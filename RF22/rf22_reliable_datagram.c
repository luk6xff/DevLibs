// rf22_reliable_datagram.c
//
// Addressed, acknowledged, retransmitted datagrams on top of rf22_datagram.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22ReliableDatagram.cpp,v 1.8 2011/02/15 01:18:03 mikem Exp $
//   ported to mbed by Karl Zweimueller
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#include <string.h>

#include "rf22_reliable_datagram.h"

//-----------------------------------------------------------------------------
// The original used rand() from the C library for the retransmit backoff. A
// self contained xorshift keeps the core free of <stdlib.h> and of the shared
// global state of rand().
static uint32_t rf22_rd_rand(rf22_reliable_datagram* const rd)
{
    uint32_t x = rd->rand_state;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rd->rand_state = x;

    return x;
}

//-----------------------------------------------------------------------------
static void rf22_rd_acknowledge(rf22_reliable_datagram* const rd, uint8_t id,
                                uint8_t from)
{
    // We would prefer to send a zero length ACK, but if an RF22 receives a
    // 0 length message with a CRC error it will never receive a 0 length
    // message again until it is reset. So we send an ACK of 1 octet.
    const uint8_t ack = (uint8_t)'!';

    rf22_set_header_id(&rd->base.base, id);
    rf22_set_header_flags(&rd->base.base, RF22_FLAGS_ACK);
    (void)rf22_datagram_sendto(&rd->base, &ack, 1u, from);
    (void)rf22_wait_packet_sent(&rd->base.base, RF22_ACK_TX_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool rf22_reliable_datagram_init(rf22_reliable_datagram* const rd,
                                 uint8_t this_address)
{
    if (!rd)
    {
        return false;
    }

    rd->retransmissions = 0;
    rd->last_sequence_number = 0;
    rd->timeout_ms = RF22_DEFAULT_ACK_TIMEOUT_MS;
    rd->retries = RF22_DEFAULT_RETRIES;
    // Sequence numbers start at 1, so 0 safely means "nothing seen yet"
    memset(rd->seen_ids, 0, sizeof(rd->seen_ids));
    // xorshift must never be seeded with zero
    rd->rand_state = rf22_get_ms_ticks() | 1u;

    return rf22_datagram_init(&rd->base, this_address);
}

//-----------------------------------------------------------------------------
void rf22_reliable_datagram_set_timeout(rf22_reliable_datagram* const rd,
                                        uint16_t timeout_ms)
{
    if (!rd)
    {
        return;
    }

    rd->timeout_ms = timeout_ms;
}

//-----------------------------------------------------------------------------
void rf22_reliable_datagram_set_retries(rf22_reliable_datagram* const rd,
                                        uint8_t retries)
{
    if (!rd)
    {
        return;
    }

    rd->retries = retries;
}

//-----------------------------------------------------------------------------
uint16_t rf22_reliable_datagram_retransmissions(
    const rf22_reliable_datagram* const rd)
{
    if (!rd)
    {
        return 0;
    }

    return rd->retransmissions;
}

//-----------------------------------------------------------------------------
bool rf22_reliable_datagram_sendto_wait(rf22_reliable_datagram* const rd,
                                        const uint8_t* buf, uint8_t len,
                                        uint8_t address)
{
    uint8_t seq;
    uint16_t attempt;

    if (!rd || !buf || len == 0)
    {
        return false;
    }

    seq = (uint8_t)(rd->last_sequence_number + 1u);
    rd->last_sequence_number = seq;

    // attempt is uint16_t so that retries == 255 cannot wrap the loop counter
    for (attempt = 0; attempt <= (uint16_t)rd->retries; attempt++)
    {
        uint32_t start;
        uint32_t timeout;

        rf22_set_header_id(&rd->base.base, seq);
        rf22_set_header_flags(&rd->base.base, 0);
        if (!rf22_datagram_sendto(&rd->base, buf, len, address))
        {
            return false;
        }
        (void)rf22_wait_packet_sent(&rd->base.base, RF22_ACK_TX_TIMEOUT_MS);

        // Never wait for ACKs to broadcasts
        if (address == RF22_BROADCAST_ADDRESS)
        {
            return true;
        }

        if (attempt > 0u)
        {
            rd->retransmissions = (uint16_t)(rd->retransmissions + 1u);
        }

        // A new timeout, random between timeout and timeout*2. This prevents
        // collisions on every retransmit if 2 nodes transmit at the same time.
        timeout = (uint32_t)rd->timeout_ms +
                  (((uint32_t)rd->timeout_ms * (rf22_rd_rand(rd) % 100u)) / 100u);
        // The timeout does not include the original transmit time
        start = rf22_get_ms_ticks();

        while ((rf22_get_ms_ticks() - start) < timeout)
        {
            uint8_t from;
            uint8_t to;
            uint8_t id;
            uint8_t flags;

            if (!rf22_available(&rd->base.base))
            {
                continue;
            }

            from = rf22_header_from(&rd->base.base);
            to = rf22_header_to(&rd->base.base);
            id = rf22_header_id(&rd->base.base);
            flags = rf22_header_flags(&rd->base.base);
            // Not using rf22_recv(), so clear the buffer ourselves
            rf22_clear_rx_buf(&rd->base.base);

            if (from == address && to == rd->base.this_address &&
                (flags & RF22_FLAGS_ACK) != 0u && id == seq)
            {
                return true; // The ACK we are waiting for
            }

            if ((flags & RF22_FLAGS_ACK) == 0u && id == rd->seen_ids[from])
            {
                // A request we have already received, ACK it again
                rf22_rd_acknowledge(rd, id, from);
            }
            // Else discard it and keep waiting until the timeout is exhausted
        }
    }

    return false;
}

//-----------------------------------------------------------------------------
bool rf22_reliable_datagram_recvfrom_ack(rf22_reliable_datagram* const rd,
                                         uint8_t* buf, uint8_t* len,
                                         uint8_t* from, uint8_t* to,
                                         uint8_t* id, uint8_t* flags)
{
    uint8_t msg_from;
    uint8_t msg_to;
    uint8_t msg_id;
    uint8_t msg_flags;

    if (!rd || !buf || !len)
    {
        return false;
    }

    // Get the message before it is clobbered by the ACK, the RF22 shares one
    // buffer between rx and tx
    if (!rf22_available(&rd->base.base))
    {
        return false;
    }
    if (!rf22_datagram_recvfrom(&rd->base, buf, len, &msg_from, &msg_to,
                                &msg_id, &msg_flags))
    {
        return false;
    }

    // Never ACK an ACK
    if ((msg_flags & RF22_FLAGS_ACK) != 0u)
    {
        return false;
    }

    if (msg_to != RF22_BROADCAST_ADDRESS)
    {
        // Not a broadcast, so ACK it with the received ID
        rf22_rd_acknowledge(rd, msg_id, msg_from);
    }

    // If we have not seen this message before then we are interested in it
    if (msg_id == rd->seen_ids[msg_from])
    {
        return false; // A duplicate, it has just been re-acked
    }

    if (from)
    {
        *from = msg_from;
    }
    if (to)
    {
        *to = msg_to;
    }
    if (id)
    {
        *id = msg_id;
    }
    if (flags)
    {
        *flags = msg_flags;
    }
    rd->seen_ids[msg_from] = msg_id;

    return true;
}

//-----------------------------------------------------------------------------
bool rf22_reliable_datagram_recvfrom_ack_timeout(
    rf22_reliable_datagram* const rd, uint8_t* buf, uint8_t* len,
    uint32_t timeout_ms, uint8_t* from, uint8_t* to, uint8_t* id,
    uint8_t* flags)
{
    uint32_t start;
    uint8_t max_len;

    if (!rd || !buf || !len)
    {
        return false;
    }

    // A rejected message (an ACK or a duplicate) shrinks *len, so the capacity
    // has to be restored before every attempt
    max_len = *len;

    // The original never started its Timer here, so the loop never terminated
    start = rf22_get_ms_ticks();
    do
    {
        *len = max_len;
        if (rf22_reliable_datagram_recvfrom_ack(rd, buf, len, from, to, id,
                                                flags))
        {
            return true;
        }
    } while ((rf22_get_ms_ticks() - start) < timeout_ms);

    return false;
}
