// rf22_router.c
//
// Multi hop routed, reliably delivered messages on top of
// rf22_reliable_datagram, with a hard wired fixed size routing table.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22Router.cpp,v 1.7 2012/05/30 01:51:25 mikem Exp $
//   ported to mbed by Karl Zweimueller
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#include <string.h>

#include "rf22_router.h"

// The routed header goes on the wire as is, so it must be tightly packed
typedef char rf22_routed_header_size_check[
    (sizeof(rf22_routed_message_header) == RF22_ROUTED_MESSAGE_HEADER_LEN)
        ? 1 : -1];

//-----------------------------------------------------------------------------
static void rf22_router_delete_route(rf22_router* const r, uint8_t index)
{
    if (index >= RF22_ROUTING_TABLE_SIZE)
    {
        return;
    }

    // Delete a route by moving the following routes on top of it. The original
    // used memcpy on overlapping regions, which is undefined behaviour.
    memmove(&r->routes[index], &r->routes[index + 1u],
            sizeof(rf22_routing_table_entry) *
                (RF22_ROUTING_TABLE_SIZE - index - 1u));

    r->routes[RF22_ROUTING_TABLE_SIZE - 1].dest = 0;
    r->routes[RF22_ROUTING_TABLE_SIZE - 1].next_hop = 0;
    r->routes[RF22_ROUTING_TABLE_SIZE - 1].state = RF22_ROUTE_INVALID;
}

//-----------------------------------------------------------------------------
// Find the next hop and hand the message to the reliable datagram layer
static uint8_t rf22_router_route(rf22_router* const r,
                                 rf22_routed_message* const message,
                                 uint8_t message_len)
{
    uint8_t next_hop = RF22_BROADCAST_ADDRESS;

    if (message->header.dest != RF22_BROADCAST_ADDRESS)
    {
        const rf22_routing_table_entry* const route =
            rf22_router_get_route_to(r, message->header.dest);
        if (!route)
        {
            return RF22_ROUTER_ERROR_NO_ROUTE;
        }
        next_hop = route->next_hop;
    }

    if (!rf22_reliable_datagram_sendto_wait(&r->base, (const uint8_t*)message,
                                            message_len, next_hop))
    {
        return RF22_ROUTER_ERROR_UNABLE_TO_DELIVER;
    }

    return RF22_ROUTER_ERROR_NONE;
}

//-----------------------------------------------------------------------------
bool rf22_router_init(rf22_router* const r, uint8_t this_address)
{
    if (!r)
    {
        return false;
    }

    r->last_e2e_sequence_number = 0;
    r->max_hops = RF22_DEFAULT_MAX_HOPS;
    rf22_router_clear_routing_table(r);
    memset(&r->tmp_message, 0, sizeof(r->tmp_message));

    return rf22_reliable_datagram_init(&r->base, this_address);
}

//-----------------------------------------------------------------------------
void rf22_router_set_max_hops(rf22_router* const r, uint8_t max_hops)
{
    if (!r)
    {
        return;
    }

    r->max_hops = max_hops;
}

//-----------------------------------------------------------------------------
void rf22_router_add_route_to(rf22_router* const r, uint8_t dest,
                              uint8_t next_hop, uint8_t state)
{
    uint8_t i;

    if (!r)
    {
        return;
    }

    // First look for an existing entry we can update. The original matched on
    // dest alone, which could also hit a stale invalid entry.
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
        if (r->routes[i].state != RF22_ROUTE_INVALID &&
            r->routes[i].dest == dest)
        {
            r->routes[i].next_hop = next_hop;
            r->routes[i].state = state;
            return;
        }
    }

    // Look for an invalid entry we can use
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
        if (r->routes[i].state == RF22_ROUTE_INVALID)
        {
            r->routes[i].dest = dest;
            r->routes[i].next_hop = next_hop;
            r->routes[i].state = state;
            return;
        }
    }

    // Need to make room for a new one
    rf22_router_retire_oldest_route(r);

    // There is an invalid slot at the end of the table now
    r->routes[RF22_ROUTING_TABLE_SIZE - 1].dest = dest;
    r->routes[RF22_ROUTING_TABLE_SIZE - 1].next_hop = next_hop;
    r->routes[RF22_ROUTING_TABLE_SIZE - 1].state = state;
}

//-----------------------------------------------------------------------------
rf22_routing_table_entry* rf22_router_get_route_to(rf22_router* const r,
                                                   uint8_t dest)
{
    uint8_t i;

    if (!r)
    {
        return NULL;
    }

    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
        if (r->routes[i].dest == dest &&
            r->routes[i].state != RF22_ROUTE_INVALID)
        {
            return &r->routes[i];
        }
    }

    return NULL;
}

//-----------------------------------------------------------------------------
bool rf22_router_delete_route_to(rf22_router* const r, uint8_t dest)
{
    uint8_t i;

    if (!r)
    {
        return false;
    }

    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
        if (r->routes[i].state != RF22_ROUTE_INVALID &&
            r->routes[i].dest == dest)
        {
            rf22_router_delete_route(r, i);
            return true;
        }
    }

    return false;
}

//-----------------------------------------------------------------------------
void rf22_router_retire_oldest_route(rf22_router* const r)
{
    if (!r)
    {
        return;
    }

    // Obliterate the first entry in the table and invalidate the last
    rf22_router_delete_route(r, 0);
}

//-----------------------------------------------------------------------------
void rf22_router_clear_routing_table(rf22_router* const r)
{
    uint8_t i;

    if (!r)
    {
        return;
    }

    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
        r->routes[i].dest = 0;
        r->routes[i].next_hop = 0;
        r->routes[i].state = RF22_ROUTE_INVALID;
    }
}

//-----------------------------------------------------------------------------
uint8_t rf22_router_sendto_wait(rf22_router* const r, const uint8_t* buf,
                                uint8_t len, uint8_t dest)
{
    if (!r)
    {
        return RF22_ROUTER_ERROR_INVALID_LENGTH;
    }

    return rf22_router_sendto_from_source_wait(r, buf, len, dest,
                                               r->base.base.this_address);
}

//-----------------------------------------------------------------------------
// Waits for delivery to the next hop, but not to the final destination
uint8_t rf22_router_sendto_from_source_wait(rf22_router* const r,
                                            const uint8_t* buf, uint8_t len,
                                            uint8_t dest, uint8_t source)
{
    if (!r || (!buf && len > 0))
    {
        return RF22_ROUTER_ERROR_INVALID_LENGTH;
    }

    if (len > RF22_ROUTER_MAX_MESSAGE_LEN)
    {
        return RF22_ROUTER_ERROR_INVALID_LENGTH;
    }

    r->tmp_message.header.source = source;
    r->tmp_message.header.dest = dest;
    r->tmp_message.header.hops = 0;
    r->tmp_message.header.id = r->last_e2e_sequence_number;
    r->last_e2e_sequence_number = (uint8_t)(r->last_e2e_sequence_number + 1u);
    r->tmp_message.header.flags = 0;
    if (len > 0)
    {
        memcpy(r->tmp_message.data, buf, (size_t)len);
    }

    return rf22_router_route(r, &r->tmp_message,
                             (uint8_t)(RF22_ROUTED_MESSAGE_HEADER_LEN + len));
}

//-----------------------------------------------------------------------------
bool rf22_router_recvfrom_ack(rf22_router* const r, uint8_t* buf, uint8_t* len,
                              uint8_t* source, uint8_t* dest, uint8_t* id,
                              uint8_t* flags)
{
    uint8_t tmp_message_len;
    uint8_t msg_len;

    if (!r || !buf || !len)
    {
        return false;
    }

    tmp_message_len = (uint8_t)sizeof(r->tmp_message);
    if (!rf22_reliable_datagram_recvfrom_ack(&r->base,
                                             (uint8_t*)&r->tmp_message,
                                             &tmp_message_len, NULL, NULL,
                                             NULL, NULL))
    {
        return false;
    }

    // A routed message must at least carry a complete header
    if (tmp_message_len < RF22_ROUTED_MESSAGE_HEADER_LEN)
    {
        return false;
    }

    // See if it is for us or has to be routed on
    if (r->tmp_message.header.dest == r->base.base.this_address ||
        r->tmp_message.header.dest == RF22_BROADCAST_ADDRESS)
    {
        if (source)
        {
            *source = r->tmp_message.header.source;
        }
        if (dest)
        {
            *dest = r->tmp_message.header.dest;
        }
        if (id)
        {
            *id = r->tmp_message.header.id;
        }
        if (flags)
        {
            *flags = r->tmp_message.header.flags;
        }

        msg_len = (uint8_t)(tmp_message_len - RF22_ROUTED_MESSAGE_HEADER_LEN);
        if (*len > msg_len)
        {
            *len = msg_len;
        }
        if (*len > 0)
        {
            memcpy(buf, r->tmp_message.data, (size_t)*len);
        }
        return true; // It is for us
    }

    if (r->tmp_message.header.hops < r->max_hops)
    {
        // Has to be routed to the next hop.
        // REVISIT: if it fails due to no route or being unable to deliver to
        // the next hop, tell the originator. But how?
        r->tmp_message.header.hops =
            (uint8_t)(r->tmp_message.header.hops + 1u);
        (void)rf22_router_route(r, &r->tmp_message, tmp_message_len);
    }

    // Discard it and maybe wait for another
    return false;
}

//-----------------------------------------------------------------------------
bool rf22_router_recvfrom_ack_timeout(rf22_router* const r, uint8_t* buf,
                                      uint8_t* len, uint32_t timeout_ms,
                                      uint8_t* source, uint8_t* dest,
                                      uint8_t* id, uint8_t* flags)
{
    uint32_t start;
    uint8_t max_len;

    if (!r || !buf || !len)
    {
        return false;
    }

    max_len = *len;

    start = rf22_get_ms_ticks();
    do
    {
        *len = max_len;
        if (rf22_router_recvfrom_ack(r, buf, len, source, dest, id, flags))
        {
            return true;
        }
    } while ((rf22_get_ms_ticks() - start) < timeout_ms);

    return false;
}
