// rf22_reliable_datagram.h
//
// Addressed, acknowledged, retransmitted datagrams on top of rf22_datagram.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22ReliableDatagram.h,v 1.7 2012/05/30 01:51:25 mikem Exp $
//   ported to mbed by Karl Zweimueller
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#ifndef __RF22_RELIABLE_DATAGRAM_H__
#define __RF22_RELIABLE_DATAGRAM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rf22_datagram.h"

/** The acknowledgement bit in the FLAGS header */
#define RF22_FLAGS_ACK                      0x80

/** Default minimum retransmit timeout [ms] */
#define RF22_DEFAULT_ACK_TIMEOUT_MS         200

/** Default number of retries, 0 means one try only */
#define RF22_DEFAULT_RETRIES                3

/** Bound for waiting on an ACK transmission to complete [ms] */
#define RF22_ACK_TX_TIMEOUT_MS              3000

/**
 * @brief rf22 reliable datagram dev object.
 *
 * Adds flags and sequence numbers to rf22_datagram. Messages are acknowledged
 * and unacknowledged messages are retransmitted until acknowledged or the
 * retries are exhausted.
 *
 * An ACK is a message with:
 *  - TO set to the FROM address of the original message
 *  - FROM set to this node address
 *  - ID set to the ID of the original message
 *  - FLAGS with the RF22_FLAGS_ACK bit set
 *
 * `base` must stay the first member.
 */
typedef struct
{
    rf22_datagram base;            // Datagram layer, must be the first member
    uint16_t retransmissions;      // Count of retransmissions we have sent
    uint8_t  last_sequence_number; // The last sequence number used
    uint16_t timeout_ms;           // Retransmit timeout
    uint8_t  retries;              // Max retries, 0 means one try only
    uint32_t rand_state;           // State of the retry backoff PRNG
    uint8_t  seen_ids[256];        // Last seen sequence number per node address
} rf22_reliable_datagram;

/**
 * @brief Initialise the reliable datagram layer.
 *
 * The radio itself must already have been brought up, that is
 * rf22_<platform>_init(&rd->base.base, &platform_dev) must have returned true.
 *
 * @param rd           rf22 reliable datagram device object
 * @param this_address The address to assign to this node
 * @return True on success, false otherwise
 */
bool rf22_reliable_datagram_init(rf22_reliable_datagram* const rd,
                                 uint8_t this_address);

/**
 * @brief Set the minimum retransmit timeout.
 *
 * The actual timeout is randomly varied between timeout and timeout*2 to
 * prevent collisions on every retransmit.
 *
 * @param rd         rf22 reliable datagram device object
 * @param timeout_ms The new timeout period in milliseconds
 */
void rf22_reliable_datagram_set_timeout(rf22_reliable_datagram* const rd,
                                        uint16_t timeout_ms);

/**
 * @brief Set the maximum number of retries. 0 means the message is sent once.
 *
 * @param rd      rf22 reliable datagram device object
 * @param retries The maximum number of retries
 */
void rf22_reliable_datagram_set_retries(rf22_reliable_datagram* const rd,
                                        uint8_t retries);

/**
 * @brief Return the number of retransmissions sent since initialisation.
 *
 * @param rd rf22 reliable datagram device object
 * @return The number of retransmissions
 */
uint16_t rf22_reliable_datagram_retransmissions(
    const rf22_reliable_datagram* const rd);

/**
 * @brief Send a message and wait for an acknowledgement.
 *
 * Blocks until an ACK is received or all retries are exhausted, that is up to
 * (retries + 1) * timeout milliseconds. Broadcasts are never acknowledged and
 * return true immediately after transmission.
 *
 * @param rd      rf22 reliable datagram device object
 * @param buf     The message to send
 * @param len     Number of octets to send, 1 to RF22_MAX_MESSAGE_LEN
 * @param address The address to send the message to
 * @return True if the message was transmitted and acknowledged
 */
bool rf22_reliable_datagram_sendto_wait(rf22_reliable_datagram* const rd,
                                        const uint8_t* buf, uint8_t len,
                                        uint8_t address);

/**
 * @brief Receive a message addressed to this node and acknowledge it.
 *
 * Duplicates of already seen messages are re-acknowledged and dropped.
 *
 * @param         rd    rf22 reliable datagram device object
 * @param[out]    buf   Location to copy the received message to
 * @param[in,out] len   On entry the space in buf, on exit the octets copied
 * @param[out]    from  The SRC address, may be NULL
 * @param[out]    to    The DEST address, may be NULL
 * @param[out]    id    The ID header, may be NULL
 * @param[out]    flags The FLAGS header, may be NULL
 * @return True if a new valid message was copied to buf
 */
bool rf22_reliable_datagram_recvfrom_ack(rf22_reliable_datagram* const rd,
                                         uint8_t* buf, uint8_t* len,
                                         uint8_t* from, uint8_t* to,
                                         uint8_t* id, uint8_t* flags);

/**
 * @brief Like rf22_reliable_datagram_recvfrom_ack() but with a timeout.
 *
 * @param         rd         rf22 reliable datagram device object
 * @param[out]    buf        Location to copy the received message to
 * @param[in,out] len        On entry the space in buf, on exit the octets copied
 * @param         timeout_ms Maximum time to wait in milliseconds
 * @param[out]    from       The SRC address, may be NULL
 * @param[out]    to         The DEST address, may be NULL
 * @param[out]    id         The ID header, may be NULL
 * @param[out]    flags      The FLAGS header, may be NULL
 * @return True if a new valid message was copied to buf
 */
bool rf22_reliable_datagram_recvfrom_ack_timeout(
    rf22_reliable_datagram* const rd, uint8_t* buf, uint8_t* len,
    uint32_t timeout_ms, uint8_t* from, uint8_t* to, uint8_t* id,
    uint8_t* flags);


#ifdef __cplusplus
}
#endif

#endif // __RF22_RELIABLE_DATAGRAM_H__
