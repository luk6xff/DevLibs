// rf22_datagram.h
//
// Addressed, unreliable datagrams on top of the portable rf22 core.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22Datagram.h,v 1.4 2012/05/30 01:50:21 mikem Exp $
//   ported to mbed by Karl Zweimueller
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#ifndef __RF22_DATAGRAM_H__
#define __RF22_DATAGRAM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rf22.h"

/**
 * @brief rf22 datagram dev object.
 *
 * Every node has an 8 bit address (defaults to 0). An address of
 * RF22_BROADCAST_ADDRESS (0xff) is reserved for broadcast.
 *
 * `base` must stay the first member, so that a pointer to an rf22_datagram can
 * be used wherever an rf22* is expected.
 */
typedef struct
{
    rf22 base;            // Radio driver state, must be the first member
    uint8_t this_address; // The address of this node
} rf22_datagram;

/**
 * @brief Initialise the datagram layer.
 *
 * The radio itself must already have been brought up, that is
 * rf22_<platform>_init(&dg->base, &platform_dev) must have returned true.
 *
 * @param dg           rf22 datagram device object
 * @param this_address The address to assign to this node
 * @return True on success, false otherwise
 */
bool rf22_datagram_init(rf22_datagram* const dg, uint8_t this_address);

/**
 * @brief Set the address of this node.
 *
 * Used as the FROM address of all messages sent by this node and as the
 * accepted TO address of received messages.
 *
 * @param dg           rf22 datagram device object
 * @param this_address The address of this node
 */
void rf22_datagram_set_this_address(rf22_datagram* const dg,
                                    uint8_t this_address);

/**
 * @brief Return the address of this node.
 *
 * @param dg rf22 datagram device object
 * @return The address of this node, 0 if dg is NULL
 */
uint8_t rf22_datagram_this_address(const rf22_datagram* const dg);

/**
 * @brief Send a message to the node(s) with the given address.
 *
 * @param dg      rf22 datagram device object
 * @param buf     The message to send
 * @param len     Number of octets to send, 1 to RF22_MAX_MESSAGE_LEN
 * @param address The address to send the message to
 * @return True if the message was queued for transmission
 */
bool rf22_datagram_sendto(rf22_datagram* const dg, const uint8_t* buf,
                          uint8_t len, uint8_t address);

/**
 * @brief Copy a received message addressed to this node out of the driver.
 *
 * @param         dg    rf22 datagram device object
 * @param[out]    buf   Location to copy the received message to
 * @param[in,out] len   On entry the space in buf, on exit the octets copied
 * @param[out]    from  The FROM address, may be NULL
 * @param[out]    to    The TO address, may be NULL
 * @param[out]    id    The ID header, may be NULL
 * @param[out]    flags The FLAGS header, may be NULL
 * @return True if a valid message was copied to buf
 */
bool rf22_datagram_recvfrom(rf22_datagram* const dg, uint8_t* buf, uint8_t* len,
                            uint8_t* from, uint8_t* to, uint8_t* id,
                            uint8_t* flags);


#ifdef __cplusplus
}
#endif

#endif // __RF22_DATAGRAM_H__
