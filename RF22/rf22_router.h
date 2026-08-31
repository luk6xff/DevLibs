// rf22_router.h
//
// Multi hop routed, reliably delivered messages on top of
// rf22_reliable_datagram, with a hard wired fixed size routing table.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22Router.h,v 1.8 2012/05/30 01:51:25 mikem Exp $
//   ported to mbed by Karl Zweimueller
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#ifndef __RF22_ROUTER_H__
#define __RF22_ROUTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rf22_reliable_datagram.h"

/** Default max number of hops we will route */
#define RF22_DEFAULT_MAX_HOPS               30

/** The size of the routing table we keep */
#ifndef RF22_ROUTING_TABLE_SIZE
#define RF22_ROUTING_TABLE_SIZE             10
#endif

/** Wire size of rf22_routed_message_header, checked at compile time */
#define RF22_ROUTED_MESSAGE_HEADER_LEN      5

/** Maximum application payload of a routed message */
#define RF22_ROUTER_MAX_MESSAGE_LEN \
    (RF22_MAX_MESSAGE_LEN - RF22_ROUTED_MESSAGE_HEADER_LEN)

/** Error codes returned by rf22_router_sendto_wait() */
#define RF22_ROUTER_ERROR_NONE              0
#define RF22_ROUTER_ERROR_INVALID_LENGTH    1
#define RF22_ROUTER_ERROR_NO_ROUTE          2
#define RF22_ROUTER_ERROR_TIMEOUT           3
#define RF22_ROUTER_ERROR_NO_REPLY          4
#define RF22_ROUTER_ERROR_UNABLE_TO_DELIVER 5

/**
 * @brief End to end delivery header, prepended to the application payload.
 */
typedef struct
{
    uint8_t dest;   // Destination node address
    uint8_t source; // Originator node address
    uint8_t hops;   // Hops traversed so far
    uint8_t id;     // Originator sequence number
    uint8_t flags;  // Originator flags
} rf22_routed_message_header;

/**
 * @brief A complete routed message.
 */
typedef struct
{
    rf22_routed_message_header header;
    uint8_t data[RF22_ROUTER_MAX_MESSAGE_LEN];
} rf22_routed_message;

/**
 * @brief Possible states of a routing table entry.
 */
typedef enum
{
    RF22_ROUTE_INVALID     = 0, // No valid route is known
    RF22_ROUTE_DISCOVERING = 1, // Discovering a route (not currently used)
    RF22_ROUTE_VALID       = 2, // Route is valid
} rf22_route_state;

/**
 * @brief An entry in the routing table.
 */
typedef struct
{
    uint8_t dest;     // Destination node address
    uint8_t next_hop; // Send via this next hop address
    uint8_t state;    // One of rf22_route_state
} rf22_routing_table_entry;

/**
 * @brief rf22 router dev object.
 *
 * Routes are hard wired: each node must be told the next hop for every
 * destination it wants to reach, using rf22_router_add_route_to().
 *
 * `base` must stay the first member. Everything, including the scratch message
 * buffer, is part of this struct, nothing is allocated dynamically.
 */
typedef struct
{
    rf22_reliable_datagram base;      // Must be the first member
    uint8_t last_e2e_sequence_number; // The last end to end sequence number
    uint8_t max_hops;                 // Max hops permitted in routed messages
    rf22_routing_table_entry routes[RF22_ROUTING_TABLE_SIZE];
    rf22_routed_message tmp_message;  // Scratch buffer, was static in the original
} rf22_router;

/**
 * @brief Initialise the router layer and clear the routing table.
 *
 * The radio itself must already have been brought up, that is
 * rf22_<platform>_init(&r->base.base.base, &platform_dev) must have returned
 * true.
 *
 * @param r            rf22 router device object
 * @param this_address The address to assign to this node
 * @return True on success, false otherwise
 */
bool rf22_router_init(rf22_router* const r, uint8_t this_address);

/**
 * @brief Set the maximum number of hops allowed between source and destination.
 *
 * @param r        rf22 router device object
 * @param max_hops The new value for max hops
 */
void rf22_router_set_max_hops(rf22_router* const r, uint8_t max_hops);

/**
 * @brief Add a route to the local routing table, or update it if present.
 *
 * If the table is full the oldest (first) route is retired.
 *
 * @param r        rf22 router device object
 * @param dest     The destination node address
 * @param next_hop The address of the next hop towards dest
 * @param state    The state of the route, one of rf22_route_state
 */
void rf22_router_add_route_to(rf22_router* const r, uint8_t dest,
                              uint8_t next_hop, uint8_t state);

/**
 * @brief Find the routing table entry for the given destination node.
 *
 * @param r    rf22 router device object
 * @param dest The desired destination node address
 * @return Pointer to the entry, NULL if there is no valid route
 */
rf22_routing_table_entry* rf22_router_get_route_to(rf22_router* const r,
                                                   uint8_t dest);

/**
 * @brief Delete any route to the destination node.
 *
 * @param r    rf22 router device object
 * @param dest The destination node address
 * @return True if the route was present
 */
bool rf22_router_delete_route_to(rf22_router* const r, uint8_t dest);

/**
 * @brief Delete the oldest (first) route from the local routing table.
 *
 * @param r rf22 router device object
 */
void rf22_router_retire_oldest_route(rf22_router* const r);

/**
 * @brief Clear all entries from the local routing table.
 *
 * @param r rf22 router device object
 */
void rf22_router_clear_routing_table(rf22_router* const r);

/**
 * @brief Send a message to the destination node via the routing table.
 *
 * Waits for an acknowledgement from the next hop, but not from the final
 * destination if that is a different node.
 *
 * @param r    rf22 router device object
 * @param buf  The application message data
 * @param len  Number of octets, 0 to RF22_ROUTER_MAX_MESSAGE_LEN
 * @param dest The destination node address
 * @return One of RF22_ROUTER_ERROR_*
 */
uint8_t rf22_router_sendto_wait(rf22_router* const r, const uint8_t* buf,
                                uint8_t len, uint8_t dest);

/**
 * @brief Like rf22_router_sendto_wait() but spoofing the source address.
 *
 * For internal use during routing.
 *
 * @param r      rf22 router device object
 * @param buf    The application message data
 * @param len    Number of octets, 0 to RF22_ROUTER_MAX_MESSAGE_LEN
 * @param dest   The destination node address
 * @param source The originating node address to claim
 * @return One of RF22_ROUTER_ERROR_*
 */
uint8_t rf22_router_sendto_from_source_wait(rf22_router* const r,
                                            const uint8_t* buf, uint8_t len,
                                            uint8_t dest, uint8_t source);

/**
 * @brief Receive a message for this node, or forward one towards its next hop.
 *
 * Call this often, it is also responsible for routing other nodes' traffic.
 *
 * @param         r      rf22 router device object
 * @param[out]    buf    Location to copy the application payload to
 * @param[in,out] len    On entry the space in buf, on exit the octets copied
 * @param[out]    source The originator address, may be NULL
 * @param[out]    dest   The final destination address, may be NULL
 * @param[out]    id     The end to end ID, may be NULL
 * @param[out]    flags  The end to end flags, may be NULL
 * @return True if a message for this node was copied to buf
 */
bool rf22_router_recvfrom_ack(rf22_router* const r, uint8_t* buf, uint8_t* len,
                              uint8_t* source, uint8_t* dest, uint8_t* id,
                              uint8_t* flags);

/**
 * @brief Like rf22_router_recvfrom_ack() but with a timeout.
 *
 * @param         r          rf22 router device object
 * @param[out]    buf        Location to copy the application payload to
 * @param[in,out] len        On entry the space in buf, on exit the octets copied
 * @param         timeout_ms Maximum time to wait in milliseconds
 * @param[out]    source     The originator address, may be NULL
 * @param[out]    dest       The final destination address, may be NULL
 * @param[out]    id         The end to end ID, may be NULL
 * @param[out]    flags      The end to end flags, may be NULL
 * @return True if a message for this node was copied to buf
 */
bool rf22_router_recvfrom_ack_timeout(rf22_router* const r, uint8_t* buf,
                                      uint8_t* len, uint32_t timeout_ms,
                                      uint8_t* source, uint8_t* dest,
                                      uint8_t* id, uint8_t* flags);


#ifdef __cplusplus
}
#endif

#endif // __RF22_ROUTER_H__
