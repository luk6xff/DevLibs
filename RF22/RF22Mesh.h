// RF22Mesh.h
//
// Author: Mike McCauley (mikem@open.com.au)
// Copyright (C) 2011 Mike McCauley
// $Id: RF22Mesh.h,v 1.4 2012/05/30 01:51:25 mikem Exp $
// ported to mbed by Karl Zweimueller

#ifndef RF22Mesh_h
#define RF22Mesh_h

#include <RF22Router.h>

// Types of RF22Mesh message, used to set msgType in the RF22MeshHeader
#define RF22_MESH_MESSAGE_TYPE_APPLICATION                    0
#define RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST        1
#define RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE       2
#define RF22_MESH_MESSAGE_TYPE_ROUTE_FAILURE                  3

/////////////////////////////////////////////////////////////////////
/// \class RF22Mesh RF22Mesh.h <RF22Mesh.h>
/// \brief RF22 subclass for sending addressed, optionally acknowledged datagrams
/// multi-hop routed across a network, with automatic route discovery
///
/// Extends RF22Router to add automatic route discovery within a mesh of adjacent nodes, 
/// and route signalling.
///
/// Unlike RF22Router, RF22Mesh can be used in networks where the network topology is fluid, or unknown, 
/// or if nodes can mode around or go in or out of service. When a node wants to send a 
/// message to another node, it will automcatically discover a route to the destaintion node and use it. 
/// If the route becomes unavailable, a new route will be discovered.
///
/// \par Route Discovery
///
/// When a RF22Mesh mesh node is initialised, it doe not know any routes to any other nodes 
/// (see RF22Router for details on route and the routing table).
/// When you attempt to send a message with sendtoWait, will first check to see if there is a route to the 
/// destinastion node in the routing tabl;e. If not, it wil initialite 'Route Discovery'.
/// When a node needs to discover a route to another node, it broadcasts MeshRouteDiscoveryMessage 
/// with a message type of RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST. 
/// Any node that receives such a request checks to see if it is a request for a route to itself
/// (in which case it makes a unicast reply to the originating node with a 
/// MeshRouteDiscoveryMessage 
/// with a message type of RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE) 
/// otherwise it rebroadcasts the request, after adding itself to the list of nodes visited so 
/// far by the request.
///
/// If a node receives a RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST that already has itself 
/// listed in the visited nodes, it knows it has already seen and rebroadcast this request, 
/// and threfore ignores it. This prevents broadcast storms.
/// When a node receives a RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST it can use the list of 
/// nodes aready visited to deduce routes back towards the originating (requesting node). 
/// This also means that when the destination node of the request is reached, it (and all 
/// the previous nodes the request visited) will have a route back to the originating node. 
/// This means the unicast RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE 
/// reply will be routed successfully back to the original route requester.
///
/// The RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE sent back by the destination node contains 
/// the full list of nodes that were visited on the way to the destination.
/// Therefore, intermediate nodes that route the reply back towards the originating node can use the 
/// node list in the reply to deduce routes to all the nodes between it and the destination node.
///
/// Therefore, RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST and 
/// RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE together ensure the original requester and all 
/// the intermediate nodes know how to route to the source and destination nodes and every node along the path.
///
/// Note that there is a race condition here that can effect routing on multipath routes. For example, 
/// if the route to the destination can traverse several paths, last reply from the destination 
/// will be the one used.
///
/// \par Route Failure
///
/// RF22Router (and therefore RF22Mesh) use reliable hop-to-hop delivery of messages using 
/// hop-to-hop acknowledgements, but not end-to-end acknowledgements. When sendtoWait() returns, 
/// you know that the message has been delivered to the next hop, but not if it is (or even if it can be) 
/// delivered to the destination node. If during the course of hop-to-hop routing of a message, 
/// one of the intermediate RF22Mesh nodes finds it cannot deliver to the next hop 
/// (say due to a lost route or no acknwledgement from the next hop), it replies to the 
/// originator with a unicast MeshRouteFailureMessage RF22_MESH_MESSAGE_TYPE_ROUTE_FAILURE message. 
/// Intermediate nodes (on the way beack to the originator)
/// and the originating node use this message to delete the route to the destination 
/// node of the original message. This means that if a route to a destination becomes unusable 
/// (either because an intermediate node is off the air, or has moved out of range) a new route 
/// will be established the next time a message is to be sent.
///
/// \par Message Format
///
/// RF22Mesh uses a number of message formats layered on top of RF22Router:
/// - MeshApplicationMessage (message type RF22_MESH_MESSAGE_TYPE_APPLICATION). 
///   Carries an application layer message for the caller of RF22Mesh
/// - MeshRouteDiscoveryMessage (message types RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST 
///   and RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_RESPONSE). Carries Route Discovery messages 
///   (broadcast) and replies (unicast).
/// - MeshRouteFailureMessage (message type RF22_MESH_MESSAGE_TYPE_ROUTE_FAILURE) Informs nodes of 
///   route failures.
///
/// Part of the Arduino RF22 library for operating with HopeRF RF22 compatible transceivers 
/// (see http://www.hoperf.com)
class RF22Mesh : public RF22Router
{
public:

    /// The maximum length permitted for the application payload data in a RF22Mesh message
    #define RF22_MESH_MAX_MESSAGE_LEN (RF22_ROUTER_MAX_MESSAGE_LEN - sizeof(RF22Mesh::MeshMessageHeader))

    /// Structure of the basic RF22Mesh header.
    typedef struct
    {
    uint8_t             msgType;  ///< Type of RF22Mesh message, one of RF22_MESH_MESSAGE_TYPE_*
    } MeshMessageHeader;

    /// Signals an application layer message for the caller of RF22Mesh
    typedef struct
    {
    MeshMessageHeader   header; ///< msgType = RF22_MESH_MESSAGE_TYPE_APPLICATION 
    uint8_t             data[RF22_MESH_MAX_MESSAGE_LEN]; ///< Applicaiotn layer payload data
    } MeshApplicationMessage;

    /// Signals a route discovery request or reply
    /// At present only supports physical dest addresses of length 1 octet
    typedef struct
    {
    MeshMessageHeader   header; ///< msgType = RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_*
    uint8_t             destlen; ///< Reserved. Must be 1.g
    uint8_t             dest;   ///< The address of the destination node whose route is being sought
    uint8_t             route[RF22_MESH_MAX_MESSAGE_LEN - 1]; ///< List of node addresses visited so far. Length is implcit
    } MeshRouteDiscoveryMessage;

    /// Signals a route failure
    typedef struct
    {
    MeshMessageHeader   header; ///< msgType = RF22_MESH_MESSAGE_TYPE_ROUTE_FAILURE
    uint8_t             dest; ///< The address of the destination towards which the route failed
    } MeshRouteFailureMessage;

    /// Constructor. 
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RF22 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega)
    /// \param[in] interrupt The interrupt number to use. Default is interrupt 0 (Arduino input pin 2)
    RF22Mesh(uint8_t thisAddress , PinName slaveSelectPin , PinName mosi, PinName miso, PinName sclk, PinName interrupt,PinName shutdownPin );

    /// Sends a message to the destination node. Initialises the RF22Router message header 
    /// (the SOURCE address is set to the address of this node, HOPS to 0) and calls 
    /// route() which looks up in the routing table the next hop to deliver to.
    /// If no route is known, initiates route discovery and waits for a reply.
    /// Then sends the message to the next hop
    /// Then waits for an acknowledgement from the next hop 
    /// (but not from the destination node (if that is different).
    /// \param [in] buf The application message data
    /// \param [in] len Number of octets in the application message data. 0 is permitted
    /// \param [in] dest The destination node address
    /// \return The result code:
    ///         - RF22_ROUTER_ERROR_NONE Message was routed and deliverd to the next hop 
    ///           (not necessarily to the final dest address)
    ///         - RF22_ROUTER_ERROR_NO_ROUTE There was no route for dest in the local routing table
    ///         - RF22_ROUTER_ERROR_UNABLE_TO_DELIVER Noyt able to deliver to the next hop 
    ///           (usually because it dod not acknowledge due to being off the air or out of range
    uint8_t sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest);

    /// Starts the receiver if it is not running already.
    /// If there is a valid application layer message available for this node (or RF22_BROADCAST_ADDRESS), 
    /// send an acknowledgement to the last hop
    /// address (blocking until this is complete), then copy the application message payload data
    /// to buf and return true
    /// else return false. 
    /// If a message is copied, *len is set to the length..
    /// If from is not NULL, the originator SOURCE address is placed in *source.
    /// If to is not NULL, the DEST address is placed in *dest. This might be this nodes address or 
    /// RF22_BROADCAST_ADDRESS. 
    /// This is the preferred function for getting messages addressed to this node.
    /// If the message is not a broadcast, acknowledge to the sender before returning.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Available space in buf. Set to the actual number of octets copied.
    /// \param[in] source If present and not NULL, the referenced uint8_t will be set to the SOURCE address
    /// \param[in] dest If present and not NULL, the referenced uint8_t will be set to the DEST address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// (not just those addressed to this node).
    /// \return true if a valid message was recvived for this node and copied to buf
    boolean recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL);

    /// Starts the receiver if it is not running already.
    /// Similar to recvfromAck(), this will block until either a valid application layer 
    /// message available for this node
    /// or the timeout expires. 
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Available space in buf. Set to the actual number of octets copied.
    /// \param[in] timeout Maximum time to wait in milliseconds
    /// \param[in] source If present and not NULL, the referenced uint8_t will be set to the SOURCE address
    /// \param[in] dest If present and not NULL, the referenced uint8_t will be set to the DEST address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// (not just those addressed to this node).
    /// \return true if a valid message was copied to buf
    boolean recvfromAckTimeout(uint8_t* buf, uint8_t* len,  uint16_t timeout, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL);

protected:

    /// Internal function that inspects messages being received and adjusts the routing table if necessary.
    /// Called by recvfromAck() immediately after it gets the message from RF22ReliableDatagram
    /// \param [in] message Pointer to the RF22Router message that was received.
    /// \param [in] messageLen Length of message in octets
    virtual void peekAtMessage(RoutedMessage* message, uint8_t messageLen);

    /// Internal function that inspects messages being received and adjusts the routing table if necessary.
    /// This is virtual, which lets subclasses override or intercept the route() function.
    /// Called by sendtoWait after the message header has been filled in.
    /// \param [in] message Pointer to the RF22Router message to be sent.
    /// \param [in] messageLen Length of message in octets
    virtual uint8_t route(RoutedMessage* message, uint8_t messageLen);

    /// Try to resolve a route for the given address. Blocks while discovering the route
    /// which may take up to 4000 msec.
    /// Virtual so subclasses can override.
    /// \param [in] address The physical addres to resolve
    /// \return true if the address was resolved and added to the local routing table
    virtual boolean doArp(uint8_t address);

    /// Tests if the given address of length addresslen is indentical to the
    /// physical addres of this node.
    /// RF22Mesh always ikmplements p[hysical addresses as the 1 octet address of the node
    /// given by _thisAddress
    /// Called by recvfromAck() to test whether a RF22_MESH_MESSAGE_TYPE_ROUTE_DISCOVERY_REQUEST
    /// is for this node.
    /// Subclasses may want to override to implemnt mode complicated or longer physical addresses
    /// \param [in] address Address of the pyysical addres being tested
    /// \param [in] addresslen Lengthof the address in bytes
    /// \return true if the physical address of this node is identical to address
    virtual boolean isPhysicalAddress(uint8_t* address, uint8_t addresslen);

private:
    /// Temporary mesage buffer
    static uint8_t _tmpMessage[RF22_ROUTER_MAX_MESSAGE_LEN];

};

#endif
