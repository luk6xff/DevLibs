// RF22Router.h
//
// Author: Mike McCauley (mikem@open.com.au)
// Copyright (C) 2011 Mike McCauley
// $Id: RF22Router.h,v 1.8 2012/05/30 01:51:25 mikem Exp $
// ported to mbed by Karl Zweimueller

#ifndef RF22Router_h
#define RF22Router_h

#include <RF22ReliableDatagram.h>

// Default max number of hops we will route
#define RF22_DEFAULT_MAX_HOPS 30

// The default size of the routing table we keep
#define RF22_ROUTING_TABLE_SIZE 10

// Error codes
#define RF22_ROUTER_ERROR_NONE              0
#define RF22_ROUTER_ERROR_INVALID_LENGTH    1
#define RF22_ROUTER_ERROR_NO_ROUTE          2
#define RF22_ROUTER_ERROR_TIMEOUT           3
#define RF22_ROUTER_ERROR_NO_REPLY          4
#define RF22_ROUTER_ERROR_UNABLE_TO_DELIVER 5

// This size of RF22_ROUTER_MAX_MESSAGE_LEN is OK for Arduino Mega, but too big for
// Duemilanova. Size of 50 works with the sample router programs on Duemilanova.
#define RF22_ROUTER_MAX_MESSAGE_LEN (RF22_MAX_MESSAGE_LEN - sizeof(RF22Router::RoutedMessageHeader))
//#define RF22_ROUTER_MAX_MESSAGE_LEN 50

// These allow us to define a simulated network topology for testing purposes
// See RF22Router.cpp for details
//#define RF22_TEST_NETWORK 1
//#define RF22_TEST_NETWORK 2
//#define RF22_TEST_NETWORK 3
//#define RF22_TEST_NETWORK 4

/////////////////////////////////////////////////////////////////////
/// \class RF22Router RF22Router.h <RF22Router.h>
/// \brief RF22 subclass for sending addressed, optionally acknowledged datagrams
/// multi-hop routed across a network.
///
/// Extends RF22ReliableDatagram to define addressed messages
/// That are reliably transmitted and routed across a network. Each message is transmitted reliably 
/// between each hop in order to get from the source node to the destination node.
///
/// With RF22Router, routes are hard wired. This means that each node must have programmed 
/// in it how to reach each of the other nodes it will be trying to communicate with. 
/// This means you must specify the next-hop node address for each of the destination nodes, 
/// using the addRouteTo() function. 
///
/// When sendtoWait() is called with a new message to deliver, and the destination address,
/// RF22Router looks up the next hop node for the destination node. It then uses 
/// RF22ReliableDatagram to (reliably) deliver the message to the next hop 
/// (which is expected also to be running an RF22Router). If that next-hop node is not
/// the final destination, it will also look up the next hop for the destination node and 
/// (reliably) deliver the message to the next hop. By this method, messages can be delivered 
/// across a network of nodes, even if each node cannot hear all of the others in the network.
/// Each time a message is received for another node and retransmitted to the next hop, 
/// the HOPS filed in teh header is incremented. If a message is received for routing to another node 
/// which has exceed the routers max_hops, the message wioll be dropped and ignored. 
/// This helps prevent infinite routing loops.
///
/// RF22Router supports messages with a dest of RF22_BROADCAST_ADDRESS. Such messages are not routed, 
/// and are broadcast (once) to all nodes within range.
///
/// The recvfromAck() function is responsible not just for receiving and delivering 
/// messages addressed to this node (or RF22_BROADCAST_ADDRESS), but 
/// it is also responsible for routing other message to their next hop. This means that it is important to 
/// call recvfromAck() or recvfromAckTimeout() frequently in your main loop. recvfromAck() will return 
/// false if it receives a message but it is not for this node.
///
/// RF22Router does not provide reliable end-to-end delivery, but uses reliable hop-to-hop delivery. 
/// If a message is unable to be delivered to an end node during to a delivery failure between 2 hops, 
/// the source node will not be told about it.
///
/// Note: This class is most useful for networks of nodes that are essentially static 
/// (i.e. the nodes dont move around), and for which the 
/// routing never changes. If that is not the case for your proposed network, see RF22Mesh instead.
///
/// \par The Routing Table
///
/// The routing table is a local table in RF22Router that holds the information about the next hop node 
/// address for each destination address you may want to send a message to. It is your responsibility 
/// to make sure every node in an RF22Router network has been configured with a unique address and the 
/// routing information so that messages are correctly routed across the network from source node to 
/// destination node. This is usually done once in setup() by calling addRouteTo(). 
/// The hardwired routing will in general be different on each node, and will depend on the physical 
/// topololgy of the network.
/// You can also use addRouteTo() to change a route and 
/// deleteRouteTo() to delete a route at run time. Youcan also clear the entire routing table
///
/// The Routing Table has limited capacity for entries (defined by RF22_ROUTING_TABLE_SIZE, which is 10)
/// if more than RF22_ROUTING_TABLE_SIZE are added, the oldest (first) one will be removed by calling 
/// retireOldestRoute()
///
/// \par Message Format
///
/// RF22Router add to the lower level RF22ReliableDatagram (and even lower level RF22) class mesage formats. 
/// In those lower level classes, the hop-to-hop message headers are in the RF22 message headers, 
/// and are handled automcatically by tyhe RF22 hardware.
/// RF22Router and its subclasses add an end-to-end addressing header in the payload of the RF22 message, 
/// and before the RF22Router application data.
/// - 1 octet DEST, the destination node address (ie the address of the final 
///   destination node for this message)
/// - 1 octet SOURCE, the source node address (ie the address of the originating node that first sent 
///   the message).
/// - 1 octet HOPS, the number of hops this message has traversed so far.
/// - 1 octet ID, an incrementing message ID for end-to-end message tracking for use by subclasses. 
///   Not used by RF22Router.
/// - 1 octet FLAGS, a bitmask for use by subclasses. Not used by RF22Router.
/// - 0 or more octets DATA, the application payload data. The length of this data is implicit 
///   in the length of the entire message.
///
/// You should be careful to note that there are ID and FLAGS fields in the low level per-hop 
/// message header too. These are used only for hop-to-hop, and in general will be different to 
/// the ones at the RF22Router level.
///
/// \par Testing
///
/// Bench testing of such networks is notoriously difficult, especially simulating limited radio 
/// connectivity between some nodes.
/// To assist testing (both during RF22 development and for your own networks) 
/// RF22Router.cpp has the ability to 
/// simulate a number of different small network topologies. Each simulated network supports 4 nodes with 
/// addresses 1 to 4. It operates by pretending to not hear RF22 messages from certain other nodes.
/// You can enable testing with a \#define TEST_NETWORK in RF22Router.h
/// The sample programs rf22_mesh_* rely on this feature.
///
/// Part of the Arduino RF22 library for operating with HopeRF RF22 compatible transceivers 
/// (see http://www.hoperf.com)
class RF22Router : public RF22ReliableDatagram
{
public:

    /// Defines the structure of the RF22Router message header, used to keep track of end-to-end delivery
    /// parameters
    typedef struct
    {
    uint8_t    dest;       ///< Destination node address
    uint8_t    source;     ///< Originator node address
    uint8_t    hops;       ///< Hops traversed so far
    uint8_t    id;         ///< Originator sequence number
    uint8_t    flags;      ///< Originator flags
    // Data follows, Length is implicit in the overall message length
    } RoutedMessageHeader;

    /// Defines the structure of a RF22Router message
    typedef struct
    {
    RoutedMessageHeader header;    ///< end-to-end delivery header
    uint8_t             data[RF22_ROUTER_MAX_MESSAGE_LEN]; ///< Applicaiton payload data
    } RoutedMessage;

    /// Values for the possible states for routes
    typedef enum
    {
    Invalid = 0,           ///< No valid route is known
    Discovering,           ///< Discovering a route (not currently used)
    Valid                  ///< Route is valid
    } RouteState;

    /// Defines an entry in the routing table
    typedef struct
    {
    uint8_t      dest;      ///< Destination node address
    uint8_t      next_hop;  ///< Send via this next hop address
    uint8_t      state;     ///< State of this route, one of RouteState
    } RoutingTableEntry;

    /// Constructor. 
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RF22 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega)
    /// \param[in] interrupt The interrupt number to use. Default is interrupt 0 (Arduino input pin 2)
    //RF22Router(uint8_t thisAddress = 0, uint8_t slaveSelectPin = 10, uint8_t interrupt = 0);
    RF22Router(uint8_t thisAddress ,PinName slaveSelectPin , PinName mosi, PinName miso, PinName sclk, PinName interrupt ,PinName shutdownPin);
    /// Initialises this instance and the radio module connected to it.
    /// Overrides the init() function in RF22.
    /// Sets max_hops to the default of RF22_DEFAULT_MAX_HOPS (30)
    boolean init();

    /// Sets the max_hops to the given value
    /// This controls the maximum number of hops allowed between source and destination nodes
    /// Messages that are not delivered by the time their HOPS field exceeds max_hops on a 
    /// routing node will be dropped and ignored.
    /// \param [in] max_hops The new value for max_hops
    void setMaxHops(uint8_t max_hops);

    /// Adds a route to the local routing table, or updates it if already present.
    /// If there is not enough room the oldest (first) route will be deleted by calling retireOldestRoute().
    /// \param [in] dest The destination node address. RF22_BROADCAST_ADDRESS is permitted.
    /// \param [in] next_hop The address of the next hop to send messages destined for dest
    /// \param [in] state The satte of the route. Defaults to Valid
    void addRouteTo(uint8_t dest, uint8_t next_hop, uint8_t state = Valid);

    /// Finds and returns a RoutingTableEntry for the given destination node
    /// \param [in] dest The desired destination node address.
    /// \return pointer to a RoutingTableEntry for dest
    RoutingTableEntry* getRouteTo(uint8_t dest);

    /// Deletes from the local routing table any route for the destination node.
    /// \param [in] dest The destination node address
    /// \return true if the route was present
    boolean deleteRouteTo(uint8_t dest);

    /// Deletes the oldest (first) route from the 
    /// local routing table
    void retireOldestRoute();

    /// Clears all entries from the 
    /// local routing table
    void clearRoutingTable();

    /// If RF22_HAVE_SERIAL is defined, this will print out the contents of the local 
    /// routing table using Serial
    void printRoutingTable();

    /// Sends a message to the destination node. Initialises the RF22Router message header 
    /// (the SOURCE address is set to the address of this node, HOPS to 0) and calls 
    /// route() which looks up in the routing table the next hop to deliver to and sends the 
    /// message to the next hop. Waits for an acknowledgement from the next hop 
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

    /// Similar to sendtoWait() above, but spoofs the source address.
    /// For internal use only during routing
    /// \param [in] buf The application message data
    /// \param [in] len Number of octets in the application message data. 0 is permitted
    /// \param [in] dest The destination node address
    /// \param [in] source The (fake) originatong node address.
    /// \return The result code:
    ///         - RF22_ROUTER_ERROR_NONE Message was routed and deliverd to the next hop 
    ///           (not necessarily to the final dest address)
    ///         - RF22_ROUTER_ERROR_NO_ROUTE There was no route for dest in the local routing table
    ///         - RF22_ROUTER_ERROR_UNABLE_TO_DELIVER Noyt able to deliver to the next hop 
    ///           (usually because it dod not acknowledge due to being off the air or out of range
    uint8_t sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t source);

    /// Starts the receiver if it is not running already.
    /// If there is a valid message available for this node (or RF22_BROADCAST_ADDRESS), 
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
    /// \return true if a valid message was recvived for this node copied to buf
    boolean recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* source = NULL, uint8_t* dest = NULL, uint8_t* id = NULL, uint8_t* flags = NULL);

    /// Starts the receiver if it is not running already.
    /// Similar to recvfromAck(), this will block until either a valid message available for this node
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

    /// Lets sublasses peek at messages going 
    /// past before routing or local delivery.
    /// Called by recvfromAck() immediately after it gets the message from RF22ReliableDatagram
    /// \param [in] message Pointer to the RF22Router message that was received.
    /// \param [in] messageLen Length of message in octets
    virtual void peekAtMessage(RoutedMessage* message, uint8_t messageLen);

    /// Finds the next-hop route and sends the message via RF22ReliableDatagram::sendtoWait().
    /// This is virtual, which lets subclasses override or intercept the route() function.
    /// Called by sendtoWait after the message header has been filled in.
    /// \param [in] message Pointer to the RF22Router message to be sent.
    /// \param [in] messageLen Length of message in octets
    virtual uint8_t route(RoutedMessage* message, uint8_t messageLen);

    /// Deletes a specific rout entry from therouting table
    /// \param [in] index The 0 based index of the routing table entry to delete
    void deleteRoute(uint8_t index);

    /// The last end-to-end sequence number to be used
    /// Defaults to 0
    uint8_t _lastE2ESequenceNumber;

    /// The maximum number of hops permitted in routed messages.
    /// If a routed message would exceed this number of hops it is dropped and ignored.
    uint8_t              _max_hops;

private:

    /// Temporary mesage buffer
    static RoutedMessage _tmpMessage;

    /// Local routing table
    RoutingTableEntry    _routes[RF22_ROUTING_TABLE_SIZE];
};

#endif
