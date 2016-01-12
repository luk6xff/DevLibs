// RF22Router.cpp
//
// Define addressed datagram
// 
// Part of the Arduino RF22 library for operating with HopeRF RF22 compatible transceivers 
// (see http://www.hoperf.com)
// RF22Datagram will be received only by the addressed node or all nodes within range if the 
// to address is RF22_BROADCAST_ADDRESS
//
// Author: Mike McCauley (mikem@open.com.au)
// Copyright (C) 2011 Mike McCauley
// $Id: RF22Router.cpp,v 1.7 2012/05/30 01:51:25 mikem Exp $
// ported to mbed by Karl Zweimueller

#include <mbed.h>
#include <RF22Router.h>
//#include <SPI.h>


RF22Router::RoutedMessage RF22Router::_tmpMessage;

////////////////////////////////////////////////////////////////////
// Constructors
RF22Router::RF22Router(uint8_t thisAddress ,PinName slaveSelectPin , PinName mosi, PinName miso, PinName sclk, PinName interrupt, PinName shutdownPin ) 
    : RF22ReliableDatagram(thisAddress, slaveSelectPin, mosi,  miso, sclk, interrupt, shutdownPin)
{
    _max_hops = RF22_DEFAULT_MAX_HOPS;
    clearRoutingTable();
}

////////////////////////////////////////////////////////////////////
// Public methods
boolean RF22Router::init()
{
    boolean ret = RF22ReliableDatagram::init();
    if (ret)
    _max_hops = RF22_DEFAULT_MAX_HOPS;
    return ret;
}

////////////////////////////////////////////////////////////////////
void RF22Router::setMaxHops(uint8_t max_hops)
{
    _max_hops = max_hops;
}

////////////////////////////////////////////////////////////////////
void RF22Router::addRouteTo(uint8_t dest, uint8_t next_hop, uint8_t state)
{
    uint8_t i;

    // First look for an existing entry we can update
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
    if (_routes[i].dest == dest)
    {
        _routes[i].dest = dest;
        _routes[i].next_hop = next_hop;
        _routes[i].state = state;
        return;
    }
    }

    // Look for an invalid entry we can use
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
    if (_routes[i].state == Invalid)
    {
        _routes[i].dest = dest;
        _routes[i].next_hop = next_hop;
        _routes[i].state = state;
        return;
    }
    }

    // Need to make room for a new one
    retireOldestRoute();
    // Should be an invalid slot now
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
    if (_routes[i].state == Invalid)
    {
        _routes[i].dest = dest;
        _routes[i].next_hop = next_hop;
        _routes[i].state = state;
    }
    }
}

////////////////////////////////////////////////////////////////////
RF22Router::RoutingTableEntry* RF22Router::getRouteTo(uint8_t dest)
{
    uint8_t i;
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    if (_routes[i].dest == dest && _routes[i].state != Invalid)
        return &_routes[i];
    return NULL;
}

////////////////////////////////////////////////////////////////////
void RF22Router::deleteRoute(uint8_t index)
{
    // Delete a route by copying following routes on top of it
    memcpy(&_routes[index], &_routes[index+1], 
       sizeof(RoutingTableEntry) * (RF22_ROUTING_TABLE_SIZE - index - 1));
    _routes[RF22_ROUTING_TABLE_SIZE - 1].state = Invalid;
}

////////////////////////////////////////////////////////////////////
void RF22Router::printRoutingTable()
{
#ifdef RF22_HAVE_SERIAL
    uint8_t i;
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
    Serial.print(i, DEC);
    Serial.print(" Dest: ");
    Serial.print(_routes[i].dest, DEC);
    Serial.print(" Next Hop: ");
    Serial.print(_routes[i].next_hop, DEC);
    Serial.print(" State: ");
    Serial.println(_routes[i].state, DEC);
    }
#endif
}

////////////////////////////////////////////////////////////////////
boolean RF22Router::deleteRouteTo(uint8_t dest)
{
    uint8_t i;
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    {
    if (_routes[i].dest == dest)
    {
        deleteRoute(i);
        return true;
    }
    }
    return false;
}

////////////////////////////////////////////////////////////////////
void RF22Router::retireOldestRoute()
{
    // We just obliterate the first in the table and clear the last
    deleteRoute(0);
}

////////////////////////////////////////////////////////////////////
void RF22Router::clearRoutingTable()
{
    uint8_t i;
    for (i = 0; i < RF22_ROUTING_TABLE_SIZE; i++)
    _routes[i].state = Invalid;
}


uint8_t RF22Router::sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest)
{
    return sendtoWait(buf, len, dest, _thisAddress);
}

////////////////////////////////////////////////////////////////////
// Waits for delivery to the next hop (but not for delivery to the final destination)
uint8_t RF22Router::sendtoWait(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t source)
{
    if (((uint16_t)len + sizeof(RoutedMessageHeader)) > RF22_MAX_MESSAGE_LEN)
    return RF22_ROUTER_ERROR_INVALID_LENGTH;

    // Construct a RF22 RouterMessage message
    _tmpMessage.header.source = source;
    _tmpMessage.header.dest = dest;
    _tmpMessage.header.hops = 0;
    _tmpMessage.header.id = _lastE2ESequenceNumber++;
    _tmpMessage.header.flags = 0;
    memcpy(_tmpMessage.data, buf, len);

    return route(&_tmpMessage, sizeof(RoutedMessageHeader)+len);
}

////////////////////////////////////////////////////////////////////
uint8_t RF22Router::route(RoutedMessage* message, uint8_t messageLen)
{
    // Reliably deliver it if possible. See if we have a route:
    uint8_t next_hop = RF22_BROADCAST_ADDRESS;
    if (message->header.dest != RF22_BROADCAST_ADDRESS)
    {
    RoutingTableEntry* route = getRouteTo(message->header.dest);
    if (!route)
        return RF22_ROUTER_ERROR_NO_ROUTE;
    next_hop = route->next_hop;
    }

    if (!RF22ReliableDatagram::sendtoWait((uint8_t*)message, messageLen, next_hop))
    return RF22_ROUTER_ERROR_UNABLE_TO_DELIVER;

    return RF22_ROUTER_ERROR_NONE;
}

////////////////////////////////////////////////////////////////////
// Subclasses may want to override this to peek at messages going past
void RF22Router::peekAtMessage(RoutedMessage* message, uint8_t messageLen)
{
    // Default does nothing
}

////////////////////////////////////////////////////////////////////
boolean RF22Router::recvfromAck(uint8_t* buf, uint8_t* len, uint8_t* source, uint8_t* dest, uint8_t* id, uint8_t* flags)
{  
    uint8_t tmpMessageLen = sizeof(_tmpMessage);
    uint8_t _from;
    uint8_t _to;
    uint8_t _id;
    uint8_t _flags;
    if (RF22ReliableDatagram::recvfromAck((uint8_t*)&_tmpMessage, &tmpMessageLen, &_from, &_to, &_id, &_flags))
    {
    // Here we simulate networks with limited visibility between nodes
    // so we can test routing
#ifdef RF22_TEST_NETWORK
    if (
#if RF22_TEST_NETWORK==1
    // This looks like 1-2-3-4
           (_thisAddress == 1 && _from == 2)
        || (_thisAddress == 2 && (_from == 1 || _from == 3))
        || (_thisAddress == 3 && (_from == 2 || _from == 4))
        || (_thisAddress == 4 && _from == 3)
        
#elif RF22_TEST_NETWORK==2
           // This looks like 1-2-4
           //                 | | |
           //                 --3--
           (_thisAddress == 1 && (_from == 2 || _from == 3))
        ||  _thisAddress == 2
        ||  _thisAddress == 3
        || (_thisAddress == 4 && (_from == 2 || _from == 3))

#elif RF22_TEST_NETWORK==3
           // This looks like 1-2-4
           //                 |   |
           //                 --3--
           (_thisAddress == 1 && (_from == 2 || _from == 3))
        || (_thisAddress == 2 && (_from == 1 || _from == 4))
        || (_thisAddress == 3 && (_from == 1 || _from == 4))
        || (_thisAddress == 4 && (_from == 2 || _from == 3))

#elif RF22_TEST_NETWORK==4
           // This looks like 1-2-3
           //                   |
           //                   4
           (_thisAddress == 1 && _from == 2)
        ||  _thisAddress == 2
        || (_thisAddress == 3 && _from == 2)
        || (_thisAddress == 4 && _from == 2)

#endif
)
    {
        // OK
    }
    else
    {
        return false; // Pretend we got nothing
    }
#endif

    peekAtMessage(&_tmpMessage, tmpMessageLen);
    // See if its for us or has to be routed
    if (_tmpMessage.header.dest == _thisAddress || _tmpMessage.header.dest == RF22_BROADCAST_ADDRESS)
    {
        // Deliver it here
        if (source) *source  = _tmpMessage.header.source;
        if (dest)   *dest    = _tmpMessage.header.dest;
        if (id)     *id      = _tmpMessage.header.id;
        if (flags)  *flags   = _tmpMessage.header.flags;
        uint8_t msgLen = tmpMessageLen - sizeof(RoutedMessageHeader);
        if (*len > msgLen)
        *len = msgLen;
        memcpy(buf, _tmpMessage.data, *len);
        return true; // Its for you!
    }
    else if (   _tmpMessage.header.dest != RF22_BROADCAST_ADDRESS
         && _tmpMessage.header.hops++ < _max_hops)
    {
        // Maybe it has to be routed to the next hop
        // REVISIT: if it fails due to no route or unable to deliver to the next hop, 
        // tell the originator. BUT HOW?
        route(&_tmpMessage, tmpMessageLen);
    }
    // Discard it and maybe wait for another
    }
    return false;
}

////////////////////////////////////////////////////////////////////
boolean RF22Router::recvfromAckTimeout(uint8_t* buf, uint8_t* len, uint16_t timeout, uint8_t* source, uint8_t* dest, uint8_t* id, uint8_t* flags)
{   
    Timer t;
    
    t.start();
    unsigned long endtime = t.read_ms() + timeout;
    while (t.read_ms() < endtime)
    {
    if (recvfromAck(buf, len, source, dest, id, flags))
        return true;
    }
    return false;
}
