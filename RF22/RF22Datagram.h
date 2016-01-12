// RF22Datagram.h
// Author: Mike McCauley (mikem@open.com.au)
// Copyright (C) 2011 Mike McCauley
// $Id: RF22Datagram.h,v 1.4 2012/05/30 01:50:21 mikem Exp $
// ported to mbed by Karl Zweimueller

#ifndef RF22Datagram_h
#define RF22Datagram_h

#include <RF22.h>

/////////////////////////////////////////////////////////////////////
/// \class RF22Datagram RF22Datagram.h <RF22Datagram.h>
/// \brief RF22 subclass for addressed, unreliable messages
///
/// Extends RF22 to define addressed, unreliable datagrams.
/// Every node has an 8 bit address (defaults to 0).
/// Addresses (DEST and SRC) are 8 bit integers with an address of RF22_BROADCAST_ADDRESS (0xff) 
/// reserved for broadcast.
///
/// Part of the Arduino RF22 library for operating with HopeRF RF22 compatible transceivers 
/// (see http://www.hoperf.com).
class RF22Datagram : public RF22
{
public:
    /// Constructor. 
    /// \param[in] thisAddress The address to assign to this node. Defaults to 0
    /// \param[in] slaveSelectPin the Arduino pin number of the output to use to select the RF22 before
    /// accessing it. Defaults to the normal SS pin for your Arduino (D10 for Diecimila, Uno etc, D53 for Mega)
    /// \param[in] interrupt The interrupt number to use. Default is interrupt 0 (Arduino input pin 2)
    RF22Datagram(uint8_t thisAddress , PinName slaveSelectPin , PinName mosi, PinName miso, PinName sclk, PinName interrupt,PinName shutdownPin );
    
    /// Initialises this instance and the radio module connected to it.
    /// Overrides the init() function in RF22
    boolean init();

    /// Sets the address of this node. Defaults to 0. 
    /// This will be used to set the FROM address of all messages sent by this node.
    /// If all the nodes leave the address unset (ie 0),
    /// In a conventional multinode system, all nodes will have a unique address 
    /// (which you could store in EEPROM).
    /// \param[in] thisAddress The address of this node
    void setThisAddress(uint8_t thisAddress);

    /// Sends a message to the node(s) with the given address
    /// RF22_BROADCAST_ADDRESS is a valid address which will cause the message
    /// to be accepted by all RF22Datagram nodes within range.
    /// \param[in] buf Pointer to the binary message to send
    /// \param[in] len Number of octets to send (> 0)
    /// \param[in] address The address to send the message to.
    /// \return true if the message was transmitted.
    boolean sendto(uint8_t* buf, uint8_t len, uint8_t address);

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available for this node, copy it to buf and return true
    /// The SRC address is placed in *from if present and not NULL.
    /// The DEST address is placed in *to if present and not NULL.
    /// If a message is copied, *len is set to the length.
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \param[in] from If present and not NULL, the referenced uint8_t will be set to the FROM address
    /// \param[in] to If present and not NULL, the referenced uint8_t will be set to the TO address
    /// \param[in] id If present and not NULL, the referenced uint8_t will be set to the ID
    /// \param[in] flags If present and not NULL, the referenced uint8_t will be set to the FLAGS
    /// (not just those addressed to this node).
    /// \return true if a valid message was copied to buf
    boolean recvfrom(uint8_t* buf, uint8_t* len, uint8_t* from = NULL, uint8_t* to = NULL, uint8_t* id = NULL, uint8_t* flags = NULL);

protected:
    /// The address of this node. Defaults to 0.
    uint8_t _thisAddress;

};

#endif
