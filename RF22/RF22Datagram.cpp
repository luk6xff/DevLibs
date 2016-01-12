// RF22Datagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RF22Datagram.cpp,v 1.3 2012/05/30 01:50:21 mikem Exp $
// ported to mbed by Karl Zweimueller

#include <RF22Datagram.h>


RF22Datagram::RF22Datagram(uint8_t thisAddress, PinName slaveSelectPin , PinName mosi, PinName miso, PinName sclk, PinName interrupt,PinName shutdownPin ) 
    : RF22(slaveSelectPin , mosi, miso, sclk, interrupt,shutdownPin )
{
    _thisAddress = thisAddress;
}

////////////////////////////////////////////////////////////////////
// Public methods
boolean RF22Datagram::init()
{
    boolean ret = this->RF22::init();
    if (ret)
    setThisAddress(_thisAddress);
    return ret;
}

void RF22Datagram::setThisAddress(uint8_t thisAddress)
{
    _thisAddress = thisAddress;
    // Check the TO header against RF22_DEFAULT_NODE_ADDRESS
    spiWrite(RF22_REG_3F_CHECK_HEADER3, _thisAddress);
    // Use this address in the transmitted FROM header
    setHeaderFrom(_thisAddress);
}

boolean RF22Datagram::sendto(uint8_t* buf, uint8_t len, uint8_t address)
{
    setHeaderTo(address);
    return send(buf, len);
}

boolean RF22Datagram::recvfrom(uint8_t* buf, uint8_t* len, uint8_t* from, uint8_t* to, uint8_t* id, uint8_t* flags)
{
    if (recv(buf, len))
    {

    if (from)  *from =  headerFrom();
    if (to)    *to =    headerTo();
    if (id)    *id =    headerId();
    if (flags) *flags = headerFlags();
    return true;
    }
    return false;
}


