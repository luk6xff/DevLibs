// rf22_datagram.c
//
// Addressed, unreliable datagrams on top of the portable rf22 core.
//
// Derived from the RF22 library:
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22Datagram.cpp,v 1.3 2012/05/30 01:50:21 mikem Exp $
//   ported to mbed by Karl Zweimueller
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#include "rf22_datagram.h"

//-----------------------------------------------------------------------------
bool rf22_datagram_init(rf22_datagram* const dg, uint8_t this_address)
{
    if (!dg)
    {
        return false;
    }

    rf22_datagram_set_this_address(dg, this_address);
    return true;
}

//-----------------------------------------------------------------------------
void rf22_datagram_set_this_address(rf22_datagram* const dg,
                                    uint8_t this_address)
{
    if (!dg)
    {
        return;
    }

    dg->this_address = this_address;
    // Accept messages addressed to this node
    rf22_spi_write_reg(&dg->base, RF22_REG_3F_CHECK_HEADER3, this_address);
    // Use this address in the transmitted FROM header
    rf22_set_header_from(&dg->base, this_address);
}

//-----------------------------------------------------------------------------
uint8_t rf22_datagram_this_address(const rf22_datagram* const dg)
{
    if (!dg)
    {
        return 0;
    }

    return dg->this_address;
}

//-----------------------------------------------------------------------------
bool rf22_datagram_sendto(rf22_datagram* const dg, const uint8_t* buf,
                          uint8_t len, uint8_t address)
{
    if (!dg || !buf)
    {
        return false;
    }

    rf22_set_header_to(&dg->base, address);
    return rf22_send(&dg->base, buf, len);
}

//-----------------------------------------------------------------------------
bool rf22_datagram_recvfrom(rf22_datagram* const dg, uint8_t* buf, uint8_t* len,
                            uint8_t* from, uint8_t* to, uint8_t* id,
                            uint8_t* flags)
{
    if (!dg || !buf || !len)
    {
        return false;
    }

    if (!rf22_recv(&dg->base, buf, len))
    {
        return false;
    }

    // The headers are read out of the device only after a successful recv, so
    // they still describe the message that was just copied
    if (from)
    {
        *from = rf22_header_from(&dg->base);
    }
    if (to)
    {
        *to = rf22_header_to(&dg->base);
    }
    if (id)
    {
        *id = rf22_header_id(&dg->base);
    }
    if (flags)
    {
        *flags = rf22_header_flags(&dg->base);
    }

    return true;
}
