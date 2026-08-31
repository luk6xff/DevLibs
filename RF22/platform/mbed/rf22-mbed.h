// rf22-mbed.h
//
// MBED platform layer for the portable rf22 core.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   ported to mbed by Karl Zweimueller
//   modified by Lukasz Uszko luszko@op.pl
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#ifndef __RF22_MBED_H__
#define __RF22_MBED_H__

#include "mbed.h"

#include "../../rf22.h"

/**
 * @brief RF22 mbed specific dev object
 */
typedef struct
{
    SPI* spi;          // SPI bus instance
    uint32_t spi_freq; // SPI clock in Hz, 0 selects 1 MHz
    DigitalOut* cs;    // nSEL, chip select output
    DigitalIn* irq;    // nIRQ, active low interrupt input
    DigitalOut* sdn;   // SDN shutdown output, NULL if tied low in hardware
} rf22_mbed;

/**
 * @brief Initialize RF22 device with MBED dependent parameters.
 *
 * @param dev      rf22 device object
 * @param mbed_dev Mbed specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool rf22_mbed_init(rf22* const dev, rf22_mbed* const mbed_dev);


#endif // __RF22_MBED_H__
