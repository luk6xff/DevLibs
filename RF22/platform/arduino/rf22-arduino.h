// rf22-arduino.h
//
// ARDUINO platform layer for the portable rf22 core.
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

#ifndef __RF22_ARDUINO_H__
#define __RF22_ARDUINO_H__

#include <Arduino.h>
#include <SPI.h>

#include "../../rf22.h"

/** Pass this as sdn_pin when SDN is tied low in hardware */
#define RF22_ARDUINO_PIN_UNUSED (-1)

/**
 * @brief RF22 arduino specific dev object
 */
typedef struct
{
    SPIClass* spi;      // SPI bus instance, e.g. &SPI
    uint32_t spi_freq;  // SPI clock in Hz, 0 selects 1 MHz
    int cs_pin;         // nSEL, chip select output
    int irq_pin;        // nIRQ, active low interrupt input
    int sdn_pin;        // SDN shutdown output, RF22_ARDUINO_PIN_UNUSED if tied low
} rf22_arduino;

/**
 * @brief Initialize RF22 device with ARDUINO dependent parameters.
 *
 * @param dev         rf22 device object
 * @param arduino_dev Arduino specific descriptor, must outlive `dev`
 * @return True on success, false otherwise
 */
bool rf22_arduino_init(rf22* const dev, rf22_arduino* const arduino_dev);


#endif // __RF22_ARDUINO_H__
