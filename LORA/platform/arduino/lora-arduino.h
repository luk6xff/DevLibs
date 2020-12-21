/**
 *  @brief:  Implementation of LORA platform dependent [ARDUINO] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-11-22
 */

#ifndef __LORA_ARDUINO_H__
#define __LORA_ARDUINO_H__

#include <Arduino.h>
#include <SPI.h>

#include "../../lora.h"

/**
 * @brief LORA arduino specific dev object.
 */
typedef struct
{
    SPISettings spi_settings;
    SPIClass* spi;
    int mosi;
    int miso;
    int sck;
    int nss;
    int reset;
    int dio0;
} lora_arduino;


/**
 * @brief LORA arduino Init function.
 *
 * @param  dev       LORA Device object pointer
 * @param  arduino_dev  Arduino device object pointer
 */
bool lora_arduino_init(lora *const dev, lora_arduino *const arduino_dev);

void lora_arduino_deinit(lora *const dev);

#endif // __LORA_ARDUINO_H__
