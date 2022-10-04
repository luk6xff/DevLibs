
/**
 *  @brief:  Implementation of a RFM69 platform dependent [ARDUINO] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */


#ifndef __RFM69_ARDUINO_H__
#define __RFM69_ARDUINO_H__

#include <Arduino.h>
#include <SPI.h>

#include "../../rfm69.h"


/** RFM69 arduino specific dev object **/
typedef struct
{
    SPISettings spi_settings;
    SPIClass* spi;
    int nss;
    int reset;
    int dio0;
    int dio1;
    int dio2;
    int dio3;
    int dio4;
    int dio5;
} rfm69_arduino;


/**
 * @brief RFM69 arduino Init function driving.
 *
 * @param  dev         RFM69 Device object pointer
 * @param  arduino_dev Arduino device object pointer
 * @return True on success, False otherwise
 */
bool rfm69_arduino_init(rfm69 *const dev, rfm69_arduino* const arduino_dev);


/**
 * @brief RFM69 arduino deinit function driving.
 *
 * @param  dev       RFM69 Device object pointer
 */
void rfm69_arduino_deinit(rfm69 *const dev);


#endif // __RFM69_ARDUINO_H__
