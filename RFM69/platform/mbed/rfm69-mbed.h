
/**
 *  @brief:  Implementation of a RFM69 platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */


#ifndef __RFM69_MBED_H__
#define __RFM69_MBED_H__

#include "mbed.h"
#include "../../rfm69.h"


/** RFM69 mbed specific dev object **/
typedef struct
{
    SPI* spi;
    DigitalOut* nss;
    DigitalInOut* reset;
    InterruptIn* dio0;
    InterruptIn* dio1;
    InterruptIn* dio2;
    InterruptIn* dio3;
    InterruptIn* dio4;
    DigitalIn* dio5;
} rfm69_mbed;


/**
 * @brief RFM69 mbed Init function driving.
 *
 * @param  dev       RFM69 Device object pointer
 * @param  mbed_dev  Mbed device object pointer
 */
void rfm69_mbed_init(rfm69* const dev, rfm69_mbed* const mbed_dev);


/**
 * @brief RFM69 mbed deinit function driving.
 *
 * @param  dev       RFM69 Device object pointer
 */
void rfm69_mbed_deinit(rfm69* const dev);

#endif // __RFM69_MBED_H__
