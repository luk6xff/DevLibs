
/**
 *  @brief:  Implementation of a LORA platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-11-22
 */


#ifndef __LORA_MBED_H__
#define __LORA_MBED_H__

#include "mbed.h"
#include "../../lora.h"

/**
 * @brief LORA mbed specific dev object.
 */
typedef struct
{
    SPI* spi;
    DigitalOut* nss;
    DigitalInOut* reset;
    InterruptIn* dio0;
} lora_mbed;


/**
 * @brief LORA mbed Init function.
 *
 * @param  dev       LORA Device object pointer
 * @param  mbed_dev  Mbed device object pointer
 */
void lora_mbed_init(lora *const dev, lora_mbed *const mbed_dev);

void lora_mbed_deinit(lora *const dev);

#endif // __LORA_MBED_H__
