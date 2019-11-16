
/**
 *  @brief:  Implementation of a SX1278 platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  luszko@op.pl
 *  @date:   2019-11-15
 */


#ifndef __SX1278_MBED_H__
#define __SX1278_MBED_H__

#include "mbed.h"
#include "../sx1278.h"

void SX1278MbedInit(RadioEvents_t *events,
                    PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                    PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5);

void SX1278MbedDeInit();

#endif // __SX1278_MBED_H__
