/**
 *  @brief:  Implementation of a RFM69 platform common utils
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-01
 */


#ifndef __RFM69_COMMON_H__
#define __RFM69_COMMON_H__

/**
 * @brief Macro declaring ISRs to IRAM for ESP boards
 */
#if (ARDUINO) && (ESP8266 || ESP32)
    #include <Arduino.h>
    #define ISR_PREFIX ICACHE_RAM_ATTR
#else
    #define ISR_PREFIX
#endif

#endif // __RFM69_COMMON_H__
