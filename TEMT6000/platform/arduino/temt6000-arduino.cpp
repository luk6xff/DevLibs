
/**
 *  @brief:  Implementation of a TEMT6000 platform dependent [ARDUINO] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-06
 */

#include "temt6000-arduino.h"

//------------------------------------------------------------------------------
void temt6000_arduino_init(temt6000* const dev, temt6000_arduino* const arduino_dev)
{
    dev->platform_dev = arduino_dev;
    #if (ARDUINO) && (ESP32)
        // For ESP32 set adc autentation to 11db for full range: https://esphome.io/components/sensor/adc.html#adc-esp32-attenuation
        analogSetPinAttenuation(arduino_dev->analog_gpio_pin, ADC_11db);
    #endif
    temt6000_init(dev);
}

//-----------------------------------------------------------------------------
bool temt6000_read_raw(temt6000* const dev, uint16_t* raw_adc_value)
{
    temt6000_arduino* const pd = (temt6000_arduino*)dev->platform_dev;
    *raw_adc_value = analogRead(pd->analog_gpio_pin);
    //Serial.printf("RAW_VALUE: %d\n", *raw_adc_value);
    return true;
}
//-----------------------------------------------------------------------------


