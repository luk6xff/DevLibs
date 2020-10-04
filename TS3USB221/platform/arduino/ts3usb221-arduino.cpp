/**
 *  @brief:   TS3USB221 High-speed USB 2.0 (480-Mbps) 1:2 multiplexer – demultiplexer library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-27
 *  @license: MIT
 */

#include "ts3usb221-arduino.h"

//------------------------------------------------------------------------------
bool ts3usb221_arduino_init(ts3usb221 *const dev, ts3usb221_arduino *const arduino_dev)
{
    dev->platform_dev = arduino_dev;
    // Set both pins as outputs
    pinMode(arduino_dev->oe_pin, OUTPUT);
    pinMode(arduino_dev->s_pin, OUTPUT);

    return ts3usb221_init(dev);
}

//-----------------------------------------------------------------------------
void ts3usb221_set_pin(const ts3usb221 *const dev, ts3usb221_pin pin,
                                ts3usb221_pin_state pin_state)
{
    ts3usb221_arduino* const pd = (ts3usb221_arduino*)dev->platform_dev;

    const uint8_t state = (pin_state == TS3USB221_PIN_STATE_HIGH) ? HIGH : LOW;


    switch (pin)
    {
        case TS3USB221_PIN_OE:
        {
            digitalWrite(pd->oe_pin, state);
            break;
        }

        case TS3USB221_PIN_S:
        default:
        {
            digitalWrite(pd->s_pin, state);
            break;
        }
    }
}

//-----------------------------------------------------------------------------
