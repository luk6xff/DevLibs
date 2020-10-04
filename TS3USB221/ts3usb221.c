 #include "ts3usb221.h"

//------------------------------------------------------------------------------
bool ts3usb221_init(ts3usb221 *const dev)
{
    ts3usb221_low_power_mode(dev);
    return true;
}

//------------------------------------------------------------------------------
void ts3usb221_low_power_mode(const ts3usb221 *const dev)
{
    ts3usb221_set_pin(dev, TS3USB221_PIN_OE, TS3USB221_PIN_STATE_HIGH);
}

//------------------------------------------------------------------------------
void ts3usb221_enable_1D_channel(const ts3usb221 *const dev)
{
    ts3usb221_set_pin(dev, TS3USB221_PIN_OE, TS3USB221_PIN_STATE_LOW);
    ts3usb221_set_pin(dev, TS3USB221_PIN_S, TS3USB221_PIN_STATE_LOW);
}

//------------------------------------------------------------------------------
void ts3usb221_enable_2D_channel(const ts3usb221 *const dev)
{
    ts3usb221_set_pin(dev, TS3USB221_PIN_OE, TS3USB221_PIN_STATE_LOW);
    ts3usb221_set_pin(dev, TS3USB221_PIN_S, TS3USB221_PIN_STATE_HIGH);
}

//-----------------------------------------------------------------------------
