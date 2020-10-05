#include "max44009.h"


//------------------------------------------------------------------------------
bool max44009_init(max44009* const dev)
{
    if (!dev)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
bool max44009_get_lux(const max44009* const dev, float* lux)
{
    uint8_t exponent;
    uint32_t mantissa;

    uint8_t high_and_low_byte[2];
    uint8_t reg_addr = 0x03 ;
    if (!max44009_read(dev, reg_addr, high_and_low_byte, 2))
    {
        *lux = -1;
        return false;
    }

    mantissa = (((high_and_low_byte[0] & 0x0F) << 4) | (high_and_low_byte[1] & 0x0F));
    exponent = (high_and_low_byte[0] & 0xF0) >> 4;
    *lux = ((float)(mantissa<<exponent) * 0.045);
    return true;
}

//-----------------------------------------------------------------------------