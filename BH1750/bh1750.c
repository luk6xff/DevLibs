/**
 *  @brief:   BH1750 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-10-02
 *  @license: MIT
 */

#include "bh1750.h"


//------------------------------------------------------------------------------
bool bh1750_init(bh1750 *const dev)
{
    if (!dev)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
bool bh1750_set_state(bh1750 *const dev, bh1750_state state)
{
    return bh1750_write_instruction(dev, state);
}

//------------------------------------------------------------------------------
bool bh1750_set_mode(bh1750 *const dev, bh1750_mode mode)
{
    return bh1750_write_instruction(dev, mode);
}

//------------------------------------------------------------------------------
bool bh1750_get_lux(const bh1750 *const dev, float *lux)
{
    uint8_t val[2];
    if (!bh1750_read(dev, val, 2))
    {
        *lux = -1;
        return false;
    }

    *lux = ((float)(val[0] << 8 | val[1]) / 1.2f);
    return true;
}

//-----------------------------------------------------------------------------