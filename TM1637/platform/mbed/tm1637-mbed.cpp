
/**
 *  @brief:  Implementation of a TM1637 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-06
 */

#include "tm1637-mbed.h"


//------------------------------------------------------------------------------
void tm1637_mbed_init(tm1637* const dev, tm1637_mbed* const mbed_dev)
{
    dev->platform_dev = mbed_dev;
    tm1637_init(dev);
}

//------------------------------------------------------------------------------
void tm1637_mbed_deinit(tm1637* const dev)
{
}

//-----------------------------------------------------------------------------
void tm1637_delay_us(tm1637* const dev, uint32_t us)
{
    wait_us(us);
}

//-----------------------------------------------------------------------------
void tm1637_set_dio_mode(tm1637* const dev, tm1637_dio_mode mode)
{
    tm1637_mbed* pd = (tm1637_mbed*)dev->platform_dev;
    if (mode == TM1637_DIO_INPUT)
    {
        pd->dio.input();
    }
    else // mode == DIO_OUTPUT
    {
        pd->dio.output();
        //pd->dio->mode(PullUp);
    }
}

//-----------------------------------------------------------------------------
void tm1637_set_dio(tm1637* const dev, tm1637_pin_state state)
{
    tm1637_mbed* pd = (tm1637_mbed*)dev->platform_dev;
    if (state == TM1637_PIN_HIGH)
    {
        pd->dio = 1;
        return;
    }
    pd->dio = 0;  // TM1637_PIN_LOW
}

//-----------------------------------------------------------------------------
tm1637_pin_state tm1637_get_dio(tm1637* const dev)
{
    tm1637_mbed* pd = (tm1637_mbed*)dev->platform_dev;
    if (pd->dio)
    {
        return TM1637_PIN_HIGH;
    }
    return TM1637_PIN_LOW;
}

//-----------------------------------------------------------------------------
void tm1637_set_clk(tm1637* const dev, tm1637_pin_state state)
{
    tm1637_mbed* pd = (tm1637_mbed*)dev->platform_dev;
    if (state == TM1637_PIN_HIGH)
    {
        pd->clk = 1;
        return;
    }
    pd->clk = 0; // TM1637_PIN_LOW
}

//-----------------------------------------------------------------------------
