
/**
 *  @brief:  Implementation of a DS1820 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */

#include "ds1820-mbed.h"

//------------------------------------------------------------------------------
void ds1820_mbed_init(ds1820 *const dev, ds1820_mbed *const mbed_dev)
{
    dev->platform_dev = mbed_dev;
    if (mbed_dev->parasite_pin == nullptr)
    {
        ds1820_init(dev, false);
        return;
    }
    ds1820_init(dev, true);
}

//------------------------------------------------------------------------------
bool ds1820_onewire_reset(ds1820 *const dev)
{
    ds1820_mbed *const pd = (ds1820_mbed*)dev->platform_dev;
    // This will return false if no devices are present on the data bus
    bool presence = false;
    pd->data_pin->output();
    *(pd->data_pin) = 0;           // bring low for 500 us
    ds1820_delay_us(500);
    pd->data_pin->input();       // let the data line float high
    ds1820_delay_us(90);            // wait 90us
    if (pd->data_pin->read() == 0) // see if any devices are pulling the data line low
    {
        presence = true;
    }
    ds1820_delay_us(410);
    return presence;
}

//------------------------------------------------------------------------------
void ds1820_onewire_bit_out(ds1820 *const dev, bool bit_data)
{
    ds1820_mbed *const pd = (ds1820_mbed*)dev->platform_dev;
    pd->data_pin->output();
    *(pd->data_pin) = 0;
    ds1820_delay_us(5);
    if (bit_data)
    {
        pd->data_pin->input();    // bring data line high
        ds1820_delay_us(55);
    }
    else
    {
        ds1820_delay_us(55); // keep data line low
        pd->data_pin->input();
    }
}

//------------------------------------------------------------------------------
bool ds1820_onewire_bit_in(ds1820 *const dev)
{
    ds1820_mbed *const pd = (ds1820_mbed*)dev->platform_dev;
    bool answer;
    pd->data_pin->output();
    *(pd->data_pin) = 0;
    ds1820_delay_us(5);
    pd->data_pin->input();
    ds1820_delay_us(5);
    answer = pd->data_pin->read();
    ds1820_delay_us(50);
    return answer;
}

//------------------------------------------------------------------------------
void ds1820_set_parasite_pin(ds1820 *const dev, bool enable)
{
    ds1820_mbed *const pd = (ds1820_mbed*)dev->platform_dev;
    if (enable)
    {
        *(pd->parasite_pin) = 1;
        return;
    }
    *(pd->parasite_pin) = 0;
}

//------------------------------------------------------------------------------
void ds1820_delay_us(uint32_t us)
{
    wait_us(us);
}

//-----------------------------------------------------------------------------
