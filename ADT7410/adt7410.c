/**
 *  @brief:   ADT7410 digital temperature sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "adt7410.h"

// CONFIG register bit layout
#define ADT7410_CFG_RES_POS         7
#define ADT7410_CFG_MODE_POS        5
#define ADT7410_CFG_INT_CT_MODE_POS 4
#define ADT7410_CFG_INT_POL_POS     3
#define ADT7410_CFG_CT_POL_POS      2
#define ADT7410_CFG_FAULT_QUEUE_POS 0

#define ADT7410_CFG_RES_MASK        (1u << ADT7410_CFG_RES_POS)

// Software reset needs at least 200 us before the device answers again
#define ADT7410_RESET_DELAY_MS      1

//-----------------------------------------------------------------------------
static bool adt7410_write_config(const adt7410* const dev, uint8_t reg)
{
    return adt7410_write(dev, ADT7410_REG_CONFIG, &reg, 1);
}

//-----------------------------------------------------------------------------
bool adt7410_init(adt7410* const dev)
{
    if (!dev)
    {
        return false;
    }

    dev->temperature = ADT7410_INVALID_TEMPERATURE;

    uint8_t id = 0;
    if (!adt7410_read_id(dev, &id, NULL) || id != ADT7410_MANUFACTURER_ID)
    {
        return false;
    }

    const adt7410_config cfg = {
        .fault_queue = ADT7410_FAULT_QUEUE_1,
        .ct_pin_polarity = ADT7410_PIN_ACTIVE_LOW,
        .int_pin_polarity = ADT7410_PIN_ACTIVE_LOW,
        .int_ct_mode = ADT7410_INT_CT_MODE_INTERRUPT,
        .operation_mode = ADT7410_MODE_CONTINUOUS,
        .resolution = ADT7410_RESOLUTION_16_BIT,
    };

    return adt7410_set_config(dev, &cfg);
}

//-----------------------------------------------------------------------------
bool adt7410_set_config(adt7410* const dev, const adt7410_config* const cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    const uint8_t reg =
        (uint8_t)(((uint8_t)cfg->resolution << ADT7410_CFG_RES_POS) |
                  ((uint8_t)cfg->operation_mode << ADT7410_CFG_MODE_POS) |
                  ((uint8_t)cfg->int_ct_mode << ADT7410_CFG_INT_CT_MODE_POS) |
                  ((uint8_t)cfg->int_pin_polarity << ADT7410_CFG_INT_POL_POS) |
                  ((uint8_t)cfg->ct_pin_polarity << ADT7410_CFG_CT_POL_POS) |
                  ((uint8_t)cfg->fault_queue << ADT7410_CFG_FAULT_QUEUE_POS));

    if (!adt7410_write_config(dev, reg))
    {
        return false;
    }

    dev->resolution = cfg->resolution;
    return true;
}

//-----------------------------------------------------------------------------
bool adt7410_set_resolution(adt7410* const dev, adt7410_resolution res)
{
    if (!dev)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!adt7410_read(dev, ADT7410_REG_CONFIG, &reg, 1))
    {
        return false;
    }

    // Clear the resolution bit before setting it, otherwise 13-bit is unreachable
    reg = (uint8_t)((reg & ~ADT7410_CFG_RES_MASK) |
                    ((uint8_t)res << ADT7410_CFG_RES_POS));

    if (!adt7410_write_config(dev, reg))
    {
        return false;
    }

    dev->resolution = res;
    return true;
}

//-----------------------------------------------------------------------------
bool adt7410_read_temperature(adt7410* const dev, float* temp)
{
    if (!dev)
    {
        return false;
    }

    uint8_t raw[2] = {0};
    if (!adt7410_read(dev, ADT7410_REG_TEMP_MSB, raw, sizeof(raw)))
    {
        dev->temperature = ADT7410_INVALID_TEMPERATURE;
        return false;
    }

    int32_t counts = (int32_t)(((uint32_t)raw[0] << 8) | (uint32_t)raw[1]);
    float result;

    if (dev->resolution == ADT7410_RESOLUTION_13_BIT)
    {
        // The three LSBs carry the alarm flags in 13-bit mode
        counts >>= 3;
        if (counts & 0x1000)
        {
            counts -= 8192;
        }
        result = (float)counts / 16.0f;
    }
    else
    {
        if (counts & 0x8000)
        {
            counts -= 65536;
        }
        result = (float)counts / 128.0f;
    }

    dev->temperature = result;
    if (temp)
    {
        *temp = result;
    }
    return true;
}

//-----------------------------------------------------------------------------
float adt7410_get_temperature(const adt7410* const dev)
{
    if (!dev)
    {
        return ADT7410_INVALID_TEMPERATURE;
    }
    return dev->temperature;
}

//-----------------------------------------------------------------------------
bool adt7410_read_id(const adt7410* const dev, uint8_t* id, uint8_t* revision)
{
    if (!dev)
    {
        return false;
    }

    uint8_t reg = 0;
    if (!adt7410_read(dev, ADT7410_REG_ID, &reg, 1))
    {
        return false;
    }

    if (id)
    {
        *id = (uint8_t)(reg >> 3);
    }
    if (revision)
    {
        *revision = (uint8_t)(reg & 0x07u);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool adt7410_reset(adt7410* const dev)
{
    if (!dev)
    {
        return false;
    }

    // A single write to the reset address is enough, no data byte is expected
    if (!adt7410_write(dev, ADT7410_REG_RESET, NULL, 0))
    {
        return false;
    }

    adt7410_delay_ms(ADT7410_RESET_DELAY_MS);
    dev->resolution = ADT7410_RESOLUTION_13_BIT; // Power-on default
    dev->temperature = ADT7410_INVALID_TEMPERATURE;
    return true;
}
