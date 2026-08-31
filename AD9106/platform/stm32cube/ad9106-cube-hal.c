/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ad9106-cube-hal.h"

#define AD9106_CUBE_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
static GPIO_TypeDef* ad9106_cube_hal_port(const ad9106_cube_hal* const pd,
                                          ad9106_pin pin, uint16_t* pin_mask)
{
    switch (pin)
    {
        case AD9106_PIN_CS:
            *pin_mask = pd->cs_pin;
            return pd->cs_port;
        case AD9106_PIN_RESET:
            *pin_mask = pd->reset_pin;
            return pd->reset_port;
        case AD9106_PIN_TRIGGER:
            *pin_mask = pd->trigger_pin;
            return pd->trigger_port;
        default:
            *pin_mask = 0;
            return NULL;
    }
}

//-----------------------------------------------------------------------------
bool ad9106_cube_hal_init(ad9106* const dev,
                          ad9106_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->spi || !cube_hal_dev->cs_port)
    {
        return false;
    }

    // The SPI peripheral has to be configured for mode 0 (CPOL=0, CPHA=0),
    // 8 bit frames, MSB first, as required by the AD9106
    HAL_GPIO_WritePin(cube_hal_dev->cs_port, cube_hal_dev->cs_pin, GPIO_PIN_SET);
    if (cube_hal_dev->reset_port)
    {
        HAL_GPIO_WritePin(cube_hal_dev->reset_port, cube_hal_dev->reset_pin,
                          GPIO_PIN_SET);
    }
    if (cube_hal_dev->trigger_port)
    {
        HAL_GPIO_WritePin(cube_hal_dev->trigger_port, cube_hal_dev->trigger_pin,
                          GPIO_PIN_SET);
    }

    dev->platform_dev = cube_hal_dev;
    return ad9106_init(dev);
}

//-----------------------------------------------------------------------------
bool ad9106_spi_transfer(const ad9106* const dev, const uint8_t* tx,
                         uint8_t* rx, size_t len)
{
    const ad9106_cube_hal* const pd = (const ad9106_cube_hal*)dev->platform_dev;

    if (!pd || !tx || !rx)
    {
        return false;
    }

    return HAL_SPI_TransmitReceive(pd->spi, (uint8_t*)tx, rx, (uint16_t)len,
                                   AD9106_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void ad9106_set_pin(const ad9106* const dev, ad9106_pin pin, bool state)
{
    const ad9106_cube_hal* const pd = (const ad9106_cube_hal*)dev->platform_dev;

    if (!pd)
    {
        return;
    }

    uint16_t pin_mask = 0;
    GPIO_TypeDef* const port = ad9106_cube_hal_port(pd, pin, &pin_mask);
    if (!port)
    {
        // Line not wired on this board
        return;
    }
    HAL_GPIO_WritePin(port, pin_mask, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
void ad9106_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
