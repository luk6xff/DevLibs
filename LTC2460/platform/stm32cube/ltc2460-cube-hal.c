/**
 *  @brief:   LTC2460 ultra-tiny 16-bit delta-sigma ADC library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ltc2460-cube-hal.h"

#define LTC2460_CUBE_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
bool ltc2460_cube_hal_init(ltc2460* const dev,
                           ltc2460_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->spi || !cube_hal_dev->cs_port)
    {
        return false;
    }

    HAL_GPIO_WritePin(cube_hal_dev->cs_port, cube_hal_dev->cs_pin, GPIO_PIN_SET);

    dev->platform_dev = cube_hal_dev;
    return ltc2460_init(dev);
}

//-----------------------------------------------------------------------------
bool ltc2460_spi_transfer(const ltc2460* const dev, const uint8_t* tx,
                          uint8_t* rx, size_t len)
{
    const ltc2460_cube_hal* const pd =
        (const ltc2460_cube_hal*)dev->platform_dev;

    if (!tx || !rx)
    {
        return false;
    }

    return HAL_SPI_TransmitReceive(pd->spi, (uint8_t*)tx, rx, (uint16_t)len,
                                   LTC2460_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void ltc2460_set_cs_pin(const ltc2460* const dev, bool state)
{
    const ltc2460_cube_hal* const pd =
        (const ltc2460_cube_hal*)dev->platform_dev;

    HAL_GPIO_WritePin(pd->cs_port, pd->cs_pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
void ltc2460_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
