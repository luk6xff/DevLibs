/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "splc501c-cube-hal.h"

#define SPLC501C_CUBE_TIMEOUT_MS 1000

//-----------------------------------------------------------------------------
bool splc501c_cube_hal_init(splc501c* const dev,
                            splc501c_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->spi ||
        !cube_hal_dev->cs_port || !cube_hal_dev->a0_port ||
        !cube_hal_dev->reset_port)
    {
        return false;
    }

    HAL_GPIO_WritePin(cube_hal_dev->cs_port, cube_hal_dev->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(cube_hal_dev->a0_port, cube_hal_dev->a0_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(cube_hal_dev->reset_port, cube_hal_dev->reset_pin,
                      GPIO_PIN_SET);

    dev->platform_dev = cube_hal_dev;
    return splc501c_init(dev);
}

//-----------------------------------------------------------------------------
bool splc501c_write_bytes(const splc501c* const dev, const uint8_t* buf,
                          size_t len)
{
    const splc501c_cube_hal* const pd =
        (const splc501c_cube_hal*)dev->platform_dev;

    if (!buf || len == 0)
    {
        return false;
    }

    return HAL_SPI_Transmit(pd->spi, (uint8_t*)buf, (uint16_t)len,
                            SPLC501C_CUBE_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
void splc501c_set_pin(const splc501c* const dev, splc501c_pin pin, bool state)
{
    const splc501c_cube_hal* const pd =
        (const splc501c_cube_hal*)dev->platform_dev;

    GPIO_TypeDef* port;
    uint16_t gpio_pin;
    switch (pin)
    {
        case SPLC501C_PIN_A0:
            port = pd->a0_port;
            gpio_pin = pd->a0_pin;
            break;
        case SPLC501C_PIN_RESET:
            port = pd->reset_port;
            gpio_pin = pd->reset_pin;
            break;
        case SPLC501C_PIN_CS:
        default:
            port = pd->cs_port;
            gpio_pin = pd->cs_pin;
            break;
    }
    HAL_GPIO_WritePin(port, gpio_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
void splc501c_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
