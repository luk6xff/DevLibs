/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "spfd5408-cube-hal.h"

//-----------------------------------------------------------------------------
bool spfd5408_cube_hal_init(spfd5408* const dev,
                            spfd5408_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->data_port ||
        !cube_hal_dev->rs_port || !cube_hal_dev->wr_port ||
        !cube_hal_dev->rd_port || !cube_hal_dev->cs_port ||
        !cube_hal_dev->reset_port)
    {
        return false;
    }

    HAL_GPIO_WritePin(cube_hal_dev->rs_port, cube_hal_dev->rs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(cube_hal_dev->wr_port, cube_hal_dev->wr_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(cube_hal_dev->rd_port, cube_hal_dev->rd_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(cube_hal_dev->cs_port, cube_hal_dev->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(cube_hal_dev->reset_port, cube_hal_dev->reset_pin,
                      GPIO_PIN_SET);

    dev->platform_dev = cube_hal_dev;
    return spfd5408_init(dev);
}

//-----------------------------------------------------------------------------
void spfd5408_write_bus(const spfd5408* const dev, uint16_t value)
{
    const spfd5408_cube_hal* const pd =
        (const spfd5408_cube_hal*)dev->platform_dev;

    // BSRR sets and clears all sixteen data lines in one atomic write, doing
    // this pin by pin with HAL_GPIO_WritePin() would be far too slow here
    pd->data_port->BSRR = ((uint32_t)((uint16_t)~value) << 16) | (uint32_t)value;
}

//-----------------------------------------------------------------------------
void spfd5408_set_pin(const spfd5408* const dev, spfd5408_pin pin, bool state)
{
    const spfd5408_cube_hal* const pd =
        (const spfd5408_cube_hal*)dev->platform_dev;

    GPIO_TypeDef* port;
    uint16_t gpio_pin;
    switch (pin)
    {
        case SPFD5408_PIN_RS:
            port = pd->rs_port;
            gpio_pin = pd->rs_pin;
            break;
        case SPFD5408_PIN_WR:
            port = pd->wr_port;
            gpio_pin = pd->wr_pin;
            break;
        case SPFD5408_PIN_RD:
            port = pd->rd_port;
            gpio_pin = pd->rd_pin;
            break;
        case SPFD5408_PIN_RESET:
            port = pd->reset_port;
            gpio_pin = pd->reset_pin;
            break;
        case SPFD5408_PIN_CS:
        default:
            port = pd->cs_port;
            gpio_pin = pd->cs_pin;
            break;
    }
    HAL_GPIO_WritePin(port, gpio_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
void spfd5408_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
