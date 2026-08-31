/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "gts4e60-cube-hal.h"

// Timeout of a blocking UART transmission
#define GTS4E60_CUBE_TX_TIMEOUT_MS 1000

// Timeout of a single byte reception, kept short to stay non blocking
#define GTS4E60_CUBE_RX_TIMEOUT_MS 1

//-----------------------------------------------------------------------------
bool gts4e60_cube_hal_init(gts4e60* const dev,
                           gts4e60_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->uart)
    {
        return false;
    }

    if (cube_hal_dev->baudrate != 0)
    {
        dev->baudrate = cube_hal_dev->baudrate;
    }

    dev->platform_dev = cube_hal_dev;
    return gts4e60_init(dev);
}

//-----------------------------------------------------------------------------
bool gts4e60_uart_write(const gts4e60* const dev, const uint8_t* buf,
                        size_t len)
{
    const gts4e60_cube_hal* const pd =
        (const gts4e60_cube_hal*)dev->platform_dev;

    return HAL_UART_Transmit(pd->uart, (uint8_t*)buf, (uint16_t)len,
                             GTS4E60_CUBE_TX_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
int gts4e60_uart_read_byte(const gts4e60* const dev)
{
    const gts4e60_cube_hal* const pd =
        (const gts4e60_cube_hal*)dev->platform_dev;

    uint8_t byte = 0;
    if (HAL_UART_Receive(pd->uart, &byte, 1, GTS4E60_CUBE_RX_TIMEOUT_MS) !=
        HAL_OK)
    {
        return -1;
    }
    return (int)byte;
}

//-----------------------------------------------------------------------------
bool gts4e60_uart_readable(const gts4e60* const dev)
{
    const gts4e60_cube_hal* const pd =
        (const gts4e60_cube_hal*)dev->platform_dev;

    return __HAL_UART_GET_FLAG(pd->uart, UART_FLAG_RXNE) != RESET;
}

//-----------------------------------------------------------------------------
bool gts4e60_set_shutdown_pin(const gts4e60* const dev, bool state)
{
    const gts4e60_cube_hal* const pd =
        (const gts4e60_cube_hal*)dev->platform_dev;

    if (!pd->shutdown_port)
    {
        return true;
    }

    const bool level = pd->shutdown_active_high ? state : !state;
    HAL_GPIO_WritePin(pd->shutdown_port, pd->shutdown_pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

//-----------------------------------------------------------------------------
void gts4e60_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t gts4e60_get_ms_ticks(void)
{
    return HAL_GetTick();
}

//-----------------------------------------------------------------------------
