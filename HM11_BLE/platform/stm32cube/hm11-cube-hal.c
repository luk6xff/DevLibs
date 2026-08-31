/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "hm11-cube-hal.h"

// Timeout of a blocking UART transmission
#define HM11_CUBE_TX_TIMEOUT_MS 1000

// Timeout of a single byte reception, kept short to stay non blocking
#define HM11_CUBE_RX_TIMEOUT_MS 1

//-----------------------------------------------------------------------------
bool hm11_cube_hal_init(hm11* const dev, hm11_cube_hal* const cube_hal_dev)
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
    return hm11_init(dev);
}

//-----------------------------------------------------------------------------
bool hm11_uart_write(const hm11* const dev, const uint8_t* buf, size_t len)
{
    const hm11_cube_hal* const pd = (const hm11_cube_hal*)dev->platform_dev;

    return HAL_UART_Transmit(pd->uart, (uint8_t*)buf, (uint16_t)len,
                             HM11_CUBE_TX_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
int hm11_uart_read_byte(const hm11* const dev)
{
    const hm11_cube_hal* const pd = (const hm11_cube_hal*)dev->platform_dev;

    uint8_t byte = 0;
    if (HAL_UART_Receive(pd->uart, &byte, 1, HM11_CUBE_RX_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }
    return (int)byte;
}

//-----------------------------------------------------------------------------
bool hm11_uart_readable(const hm11* const dev)
{
    const hm11_cube_hal* const pd = (const hm11_cube_hal*)dev->platform_dev;

    return __HAL_UART_GET_FLAG(pd->uart, UART_FLAG_RXNE) != RESET;
}

//-----------------------------------------------------------------------------
void hm11_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t hm11_get_ms_ticks(void)
{
    return HAL_GetTick();
}

//-----------------------------------------------------------------------------
