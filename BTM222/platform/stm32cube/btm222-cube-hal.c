/**
 *  @brief:   BTM222 Bluetooth SPP module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "btm222-cube-hal.h"

// Timeout of a blocking UART transmission
#define BTM222_CUBE_TX_TIMEOUT_MS 1000

// Timeout of a single byte reception, kept short to stay non blocking
#define BTM222_CUBE_RX_TIMEOUT_MS 1

//-----------------------------------------------------------------------------
bool btm222_cube_hal_init(btm222* const dev,
                          btm222_cube_hal* const cube_hal_dev)
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
    return btm222_init(dev);
}

//-----------------------------------------------------------------------------
bool btm222_uart_write(const btm222* const dev, const uint8_t* buf, size_t len)
{
    const btm222_cube_hal* const pd = (const btm222_cube_hal*)dev->platform_dev;

    return HAL_UART_Transmit(pd->uart, (uint8_t*)buf, (uint16_t)len,
                             BTM222_CUBE_TX_TIMEOUT_MS) == HAL_OK;
}

//-----------------------------------------------------------------------------
int btm222_uart_read_byte(const btm222* const dev)
{
    const btm222_cube_hal* const pd = (const btm222_cube_hal*)dev->platform_dev;

    uint8_t byte = 0;
    if (HAL_UART_Receive(pd->uart, &byte, 1, BTM222_CUBE_RX_TIMEOUT_MS) !=
        HAL_OK)
    {
        return -1;
    }
    return (int)byte;
}

//-----------------------------------------------------------------------------
bool btm222_uart_readable(const btm222* const dev)
{
    const btm222_cube_hal* const pd = (const btm222_cube_hal*)dev->platform_dev;

    return __HAL_UART_GET_FLAG(pd->uart, UART_FLAG_RXNE) != RESET;
}

//-----------------------------------------------------------------------------
void btm222_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t btm222_get_ms_ticks(void)
{
    return HAL_GetTick();
}

//-----------------------------------------------------------------------------
