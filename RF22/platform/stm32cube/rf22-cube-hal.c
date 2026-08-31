// rf22-cube-hal.c
//
// STM32 CUBE HAL platform layer for the portable rf22 core.
//
// Derived from the RF22 library:
//   Copyright (C) 2011 Mike McCauley
//   ported to mbed by Karl Zweimueller
//   modified by Lukasz Uszko luszko@op.pl
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#include <string.h>

#include "rf22-cube-hal.h"

#define RF22_CUBE_TIMEOUT_MS 1000

// Half duplex directions are emulated with a stack scratch buffer so that no
// dynamic allocation is needed for the 255 octet burst transfers
#define RF22_CUBE_CHUNK_SIZE 32

//-----------------------------------------------------------------------------
bool rf22_cube_hal_init(rf22* const dev, rf22_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->spi ||
        !cube_hal_dev->cs_port || !cube_hal_dev->irq_port)
    {
        return false;
    }

    HAL_GPIO_WritePin(cube_hal_dev->cs_port, cube_hal_dev->cs_pin, GPIO_PIN_SET);
    if (cube_hal_dev->sdn_port)
    {
        // Held in shutdown until rf22_init() powers the module up
        HAL_GPIO_WritePin(cube_hal_dev->sdn_port, cube_hal_dev->sdn_pin,
                          GPIO_PIN_SET);
    }

    dev->platform_dev = cube_hal_dev;
    return rf22_init(dev);
}

//-----------------------------------------------------------------------------
bool rf22_spi_transfer(const rf22* const dev, const uint8_t* tx, uint8_t* rx,
                       size_t len)
{
    const rf22_cube_hal* const pd = (const rf22_cube_hal*)dev->platform_dev;
    uint8_t zeros[RF22_CUBE_CHUNK_SIZE];
    uint8_t sink[RF22_CUBE_CHUNK_SIZE];
    size_t offset = 0;

    if (!pd || !pd->spi || len == 0)
    {
        return false;
    }

    memset(zeros, 0, sizeof(zeros));

    while (offset < len)
    {
        size_t chunk = len - offset;
        const uint8_t* tx_ptr;
        uint8_t* rx_ptr;

        if (chunk > RF22_CUBE_CHUNK_SIZE)
        {
            chunk = RF22_CUBE_CHUNK_SIZE;
        }

        tx_ptr = tx ? (tx + offset) : zeros;
        rx_ptr = rx ? (rx + offset) : sink;

        if (HAL_SPI_TransmitReceive(pd->spi, (uint8_t*)tx_ptr, rx_ptr,
                                    (uint16_t)chunk,
                                    RF22_CUBE_TIMEOUT_MS) != HAL_OK)
        {
            return false;
        }

        offset += chunk;
    }

    return true;
}

//-----------------------------------------------------------------------------
void rf22_set_cs_pin(const rf22* const dev, bool state)
{
    const rf22_cube_hal* const pd = (const rf22_cube_hal*)dev->platform_dev;

    HAL_GPIO_WritePin(pd->cs_port, pd->cs_pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
void rf22_set_sdn_pin(const rf22* const dev, bool state)
{
    const rf22_cube_hal* const pd = (const rf22_cube_hal*)dev->platform_dev;

    if (!pd->sdn_port)
    {
        return; // SDN tied low in hardware
    }

    HAL_GPIO_WritePin(pd->sdn_port, pd->sdn_pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
bool rf22_get_irq_pin(const rf22* const dev)
{
    const rf22_cube_hal* const pd = (const rf22_cube_hal*)dev->platform_dev;

    return HAL_GPIO_ReadPin(pd->irq_port, pd->irq_pin) != GPIO_PIN_RESET;
}

//-----------------------------------------------------------------------------
void rf22_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

//-----------------------------------------------------------------------------
uint32_t rf22_get_ms_ticks(void)
{
    return HAL_GetTick();
}

//-----------------------------------------------------------------------------
