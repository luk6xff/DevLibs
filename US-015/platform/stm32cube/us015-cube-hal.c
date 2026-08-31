/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "us015-cube-hal.h"

//-----------------------------------------------------------------------------
bool us015_cube_hal_init(us015* const dev, us015_cube_hal* const cube_hal_dev)
{
    if (!dev || !cube_hal_dev || !cube_hal_dev->trigger_port ||
        !cube_hal_dev->echo_port)
    {
        return false;
    }

    HAL_GPIO_WritePin(cube_hal_dev->trigger_port, cube_hal_dev->trigger_pin,
                      GPIO_PIN_RESET);

    dev->platform_dev = cube_hal_dev;
    return us015_init(dev);
}

//-----------------------------------------------------------------------------
void us015_set_trigger_pin(const us015* const dev, bool state)
{
    const us015_cube_hal* const pd = (const us015_cube_hal*)dev->platform_dev;

    HAL_GPIO_WritePin(pd->trigger_port, pd->trigger_pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//-----------------------------------------------------------------------------
bool us015_get_echo_pin(const us015* const dev)
{
    const us015_cube_hal* const pd = (const us015_cube_hal*)dev->platform_dev;

    return HAL_GPIO_ReadPin(pd->echo_port, pd->echo_pin) == GPIO_PIN_SET;
}

//-----------------------------------------------------------------------------
uint32_t us015_get_us_ticks(void)
{
    // Cortex-M0 has no DWT cycle counter, so the SysTick reload counter is
    // used to interpolate microseconds within the current HAL millisecond.
    const uint32_t reload = SysTick->LOAD + 1U;
    uint32_t ms;
    uint32_t val;

    do
    {
        ms = HAL_GetTick();
        val = SysTick->VAL;
    } while (ms != HAL_GetTick());

    return (ms * 1000U) + (((reload - val) * 1000U) / reload);
}

//-----------------------------------------------------------------------------
void us015_delay_us(uint32_t delay_us)
{
    const uint32_t start = us015_get_us_ticks();

    while ((us015_get_us_ticks() - start) < delay_us)
    {
    }
}

//-----------------------------------------------------------------------------
