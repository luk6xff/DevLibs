#include "at24cxx-cube.h"


//-------------------------------------------------------------------------------
void at24cxx_cube_init(at24cxx* const dev, at24cxx_cube* const cube_dev, uint8_t i2c_addr_pins)
{
    dev->platform_dev = cube_dev;
    at24cxx_init(dev, i2c_addr_pins);
}

//------------------------------------------------------------------------------
void at24cxx_cube_deinit(at24cxx* const dev)
{
    at24cxx_io_deinit(dev);
}

//------------------------------------------------------------------------------
at24cxx_status at24cxx_io_init(at24cxx* const dev)
{
    at24cxx_enable_wp(dev, false); // Disable Write protection
    return AT24CXX_NOERR;
}

//------------------------------------------------------------------------------
at24cxx_status at24cxx_io_deinit(at24cxx* const dev)
{
    // Empty
    return AT24CXX_NOERR;
}

//------------------------------------------------------------------------------
at24cxx_status at24cxx_write_buffer(const at24cxx* const dev, uint32_t addr,
                                    const uint8_t* buf, size_t buf_size)
{
    // Check space
	if (!at24cxx_check_space(dev, addr, buf_size))
    {
        return AT24CXX_OUT_OF_RANGE;
    }

    const at24cxx_cube* const pd = (at24cxx_cube*)dev->platform_dev;
    const uint8_t addr_len = at24cxx_devices[dev->type].word_addr_len;

    uint8_t data[addr_len+buf_size];
    if (addr_len == 1)
    {
    	data[0] = addr;
    }
    else if (addr_len == 2)
    {
    	data[0] = addr >> 8;
    	data[1] = addr & 0xFF;
    }
    else
    {
    	// Word address length not supported
    	return AT24CXX_PARAM_ERR;
    }

    memcpy(&data[addr_len], buf, buf_size);

    while (HAL_I2C_IsDeviceReady(pd->i2c, (uint16_t)dev->addr, 3, 100) != HAL_OK) {}

    int ack = HAL_I2C_Master_Transmit(pd->i2c, (uint16_t)dev->addr, data, sizeof(data), 1000);
    if (ack != HAL_OK)
    {
        return AT24CXX_ERR;
    }
    return AT24CXX_NOERR;
}

//------------------------------------------------------------------------------
at24cxx_status at24cxx_read_buffer(const at24cxx* const dev, uint32_t addr,
                                   uint8_t* buf, size_t buf_size)
{
    const at24cxx_cube* const pd = (at24cxx_cube*)dev->platform_dev;

    const uint8_t addr_len = at24cxx_devices[dev->type].word_addr_len;
    uint8_t address[addr_len];
    if (addr_len == 1)
    {
        address[0] = addr;
    }
    else if (addr_len == 2)
    {
        address[0] = addr >> 8;
        address[1] = addr & 0xFF;
    }
    else
    {
    	// word address length not supported
    	return AT24CXX_PARAM_ERR;
    }

    while (HAL_I2C_IsDeviceReady(pd->i2c, (uint16_t)dev->addr, 3, 100) != HAL_OK) {};

    // Write addr
    int ack = HAL_I2C_Master_Transmit(pd->i2c, (uint16_t)dev->addr, address, addr_len, 1000);
    if (ack != HAL_OK)
    {
        return AT24CXX_I2C_ERR;
    }

    while (HAL_I2C_IsDeviceReady(pd->i2c, (uint16_t)dev->addr, 3, 100) != HAL_OK) {};

    // Sequential Read
    int retVal = HAL_I2C_Master_Receive(pd->i2c, (uint16_t)dev->addr, buf, buf_size, 1000);
    if (retVal != HAL_OK)
    {
        return AT24CXX_ERR;
    }

    return AT24CXX_NOERR;
}

//------------------------------------------------------------------------------
void at24cxx_enable_wp(at24cxx* const dev, bool enable)
{
    const at24cxx_cube* const pd = (at24cxx_cube*)dev->platform_dev;
    if (enable)
    {
    	HAL_GPIO_WritePin(pd->gpio_wp_port, pd->gpio_wp_pin, GPIO_PIN_SET);
        return;
    }
	HAL_GPIO_WritePin(pd->gpio_wp_port, pd->gpio_wp_pin, GPIO_PIN_RESET);
}


//------------------------------------------------------------------------------
void at24cxx_delay_ms(uint32_t delay_ms)
{
	  uint32_t tickstart_ms = HAL_GetTick();
	  while((HAL_GetTick()-tickstart_ms) < delay_ms);
}

//------------------------------------------------------------------------------
