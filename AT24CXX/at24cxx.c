 #include "at24cxx.h"

/** AT24CXX device capability table, indexed by at24cxx_type **/
const at24cxx_mem at24cxx_devices[] =
{
    { AT24C01,   128,    8,   1 },  //128 bytes=1Kbit, 8bytes=64bit
    { AT24C02,   256,    8,   1 },
    { AT24C04,   512,    16,  1 },
    { AT24C08,   1024,   16,  1 },
    { AT24C16,   2048,   16,  1 },
    { AT24C32,   4096,   32,  2 },
    { AT24C64,   8192,   32,  2 },
    { AT24C128,  16384,  64,  2 },
    { AT24C256,  32768,  64,  2 },
    { AT24C512,  65536,  128, 2 },
    { AT24C1024, 131072, 128, 2 },
};

//------------------------------------------------------------------------------
at24cxx_status at24cxx_init(at24cxx* const dev, uint8_t i2c_addr_pins)
{
    if (!dev)
    {
        return AT24CXX_ERR;
    }
    uint8_t i2c_addr = i2c_addr_pins;
    switch (dev->type)
    {
        case AT24C01:
        case AT24C02:
            if (i2c_addr > 0x07)
            {
                return AT24CXX_BAD_ADDR;
            }
            dev->addr = AT24CXX_ADDR | (i2c_addr << 1);
            break;

        case AT24C04:
            if (i2c_addr > 0x07)
            {
                return AT24CXX_BAD_ADDR;
            }
            dev->addr = AT24CXX_ADDR | (i2c_addr & 0xFE) << 1;
            break;

        case AT24C08:
            if (i2c_addr > 0x07)
            {
                return AT24CXX_BAD_ADDR;
            }
            dev->addr = AT24CXX_ADDR | (i2c_addr & 0xFC) << 1;
            break;

        case AT24C16:
            dev->addr = AT24CXX_ADDR;
            break;

        case AT24C32:
        case AT24C64:
            if (i2c_addr > 0x07)
            {
                return AT24CXX_BAD_ADDR;
            }
            dev->addr = AT24CXX_ADDR | (i2c_addr << 1);
            break;

        case AT24C128:
        case AT24C256:
        case AT24C512:
            if (i2c_addr > 0x03)
            {
                return AT24CXX_BAD_ADDR;
            }
            dev->addr = AT24CXX_ADDR | (i2c_addr << 1);
            break;

        case AT24C1024:
            if (i2c_addr > 0x07)
            {
                return AT24CXX_BAD_ADDR;
            }
            dev->addr = AT24CXX_ADDR | (i2c_addr & 0xFE) << 1;
            break;

        default:
            return AT24CXX_PARAM_ERR;
    }

    at24cxx_io_init(dev);

    return AT24CXX_NOERR;
}


//------------------------------------------------------------------------------
at24cxx_status at24cxx_write(const at24cxx* const dev, uint32_t addr,
                             const uint8_t* data, uint32_t data_size)
{
    at24cxx_status ret = AT24CXX_NOERR;
    const uint8_t page_size = at24cxx_devices[dev->type].page_size;
    size_t bytes_sent = 0;

    if (!at24cxx_check_space(dev, addr, data_size))
    {
        return AT24CXX_OUT_OF_RANGE;
    }

    size_t page_num = 0;
    size_t bytes_to_be_send_left = 0;
    size_t bytes_to_be_send_right = 0;

    size_t page_constraint_left = addr % page_size;
    size_t page_constraint_right = (addr+data_size) % page_size;

    if (page_constraint_left == 0)
    {
        bytes_to_be_send_left = 0;
    }
    else
    {
        bytes_to_be_send_left = page_size - page_constraint_left;
        // Send left side of the data
        ret = at24cxx_write_buffer(dev, addr, data, bytes_to_be_send_left);
        at24cxx_wait_for_ready(dev);
        if (ret != AT24CXX_NOERR)
        {
            return ret;
        }
        bytes_sent += bytes_to_be_send_left;
    }

    if (page_constraint_right == 0)
    {
        bytes_to_be_send_right = 0;
    }
    else
    {
        bytes_to_be_send_right = page_constraint_right;
        // Send right side of the data
        ret = at24cxx_write_buffer(dev, (addr+data_size-bytes_to_be_send_right), &data[data_size-bytes_to_be_send_right], bytes_to_be_send_right);
        at24cxx_wait_for_ready(dev);
        if (ret != AT24CXX_NOERR)
        {
            return ret;
        }
        bytes_sent += bytes_to_be_send_right;
    }

    // Send full pages
    while (bytes_sent < data_size)
    {
        const size_t offset = bytes_to_be_send_left + (page_num*page_size);
        ret = at24cxx_write_buffer(dev, addr + offset, &data[offset], page_size);

        at24cxx_wait_for_ready(dev);
        if (ret != AT24CXX_NOERR)
        {
            return ret;
        }
        page_num++;
        bytes_sent += page_size;
    }
    return ret;
}

//-----------------------------------------------------------------------------
at24cxx_status at24cxx_read(const at24cxx* const dev, uint32_t addr,
                            uint8_t* data, uint32_t data_size)
{
    at24cxx_status ret = AT24CXX_NOERR;

    if (!at24cxx_check_space(dev, addr, data_size))
    {
        return AT24CXX_OUT_OF_RANGE;
    }

    ret = at24cxx_read_buffer(dev, addr, data, data_size);
    at24cxx_wait_for_ready(dev);

    if (ret != AT24CXX_NOERR)
    {
        return ret;
    }

    return ret;
}

//-----------------------------------------------------------------------------
bool at24cxx_check_space(const at24cxx* const dev, uint32_t addr, size_t size)
{
    if (dev->type < AT24CINVALID)
    {
        // Check that the address start in the chip and doesn't extend too broad
        if ((addr >= at24cxx_devices[dev->type].size) ||
           ((addr + size) >= at24cxx_devices[dev->type].size))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
void at24cxx_wait_for_ready(const at24cxx* const dev)
{
    // The EEPROM does not ACK while an internal write cycle is in progress.
    // A fixed delay covers the worst case tWR of all supported devices.
    (void)dev;
    at24cxx_delay_ms(10);
}


//-----------------------------------------------------------------------------
