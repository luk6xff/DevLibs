
/**
 *  @brief:  Implementation of a RFM69 platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */

#include "rfm69-mbed.h"


/**
 * Tx and Rx timers
 */
static Timer timer_ms;

//-----------------------------------------------------------------------------
void rfm69_mbed_init(rfm69* const dev, rfm69_mbed* const mbed_dev)
{
    dev->platform_dev = mbed_dev;

    timer_ms.start();

    rfm69_init(dev);
}

//-----------------------------------------------------------------------------
void rfm69_mbed_deinit(rfm69* const dev)
{
    // IO
    rfm69_io_deinit(dev);
}

//-----------------------------------------------------------------------------
void rfm69_io_init(rfm69* const dev)
{
    rfm69_mbed* const pd = (rfm69_mbed*)dev->platform_dev;
    // Init SPI
    *(pd->nss) = 1;
    pd->spi->format(8,0);
    uint32_t frequencyToSet = 4000000;
    pd->spi->frequency(frequencyToSet);
    rfm69_delay_ms(10);
}

//-----------------------------------------------------------------------------
void rfm69_io_deinit(rfm69* const dev)
{
    // EMPTY
}


//-----------------------------------------------------------------------------
void rfm69_ioirq_init(rfm69* const dev)
{
    rfm69_mbed* const pd = (rfm69_mbed*)dev->platform_dev;
    // dio0
    pd->dio0->rise(mbed::callback(dev->dio_irq[0], (void*)dev));
}

//-----------------------------------------------------------------------------
void rfm69_reset(rfm69* const dev)
{
    rfm69_mbed* const pd = (rfm69_mbed*)dev->platform_dev;
    pd->reset->output();
    *(pd->reset) = 1;
    rfm69_delay_ms(10);
    *(pd->reset) = 0;
    rfm69_delay_ms(10);
}

//-----------------------------------------------------------------------------
void rfm69_write_buffer(rfm69* const dev, uint8_t addr, const uint8_t *buffer, uint8_t size)
{
    rfm69_mbed* const pd = (rfm69_mbed*)dev->platform_dev;
    uint8_t i;

    *(pd->nss) = 0;
    pd->spi->write(addr | 0x80);
    for(i = 0; i < size; i++)
    {
        pd->spi->write(buffer[i]);
    }
    *(pd->nss) = 1;
}

//-----------------------------------------------------------------------------
void rfm69_read_buffer(rfm69* const dev, uint8_t addr, uint8_t *buffer, uint8_t size)
{
    rfm69_mbed* const pd = (rfm69_mbed*)dev->platform_dev;
    uint8_t i;

    *(pd->nss) = 0;
    pd->spi->write(addr & 0x7F);
    for(i = 0; i < size; i++)
    {
        buffer[i] = pd->spi->write(0);
    }
    *(pd->nss) = 1;
}

//-----------------------------------------------------------------------------
void rfm69_delay_ms(int ms)
{
    wait_us(ms*1000);
}

//-----------------------------------------------------------------------------
uint32_t rfm69_timer_read_ms()
{
    return timer_ms.read_ms();
}

//-----------------------------------------------------------------------------
void rfm69_print_all_regs(rfm69* const dev)
{
    uint8_t reg_val;
    debug("\r\n<<<RFM69 REGISTERS>>>\r\nADDR - HEX\r\n");
    for (uint8_t reg_addr = 1; reg_addr <= 0x71; reg_addr++)
    {
        reg_val = rfm69_read_reg(dev, reg_addr);
        debug("0x%x", reg_addr);
        debug(" - ");
        debug("0x%x\r\n", reg_val);
    }
}

//-----------------------------------------------------------------------------