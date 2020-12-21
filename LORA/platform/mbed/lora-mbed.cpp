/**
 *  @brief:  Implementation of a LORA platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-08-07
 */

#include "lora-mbed.h"

/**
 * Miliseconds timer
 */
static Timer timer_ms;

//-----------------------------------------------------------------------------
void lora_mbed_init(lora *const dev, lora_mbed *const mbed_dev)

{
    dev->platform_dev = mbed_dev;
    lora_init(dev);
}

//-----------------------------------------------------------------------------
void lora_mbed_deinit(lora *const dev)
{
    // IO
    lora_io_deinit(dev);
}

//-----------------------------------------------------------------------------
void lora_io_init(lora *const dev)
{
    lora_mbed *const pd = (lora_mbed*)dev->platform_dev;
    // Init SPI
    *(pd->nss) = 1;
    pd->spi->format(8,0);
    uint32_t frequencyToSet = 8000000;
    pd->spi->frequency(frequencyToSet);
    lora_delay_ms(100);
}

//-----------------------------------------------------------------------------
void lora_io_deinit(lora *const dev)
{
    lora_io_deinit(dev);
}

//-----------------------------------------------------------------------------
void lora_ioirq_init(lora *const dev)
{
    lora_mbed *const pd = (lora_mbed*)dev->platform_dev;
    // dio0
    pd->dio0->rise(mbed::callback(dev->dio_irq, (void*)dev));
}

//-----------------------------------------------------------------------------
void lora_ioirq_deinit(lora *const dev)
{
    lora_mbed *const pd = (lora_mbed*)dev->platform_dev;
    // dio0
    pd->dio0->fall(mbed::callback(dev->dio_irq, (void*)dev));
}


//-----------------------------------------------------------------------------
void lora_reset(lora *const dev)
{
    lora_mbed *const pd = (lora_mbed*)dev->platform_dev;
    pd->reset->output();
    *(pd->reset) = 0;
    lora_delay_ms(1);
    pd->reset->input();
    lora_delay_ms(6);
}

//-----------------------------------------------------------------------------
void lora_write_buffer(lora *const dev, uint8_t addr, const uint8_t *buffer, const uint8_t size)
{
    lora_mbed *const pd = (lora_mbed*)dev->platform_dev;
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
void lora_read_buffer(lora *const dev, uint8_t addr, uint8_t *buffer, uint8_t size)
{
    lora_mbed *const pd = (lora_mbed*)dev->platform_dev;
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
void lora_delay_ms(int ms)
{
    wait_us(ms*1000);
}

//-----------------------------------------------------------------------------
uint32_t lora_timer_read_ms()
{
    return timer_ms.read_ms();
}

//-----------------------------------------------------------------------------
void lora_dump_registers(lora *const dev)
{
    printf("\r\n<<<LORA DEV REGISTERS>>>\r\nADDR - HEX\r\n");
    for (int i = 0; i < 0x80; i++)
    {
        printf("0x%x: 0x%x\n", i, lora_read_reg(dev, i));
    }
}

//-----------------------------------------------------------------------------

