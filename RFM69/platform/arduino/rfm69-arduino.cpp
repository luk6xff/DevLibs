
/**
 *  @brief:  Implementation of a RFM69 platform dependent [ARDUINO] radio functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */
#include "rfm69-arduino.h"


//------------------------------------------------------------------------------
bool rfm69_arduino_init(rfm69* const dev, rfm69_arduino* const arduino_dev)
{
    dev->platform_dev = arduino_dev;
    return rfm69_init(dev);
}

//------------------------------------------------------------------------------
void rfm69_arduino_deinit(rfm69* const dev)
{
    // IO
    rfm69_io_deinit(dev);
}

//------------------------------------------------------------------------------
void rfm69_io_init(rfm69* const dev)
{
    // Get arduino platform data
    rfm69_arduino* const pd = (rfm69_arduino*)dev->platform_dev;
    // Init SPI
    pd->spi_settings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
    // Start SPI
    pd->spi->begin();
    // Setup pins
    pinMode(pd->nss, OUTPUT);
    // Set SS high
    digitalWrite(pd->nss, HIGH);
    rfm69_delay_ms(10);
}

//------------------------------------------------------------------------------
void rfm69_io_deinit(rfm69* const dev)
{
    // Empty
}


//------------------------------------------------------------------------------
void rfm69_ioirq_init(rfm69* const dev)
{
    rfm69_arduino* const pd = (rfm69_arduino*)dev->platform_dev;
    if (dev->dio0_irq == NULL)
    {
        // TODO print error here
        while(1);
        return;
    }
    // dio0
    int interrupt_num = digitalPinToInterrupt(pd->dio0);
    if (interrupt_num == NOT_AN_INTERRUPT)
    {
        // TODO print error here
        while(1);
        return;
    }
    pinMode(pd->dio0, INPUT);
    attachInterruptArg(digitalPinToInterrupt(pd->dio0), dev->dio0_irq, dev, RISING);
}

//------------------------------------------------------------------------------
void rfm69_reset(rfm69* const dev)
{
    rfm69_arduino* const pd = (rfm69_arduino*)dev->platform_dev;

    pinMode(pd->reset, OUTPUT);
    digitalWrite(pd->reset, HIGH);
    rfm69_delay_ms(10);
    digitalWrite(pd->reset, LOW);
    rfm69_delay_ms(10);
}

//------------------------------------------------------------------------------
void rfm69_write_buffer(rfm69* const dev, uint8_t addr, const uint8_t* buffer, uint8_t size)
{
    rfm69_arduino* const pd = (rfm69_arduino*)dev->platform_dev;

    pd->spi->beginTransaction(pd->spi_settings);
    digitalWrite(pd->nss, LOW);
    pd->spi->transfer(addr | 0x80);
    pd->spi->writeBytes(buffer, size);
    digitalWrite(pd->nss, HIGH);
    pd->spi->endTransaction();
}

//------------------------------------------------------------------------------
void rfm69_read_buffer(rfm69* const dev, uint8_t addr, uint8_t* buffer, uint8_t size)
{
    rfm69_arduino* const pd = (rfm69_arduino*)dev->platform_dev;

    pd->spi->beginTransaction(pd->spi_settings);
    digitalWrite(pd->nss, LOW);
    pd->spi->transfer(addr & 0x7F);
    for(int i = 0; i < size; i++)
    {
        buffer[i] = pd->spi->transfer(0);
    }
    digitalWrite(pd->nss, HIGH);
    pd->spi->endTransaction();
}

//------------------------------------------------------------------------------
void rfm69_delay_ms(int ms)
{
    delay(ms);
}

//------------------------------------------------------------------------------
uint32_t rfm69_timer_read_ms()
{
    return millis();
}

//------------------------------------------------------------------------------
void rfm69_print_all_regs(rfm69* const dev)
{
    uint8_t reg_val;
    Serial.printf("\r\n<<<RFM69 REGISTERS>>>\r\nADDR - HEX\r\n");
    for (uint8_t reg_addr = 1; reg_addr <= 0x71; reg_addr++)
    {
        reg_val = rfm69_read_reg(dev, reg_addr);
        Serial.printf("0x%x", reg_addr);
        Serial.printf(" - ");
        Serial.printf("0x%x\r\n", reg_val);
    }
}

//------------------------------------------------------------------------------