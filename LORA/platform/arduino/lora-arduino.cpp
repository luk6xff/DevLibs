/**
 *  @brief:  Implementation of LORA platform dependent [ARDUINO] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-11-22
 */

#include "lora-arduino.h"


//-----------------------------------------------------------------------------
bool lora_arduino_init(lora *const dev, lora_arduino *const arduino_dev)

{
    dev->platform_dev = arduino_dev;
    return lora_init(dev);
}

//-----------------------------------------------------------------------------
void lora_arduino_deinit(lora *const dev)
{
    lora_end(dev);
}

//-----------------------------------------------------------------------------
void lora_io_init(lora *const dev)
{
    // Get arduino platform data
    lora_arduino *const pd = (lora_arduino*)dev->platform_dev;
    // Init SPI
    pd->spi_settings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
    // Setup pins
    pinMode(pd->nss, OUTPUT);
    // Set SS high
    digitalWrite(pd->nss, HIGH);
    // Start SPI
    pd->spi->begin(pd->sck, pd->miso, pd->mosi, pd->nss);
    lora_delay_ms(10);
}

//-----------------------------------------------------------------------------
void lora_io_deinit(lora *const dev)
{
    // Get arduino platform data
    lora_arduino *const pd = (lora_arduino*)dev->platform_dev;
    // Init SPI
    pd->spi->end();
    lora_delay_ms(10);
}

//-----------------------------------------------------------------------------
void lora_ioirq_init(lora *const dev)
{
    lora_arduino *const pd = (lora_arduino*)dev->platform_dev;
    if (dev->dio_irq == NULL)
    {
        // TODO print error here
        return;
    }
    // dio0
    int interrupt_num = digitalPinToInterrupt(pd->dio0);
    if (interrupt_num == NOT_AN_INTERRUPT)
    {
        // TODO print error here
        return;
    }
    pinMode(pd->dio0, INPUT);
    attachInterruptArg(digitalPinToInterrupt(pd->dio0), dev->dio_irq, dev, RISING);
}

//-----------------------------------------------------------------------------
void lora_ioirq_deinit(lora *const dev)
{
    lora_arduino *const pd = (lora_arduino*)dev->platform_dev;
    if (dev->dio_irq == NULL)
    {
        return;
    }
    // dio0
    pinMode(pd->dio0, INPUT);
    detachInterrupt(digitalPinToInterrupt(pd->dio0));
}

//-----------------------------------------------------------------------------
void lora_reset(lora *const dev)
{
    lora_arduino *const pd = (lora_arduino*)dev->platform_dev;

    pinMode(pd->reset, OUTPUT);
    digitalWrite(pd->reset, LOW);
    lora_delay_ms(10);
    pinMode(pd->reset, INPUT);
    //digitalWrite(pd->reset, HIGH);
    lora_delay_ms(10);
}

//-----------------------------------------------------------------------------
void lora_write_buffer(lora *const dev, uint8_t addr, const uint8_t *buffer, const uint8_t size)
{
    lora_arduino *const pd = (lora_arduino*)dev->platform_dev;

    digitalWrite(pd->nss, LOW);
    pd->spi->beginTransaction(pd->spi_settings);
    pd->spi->transfer(addr | 0x80);
    pd->spi->writeBytes(buffer, size);
    pd->spi->endTransaction();
    digitalWrite(pd->nss, HIGH);
}

//-----------------------------------------------------------------------------
void lora_read_buffer(lora *const dev, uint8_t addr, uint8_t *buffer, uint8_t size)
{
    lora_arduino *const pd = (lora_arduino*)dev->platform_dev;

    digitalWrite(pd->nss, LOW);
    pd->spi->beginTransaction(pd->spi_settings);
    pd->spi->transfer(addr & 0x7F);
    for(int i = 0; i < size; i++)
    {
        buffer[i] = pd->spi->transfer(0);
    }
    pd->spi->endTransaction();
    digitalWrite(pd->nss, HIGH);
}

//-----------------------------------------------------------------------------
void lora_delay_ms(int ms)
{
    delay(ms);
}

//-----------------------------------------------------------------------------
uint32_t lora_timer_read_ms()
{
    return millis();
}

//-----------------------------------------------------------------------------
void lora_dump_registers(lora *const dev)
{
    Serial.printf("\r\n<<<LORA DEV REGISTERS>>>\r\nADDR - HEX\r\n");
    for (int i = 0; i < 0x80; i++)
    {
        Serial.printf("0x%X:0x%X, ", i, lora_read_reg(dev, i));
    }
}

//-----------------------------------------------------------------------------
