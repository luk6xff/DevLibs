/**
 *  @brief:   Implementation of a BME280 platform dependent [ARDUINO] functions
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2022-10-04
 *  @license: MIT
 */

#include "bme280-arduino.h"

//-------------------------------------------------------------------------------
bool bme280_arduino_init(bme280 *const dev, bme280_arduino *const arduino_dev)
{
    dev->platform_dev = arduino_dev;
    bme280_arduino *const pd = (bme280_arduino*)dev->platform_dev;

    if (dev->intf == BME280_INTF_I2C)
    {
        // Init I2C
        pd->i2c.hndl->begin(pd->i2c.sda_pin, pd->i2c.scl_pin);
        pd->i2c.hndl->setClock(400000);
    }
    else // dev->intf == BME280_INTF_SPI
    {
        // Init SPI
        pd->spi.settings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
        // Setup pins
        pinMode(pd->spi.nss, OUTPUT);
        // Set SS high
        digitalWrite(pd->spi.nss, HIGH);
        // Start SPI
        pd->spi.hndl->begin(pd->spi.sck, pd->spi.miso, pd->spi.mosi, pd->spi.nss);
    }

    return bme280_init(dev);
}

//------------------------------------------------------------------------------
bool bme280_arduino_deinit(bme280 *const dev)
{
    bme280_arduino *const pd = (bme280_arduino*)dev->platform_dev;
    if (dev->intf == BME280_INTF_SPI)
    {
        // Deinit SPI
        pd->spi.hndl->end();
    }
	return true;
}

//------------------------------------------------------------------------------
bool bme280_write(bme280 *const dev, const uint8_t* buf, const size_t buf_size)
{
    bme280_arduino *const pd = (bme280_arduino*)dev->platform_dev;
    if (dev->intf == BME280_INTF_I2C)
    {
        pd->i2c.hndl->beginTransmission(dev->i2c_addr);
        for (size_t i = 0; i < buf_size; i++)
        {
            pd->i2c.hndl->write((uint8_t) buf[i]);
        }
        return pd->i2c.hndl->endTransmission() == 0;
    }
    else // dev->intf == BME280_INTF_SPI
    {
        bme280_arduino *const pd = (bme280_arduino*)dev->platform_dev;
        digitalWrite(pd->spi.nss, LOW);
        pd->spi.hndl->beginTransaction(pd->spi.settings);
        //pd->spi.hndl->transfer(buf[0] | 0x80);
        pd->spi.hndl->writeBytes(buf, buf_size);
        pd->spi.hndl->endTransaction();
        digitalWrite(pd->spi.nss, HIGH);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool bme280_read(bme280 *const dev, uint8_t* buf, const size_t buf_size)
{
    bme280_arduino *const pd = (bme280_arduino*)dev->platform_dev;
    if (dev->intf == BME280_INTF_I2C)
    {
        const size_t timeout_ms = 1000; // 1 second timeout
        size_t recv_data_cntr = 0;
        bme280_arduino *const pd = (bme280_arduino*)dev->platform_dev;

        pd->i2c.hndl->beginTransmission(dev->i2c_addr);
        // Read data
        pd->i2c.hndl->requestFrom(dev->i2c_addr, buf_size);
        const size_t t1 = millis();
        for (; pd->i2c.hndl->available() && (timeout_ms == 0 || (millis() - t1) < timeout_ms);
                recv_data_cntr++)
        {
            buf[recv_data_cntr] = pd->i2c.hndl->read();
        }
        return pd->i2c.hndl->endTransmission() == 0 && recv_data_cntr == buf_size;
    }
    else // dev->intf == BME280_INTF_SPI
    {
        bme280_arduino *const pd = (bme280_arduino*)dev->platform_dev;

        digitalWrite(pd->spi.nss, LOW);
        pd->spi.hndl->beginTransaction(pd->spi.settings);
        for(int i = 0; i < buf_size; i++)
        {
            buf[i] = pd->spi.hndl->transfer(0);
        }
        pd->spi.hndl->endTransaction();
        digitalWrite(pd->spi.nss, HIGH);
    }
    return true;
}

//-----------------------------------------------------------------------------
void bme280_delay_ms(uint32_t delay_ms)
{
    delay(delay_ms);
}