/**
 *  @brief:   BMP180 digital pressure sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-08-12
 *  @license: MIT
 */

#include "bmp180.h"
#include <math.h> // pow

// Uncomment to test the documentation algorithm against the documentation example
// Result should be 699.64 hPa and 15°C
// #define BMP180_TEST_FORMULA


//------------------------------------------------------------------------------
// @brief PRIVATE FUNCTIONS
//------------------------------------------------------------------------------

/**
 * @brief Reads raw temperature value from the sensor
 *
 * @return temperature [C]
 */
static int bmp180_read_raw_temperature(bmp180 *const dev, int32_t* pUt);

/**
 * @brief Reads raw pressure value from the sensor
 *
 * @return pressure [hPa]
 */
static int bmp180_read_raw_pressure(bmp180 *const dev, int32_t* pUp);

/**
 * @brief Calculation of the temperature from the digital output
 */
static float bmp180_calc_true_temperature(bmp180 *const dev, int32_t ut);

/**
 * @brief Calculation of the pressure from the digital output
 */
static float bmp180_calc_true_pressure(bmp180 *const dev, int32_t up);


//------------------------------------------------------------------------------
// @brief FUNCTIONS DEFINITIONS
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
bool bmp180_init(bmp180 *const dev)
{
    return bmp180_set_configuration(dev, dev->altitude, dev->oss);
}

//------------------------------------------------------------------------------
bool bmp180_set_configuration(bmp180 *const dev, float altitude, bmp180_oversampling_t oss)
{
    uint8_t data[22];
    int errors = 0;

    dev->altitude = altitude;
    dev->oss = oss;

    // read calibration data
    data[0] = BMP180_REG_AC1;
    errors += bmp180_write(dev, data, 1);
    errors += bmp180_read(dev, data, sizeof(data)); // // read 11 x 16 bits at this position
    bmp180_delay_ms(10);

    // store calibration data for further calculus
    dev->calib.ac1 = data[0]  << 8 | data[1];
    dev->calib.ac2 = data[2]  << 8 | data[3];
    dev->calib.ac3 = data[4]  << 8 | data[5];
    dev->calib.ac4 = data[6]  << 8 | data[7];
    dev->calib.ac5 = data[8]  << 8 | data[9];
    dev->calib.ac6 = data[10] << 8 | data[11];
    dev->calib.b1  = data[12] << 8 | data[13];
    dev->calib.b2  = data[14] << 8 | data[15];
    dev->calib.mb  = data[16] << 8 | data[17];
    dev->calib.mc  = data[18] << 8 | data[19];
    dev->calib.md  = data[20] << 8 | data[21];

#ifdef BMP180_TEST_FORMULA
    dev->calib.ac1 = 408;
    dev->calib.ac2 = -72;
    dev->calib.ac3 = -14383;
    dev->calib.ac4 = 32741;
    dev->calib.ac5 = 32757;
    dev->calib.ac6 = 23153;
    dev->calib.b1 = 6190;
    dev->calib.b2 = 4;
    dev->calib.mb = -32768;
    dev->calib.mc = -8711;
    dev->calib.md = 2868;
    dev->oss = 0;
#endif // #ifdef BMP180_TEST_FORMULA

    return errors ? true : false;
}

//------------------------------------------------------------------------------
bool bmp180_reset(bmp180 *const dev)
{
    uint8_t data;
    data = BMP180_REG_RESET;
    if (!bmp180_write(dev, &data, 1))
    {
    	return false;
    }

    data = 0xB6;
    if (!bmp180_write(dev, &data, 1))
    {
    	return false;
    }

    return true;
}

//------------------------------------------------------------------------------
bool bmp180_check_id(bmp180 *const dev)
{
    uint8_t data[2];
    data[0] = BMP180_REG_CHIP_ID;
    bmp180_write(dev, data, 1);
    bmp180_read(dev, data, 1);

    if (data[0] != 0x55)
    {
        return false;
    }

    return true;
}

//------------------------------------------------------------------------------
bool bmp180_read_data(bmp180 *const dev, float *temperature, float *pressure)
{
    if (!bmp180_get_temperature(dev, temperature) || !bmp180_get_pressure(dev, pressure))
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
bool bmp180_get_temperature(bmp180 *const dev, float *temperature)
{
    int32_t t;

    if (!bmp180_read_raw_temperature(dev, &t))
    {
        return false;
    }

    *temperature = bmp180_calc_true_temperature(dev, t);
    return true;
}

//------------------------------------------------------------------------------
bool bmp180_get_pressure(bmp180 *const dev, float *pressure)
{
    int32_t p;

    if (!bmp180_read_raw_pressure(dev, &p))
    {
        return false;
    }

    *pressure = bmp180_calc_true_pressure(dev, p);

    return true;
}

//------------------------------------------------------------------------------
int bmp180_read_raw_temperature(bmp180 *const dev, int32_t* pUt)
{
    int errors = 0;
    uint8_t data[2];

    // Request temperature measurement
    data[0] = BMP180_REG_CTRL;
    data[1] = BMP180_CMD_MEASURE_TEMP;
    errors = bmp180_write(dev, data, sizeof(data)); // write 0x2E into reg 0xF4

    bmp180_delay_ms(4.5F);

    // Read raw temperature data
    data[0] = BMP180_REG_DATA;
    errors += bmp180_write(dev, data, 1);
    errors += bmp180_read(dev, data, sizeof(data)); // get 16 bits at this position

#ifdef BMP180_TEST_FORMULA
    errors = 0;
#endif // #ifdef BMP180_TEST_FORMULA

    if (errors < 3)
    {
        return 0;
    }
    else
    {
        *pUt = ((data[0] << 8) | data[1]);
    }


#ifdef BMP180_TEST_FORMULA
    *pUt = 27898;
#endif

    return 1;
}

//------------------------------------------------------------------------------
int bmp180_read_raw_pressure(bmp180 *const dev, int32_t* pUp)
{
    int errors = 0;
    uint8_t data[2];

    // Request pressure measurement
    data[0] = BMP180_REG_CTRL;
    data[1] = BMP180_CMD_MEASURE_PRESSURE_0  + (dev->oss << 6);
    errors = bmp180_write(dev, data, sizeof(data)); // write 0x34 + (dev->oss << 6) into reg 0xF4

    switch (dev->oss)
    {
        case BMP180_OSS_ULTRA_LOW_POWER:
        {
            bmp180_delay_ms(4.5);
            break;
        }
        case BMP180_OSS_NORMAL:
        {
            bmp180_delay_ms(7.5);
            break;
        }
        case BMP180_OSS_HIGH_RESOLUTION:
        {
            bmp180_delay_ms(13.5);
            break;
        }
        case BMP180_OSS_ULTRA_HIGH_RESOLUTION:
        {
            bmp180_delay_ms(25.5);
            break;
        }
    }

    // Read raw pressure data
    data[0] = BMP180_REG_DATA;
    errors += bmp180_write(dev, data, 1);
    errors += bmp180_read(dev, data, sizeof(data)); // get 16 bits at this position

#ifdef BMP180_TEST_FORMULA
    errors = 0;
#endif // #ifdef BMP180_TEST_FORMULA

    if (errors < 3)
    {
        return 0;
    }
    else
    {
        *pUp = (data[0] << 16 | data[1] << 8) >> (8 - dev->oss);
    }

#ifdef BMP180_TEST_FORMULA
    *pUp = 23843;
#endif // #ifdef BMP180_TEST_FORMULA

    return 1;
}

//------------------------------------------------------------------------------
float bmp180_calc_true_temperature(bmp180 *const dev, int32_t ut)
{
    int32_t t, x1, x2;

    // Straight out from the documentation
    x1 = ((ut - dev->calib.ac6) * dev->calib.ac5) >> 15;
    x2 = ((int32_t)dev->calib.mc << 11) / (x1 + dev->calib.md);
    dev->b5 = x1 + x2;
    t = (dev->b5 + 8) >> 4;

    // convert to celcius
    return t / 10.F;
}

//------------------------------------------------------------------------------
float bmp180_calc_true_pressure(bmp180 *const dev, int32_t up)
{
    int32_t p, x1, x2, x3, b3, b6;
    uint32_t b4, b7;

    // Straight out from the documentation
    b6 = dev->b5 - 4000;
    x1 = (dev->calib.b2 * (b6 * b6 >> 12)) >> 11;
    x2 = dev->calib.ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((dev->calib.ac1 * 4 + x3) << dev->oss) + 2) >> 2;
    x1 = (dev->calib.ac3 * b6) >> 13;
    x2 = (dev->calib.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = dev->calib.ac4 * (uint32_t)(x3 + 32768) >> 15;
    b7 = ((uint32_t)up - b3)* (50000 >> dev->oss);
    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) << 1;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    // Convert to hPa and, if altitude has been initialized, to sea level pressure
    if (dev->altitude == 0.F)
    {
        return p / 100.F;
    }

    return  p / (100.F * pow((1.F - dev->altitude / 44330.0L), 5.255L));
}

//------------------------------------------------------------------------------
