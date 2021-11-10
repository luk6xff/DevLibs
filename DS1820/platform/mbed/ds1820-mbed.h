
/**
 *  @brief:  Implementation of a DS1820 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-02
 */


#ifndef __DS1820_MBED_H__
#define __DS1820_MBED_H__

#include "mbed.h"
#include "../../ds1820.h"


/** DS1820 Dallas 1-Wire Temperature Probe
 *
 * Example:
 * @code
 *   #include "DS1820/platform/mbed/ds1820-mbed.h"
 *
 *   #define DS18B20_DATA_PIN PE_12
 *   #define DS18B20_SENSORS_NUM = 16;
 *   int main()
 *   {
 *       int i;
 *
 *       ds1820 dev;
 *       ds1820_mbed mbed_dev;
 *       mbed_dev.data_pin = new DigitalInOut(DS18B20_DATA_PIN);
 *       mbed_dev.parasite_pin = nullptr;
 *       dev.sensors_num = DS18B20_SENSORS_NUM;
 *       ds1820_mbed_init(&dev, &mbed_dev);
 *
 *       while (true)
 *       {
 *           ds1820_convert_temperature(ALL);
 *           float temp = ds1820_read_temperature(CELSIUS);
 *           debug("%3.1f\r\n",temp);
 *           wait_ms(2000);
 *       }
 *   }
 * @endcode
 */

/** DS1820 mbed specific dev object **/
typedef struct
{
    DigitalInOut *data_pin;
    DigitalOut   *parasite_pin;
} ds1820_mbed;


void ds1820_mbed_init(ds1820 *const dev, ds1820_mbed *const mbed_dev);


#endif // __DS1820_MBED_H__

