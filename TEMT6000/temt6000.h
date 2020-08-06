/**
 *  @brief:   TEMT6000 - TEMT6000 ambient light sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-08-06
 *  @license: MIT
 */


#ifndef __TEMT6000_H__
#define __TEMT6000_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


/**
 * @brief TEMT6000 dev object
 */
typedef struct
{
    float vdd_value;            // Sensor power supply voltage [V]
    uint16_t adc_max_value;     // ADC resolution
    uint16_t adc_samples_num;   // Number of samples
    void* platform_dev;         // platform device object
} temt6000;


/**
 * @brief Init function
 *
 * @param[in] dev - temt6000 device object
 */
void temt6000_init(temt6000* const dev);

/**
 * @brief Init function
 *
 * @param[in] dev - temt6000 device object
 * @return Illuminance value in lux [lx]
 */
float temt6000_get_light(temt6000* const dev);

//-----------------------------------------------------------------------------
// @brief HW DEPENDENT FUNCTIONS - must be implemented for each platform/
//-----------------------------------------------------------------------------

/**
 * @brief Read data via I2C from TEMT6000 registers
 *
 * @param[in] dev - temt6000 device object
 * @param[out] raw_adc_value - raw value from the sensor
 * @return return value = true on success, false on failure
 */
extern bool temt6000_read_raw(temt6000* const dev, uint16_t* raw_adc_value);


#ifdef __cplusplus
}
#endif


#endif /* __TEMT6000_H__*/

