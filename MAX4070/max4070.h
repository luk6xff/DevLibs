/**
 *  @brief:   MAX4070 bidirectional high-side current-sense amplifier library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __MAX4070_H__
#define __MAX4070_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Nominal voltage gain of the MAX4070 [V/V], check the part variant you use */
#define MAX4070_DEFAULT_GAIN        50.0f

/** Values reported by the getters when no valid reading exists */
#define MAX4070_INVALID_VOLTAGE     (-999.0f)
#define MAX4070_INVALID_CURRENT     (-999.0f)

/**
 * @brief max4070 dev object
 *
 * The amplifier output follows OUT = ref_voltage + gain * shunt_resistance * I,
 * so a current flowing in the opposite direction pulls OUT below ref_voltage.
 * For unidirectional sensing simply leave ref_voltage at 0.0f.
 */
typedef struct
{
    float adc_vref;           // ADC full scale reference voltage [V]
    uint16_t adc_max_value;   // ADC full scale count, e.g. 4095 for a 12-bit ADC
    uint16_t adc_samples_num; // Number of samples averaged per reading, min 1
    float ref_voltage;        // OUT voltage for a zero sensed current [V]
    float gain;               // Amplifier voltage gain [V/V]
    float shunt_resistance;   // Sense resistor value [Ohm]
    void* platform_dev;       // Platform descriptor, set by max4070_<platform>_init()
    float out_voltage;        // Last measured amplifier output voltage [V]
    float current;            // Last measured sensed current [A]
} max4070;

/**
 * @brief Validate the device configuration and reset the cached readings.
 *
 * All of adc_vref, adc_max_value, adc_samples_num, gain and shunt_resistance
 * have to be filled in by the caller before this function runs.
 *
 * @param dev max4070 device object
 * @return True on success, false otherwise
 */
bool max4070_init(max4070* const dev);

/**
 * @brief Measure the amplifier output voltage.
 *
 * adc_samples_num samples are taken and averaged.
 *
 * @param      dev     max4070 device object
 * @param[out] voltage Amplifier output voltage in volts [V], may be NULL
 * @return True on success, false otherwise
 */
bool max4070_read_output_voltage(max4070* const dev, float* voltage);

/**
 * @brief Measure the current flowing through the shunt resistor.
 *
 * @param      dev     max4070 device object
 * @param[out] current Sensed current in amperes [A], may be NULL. The sign
 *                     follows the direction of the current flow
 * @return True on success, false otherwise
 */
bool max4070_read_current(max4070* const dev, float* current);

/**
 * @brief Return the output voltage cached by the last successful read.
 *
 * @param dev max4070 device object
 * @return Amplifier output voltage in volts [V] or MAX4070_INVALID_VOLTAGE
 */
float max4070_get_output_voltage(const max4070* const dev);

/**
 * @brief Return the current cached by the last successful read.
 *
 * @param dev max4070 device object
 * @return Sensed current in amperes [A] or MAX4070_INVALID_CURRENT
 */
float max4070_get_current(const max4070* const dev);

/**
 * @brief Scale a voltage measured behind a resistive divider back to its source.
 *
 * Useful when the same ADC also monitors the battery rail through a divider:
 * Vsource -> r_top -> ADC input -> r_bottom -> GND
 *
 * @param measured_voltage Voltage present on the ADC input [V]
 * @param r_top_ohms       Upper divider resistor [Ohm]
 * @param r_bottom_ohms    Lower divider resistor [Ohm]
 * @return The source voltage in volts [V] or MAX4070_INVALID_VOLTAGE
 */
float max4070_scale_divider_voltage(float measured_voltage, float r_top_ohms,
                                    float r_bottom_ohms);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Read one raw sample from the ADC channel wired to the OUT pin
 *
 * @param      dev           max4070 device object
 * @param[out] raw_adc_value Raw ADC conversion result
 * @return True on success, false otherwise
 */
extern bool max4070_read_adc(const max4070* const dev, uint16_t* raw_adc_value);


#ifdef __cplusplus
}
#endif

#endif /* __MAX4070_H__ */
