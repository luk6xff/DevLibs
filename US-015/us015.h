/**
 *  @brief:   US-015 (HC-SR04 compatible) ultrasonic distance sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __US015_H__
#define __US015_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Maximum distance the sensor is specified for, in centimeters */
#define US015_MAX_DISTANCE_CM       400u

/** Echo pulse width in microseconds per centimeter of distance */
#define US015_US_PER_CM             58.0f

/** Echo pulse width in microseconds per inch of distance */
#define US015_US_PER_INCH           148.0f

/** Width of the trigger pulse that starts a measurement, in microseconds */
#define US015_TRIGGER_PULSE_US      10u

/** Settling time applied before the trigger pulse, in microseconds */
#define US015_TRIGGER_SETTLE_US     4u

/**
 * @brief Default echo timeout in microseconds.
 *
 * US015_MAX_DISTANCE_CM * US015_US_PER_CM rounded up, so a missing echo is
 * detected shortly after the sensor range has been exceeded.
 */
#define US015_DEFAULT_TIMEOUT_US    24000u

/**
 * @brief us015 dev object
 */
typedef struct
{
    uint32_t timeout_us; // Echo timeout in us, 0 selects US015_DEFAULT_TIMEOUT_US
    void* platform_dev;  // Platform descriptor, assigned by us015_<platform>_init()
} us015;

/**
 * @brief Initialize the sensor and bring the trigger pin into its idle state.
 *
 * @param dev us015 device object
 * @return True on success, false otherwise
 */
bool us015_init(us015* const dev);

/**
 * @brief Trigger a measurement and return the raw width of the echo pulse.
 *
 * @param      dev             us015 device object
 * @param[out] pulse_width_us  Echo pulse width in microseconds
 * @return True on success, false on a timeout (no echo received)
 */
bool us015_read_pulse_width(const us015* const dev, uint32_t* pulse_width_us);

/**
 * @brief Trigger a measurement and return the distance in centimeters.
 *
 * @param      dev         us015 device object
 * @param[out] distance_cm Measured distance in centimeters
 * @return True on success, false on a timeout (no echo received)
 */
bool us015_read_distance_cm(const us015* const dev, float* distance_cm);

/**
 * @brief Trigger a measurement and return the distance in inches.
 *
 * @param      dev           us015 device object
 * @param[out] distance_inch Measured distance in inches
 * @return True on success, false on a timeout (no echo received)
 */
bool us015_read_distance_inch(const us015* const dev, float* distance_inch);

/**
 * @brief Convert an echo pulse width into centimeters.
 *
 * @param pulse_width_us Echo pulse width in microseconds
 * @return Distance in centimeters
 */
float us015_pulse_width_to_cm(uint32_t pulse_width_us);

/**
 * @brief Convert an echo pulse width into inches.
 *
 * @param pulse_width_us Echo pulse width in microseconds
 * @return Distance in inches
 */
float us015_pulse_width_to_inch(uint32_t pulse_width_us);

/**
 * @brief Convert a maximum distance into the matching echo timeout.
 *
 * @param distance_cm Maximum distance of interest in centimeters
 * @return Echo timeout in microseconds
 */
uint32_t us015_distance_cm_to_timeout_us(uint32_t distance_cm);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Drive the trigger pin.
 *
 * @param dev   us015 device object
 * @param state True drives the pin HIGH, false LOW
 */
extern void us015_set_trigger_pin(const us015* const dev, bool state);

/**
 * @brief Read the echo pin.
 *
 * @param dev us015 device object
 * @return True when the pin is HIGH, false when it is LOW
 */
extern bool us015_get_echo_pin(const us015* const dev);

/**
 * @brief Block for a given number of microseconds.
 *
 * @param delay_us Delay in microseconds
 */
extern void us015_delay_us(uint32_t delay_us);

/**
 * @brief Return the value of a free running microsecond counter.
 *
 * The counter is allowed to wrap around, the library only uses differences.
 *
 * @return Current number of microsecond ticks
 */
extern uint32_t us015_get_us_ticks(void);


#ifdef __cplusplus
}
#endif

#endif /* __US015_H__ */
