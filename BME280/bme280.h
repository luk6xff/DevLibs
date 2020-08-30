/**
 *  @brief:   BME280 digital pressure, temperature and humidity sensor library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-08-18
 *  @license: MIT
 */


#ifndef __BME280_H__
#define __BME280_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// BME280 Default I2C address
#define BME280_DEFAULT_I2C_ADDRESS  (0x77)

// Sea level atmospheric pressure
#define SEA_LEVEL_PRESSURE_HPA      (1013.25f) //[hPa]


/**
 * @brief BME280 Communnication interface type SPI/I2C
 */
typedef enum
{
    BME280_INTF_SPI = 0,
    BME280_INTF_I2C = 1,
} bme280_intf_type;

/**
 * @brief BME280 Registers
 */
typedef enum
{
    BME280_REGISTER_DIG_T1 = 0x88,
    BME280_REGISTER_DIG_T2 = 0x8A,
    BME280_REGISTER_DIG_T3 = 0x8C,

    BME280_REGISTER_DIG_P1 = 0x8E,
    BME280_REGISTER_DIG_P2 = 0x90,
    BME280_REGISTER_DIG_P3 = 0x92,
    BME280_REGISTER_DIG_P4 = 0x94,
    BME280_REGISTER_DIG_P5 = 0x96,
    BME280_REGISTER_DIG_P6 = 0x98,
    BME280_REGISTER_DIG_P7 = 0x9A,
    BME280_REGISTER_DIG_P8 = 0x9C,
    BME280_REGISTER_DIG_P9 = 0x9E,

    BME280_REGISTER_DIG_H1 = 0xA1,
    BME280_REGISTER_DIG_H2 = 0xE1,
    BME280_REGISTER_DIG_H3 = 0xE3,
    BME280_REGISTER_DIG_H4 = 0xE4,
    BME280_REGISTER_DIG_H5 = 0xE5,
    BME280_REGISTER_DIG_H6 = 0xE7,

    BME280_REGISTER_CHIPID = 0xD0,
    BME280_REGISTER_VERSION = 0xD1,
    BME280_REGISTER_SOFTRESET = 0xE0,

    BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

    BME280_REGISTER_CTRL_HUM = 0xF2,
    BME280_REGISTER_STATUS = 0XF3,
	BME280_REGISTER_CTRL_MEAS = 0xF4,
    BME280_REGISTER_CONFIG = 0xF5,
    BME280_REGISTER_PRESSUREDATA = 0xF7,
    BME280_REGISTER_TEMPDATA = 0xFA,
    BME280_REGISTER_HUMIDDATA = 0xFD
} bme280_registers;


/**
 * @brief BME280 calibration data
 */
typedef struct
{
    uint16_t dig_T1; ///< temperature compensation value
    int16_t dig_T2;  ///< temperature compensation value
    int16_t dig_T3;  ///< temperature compensation value

    uint16_t dig_P1; ///< pressure compensation value
    int16_t dig_P2;  ///< pressure compensation value
    int16_t dig_P3;  ///< pressure compensation value
    int16_t dig_P4;  ///< pressure compensation value
    int16_t dig_P5;  ///< pressure compensation value
    int16_t dig_P6;  ///< pressure compensation value
    int16_t dig_P7;  ///< pressure compensation value
    int16_t dig_P8;  ///< pressure compensation value
    int16_t dig_P9;  ///< pressure compensation value

    uint8_t dig_H1; ///< humidity compensation value
    int16_t dig_H2; ///< humidity compensation value
    uint8_t dig_H3; ///< humidity compensation value
    int16_t dig_H4; ///< humidity compensation value
    int16_t dig_H5; ///< humidity compensation value
    int8_t dig_H6;  ///< humidity compensation value
} bme280_calibration;


/**
 * @brief sampling rates
 */
typedef enum
{
    SAMPLING_NONE = 0b000,
    SAMPLING_X1   = 0b001,
    SAMPLING_X2   = 0b010,
    SAMPLING_X4   = 0b011,
    SAMPLING_X8   = 0b100,
    SAMPLING_X16  = 0b101
} bme280_sensor_sampling;

/**
 * @brief power modes
 */
typedef enum
{
    MODE_SLEEP  = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
} bme280_sensor_mode;

/**
 * @brief filter values
 */
typedef enum
{
    FILTER_OFF = 0b000,
    FILTER_X2  = 0b001,
    FILTER_X4  = 0b010,
    FILTER_X8  = 0b011,
    FILTER_X16 = 0b100
} bme280_sensor_filter;

/**
 * @brief standby duration in ms
 */
typedef enum
{
    STANDBY_MS_0_5  = 0b000,
    STANDBY_MS_10   = 0b110,
    STANDBY_MS_20   = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125  = 0b010,
    STANDBY_MS_250  = 0b011,
    STANDBY_MS_500  = 0b100,
    STANDBY_MS_1000 = 0b101
} bme280_standby_duration;


/**
 * @brief BME280_REGISTER_CONFIG [0xF5] structure.
 */
typedef union
{
    struct
    {
        uint8_t spi3w_en : 1; ///< unused - don't set
        uint8_t none : 1;     ///< unused - don't set
        bme280_sensor_filter filter : 3; ///< filter settings
        bme280_standby_duration t_sb : 3; ///< inactive duration (standby time) in normal mode
    };
    uint8_t reg;

} config_reg;

/**
 * @brief BME280_REGISTER_CTRL_MEAS [0xF4] structure.
 */
typedef union
{
    struct
    {
        bme280_sensor_mode mode : 2; ///< device mode
        bme280_sensor_sampling osrs_p : 3; ///< pressure oversampling
        bme280_sensor_sampling osrs_t : 3; ///< temperature oversampling
    };
    uint8_t reg;

} ctrl_meas_reg;

/**
 * @brief BME280_REGISTER_CTRL_HUM [oxF2] structure.
 */
typedef union
{
    struct
    {
        bme280_sensor_sampling osrs_h : 3; ///< pressure oversampling
        uint8_t none : 5;                  ///< unused - don't set
    };
    uint8_t reg;

} ctrl_hum_reg;


/**
 * @brief BME280 dev object
 */
typedef struct
{
    bme280_intf_type intf; ///< Interface type
    uint8_t i2c_addr;      ///< I2C device address
    bme280_calibration calib;
    float t_fine;
    int32_t t_fine_adjust; ///< Add to compensate temp readings and in turn to pressure and humidity readings
    void* platform_dev;
} bme280;

/**
 * @brief Init function
 *
 * @param[in] dev - bme280 device object
 */
bool bme280_init(bme280 *const dev);

/**
 * @brief Set config params and read BME280 calibration parameters
 * @param[in] dev - bme280 device object
 * @param altitude (in meter)
 * @param oss
 * @return 0 if no errors, 1 on error
 */
bool bme280_set_configuration(bme280 *const dev);

/**
 * @brief Resets BME280.
 *
 * @details Performs a soft reset of the device. Same sequence as power on reset.
 * @param[in] dev - bme280 device object
 * @return 0 if no errors, 1 on error
 */
bool bme280_reset(bme280 *const dev);

/**
 * @brief Returns sensor ID.
 *
 * @param[in] dev - bme280 device object
 * @return Sensor ID 0x60 for BME280, 0x56, 0x57, 0x58 BMP280
 */
uint8_t bme280_sensor_id(bme280 *const dev);

/**
 * @brief Read pressure and temperature from the BME280.
 *
 * @param[in] dev - bme280 device object
 * @param temperature [C]
 * @param pressure [hPa]
 * @param humidity [%]
 * @return true on success, false on error
 */
bool bme280_read_data(bme280 *const dev, float *temperature, float *pressure,  float *humidity);

/**
 * @brief Returns the temperature from the sensor
 *
 * @param[in] dev - bme280 device object
 * @param[out] temperature - temperature  value in [C]
 * @return true on success, false on error
 */
bool bme280_read_temperature(bme280 *const dev, float *temperature);

/**
 * @brief Returns the pressure from the sensor
 *
 * @param[in] dev - bme280 device object
 * @param[out] pressure - pressure value in Pascal [Pa]
 * @return true on success, false on error
 */
bool bme280_read_pressure(bme280 *const dev, float *pressure);

/**
 * @brief  Returns the humidity from the sensor
 *
 * @param[out] pressure - pressure value in [%]
 * @return true on success, false on error
 */
bool bme280_read_humidity(bme280 *const dev, float *humidity);

/**
 * @brief Calculates the altitude (in meters) from the specified atmospheric
 *        pressure (in hPa), and sea-level pressure (in hPa).
 * @note  Equation taken from BMP180 datasheet (page 16).
 * @param[in]  sea_level - Sea-level pressure in hPa
 * @param[out] altitude  - The altitude value read from the device
 * @return true on success, false on error
 */
bool bme280_read_altitude(bme280 *const dev, const float sea_level_pressure, float *altitude);

/**
 * @brief Calculates the pressure at sea level (in hPa) from the specified
 *        altitude (in meters), and atmospheric pressure (in hPa).
 * @note  Equation taken from BMP180 datasheet (page 17):
 * @param  altitude      Altitude in meters
 * @param  atmospheric   Atmospheric pressure in hPa
 * @return the pressure at sea level (in hPa) from the specified altitude
 */
float bme280_sea_level_for_altitude(float altitude, float atmospheric);

/**
 * @brief Returns the current temperature compensation value in degrees Celcius
 * @return the current temperature compensation value in degrees Celcius
 */
float bme280_get_temperature_compensation(bme280 *const dev);

/**
 * @brief Sets a value to be added to each temperature reading. This adjusted
 *        temperature is used in pressure and humidity readings.
 * @param adjustment  Value to be added to each tempature reading in Celcius
 */
void bme280_set_temperature_compensation(bme280 *const dev, float adjustment);


//-----------------------------------------------------------------------------
// @brief HW DEPENDENT FUNCTIONS - must be implemented for each platform/
//-----------------------------------------------------------------------------
/**
 * @brief Write data via I2C to BME280 registers
 *
 * @param[in] dev - bme280 device object
 * @param[in] buf - pointer to buffer which will be send to device
 * @param[in] buf_size - number of bytes to be written
 * @return return value = true on success, false on failure
 */
extern bool bme280_write(bme280 *const dev, const uint8_t* buf, const size_t buf_size);

/**
 * @brief Read data via I2C from BME280 registers
 *
 * @param[in] dev - bme280 device object
 * @param[out] buf - pointer to buffer which will be filled with read data from the device
 * @param[in] buf_size - number of bytes to be read
 * @return return value = true on success, false on failure
 */
extern bool bme280_read(bme280 *const dev, uint8_t* buf, const size_t buf_size);

/**
 * @brief Miliseconds delay function.
 *
 * @param delay_ms: Delay in miliseconds.
 */
extern void bme280_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif


#endif /* __BME280_H__*/

