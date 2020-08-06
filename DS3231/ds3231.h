/**
 *  @brief:   DS3231 - RTC module library, inspired by Justin Jordan code.
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2019-12-06
 *  @license: MIT
 */


#ifndef __DS3231_H__
#define __DS3231_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>


#define I2C_WRITE 0
#define I2C_READ  1

#define AM_PM     (1 << 5)
#define MODE      (1 << 6)
#define DY_DT     (1 << 6)
#define ALRM_MASK (1 << 7)

// Control register bit masks
#define A1IE  (1 << 0)
#define A2IE  (1 << 1)
#define INTCN (1 << 2)
#define RS1   (1 << 3)
#define RS2   (1 << 4)
#define CONV  (1 << 5)
#define BBSQW (1 << 6)
#define EOSC  (1 << 7)

// Status register bit masks
#define A1F     (1 << 0)
#define A2F     (1 << 1)
#define BSY     (1 << 2)
#define EN32KHZ (1 << 3)
#define OSF     (1 << 7)

#define SEC_1970_TO_2000 946684800

static const uint8_t days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };


/**
 * @brief ds3231_day_of_week_t - Enum describing day of the week.
 */
typedef enum
{
    MONDAY    = 1,
    TUESDAY	  = 2,
    WEDNESDAY = 3,
    THURSDAY  = 4,
    FRIDAY	  = 5,
    SATURDAY  = 6,
    SUNDAY	  = 7
} ds3231_day_of_week_t;


/**
 * @brief ds3231_time_t - Struct for containing time data.
 *
 * Members:
 *
 * - uint32_t seconds - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t minutes - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t hours   - Use decimal value. Member fx's convert to BCD
 *
 * - bool am_pm      - TRUE for PM, same logic as datasheet
 *
 * - bool mode       - TRUE for 12 hour, same logic as datasheet
 */
typedef struct
{
    uint32_t seconds;
    uint32_t minutes;
    uint32_t hours;
    bool am_pm;
    bool mode;
} ds3231_time_t;


/**
 * @brief ds3231_calendar_t - Struct for containing calendar data.
 *
 * Members:
 *
 * - uint32_t day   - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t date  - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t month - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t year  - Use decimal value. Member fx's convert to BCD
 */
typedef struct
{
    uint32_t day;
    uint32_t date;
    uint32_t month;
    uint32_t year;
} ds3231_calendar_t;


/**
 * @brief ds3231_alrm_t - Struct for containing alarm data.
 *
 * Members:
 *
 * - uint32_t seconds - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t minutes - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t hours   - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t day     - Use decimal value. Member fx's convert to BCD
 *
 * - uint32_t date    - Use decimal value. Member fx's convert to BCD
 *
 * - bool am1        - Flag for setting alarm rate
 *
 * - bool am2        - Flag for setting alarm rate
 *
 * - bool am3        - Flag for setting alarm rate
 *
 * - bool am4        - Flag for setting alarm rate
 *
 * - bool am_pm      - TRUE for PM, same logic as datasheet
 *
 * - bool mode       - TRUE for 12 hour, same logic as datasheet
 *
 * - bool dy_dt      - TRUE for Day, same logic as datasheet
 */
typedef struct
{
    //Seconds and am1 not used for alarm2
    uint32_t seconds;
    uint32_t minutes;
    uint32_t hours;
    uint32_t day;
    uint32_t date;
    bool am1;
    bool am2;
    bool am3;
    bool am4;
    bool am_pm;
    bool mode;
    bool dy_dt;
} ds3231_alrm_t;


/**
 * @brief ds3231_cntl_stat_t - Struct for containing control and status
 * register data.
 *
 * Members:
 *
 * - uint8_t control - Binary data for read/write of control register
 *
 * - uint8_t status  - Binary data  for read/write of status register
 */
typedef struct
{
    uint8_t control;
    uint8_t status;
} ds3231_cntl_stat_t;


/**
 * @brief ds3231_regs_t - enumerated DS3231 registers
 */
typedef enum
{
    SECONDS,
    MINUTES,
    HOURS,
    DAY,
    DATE,
    MONTH,
    YEAR,
    ALRM1_SECONDS,
    ALRM1_MINUTES,
    ALRM1_HOURS,
    ALRM1_DAY_DATE,
    ALRM2_MINUTES,
    ALRM2_HOURS,
    ALRM2_DAY_DATE,
    CONTROL,
    STATUSS,
    AGING_OFFSET, //don't touch this register
    MSB_TEMP,
    LSB_TEMP
} ds3231_regs_t;


/**
 * @brief DS3231 dev object
 */
typedef struct
{
    uint8_t i2c_addr; // i2c device address
    uint8_t w_addr;   // write address
    uint8_t r_addr;   // read address
    void* platform_dev;
} ds3231;



/**
 * @brief Init function
 *
 * @param[in] I2C bus module addr;
 *
 */
void ds3231_init(ds3231* const dev);


/**
 * @brief Sets the time on DS3231
 *        Struct data is in integrer format, not BCD.  Fx will convert
 *        to BCD for you.
 *
 * @param[in] time - struct cotaining time data;
 *
 * @return return value = 0 on success, non-0 on failure
 */
uint16_t ds3231_set_time(ds3231* const dev, ds3231_time_t time);


/**
 * @brief Sets the calendar on DS3231
 *
 * @param[in] calendar - struct cotaining calendar data
 *
 * @return return value = 0 on success, non-0 on failure
 */
uint16_t ds3231_set_calendar(ds3231* const dev, ds3231_calendar_t calendar);


/**
 * @brief Set either Alarm1 or Alarm2 of DS3231
 *
 * @param[in] alarm - struct cotaining alarm data
 * @param[in] one_r_two - TRUE for Alarm1 and FALSE for
 *                        Alarm2
 *
 * @return return value = 0 on success, non-0 on failure
 */
uint16_t ds3231_set_alarm(ds3231* const dev, ds3231_alrm_t alarm, bool one_r_two);


/**
 * @brief Set control and status registers of DS3231
 *
 * @param[in] data - Struct containing control and status
 *                   register data
 * @return return value = 0 on success, non-0 on failure
 *
*/
uint16_t ds3231_set_cntl_stat_reg(ds3231* const dev, ds3231_cntl_stat_t data);


/**
 * @brief Gets the time on DS3231
 *
 * @param[in] time - pointer to struct for storing time data
 * @param[out] time - contains current integrer rtc time
 *                    data
 * @return return value = 0 on success, non-0 on failure
 */
uint16_t ds3231_get_time(ds3231* const dev, ds3231_time_t* time);


/**
 * @brief Gets the calendar on DS3231
 *
 * @param[in] calendar - pointer to struct for storing
 *                       calendar data
 * @param[out] calendar - contains current integer rtc
 *                        calendar data
 * @return return value = 0 on success, non-0 on failure
 */
uint16_t ds3231_get_calendar(ds3231* const dev, ds3231_calendar_t* calendar);


/**
 * @brief Get either Alarm1 or Alarm2 of DS3231
 *
 * @param[in] alarm - pointer to struct for storing alarm
 *                    data;
 * @param[in] one_r_two - TRUE for Alarm1 and FALSE for
 *                        Alarm
 * @param[out] alarm - contains integer alarm data
 * @return return value = 0 on success, non-0 on failure
 */
uint16_t ds3231_get_alarm(ds3231* const dev, ds3231_alrm_t* alarm, bool one_r_two);


/**
 * @brief Get control and status registers of DS3231
 *
 * @param[in] data - pointer to struct for storing control
 *                    and status register data
 * @param[out] data - contains control and status registers
 *                    data
 * @return return value = 0 on success, non-0 on failure
 */
uint16_t ds3231_get_cntl_stat_reg(ds3231* const dev, ds3231_cntl_stat_t* data);


/**
 * @brief Get temperature data of DS3231
 *
 * @return return value = raw temperature data
 */
uint16_t ds3231_get_temperature(ds3231* const dev);


/**
 * @brief Get epoch time based on current RTC time and date.
 *        DS3231 must be configured and running before this fx is
 *        called
 *
 * @return return value = epoch time
 */
time_t ds3231_get_epoch(ds3231* const dev);


//-----------------------------------------------------------------------------
// @brief HW DEPENDENT FUNCTIONS - must be implemented for each platform/
//-----------------------------------------------------------------------------
/**
 * @brief Write data via I2C to DS3231 registers
 *
 * @param[in] buf - pointer to buffer which will be send to device
 * @param[in] buf_size - number of bytes to be written
 * @return return value = 0 on success, non-0 on failure
 */
extern bool ds3231_write(ds3231* const dev, const uint8_t* buf, const size_t buf_size);

/**
 * @brief Read data via I2C from DS3231 registers
 *
 * @param[out] buf - pointer to buffer which will be filled with read data from the device
 * @param[in] buf_size - number of bytes to be read
 * @return return value = 0 on success, non-0 on failure
 */
extern bool ds3231_read(ds3231* const dev, uint8_t* buf, const size_t buf_size);


#ifdef __cplusplus
}
#endif


#endif /* __DS3231_H__*/

