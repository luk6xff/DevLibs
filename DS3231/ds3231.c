#include "ds3231.h"

//------------------------------------------------------------------------------
// @brief PRIVATE FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
 * @brief Convert unsigned char to BCD
 *
 * @param[in] data - 0-255
 *
 * @return bcd_result = BCD representation of data
 */
static uint16_t uchar_2_bcd(uint8_t data);


//------------------------------------------------------------------------------
/**
 * @brief Convert BCD to a uint8_t
 *
 * @param[in] bcd - 0-99
 *
 * @return rtn_val = integer rep. of BCD
 */
static uint8_t bcd_2_uchar(uint8_t bcd);


//------------------------------------------------------------------------------
// @brief FUNCTIONS DEFINITIONS
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void ds3231_init(ds3231* const dev)
{
    dev->w_addr = ((dev->i2c_addr << 1) | I2C_WRITE);
    dev->r_addr = ((dev->i2c_addr << 1) | I2C_READ);
}

//------------------------------------------------------------------------------
uint16_t ds3231_set_time(ds3231* const dev, ds3231_time_t time)
{
    uint8_t data[] = {0,0,0,0};
    uint8_t data_length = 0;
    uint8_t max_hour = 24;

    data[data_length++] = SECONDS;
    data[data_length++] = uchar_2_bcd(time.seconds);
    data[data_length++] = uchar_2_bcd(time.minutes);

    //format Hours register
    data[data_length] = uchar_2_bcd(time.hours);
    if (time.mode)
    {
        max_hour = max_hour/2;

        data[data_length] |= MODE;
        if (time.am_pm)
        {
            data[data_length] |= AM_PM;
        }

    }
    else
    {
        max_hour = max_hour - 1;
    }
    data_length++;

    //Make sure data is within range.
    if ((time.seconds > 59) || (time.minutes > 59) || (time.hours > max_hour))
    {
        return (1);
    }
    else
    {
        return (ds3231_write(dev, data, data_length));
    }
}

//------------------------------------------------------------------------------
uint16_t ds3231_set_calendar(ds3231* const dev, ds3231_calendar_t calendar)
{
    uint8_t data[] = {0,0,0,0,0};
    uint8_t data_length = 0;

    calendar.year -= 2000; // fix year
    data[data_length++] = DAY;
    data[data_length++] = uchar_2_bcd(calendar.day);
    data[data_length++] = uchar_2_bcd(calendar.date);
    data[data_length++] = uchar_2_bcd(calendar.month);
    data[data_length++] = uchar_2_bcd(calendar.year);

    //Make sure data is within range.
    if (((calendar.day < 1) || (calendar.day > 7)) ||
       ((calendar.date < 1) || (calendar.date > 31)) ||
       ((calendar.month < 1) || (calendar.month > 12)) ||
       (calendar.year > 99))
    {
        return(1);
    }
    else
    {
        return(ds3231_write(dev, data, data_length));
    }
}

//------------------------------------------------------------------------------
uint16_t ds3231_set_alarm(ds3231* const dev, ds3231_alrm_t alarm, bool one_r_two)
{
    uint8_t data[] = {0,0,0,0,0};
    uint8_t data_length = 0;
    uint8_t max_hour = 24;
    uint8_t mask_var = 0;

    //setting alarm 1 or 2?
    if (one_r_two)
    {
        data[data_length++] = ALRM1_SECONDS;

        //config seconds register
        if (alarm.am1)
        {
           mask_var |= ALRM_MASK;
        }
        data[data_length++] =  (mask_var | uchar_2_bcd(alarm.seconds));
        mask_var = 0;

        //config minutes register
        if (alarm.am2)
        {
           mask_var |= ALRM_MASK;
        }
        data[data_length++] =  (mask_var | uchar_2_bcd(alarm.minutes));
        mask_var = 0;

        //config hours register
        if (alarm.am3)
        {
           mask_var |= ALRM_MASK;
        }
        if (alarm.mode)
        {
            max_hour = max_hour/2;
            mask_var |= MODE;
            if(alarm.am_pm)
            {
                mask_var |= AM_PM;
            }
        }
        else
        {
            max_hour = max_hour - 1;
        }
        data[data_length++] =  (mask_var | uchar_2_bcd(alarm.hours));
        mask_var = 0;

        //config day/date register
        if (alarm.am4)
        {
           mask_var |= ALRM_MASK;
        }
        if (alarm.dy_dt)
        {
            mask_var |= DY_DT;
            data[data_length++] =  (mask_var | uchar_2_bcd(alarm.day));
        }
        else
        {
            data[data_length++] =  (mask_var | uchar_2_bcd(alarm.date));
        }
        mask_var = 0;
    }
    else
    {
        data[data_length++] = ALRM2_MINUTES;

        //config minutes register
        if (alarm.am2)
        {
           mask_var |= ALRM_MASK;
        }
        data[data_length++] =  (mask_var | uchar_2_bcd(alarm.minutes));
        mask_var = 0;

        //config hours register
        if (alarm.am3)
        {
           mask_var |= ALRM_MASK;
        }
        if (alarm.mode)
        {
            max_hour = max_hour/2;
            mask_var |= MODE;
            if(alarm.am_pm)
            {
                mask_var |= AM_PM;
            }
        }
        else
        {
            max_hour = max_hour - 1;
        }
        data[data_length++] =  (mask_var | uchar_2_bcd(alarm.hours));
        mask_var = 0;

        //config day/date register
        if (alarm.am4)
        {
           mask_var |= ALRM_MASK;
        }
        if (alarm.dy_dt)
        {
            mask_var |= DY_DT;
            data[data_length++] =  (mask_var | uchar_2_bcd(alarm.day));
        }
        else
        {
            data[data_length++] =  (mask_var | uchar_2_bcd(alarm.date));
        }
        mask_var = 0;
    }

    //Make sure data is within range.
    if ((alarm.seconds > 59) || (alarm.minutes > 59) || (alarm.hours > max_hour) ||
       ((alarm.day < 1) || (alarm.day > 7)) ||
       ((alarm.date < 1) || (alarm.date > 31)))
    {
        return(1);
    }
    else
    {
        return(ds3231_write(dev, data, data_length));
    }
}

//------------------------------------------------------------------------------
uint16_t ds3231_set_cntl_stat_reg(ds3231* const dev, ds3231_cntl_stat_t data)
{
    uint8_t local_data[] = {0,0,0};
    uint8_t data_length = 0;

    local_data[data_length++] = CONTROL;
    local_data[data_length++] = data.control;
    local_data[data_length++] = data.status;

    //users responsibility to make sure data is logical
    return(ds3231_write(dev, local_data, data_length));
}

//------------------------------------------------------------------------------
uint16_t ds3231_get_time(ds3231* const dev, ds3231_time_t* time)
{
    uint16_t rtn_val = 1;
    uint8_t data[3];

    data[0] = SECONDS;
    rtn_val = ds3231_write(dev, data, 1);

    if (!rtn_val)
    {
        rtn_val = ds3231_read(dev, data, 3);

        time->seconds = bcd_2_uchar(data[0]);
        time->minutes = bcd_2_uchar(data[1]);
        time->am_pm = (data[2]&AM_PM);
        time->mode = (data[2]&MODE);

        if(time->mode)
        {
            time->hours = bcd_2_uchar((data[2]&0x1F));
        }
        else
        {
            time->hours = bcd_2_uchar((data[2]&0x3F));
        }
    }

    return(rtn_val);
}

//------------------------------------------------------------------------------
uint16_t ds3231_get_calendar(ds3231* const dev, ds3231_calendar_t* calendar)
{
    uint16_t rtn_val = 1;
    uint8_t data[4];

    data[0] = DAY;
    rtn_val = ds3231_write(dev, data, 1);

    if (!rtn_val)
    {
        rtn_val = ds3231_read(dev, data, 4);

        calendar->day = bcd_2_uchar(data[0]);
        calendar->date = bcd_2_uchar(data[1]);
        calendar->month = bcd_2_uchar((data[2]&0x1F));
        calendar->year = bcd_2_uchar(data[3]);
        // fix year
        calendar->year += 2000;
        // determine day of the week (Zeller algorithm)
        const uint8_t arr[12] = {6, 2, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
        int day_of_week = (calendar->year % 100);
        day_of_week = day_of_week * 1.25;
        day_of_week += calendar->date;
        day_of_week += arr[calendar->month-1];
        if (((calendar->year % 4)==0) && (calendar->month < 3))
        {
            day_of_week -= 1;
        }
        while (day_of_week > 7)
        {
            day_of_week -= 7;
        }
        calendar->day = day_of_week;
    }

    return(rtn_val);
}

//------------------------------------------------------------------------------
uint16_t ds3231_get_alarm(ds3231* const dev, ds3231_alrm_t* alarm, bool one_r_two)
{
    uint16_t rtn_val = 1;
    uint8_t data[4];

    if (one_r_two)
    {
        data[0] = ALRM1_SECONDS;
        rtn_val = ds3231_write(dev, data, 1);

        if (!rtn_val)
        {
            rtn_val = ds3231_read(dev, data, 4);

            alarm->seconds = bcd_2_uchar(data[0]&0x7F);
            alarm->am1 = (data[0]&ALRM_MASK);
            alarm->minutes = bcd_2_uchar(data[1]&0x7F);
            alarm->am2 = (data[1]&ALRM_MASK);
            alarm->am3 = (data[2]&ALRM_MASK);
            alarm->am_pm = (data[2]&AM_PM);
            alarm->mode = (data[2]&MODE);

            if (alarm->mode)
            {
                alarm->hours = bcd_2_uchar((data[2]&0x1F));
            }
            else
            {
                alarm->hours = bcd_2_uchar((data[2]&0x3F));
            }

            if (data[3] & DY_DT)
            {
                alarm->dy_dt = 1;
                alarm->day = bcd_2_uchar(data[3]&0x0F);
            }
            else
            {
                alarm->date = bcd_2_uchar(data[3]&0x3F);
            }
            alarm->am4 = (data[3]&ALRM_MASK);
        }
    }
    else
    {
        data[0] = ALRM2_MINUTES;
        rtn_val = ds3231_write(dev, data, 1);

        if (!rtn_val)
        {
            rtn_val = ds3231_read(dev, data, 4);

            alarm->minutes = bcd_2_uchar(data[0]&0x7F);
            alarm->am2 = (data[0]&ALRM_MASK);
            alarm->am3 = (data[1]&ALRM_MASK);
            alarm->am_pm = (data[1]&AM_PM);
            alarm->mode = (data[1]&MODE);

            if (alarm->mode)
            {
                alarm->hours = bcd_2_uchar((data[2]&0x1F));
            }
            else
            {
                alarm->hours = bcd_2_uchar((data[2]&0x3F));
            }

            if (data[2] & DY_DT)
            {
                alarm->dy_dt = 1;
                alarm->day = bcd_2_uchar(data[2]&0x0F);
            }
            else
            {
                alarm->date = bcd_2_uchar(data[2]&0x3F);
            }
            alarm->am4 = (data[2]&ALRM_MASK);
        }
    }

    return(rtn_val);
}

//------------------------------------------------------------------------------
uint16_t ds3231_get_cntl_stat_reg(ds3231* const dev, ds3231_cntl_stat_t* data)
{
    uint16_t rtn_val = 1;
    uint8_t local_data[2];

    local_data[0] = CONTROL;
    rtn_val = ds3231_write(dev, local_data, 1);

    if (!rtn_val)
    {
        rtn_val = ds3231_read(dev, local_data, 2);

        data->control = local_data[0];
        data->status = local_data[1];
    }

    return(rtn_val);
}

//------------------------------------------------------------------------------
uint16_t ds3231_get_temperature(ds3231* const dev)
{
    uint16_t rtn_val = 1;
    uint8_t data[2];

    data[0] = MSB_TEMP;
    rtn_val = ds3231_write(dev, data, 1);

    if (!rtn_val)
    {
        ds3231_read(dev, data, 2);

        rtn_val = data[0] << 8;
        rtn_val |= data[1];
    }

    return (rtn_val);
}

//------------------------------------------------------------------------------
time_t ds3231_get_epoch(ds3231* const dev)
{
    //system vars
    struct tm sys_time;

    //RTC vars
    ds3231_time_t rtc_time = {0,0,0,0,0};
    ds3231_calendar_t rtc_calendar = {0,0,0,0};

    ds3231_get_calendar(dev, &rtc_calendar);
    ds3231_get_time(dev, &rtc_time);

    sys_time.tm_wday = rtc_calendar.day - 1;
    sys_time.tm_mday = rtc_calendar.date;
    sys_time.tm_mon = rtc_calendar.month - 1;
    sys_time.tm_year = rtc_calendar.year + 100;

    //check for 12hr or 24hr mode
    if (rtc_time.mode)
    {
        //check am/pm
        if(rtc_time.am_pm  && (rtc_time.hours != 12))
        {
            sys_time.tm_hour = rtc_time.hours + 12;
        }
        else
        {
            sys_time.tm_hour = rtc_time.hours;
        }

    }
    else
    {
        //24hr mode
        sys_time.tm_hour = rtc_time.hours;
    }

    sys_time.tm_min = rtc_time.minutes;
    sys_time.tm_sec = rtc_time.seconds;

    //make epoch time
    return (mktime(&sys_time));
}

//------------------------------------------------------------------------------
static uint16_t uchar_2_bcd(uint8_t data)
{
   uint16_t bcd_result = 0;

   //Get hundreds
   bcd_result |= ((data/100) << 8);
   data = (data - (data/100)*100);

   //Get tens
   bcd_result |= ((data/10) << 4);
   data = (data - (data/10)*10);

   //Get ones
   bcd_result |= data;

   return(bcd_result);
}

//------------------------------------------------------------------------------
static uint8_t bcd_2_uchar(uint8_t bcd)
{
    uint8_t rtn_val = 0;

    rtn_val += ((bcd&0xf0)>>4)*10;
    rtn_val += (bcd&0x000f);

    return rtn_val;
}

//------------------------------------------------------------------------------
