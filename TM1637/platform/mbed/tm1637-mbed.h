
/**
 *  @brief:  Implementation of a TM1637 platform dependent [MBED] functions
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-12-02
 */


#ifndef __TM1637_MBED_H__
#define __TM1637_MBED_H__

#include "mbed.h"
#include "../../tm1637.h"


/** TM1637 7-segment led driver
 *
 * Example:
 * @code
 *   #include "mbed.h"
 *
 *   @endcode
 */

typedef struct
{
    DigitalInOut dio; // Serial bus DIO pin
    DigitalOut   clk; // Serial bus CLK pin
} tm1637_mbed;


/**
 * @brief Init function driving TM1637 LED controller.
 *
 * @param  dev       Device object pointer
 * @param  mbed_dev  Mbed device object pointer
 */
void tm1637_mbed_init(tm1637* const dev, tm1637_mbed* const mbed_dev);

void tm1637_mbed_deinit(tm1637* const dev);

#endif // __TM1637_MBED_H__


