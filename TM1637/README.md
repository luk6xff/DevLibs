# TM1637 RTC module library

## Description
TM1637 LED controller platform independent library

## Usage
Simple demo code how to use the library on [MBED] platform
```cpp

#include "TM1637/platform/tm1637-mbed.h"

// PINOUT FOR MBED BOARD_DISCO_L476VG
// TM1637
#define LED_DIO     PA_0
#define LED_CLK     PA_5
int main(void)
{

    tm1637_mbed disp_mbed =
    {
        DigitalInOut(LED_DIO),
        DigitalOut(LED_CLK)
    };
    tm1637 disp;
    const char str[]  = {"HI.L"};
    const uint8_t raw[]  = {0x31, 0x32, 0x33, 0x34};
    tm1637_mbed_init(&disp, &disp_mbed);
    tm1637_clear(&disp);
    tm1637_print(&disp, (const uint8_t*)str, sizeof(str));
    wait_us(1000*2000);
    tm1637_print(&disp, raw, sizeof(raw));
    wait_us(1000*2000);
    tm1637_print(&disp, (const uint8_t*)str, sizeof(str));
    tm1637_set_brightness(&disp, TM1637_BRT0);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT1);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT2);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT3);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT4);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT5);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT6);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT7);
    wait_us(1000*500);
    tm1637_set_brightness(&disp, TM1637_BRT3);
    tm1637_clear(&disp);
    tm1637_print(&disp, (const uint8_t*)str, sizeof(str));
    wait_us(1000*1000);
    tm1637_clear(&disp);
    return 0;
}
```

## Porting to different platforms
If you want to port this library on other platform, the only thing you have to do is to implement HW/Platform dependent functions from `tm1637.h` file as it is done for example for [MBED] platform in `platform/tm1637-mbed.cpp` file.


## Author
* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
