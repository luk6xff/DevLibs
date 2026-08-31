# SPFD5408 TFT LCD controller

## Description
Platform independent driver for the Sunplus SPFD5408 TFT LCD controller, the
one found on the common 2.4" 240x320 display modules.

The core (`spfd5408.c` / `spfd5408.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Resolution: 240 x 320 pixels, usable in portrait (240x320) or landscape
  (320x240) orientation
* Colour depth: 16 bits per pixel, RGB565
* Interface: 16-bit 8080 parallel bus plus the RS, WR, RD, CS and RESET control
  lines
* No framebuffer and no dynamic allocation, the pixels go straight into the
  GRAM of the controller
* Built-in 8x8 font, additional fonts in the UTFT layout can be loaded with
  `spfd5408_set_font()`

Every drawing primitive clips against the screen, coordinates outside the
current orientation are simply discarded and never reach the controller.

Documentation regarding the SPFD5408 can be found here:
http://www.rhydolabz.com/documents/SPFD5408A.pdf

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/spfd5408-arduino.{h,cpp}` |
| Mbed | `platform/mbed/spfd5408-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/spfd5408-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "SPFD5408/platform/arduino/spfd5408-arduino.h"

spfd5408 dev;
spfd5408_arduino arduino_dev = {
    // D0 .. D15
    {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37},
    38, // rs_pin
    39, // wr_pin
    40, // rd_pin
    41, // cs_pin
    42, // reset_pin
};

void setup()
{
    Serial.begin(115200);

    dev.orientation = SPFD5408_ORIENTATION_LANDSCAPE;
    dev.fg_color = SPFD5408_COLOR_WHITE;
    dev.bg_color = SPFD5408_COLOR_BLACK;

    if (!spfd5408_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("SPFD5408 init failed!");
        return;
    }

    spfd5408_fill_screen(&dev, SPFD5408_COLOR_BLACK);
    spfd5408_draw_rect(&dev, 0, 0, spfd5408_get_width(&dev),
                       spfd5408_get_height(&dev), SPFD5408_COLOR_BLUE);
    spfd5408_draw_string(&dev, "Hello SPFD5408", SPFD5408_X_CENTER, 10,
                         SPFD5408_COLOR_YELLOW);
    spfd5408_fill_round_rect(&dev, 20, 40, 120, 60, SPFD5408_COLOR_GRAY1);
    spfd5408_draw_circle(&dev, 240, 120, 50, SPFD5408_COLOR_GREEN);
    spfd5408_fill_circle(&dev, 240, 120, 20, SPFD5408_COLOR_RED);
    spfd5408_draw_line(&dev, 0, 0, 319, 239, SPFD5408_COLOR_CYAN);
}

void loop()
{
    static int32_t counter = 0;

    spfd5408_fill_rect(&dev, 20, 180, 200, 20, SPFD5408_COLOR_BLACK);
    spfd5408_draw_number(&dev, counter, 20, 180, 6, '0', SPFD5408_COLOR_WHITE);
    spfd5408_draw_float(&dev, (float)counter / 3.0f, 2, 120, 180, '.', 0, ' ',
                        SPFD5408_COLOR_WHITE);
    counter++;
    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `spfd5408.h`:

* `spfd5408_write_bus()`
* `spfd5408_set_pin()`
* `spfd5408_delay_ms()`

and provide a `spfd5408_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/spfd5408-arduino.cpp`
for a reference implementation.

The library generates the write strobe itself, so `spfd5408_write_bus()` only
has to update the sixteen data lines.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
