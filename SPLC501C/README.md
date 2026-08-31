# SPLC501C monochrome graphic LCD

## Description
Platform independent driver for the Sunplus SPLC501C monochrome graphic LCD
controller.

The core (`splc501c.c` / `splc501c.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Resolution: 128 x 64 pixels (the controller RAM is 132 columns wide, the
  visible window is skewed automatically when the panel is rotated)
* Colour depth: 1 bit per pixel, monochrome
* Interface: 4-wire serial (SPI mode 0, MSB first) plus the A0, RESET and CS
  control lines
* Framebuffer: 1024 bytes held inside the `splc501c` object, page organised
  exactly like the controller RAM, no dynamic allocation at all
* Built-in 5x7 font covering ASCII 32..127

All the drawing primitives only modify the framebuffer, call `splc501c_update()`
to push it to the panel. Every primitive clips against the panel, coordinates
outside 0..127 / 0..63 are simply discarded.

Documentation regarding the SPLC501C might be found here:
http://www.densitron.com/uploadedFiles/Displays/Support/sunplussplc501c.pdf

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/splc501c-arduino.{h,cpp}` |
| Mbed | `platform/mbed/splc501c-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/splc501c-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "SPLC501C/platform/arduino/splc501c-arduino.h"

splc501c dev;
splc501c_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.spi = &SPI;
    arduino_dev.cs_pin = 10;
    arduino_dev.a0_pin = 9;
    arduino_dev.reset_pin = 8;
    arduino_dev.spi_freq = 4000000;

    dev.orientation = SPLC501C_ORIENTATION_NORMAL;

    if (!splc501c_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("SPLC501C init failed!");
        return;
    }

    splc501c_clear(&dev, SPLC501C_COLOR_OFF);
    splc501c_draw_rect(&dev, 0, 0, SPLC501C_WIDTH, SPLC501C_HEIGHT,
                       SPLC501C_COLOR_ON);
    splc501c_draw_string(&dev, "Hello SPLC501C", 6, 4, SPLC501C_COLOR_ON);
    splc501c_draw_line(&dev, 4, 14, 123, 14, SPLC501C_COLOR_ON);
    splc501c_fill_circle(&dev, 32, 40, 15, SPLC501C_COLOR_ON);
    splc501c_draw_circle(&dev, 96, 40, 15, SPLC501C_COLOR_ON);
    splc501c_update(&dev);
}

void loop()
{
    static uint8_t contrast = SPLC501C_CONTRAST_DEFAULT;

    splc501c_fill_rect(&dev, 40, 32, 48, 16, SPLC501C_COLOR_OFF);
    splc501c_set_cursor(&dev, 44, 36);
    splc501c_set_text_color(&dev, SPLC501C_COLOR_ON);
    splc501c_print(&dev, "DevLibs");
    splc501c_update(&dev);

    contrast = (uint8_t)((contrast + 1) & SPLC501C_CONTRAST_MAX);
    splc501c_set_contrast(&dev, contrast);
    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `splc501c.h`:

* `splc501c_write_bytes()`
* `splc501c_set_pin()`
* `splc501c_delay_ms()`

and provide a `splc501c_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/splc501c-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
