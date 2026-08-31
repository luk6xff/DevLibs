# AD9106 quad 12-bit 180 MSPS DAC and waveform generator

## Description
Platform independent driver for the Analog Devices AD9106 waveform generator.

The core (`ad9106.c` / `ad9106.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Quad, low power, 12-bit, 180 MSPS digital-to-analog converter
* Waveform generator: DDS sine/cosine, sawtooth/triangle, pseudorandom noise,
  constant value and arbitrary patterns played from the on-chip 4096 word SRAM
* Interface: 4-wire SPI, mode 0 (CPOL=0, CPHA=0), MSB first
* Registers: 16-bit wide, addressed by a 15-bit address with the R/W flag in the
  MSB of the first instruction byte
* Control lines: `CS` (active low), `RESET` (active low, optional) and
  `TRIGGER` (active low, optional)

Documentation regarding the AD9106 might be found here:
http://www.analog.com/en/digital-to-analog-converters/high-speed-da-converters/ad9106/products/product.html?src=ad9106.pdf

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/ad9106-arduino.{h,cpp}` |
| Mbed | `platform/mbed/ad9106-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/ad9106-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "AD9106/platform/arduino/ad9106-arduino.h"

ad9106 dev;
ad9106_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.spi = &SPI;
    arduino_dev.spi_freq = 1000000;
    arduino_dev.cs_pin = 10;
    arduino_dev.reset_pin = 9;   // negative if not connected
    arduino_dev.trigger_pin = 8; // negative if not connected
    dev.dac_clk_hz = 180000000;  // clock present on the DACCLK pins

    if (!ad9106_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("AD9106 init failed!");
        return;
    }

    // DAC4 outputs a 1 MHz DDS sine wave
    ad9106_set_wave_config(&dev, AD9106_CHANNEL_4, AD9106_WAVE_PRESTORED,
                           AD9106_PRESTORE_DDS);
    ad9106_set_dds_freq(&dev, 1000000);
    ad9106_set_dds_phase(&dev, AD9106_CHANNEL_4, 0);
    ad9106_set_digital_gain(&dev, AD9106_CHANNEL_4, 0x400);
    ad9106_set_rset(&dev, AD9106_CHANNEL_4, true, 2);
    ad9106_set_gain_range(&dev, AD9106_CHANNEL_4, 2);
    ad9106_set_pattern_timebase(&dev, 0, 1, 1);
    ad9106_set_pattern_period(&dev, 0x8000);
    ad9106_start_pattern(&dev);

    // Start the pattern generator by pulling the TRIGGER line low
    ad9106_set_trigger(&dev, true);
}

void loop()
{
    uint16_t errors = 0;
    if (ad9106_get_error_flags(&dev, &errors) && errors != 0)
    {
        Serial.print("AD9106 config error flags: 0x");
        Serial.println(errors, HEX);
        ad9106_clear_error_flags(&dev);
    }
    delay(1000);
}
```

Playing an arbitrary pattern out of the on-chip SRAM:

```cpp
static const uint16_t pattern[256] = { /* 12-bit samples in bits [15:4] */ };

ad9106_write_sram(&dev, 0, pattern, 256);
ad9106_set_wave_config(&dev, AD9106_CHANNEL_4, AD9106_WAVE_SRAM,
                       AD9106_PRESTORE_DDS);
ad9106_config_sram_playback(&dev, AD9106_CHANNEL_4, 0, 255, 0);
ad9106_set_pattern_repeat_mode(&dev, true);
ad9106_set_pattern_repeat_cycles(&dev, AD9106_CHANNEL_4, 2);
ad9106_start_pattern(&dev);
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `ad9106.h`:

* `ad9106_spi_transfer()`
* `ad9106_set_pin()`
* `ad9106_delay_ms()`

and provide a `ad9106_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/ad9106-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
