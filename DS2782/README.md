# DS2782 stand-alone battery fuel gauge

## Description
Platform independent driver for the Maxim DS2782 stand-alone battery fuel gauge.

The core (`ds2782.c` / `ds2782.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C (up to 400 kHz)
* Address: `0x34` in the 7-bit form (`0x68` in the 8-bit form used by the
  datasheet), reprogrammable through the `I2C_SLAVE_ADDR` EEPROM cell
* Reports voltage [mV], instantaneous and average current [mA], accumulated
  current [mAh], temperature [degC], remaining relative [%] and absolute [mAh]
  capacity, the age scalar [%] and the status flags
* Current, accumulated current, voltage and temperature are two's complement
  signed values, the driver sign extends them correctly
* The current related readings scale with the sense resistor, set
  `dev.rsense_mohm` to the value used on your board (20 mOhm by default)
* Exposes the EEPROM shadow RAM together with the copy and recall function
  commands so the battery model can be programmed and made persistent

Documentation regarding the DS2782 might be found here:
http://www.maximintegrated.com/en/products/power/battery-management/DS2782.html

and some very useful tutorials:
* http://www.maximintegrated.com/en/products/power/battery-management/DS2782.html/tb_tab2
* http://www.maximintegrated.com/en/app-notes/index.mvp/id/3584
* http://www.maximintegrated.com/en/app-notes/index.mvp/id/3463

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/ds2782-arduino.{h,cpp}` |
| Mbed | `platform/mbed/ds2782-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/ds2782-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "DS2782/platform/arduino/ds2782-arduino.h"

ds2782 dev;
ds2782_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c = &Wire;
    arduino_dev.i2c_freq = 400000;
    dev.i2c_addr = DS2782_I2C_ADDR;
    dev.rsense_mohm = 20;

    if (!ds2782_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("DS2782 init failed!");
        return;
    }

    uint8_t status = 0;
    if (ds2782_read_status(&dev, &status) && (status & DS2782_STATUS_PORF))
    {
        // The gauge has been reset, start the accumulator from a known state
        ds2782_write_accumulated_current(&dev, 0.0f);
    }
}

void loop()
{
    float voltage_mv = 0.0f;
    float current_ma = 0.0f;
    float charge_mah = 0.0f;
    float temp_c = 0.0f;
    uint8_t percent = 0;

    if (ds2782_read_voltage(&dev, &voltage_mv) &&
        ds2782_read_current(&dev, &current_ma) &&
        ds2782_read_accumulated_current(&dev, &charge_mah) &&
        ds2782_read_temperature(&dev, &temp_c) &&
        ds2782_read_active_relative_capacity(&dev, &percent))
    {
        Serial.print("Voltage: ");
        Serial.print(voltage_mv);
        Serial.print(" [mV], current: ");
        Serial.print(current_ma);
        Serial.print(" [mA], charge: ");
        Serial.print(charge_mah);
        Serial.print(" [mAh], temperature: ");
        Serial.print(temp_c);
        Serial.print(" [degC], capacity: ");
        Serial.print(percent);
        Serial.println(" [%]");
    }

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `ds2782.h`:

* `ds2782_write()`
* `ds2782_read()`
* `ds2782_delay_ms()`

and provide a `ds2782_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/ds2782-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
