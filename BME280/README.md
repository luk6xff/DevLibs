# BME280 digital pressure, temperature and humidity sensor

## Description
Platform independent driver for the Bosch BME280 combined pressure, temperature
and humidity sensor.

The core (`bme280.c` / `bme280.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C or SPI, selected with `dev.intf` (`BME280_INTF_I2C` /
  `BME280_INTF_SPI`)
* Default I2C address: `BME280_DEFAULT_I2C_ADDRESS` (`0x77`)
* Oversampling (`bme280_sensor_sampling`), IIR filter (`bme280_sensor_filter`),
  standby time (`bme280_standby_duration`) and power mode
  (`bme280_sensor_mode`: sleep / forced / normal) are configurable
* Derived readings: altitude via `bme280_read_altitude()` and sea level pressure
  via `bme280_sea_level_for_altitude()`, referenced to `SEA_LEVEL_PRESSURE_HPA`

## Supported platforms
| Platform | Files |
|---|---|
| Arduino | `platform/arduino/bme280-arduino.{h,cpp}` |
| Mbed | `platform/mbed/bme280-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/bme280-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `ARDUINO` platform:

```cpp
#include "BME280/platform/arduino/bme280-arduino.h"

#define BME280_SDA_PIN 21
#define BME280_SCL_PIN 22

bme280 dev;
bme280_arduino arduino_dev;

void setup()
{
    Serial.begin(115200);

    arduino_dev.i2c.hndl = &Wire;
    arduino_dev.i2c.sda_pin = BME280_SDA_PIN;
    arduino_dev.i2c.scl_pin = BME280_SCL_PIN;

    dev.intf = BME280_INTF_I2C;
    dev.i2c_addr = BME280_DEFAULT_I2C_ADDRESS;
    dev.t_fine = 0;
    dev.t_fine_adjust = 0;
    dev.burst_read_mode = false;

    if (!bme280_arduino_init(&dev, &arduino_dev))
    {
        Serial.println("BME280 init failed!");
    }
}

void loop()
{
    float temperature = 0.0f;
    float pressure = 0.0f;
    float humidity = 0.0f;
    float altitude = 0.0f;

    if (bme280_read_data(&dev, &temperature, &pressure, &humidity))
    {
        Serial.print("T: "); Serial.print(temperature); Serial.print(" [C], ");
        Serial.print("P: "); Serial.print(pressure / 100.0f); Serial.print(" [hPa], ");
        Serial.print("H: "); Serial.print(humidity); Serial.println(" [%]");
    }

    if (bme280_read_altitude(&dev, SEA_LEVEL_PRESSURE_HPA, &altitude))
    {
        Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" [m]");
    }

    delay(1000);
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `bme280.h`:

* `bme280_write()`
* `bme280_read()`
* `bme280_delay_ms()`

and provide a `bme280_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/arduino/bme280-arduino.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
