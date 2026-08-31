# DS1820 1-Wire digital temperature sensor

## Description
Platform independent driver for the Dallas/Maxim DS1820 / DS18S20 / DS18B20
1-Wire digital temperature probes.

The core (`ds1820.c` / `ds1820.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: 1-Wire (single data line, optional parasite power pin)
* Multi-drop bus: ROM search via `ds1820_search_rom_setup()` /
  `ds1820_search_rom()` and alarm search via `ds1820_search_alarm()`
* Resolution: 9..12 bits, set with `ds1820_set_configuration_bits()`
* Scales (`ds1820_scale`): `CELSIUS`, `FAHRENHEIT`
* Command scope (`ds1820_devices`): `THIS` (addressed device) or `ALL`
  (every device on the bus)
* CRC checks on ROM and scratchpad: `ds1820_rom_checksum_error()`,
  `ds1820_ram_checksum_error()`

## Supported platforms
| Platform | Files |
|---|---|
| Mbed | `platform/mbed/ds1820-mbed.{h,cpp}` |

## Usage
Demo code showing how to use the library on the `MBED` platform:

```cpp
#include "DS1820/platform/mbed/ds1820-mbed.h"

#define DS1820_DATA_PIN    PE_12
#define DS1820_SENSORS_NUM 1

int main()
{
    DigitalInOut data_pin(DS1820_DATA_PIN);

    ds1820 dev;
    ds1820_mbed mbed_dev;
    mbed_dev.data_pin = &data_pin;
    mbed_dev.parasite_pin = nullptr; // non parasite-powered setup

    dev.sensors_num = DS1820_SENSORS_NUM;
    ds1820_mbed_init(&dev, &mbed_dev);

    while (true)
    {
        ds1820_convert_temperature(&dev, ALL);
        float temp = ds1820_read_temperature(&dev, CELSIUS);
        printf("Temperature: %3.1f [C]\r\n", temp);
    }

    return 0;
}
```

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `ds1820.h`:

* `ds1820_onewire_reset()`
* `ds1820_onewire_bit_out()`
* `ds1820_onewire_bit_in()`
* `ds1820_set_parasite_pin()`
* `ds1820_delay_us()`

and provide a `ds1820_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/mbed/ds1820-mbed.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
