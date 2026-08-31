# AT24CXX I2C EEPROM memory

## Description
Platform independent driver for the Atmel/Microchip AT24Cxx family of serial I2C
EEPROM memories.

The core (`at24cxx.c` / `at24cxx.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: I2C
* Base address: `AT24CXX_ADDR` (`0xA0`) OR-ed with the A2/A1/A0 pin strapping
* Supported parts (`at24cxx_type`): AT24C01, AT24C02, AT24C04, AT24C08, AT24C16,
  AT24C32, AT24C64, AT24C128, AT24C256, AT24C512, AT24C1024
* Memory size, page size and word address length are taken from the
  `at24cxx_devices[]` capability table, indexed by `at24cxx_type`
* Optional write protection pin driven through `at24cxx_enable_wp()`

## Supported platforms
| Platform | Files |
|---|---|
| Mbed | `platform/mbed/at24cxx-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/at24cxx-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `MBED` platform:

```cpp
#include "AT24CXX/platform/mbed/at24cxx-mbed.h"

#define AT24CXX_SDA PB_7
#define AT24CXX_SCL PB_6
#define AT24CXX_WP  PB_5

int main()
{
    I2C i2c(AT24CXX_SDA, AT24CXX_SCL);
    DigitalOut wp(AT24CXX_WP);

    at24cxx dev;
    at24cxx_mbed mbed_dev;
    mbed_dev.i2c = &i2c;
    mbed_dev.wp = &wp;

    dev.type = AT24C32;

    // The last argument is the A2/A1/A0 pin strapping
    at24cxx_mbed_init(&dev, &mbed_dev, 0);

    const uint8_t tx[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    uint8_t rx[4] = { 0 };

    if (at24cxx_write(&dev, 0x0000, tx, sizeof(tx)) != AT24CXX_NOERR)
    {
        printf("AT24CXX write failed!\r\n");
        return -1;
    }

    if (at24cxx_read(&dev, 0x0000, rx, sizeof(rx)) != AT24CXX_NOERR)
    {
        printf("AT24CXX read failed!\r\n");
        return -1;
    }

    printf("EEPROM size: %lu [B], read back: %02X %02X %02X %02X\r\n",
           (unsigned long)at24cxx_size(&dev), rx[0], rx[1], rx[2], rx[3]);
    return 0;
}
```

Another usage example can be found here:
https://github.com/luk6xff/GarageDoorKeypad/blob/master/common/Storage/eeprom.c

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `at24cxx.h`:

* `at24cxx_io_init()`
* `at24cxx_io_deinit()`
* `at24cxx_write_buffer()`
* `at24cxx_read_buffer()`
* `at24cxx_enable_wp()`
* `at24cxx_delay_ms()`

and provide an `at24cxx_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/mbed/at24cxx-mbed.cpp`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
