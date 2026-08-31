# NRF24L01+ 2.4GHz RF transceiver

## Description
Platform independent driver for the Nordic nRF24L01+, a highly integrated, ultra
low power (ULP) 2 Mbps RF transceiver for the 2.4 GHz ISM (Industrial,
Scientific and Medical) band. Based on https://os.mbed.com/users/Owen/code/nRF24L01P/

The core (`nrf24l01p.c` / `nrf24l01p.h`) is written in plain C and knows nothing
about the underlying hardware. Everything hardware related is provided by a thin
platform layer living in `platform/<platform>`.

* Interface: SPI + CE (chip enable) and CSN (chip select not) pins
* RF frequency: `NRF24L01P_MIN_RF_FREQUENCY` (2400 MHz) ..
  `NRF24L01P_MAX_RF_FREQUENCY` (2525 MHz)
* Data rates: 250 kbps, 1 Mbps, 2 Mbps
* TX power: 0, -6, -12, -18 dBm
* CRC: none, 8-bit or 16-bit
* 6 receive pipes (`NRF24L01P_PIPE_P0` .. `NRF24L01P_PIPE_P5`), 3..5 byte
  addresses, 1..32 byte payloads
* Auto acknowledge and auto retransmit support
* Tested on FRDM-KL46Z, FRDM-KL25Z, STM32L and STM32F microcontrollers

Device documentation:
https://www.nordicsemi.com/Products/nRF24-series

## Supported platforms
| Platform | Files |
|---|---|
| Mbed | `platform/mbed/nrf24l01p-mbed.{h,cpp}` |
| STM32 Cube HAL | `platform/stm32cube/nrf24l01p-cube-hal.{h,c}` |

## Usage
Demo code showing how to use the library on the `STM32CUBE` platform:

```c
#include "NRF24L01+/platform/stm32cube/nrf24l01p-cube-hal.h"

extern SPI_HandleTypeDef hspi1;

static nrf24l01p dev;
static nrf24l01p_cube cube_dev;

void radio_init(void)
{
    cube_dev.spi = &hspi1;
    cube_dev.ce_port = GPIOA;
    cube_dev.ce_pin = GPIO_PIN_1;
    cube_dev.csn_port = GPIOA;
    cube_dev.csn_pin = GPIO_PIN_2;

    nrf24l01p_cube_init(&dev, &cube_dev);

    nrf24l01p_power_up(&dev);
    nrf24l01p_set_rf_freq(&dev, DEFAULT_NRF24L01P_RF_FREQUENCY);
    nrf24l01p_set_data_rate(&dev, NRF24L01P_DATARATE_1_MBPS);
    nrf24l01p_set_rf_tx_power(&dev, NRF24L01P_TX_PWR_ZERO_DB);
    nrf24l01p_set_crc_length(&dev, NRF24L01P_CRC_8_BIT);
    nrf24l01p_set_rx_addr(&dev, DEFAULT_NRF24L01P_ADDRESS,
                          DEFAULT_NRF24L01P_ADDRESS_WIDTH, NRF24L01P_PIPE_P0);
    nrf24l01p_set_tx_addr(&dev, DEFAULT_NRF24L01P_ADDRESS,
                          DEFAULT_NRF24L01P_ADDRESS_WIDTH);
    nrf24l01p_set_transfer_size(&dev, DEFAULT_NRF24L01P_TRANSFER_SIZE,
                                NRF24L01P_PIPE_P0);
    nrf24l01p_enable(&dev);
}

void radio_loop(void)
{
    uint8_t tx[DEFAULT_NRF24L01P_TRANSFER_SIZE] = { 'L', 'U', 'K', '6' };
    uint8_t rx[DEFAULT_NRF24L01P_TRANSFER_SIZE] = { 0 };

    nrf24l01p_set_tx_mode(&dev);
    nrf24l01p_write(&dev, NRF24L01P_PIPE_P0, tx, sizeof(tx));

    nrf24l01p_set_rx_mode(&dev);
    if (nrf24l01p_available(&dev, NRF24L01P_PIPE_P0))
    {
        nrf24l01p_read(&dev, NRF24L01P_PIPE_P0, rx, sizeof(rx));
    }

    nrf24l01p_delay_ms(1000);
}
```

Another usage example can be found here:
https://github.com/luk6xff/GarageDoorKeypad/blob/master/common/Radio/radio.c

## Porting to a different platform
To port this library to another platform you only have to implement the
HW dependent functions declared at the bottom of `nrf24l01p.h`:

* `nrf24l01p_spi_write()`
* `nrf24l01p_set_ce_pin()`
* `nrf24l01p_get_ce_pin()`
* `nrf24l01p_set_csn_pin()`
* `nrf24l01p_delay_ms()`
* `nrf24l01p_print_all_regs()`
* `nrf24l01p_print_chip_info()`

and provide an `nrf24l01p_<platform>_init()` helper that stores your platform
descriptor in `dev->platform_dev`. See `platform/stm32cube/nrf24l01p-cube-hal.c`
for a reference implementation.

## License
MIT - see the repository root for details.

## Author
Lukasz Uszko - lukasz.uszko@gmail.com

* Lukasz Uszko aka `luk6xff` [lukasz.uszko@gmail.com]
