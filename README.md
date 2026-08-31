# DevLibs

Drivers for various electronic modules, written in plain C so they can be
dropped into just about any embedded project.

## Library layout
Every library follows the same structure: a **portable C core** that contains all
the device logic and knows nothing about the hardware, plus a thin
**platform layer** that implements a handful of hardware dependent functions.

```
<LIBRARY>/
├── <name>.h                 portable C core, public API
├── <name>.c                 portable C core, device logic
├── platform/
│   ├── arduino/<name>-arduino.{h,cpp}
│   ├── mbed/<name>-mbed.{h,cpp}
│   └── stm32cube/<name>-cube-hal.{h,c}
├── docs/                    datasheets (where available)
├── README.md
├── library.json             PlatformIO manifest
├── library.properties       Arduino IDE manifest
└── lib_build.py             PlatformIO platform selection script
```

The core declares its hardware dependencies as `extern` functions at the bottom
of `<name>.h`, and the device object carries a `void* platform_dev` pointer that
each platform layer fills in through its `<name>_<platform>_init()` helper.

## Using a library with PlatformIO
Select the platform implementation with the `PLATFORM` build flag; `lib_build.py`
takes care of the include path and source filtering:

```ini
[env:nucleo_f091rc]
platform = ststm32
framework = arduino
build_flags = -D PLATFORM=arduino
```

## Porting to a new platform
1. Create `platform/<yours>/<name>-<yours>.{h,c}`.
2. Implement the `extern` functions listed at the bottom of `<name>.h`.
3. Add a `<name>_<yours>_init()` helper that stores your descriptor in
   `dev->platform_dev` and then calls `<name>_init(dev)`.

See any existing platform layer for a reference implementation.

## Libraries

### Sensors
| Library | Device | Interface |
|---|---|---|
| [ADT7410](ADT7410) | Digital temperature sensor, 16-bit | I2C |
| [AS3935](AS3935) | Franklin lightning sensor | I2C |
| [BH1750](BH1750) | Ambient light sensor | I2C |
| [BME280](BME280) | Temperature, humidity and pressure sensor | I2C |
| [BMP180](BMP180) | Barometric pressure and temperature sensor | I2C |
| [DS1621](DS1621) | Digital thermometer and thermostat | I2C |
| [DS1820](DS1820) | DS1820/DS18S20/DS18B20 temperature probes | 1-Wire |
| [GPS_GTS-4E-60](GPS_GTS-4E-60) | Fibocom GTS-4E-60 GPS receiver, NMEA 0183 | UART |
| [MAX44009](MAX44009) | Ambient light sensor | I2C |
| [MPU6050](MPU6050) | 6-axis IMU, 3-axis gyro + 3-axis accelerometer | I2C |
| [SI7020](SI7020) | Humidity and temperature sensor | I2C |
| [TEMT6000](TEMT6000) | Ambient light sensor | Analog |
| [US-015](US-015) | US-015 / HC-SR04 ultrasonic distance sensor | GPIO |

### Measurement and conversion
| Library | Device | Interface |
|---|---|---|
| [AD9106](AD9106) | Quad 12-bit 180 MSPS DAC and waveform generator | SPI |
| [LTC2460](LTC2460) | 16-bit delta-sigma ADC | SPI |
| [MAX4070](MAX4070) | Bidirectional high-side current-sense amplifier | Analog |
| [MAX9611](MAX9611) | Current-sense amplifier with 12-bit ADC | I2C |

### Communication
| Library | Device | Interface |
|---|---|---|
| [BTM222](BTM222) | Bluetooth SPP module, AT commands | UART |
| [HM11_BLE](HM11_BLE) | HM-11 Bluetooth Low Energy 4.0 module, AT commands | UART |
| [LORA](LORA) | SX127x LoRa transceiver | SPI |
| [NRF24L01+](NRF24L01+) | 2.4 GHz transceiver | SPI |
| [RF22](RF22) | HopeRF RFM22B/RFM23B ISM transceiver | SPI |
| [RFM69](RFM69) | HopeRF RFM69 sub-GHz transceiver | SPI |
| [SX1278](SX1278) | Semtech SX1278 LoRa transceiver | SPI |

### Display
| Library | Device | Interface |
|---|---|---|
| [SPFD5408](SPFD5408) | 240x320 TFT LCD controller | 8080 parallel |
| [SPLC501C](SPLC501C) | 128x64 monochrome graphic LCD | SPI |
| [TM1637](TM1637) | 7-segment LED display driver and key scan | 2-wire |

### Power, storage and other
| Library | Device | Interface |
|---|---|---|
| [AT24CXX](AT24CXX) | AT24Cxx EEPROM memory family | I2C |
| [DS2782](DS2782) | Stand-alone battery fuel gauge | I2C |
| [DS3231](DS3231) | Real time clock with TCXO | I2C |
| [TS3USB221](TS3USB221) | High-speed USB 2.0 1:2 multiplexer/demultiplexer | GPIO |

## License
MIT, except where a library ships its own `LICENSE.txt`:

* [RF22](RF22/LICENSE.txt) - Copyright (C) 2011 Mike McCauley, GPL V2 or commercial
* [RFM69](RFM69/LICENSE.txt)
* [SX1278](SX1278/LICENSE.txt)

## Author
Lukasz Uszko - lukasz.uszko@gmail.com



