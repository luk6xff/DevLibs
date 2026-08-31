// rf22.c
//
// Portable C driver for the HopeRF RFM22B / RFM23B (RF22B) ISM transceiver.
//
// Derived from the RF22 library:
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22.cpp,v 1.17 2013/02/06 21:33:56 mikem Exp mikem $
//   ported to mbed by Karl Zweimueller
//   modified by Lukasz Uszko luszko@op.pl
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#include <string.h>

#include "rf22.h"

// The modem configuration registers are written with burst writes, so the
// struct must be exactly RF22_NUM_MODEM_CONFIG_REGS tightly packed octets.
typedef char rf22_modem_config_size_check[
    (sizeof(rf22_modem_config) == RF22_NUM_MODEM_CONFIG_REGS) ? 1 : -1];

// These are indexed by the values of rf22_modem_config_choice.
// Canned modem configurations generated with
// http://www.hoperf.com/upload/rf/RF22B%2023B%2031B%2042B%2043B%20Register%20Settings_RevB1-v5.xls
static const rf22_modem_config MODEM_CONFIG_TABLE[RF22_MODEM_CONFIG_CHOICE_NUM] =
{
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x00, 0x08 }, // Unmodulated carrier
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x33, 0x08 }, // FSK, PN9 random modulation, 2, 5

    // All the following enable FIFO with reg 71
    //  1c,   1f,   20,   21,   22,   23,   24,   25,   2c,   2d,   2e,   58,   69,   6e,   6f,   70,   71,   72
    // FSK, No Manchester, Max Rb err <1%, Xtal Tol 20ppm
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x22, 0x08 }, // 2, 5
    { 0x1b, 0x03, 0x41, 0x60, 0x27, 0x52, 0x00, 0x07, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x13, 0xa9, 0x2c, 0x22, 0x3a }, // 2.4, 36
    { 0x1d, 0x03, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x13, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x27, 0x52, 0x2c, 0x22, 0x48 }, // 4.8, 45
    { 0x1e, 0x03, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x45, 0x40, 0x0a, 0x20, 0x80, 0x60, 0x4e, 0xa5, 0x2c, 0x22, 0x48 }, // 9.6, 45
    { 0x2b, 0x03, 0x34, 0x02, 0x75, 0x25, 0x07, 0xff, 0x40, 0x0a, 0x1b, 0x80, 0x60, 0x9d, 0x49, 0x2c, 0x22, 0x0f }, // 19.2, 9.6
    { 0x02, 0x03, 0x68, 0x01, 0x3a, 0x93, 0x04, 0xd5, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x09, 0xd5, 0x0c, 0x22, 0x1f }, // 38.4, 19.6
    { 0x06, 0x03, 0x45, 0x01, 0xd7, 0xdc, 0x07, 0x6e, 0x40, 0x0a, 0x2d, 0x80, 0x60, 0x0e, 0xbf, 0x0c, 0x22, 0x2e }, // 57.6, 28.8
    { 0x8a, 0x03, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x40, 0x0a, 0x50, 0x80, 0x60, 0x20, 0x00, 0x0c, 0x22, 0xc8 }, // 125, 125

    // GFSK, No Manchester, Max Rb err <1%, Xtal Tol 20ppm
    // These differ from FSK only in register 71, for the modulation type
    { 0x2b, 0x03, 0xf4, 0x20, 0x41, 0x89, 0x00, 0x36, 0x40, 0x0a, 0x1d, 0x80, 0x60, 0x10, 0x62, 0x2c, 0x23, 0x08 }, // 2, 5
    { 0x1b, 0x03, 0x41, 0x60, 0x27, 0x52, 0x00, 0x07, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x13, 0xa9, 0x2c, 0x23, 0x3a }, // 2.4, 36
    { 0x1d, 0x03, 0xa1, 0x20, 0x4e, 0xa5, 0x00, 0x13, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x27, 0x52, 0x2c, 0x23, 0x48 }, // 4.8, 45
    { 0x1e, 0x03, 0xd0, 0x00, 0x9d, 0x49, 0x00, 0x45, 0x40, 0x0a, 0x20, 0x80, 0x60, 0x4e, 0xa5, 0x2c, 0x23, 0x48 }, // 9.6, 45
    { 0x2b, 0x03, 0x34, 0x02, 0x75, 0x25, 0x07, 0xff, 0x40, 0x0a, 0x1b, 0x80, 0x60, 0x9d, 0x49, 0x2c, 0x23, 0x0f }, // 19.2, 9.6
    { 0x02, 0x03, 0x68, 0x01, 0x3a, 0x93, 0x04, 0xd5, 0x40, 0x0a, 0x1e, 0x80, 0x60, 0x09, 0xd5, 0x0c, 0x23, 0x1f }, // 38.4, 19.6
    { 0x06, 0x03, 0x45, 0x01, 0xd7, 0xdc, 0x07, 0x6e, 0x40, 0x0a, 0x2d, 0x80, 0x60, 0x0e, 0xbf, 0x0c, 0x23, 0x2e }, // 57.6, 28.8
    { 0x8a, 0x03, 0x60, 0x01, 0x55, 0x55, 0x02, 0xad, 0x40, 0x0a, 0x50, 0x80, 0x60, 0x20, 0x00, 0x0c, 0x23, 0xc8 }, // 125, 125

    // OOK, No Manchester, Max Rb err <1%, Xtal Tol 20ppm
    { 0x51, 0x03, 0x68, 0x00, 0x3a, 0x93, 0x01, 0x3d, 0x2c, 0x11, 0x28, 0x80, 0x60, 0x09, 0xd5, 0x2c, 0x21, 0x08 }, // 1.2, 75
    { 0xc8, 0x03, 0x39, 0x20, 0x68, 0xdc, 0x00, 0x6b, 0x2a, 0x08, 0x2a, 0x80, 0x60, 0x13, 0xa9, 0x2c, 0x21, 0x08 }, // 2.4, 335
    { 0xc8, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x29, 0x04, 0x29, 0x80, 0x60, 0x27, 0x52, 0x2c, 0x21, 0x08 }, // 4.8, 335
    { 0xb8, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x28, 0x82, 0x29, 0x80, 0x60, 0x4e, 0xa5, 0x2c, 0x21, 0x08 }, // 9.6, 335
    { 0xa8, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x28, 0x41, 0x29, 0x80, 0x60, 0x9d, 0x49, 0x2c, 0x21, 0x08 }, // 19.2, 335
    { 0x98, 0x03, 0x9c, 0x00, 0xd1, 0xb7, 0x00, 0xd4, 0x28, 0x20, 0x29, 0x80, 0x60, 0x09, 0xd5, 0x0c, 0x21, 0x08 }, // 38.4, 335
    { 0x98, 0x03, 0x96, 0x00, 0xda, 0x74, 0x00, 0xdc, 0x28, 0x1f, 0x29, 0x80, 0x60, 0x0a, 0x3d, 0x0c, 0x21, 0x08 }, // 40, 335
};


//-----------------------------------------------------------------------------
// Internal helpers
//-----------------------------------------------------------------------------

static bool rf22_append_tx_buf(rf22* const dev, const uint8_t* data, uint8_t len);
static bool rf22_fill_tx_buf(rf22* const dev, const uint8_t* data, uint8_t len);
static void rf22_send_next_fragment(rf22* const dev);
static void rf22_read_next_fragment(rf22* const dev);
static void rf22_start_transmit(rf22* const dev);
static void rf22_restart_transmit(rf22* const dev);

//-----------------------------------------------------------------------------
static bool rf22_timed_out(uint32_t start_ms, uint32_t timeout_ms)
{
    return (rf22_get_ms_ticks() - start_ms) > timeout_ms;
}

//-----------------------------------------------------------------------------
bool rf22_init(rf22* const dev)
{
    static const uint8_t sync_words[] = { 0x2d, 0xd4 };

    if (!dev)
    {
        return false;
    }

    dev->idle_mode = RF22_XTON; // Default idle state is READY mode
    dev->mode = RF22_MODE_IDLE;
    dev->device_type = 0;
    dev->buf_len = 0;
    dev->tx_buf_sent_index = 0;
    dev->rx_buf_valid = false;
    dev->last_rssi = 0;
    dev->rx_bad = 0;
    dev->rx_good = 0;
    dev->tx_good = 0;

    // Power the module up and wait for the RF22 POR (up to 16 ms)
    rf22_set_sdn_pin(dev, false);
    rf22_delay_ms(16);

    // Deselect the device and let the supply settle
    rf22_set_cs_pin(dev, true);
    rf22_delay_ms(100);

    rf22_reset(dev);

    // Get the device type and check it. This also tests whether we are really
    // connected to a device.
    dev->device_type = rf22_spi_read_reg(dev, RF22_REG_00_DEVICE_TYPE);
    if (dev->device_type != RF22_DEVICE_TYPE_RX_TRX &&
        dev->device_type != RF22_DEVICE_TYPE_TX)
    {
        return false;
    }

    rf22_clear_tx_buf(dev);
    rf22_clear_rx_buf(dev);

    // Most of these are the POR default
    rf22_spi_write_reg(dev, RF22_REG_7D_TX_FIFO_CONTROL2, RF22_TXFFAEM_THRESHOLD);
    rf22_spi_write_reg(dev, RF22_REG_7E_RX_FIFO_CONTROL, RF22_RXFFAFULL_THRESHOLD);
    rf22_spi_write_reg(dev, RF22_REG_30_DATA_ACCESS_CONTROL,
                       RF22_ENPACRX | RF22_ENPACTX | RF22_ENCRC | RF22_CRC_CRC_16_IBM);

    // Standard packet format used by this library:
    //   8 nibbles preamble
    //   2 SYNC words 0x2d, 0xd4
    //   4 octets header (to, from, id, flags)
    //   1 octet of data length (0 to 255)
    //   0 to 255 octets data
    //   2 CRC octets as CRC16(IBM), computed on the header, length and data
    rf22_spi_write_reg(dev, RF22_REG_32_HEADER_CONTROL1,
                       RF22_BCEN_HEADER3 | RF22_HDCH_HEADER3);
    rf22_spi_write_reg(dev, RF22_REG_33_HEADER_CONTROL2,
                       RF22_HDLEN_4 | RF22_SYNCLEN_2);
    rf22_set_preamble_length(dev, 8);
    rf22_set_sync_words(dev, sync_words, (uint8_t)sizeof(sync_words));
    rf22_set_promiscuous(dev, false);

    // Check the TO header against RF22_DEFAULT_NODE_ADDRESS
    rf22_spi_write_reg(dev, RF22_REG_3F_CHECK_HEADER3, RF22_DEFAULT_NODE_ADDRESS);
    rf22_set_header_to(dev, RF22_DEFAULT_NODE_ADDRESS);
    rf22_set_header_from(dev, RF22_DEFAULT_NODE_ADDRESS);
    rf22_set_header_id(dev, 0);
    rf22_set_header_flags(dev, 0);

    // Ensure the antenna can be switched automatically according to transmit
    // and receive. This assumes GPIO0(out) is connected to TX_ANT(in) and
    // GPIO1(out) is connected to RX_ANT(in).
    rf22_spi_write_reg(dev, RF22_REG_0B_GPIO_CONFIGURATION0, 0x12); // TX state
    rf22_spi_write_reg(dev, RF22_REG_0C_GPIO_CONFIGURATION1, 0x15); // RX state

    // Enable interrupts. RF22_ENFFERR is deliberately left out, with it the
    // mbed port was observed to lock up after some hours, see
    // https://groups.google.com/forum/?fromgroups#!topic/rf22-arduino/Ezkw256yQI8
    rf22_spi_write_reg(dev, RF22_REG_05_INTERRUPT_ENABLE1,
                       RF22_ENTXFFAEM | RF22_ENRXFFAFULL | RF22_ENPKSENT |
                       RF22_ENPKVALID | RF22_ENCRCERROR);
    rf22_spi_write_reg(dev, RF22_REG_06_INTERRUPT_ENABLE2, RF22_ENPREAVAL);

    // Set some defaults. An innocuous ISM frequency, and reasonable pull in.
    if (!rf22_set_frequency(dev, 434.0f, 0.05f))
    {
        return false;
    }
    if (!rf22_set_modem_config(dev, RF22_FSK_RB125FD125))
    {
        return false;
    }
    rf22_set_tx_power(dev, RF22_TXPOW_8DBM);

    return true;
}

//-----------------------------------------------------------------------------
void rf22_reset(const rf22* const dev)
{
    if (!dev)
    {
        return;
    }

    rf22_spi_write_reg(dev, RF22_REG_07_OPERATING_MODE1, RF22_SWRES);
    // SWReset time is nominally 100 usec, wait for it to settle
    rf22_delay_ms(1);
}

//-----------------------------------------------------------------------------
uint8_t rf22_spi_read_reg(const rf22* const dev, uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    if (!dev)
    {
        return 0;
    }

    tx[0] = (uint8_t)(reg & (uint8_t)~RF22_SPI_WRITE_MASK);
    tx[1] = 0;
    rx[0] = 0;
    rx[1] = 0;

    rf22_set_cs_pin(dev, false);
    (void)rf22_spi_transfer(dev, tx, rx, sizeof(tx));
    rf22_set_cs_pin(dev, true);

    return rx[1];
}

//-----------------------------------------------------------------------------
void rf22_spi_write_reg(const rf22* const dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2];

    if (!dev)
    {
        return;
    }

    tx[0] = (uint8_t)(reg | RF22_SPI_WRITE_MASK);
    tx[1] = val;

    rf22_set_cs_pin(dev, false);
    (void)rf22_spi_transfer(dev, tx, NULL, sizeof(tx));
    rf22_set_cs_pin(dev, true);
}

//-----------------------------------------------------------------------------
void rf22_spi_burst_read(const rf22* const dev, uint8_t reg, uint8_t* dest,
                         uint8_t len)
{
    uint8_t addr;

    if (!dev || !dest || len == 0)
    {
        return;
    }

    addr = (uint8_t)(reg & (uint8_t)~RF22_SPI_WRITE_MASK);

    rf22_set_cs_pin(dev, false);
    (void)rf22_spi_transfer(dev, &addr, NULL, 1);
    (void)rf22_spi_transfer(dev, NULL, dest, (size_t)len);
    rf22_set_cs_pin(dev, true);
}

//-----------------------------------------------------------------------------
void rf22_spi_burst_write(const rf22* const dev, uint8_t reg,
                          const uint8_t* src, uint8_t len)
{
    uint8_t addr;

    if (!dev || !src || len == 0)
    {
        return;
    }

    addr = (uint8_t)(reg | RF22_SPI_WRITE_MASK);

    rf22_set_cs_pin(dev, false);
    (void)rf22_spi_transfer(dev, &addr, NULL, 1);
    (void)rf22_spi_transfer(dev, src, NULL, (size_t)len);
    rf22_set_cs_pin(dev, true);
}

//-----------------------------------------------------------------------------
uint8_t rf22_status_read(const rf22* const dev)
{
    return rf22_spi_read_reg(dev, RF22_REG_02_DEVICE_STATUS);
}

//-----------------------------------------------------------------------------
uint8_t rf22_ezmac_status_read(const rf22* const dev)
{
    return rf22_spi_read_reg(dev, RF22_REG_31_EZMAC_STATUS);
}

//-----------------------------------------------------------------------------
uint8_t rf22_rssi_read(const rf22* const dev)
{
    return rf22_spi_read_reg(dev, RF22_REG_26_RSSI);
}

//-----------------------------------------------------------------------------
uint8_t rf22_last_rssi(const rf22* const dev)
{
    if (!dev)
    {
        return 0;
    }

    return dev->last_rssi;
}

//-----------------------------------------------------------------------------
bool rf22_adc_read(const rf22* const dev, uint8_t adcsel, uint8_t adcref,
                   uint8_t adcgain, uint8_t adcoffs, uint8_t* value)
{
    uint8_t configuration;
    uint32_t start;

    if (!dev || !value)
    {
        return false;
    }

    configuration = (uint8_t)(adcsel | adcref | (adcgain & RF22_ADCGAIN));
    rf22_spi_write_reg(dev, RF22_REG_0F_ADC_CONFIGURATION,
                       (uint8_t)(configuration | RF22_ADCSTART));
    rf22_spi_write_reg(dev, RF22_REG_10_ADC_SENSOR_AMP_OFFSET, adcoffs);

    // Conversion time is nominally 305 usec, wait for the DONE bit
    start = rf22_get_ms_ticks();
    while ((rf22_spi_read_reg(dev, RF22_REG_0F_ADC_CONFIGURATION) & RF22_ADCDONE) == 0)
    {
        if (rf22_timed_out(start, RF22_ADC_TIMEOUT_MS))
        {
            return false;
        }
    }

    *value = rf22_spi_read_reg(dev, RF22_REG_11_ADC_VALUE);
    return true;
}

//-----------------------------------------------------------------------------
bool rf22_temperature_read(const rf22* const dev, uint8_t tsrange,
                           uint8_t tvoffs, uint8_t* value)
{
    if (!dev || !value)
    {
        return false;
    }

    rf22_spi_write_reg(dev, RF22_REG_12_TEMPERATURE_SENSOR_CALIBRATION,
                       (uint8_t)(tsrange | RF22_ENTSOFFS));
    rf22_spi_write_reg(dev, RF22_REG_13_TEMPERATURE_VALUE_OFFSET, tvoffs);

    return rf22_adc_read(dev, RF22_ADCSEL_INTERNAL_TEMPERATURE_SENSOR,
                         RF22_ADCREF_BANDGAP_VOLTAGE, 0, 0, value);
}

//-----------------------------------------------------------------------------
uint16_t rf22_wut_read(const rf22* const dev)
{
    uint8_t buf[2] = { 0, 0 };

    rf22_spi_burst_read(dev, RF22_REG_17_WAKEUP_TIMER_VALUE1, buf, sizeof(buf));

    // Do not rely on byte order
    return (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

//-----------------------------------------------------------------------------
// RFM-22 doc appears to be wrong: WUT for wtm = 10000, r = 0, d = 0 is about 1s
void rf22_set_wut_period(const rf22* const dev, uint16_t wtm, uint8_t wtr,
                         uint8_t wtd)
{
    uint8_t period[3];

    period[0] = (uint8_t)(((wtr & 0x0fu) << 2) | (wtd & 0x03u));
    period[1] = (uint8_t)(wtm >> 8);
    period[2] = (uint8_t)(wtm & 0xffu);

    rf22_spi_burst_write(dev, RF22_REG_14_WAKEUP_TIMER_PERIOD1, period,
                         (uint8_t)sizeof(period));
}

//-----------------------------------------------------------------------------
// Returns true if centre + (fhch * fhs) is within limits. Caution, different
// versions of the RF22 support different max frequencies, so YMMV.
bool rf22_set_frequency(const rf22* const dev, float centre_mhz,
                        float afc_pull_in_mhz)
{
    uint8_t fbsel = RF22_SBSEL;
    uint8_t afclimiter;
    uint8_t fb;
    uint16_t fc;
    float centre = centre_mhz;
    float integer_part;
    float fractional_part;

    if (!dev)
    {
        return false;
    }
    // 930.0 for early silicon
    if (centre < 240.0f || centre > 960.0f)
    {
        return false;
    }

    if (centre >= 480.0f)
    {
        if (afc_pull_in_mhz < 0.0f || afc_pull_in_mhz > 0.318750f)
        {
            return false;
        }
        centre /= 2.0f;
        fbsel = (uint8_t)(fbsel | RF22_HBSEL);
        afclimiter = (uint8_t)(afc_pull_in_mhz * 1000000.0f / 1250.0f);
    }
    else
    {
        if (afc_pull_in_mhz < 0.0f || afc_pull_in_mhz > 0.159375f)
        {
            return false;
        }
        afclimiter = (uint8_t)(afc_pull_in_mhz * 1000000.0f / 625.0f);
    }

    // centre is now 24.0 .. 48.0, so the truncation below is a valid floor()
    centre /= 10.0f;
    integer_part = (float)(uint32_t)centre;
    fractional_part = centre - integer_part;

    fb = (uint8_t)((uint32_t)integer_part - 24u); // Range 0 to 23
    fbsel = (uint8_t)(fbsel | fb);
    fc = (uint16_t)(fractional_part * 64000.0f);

    rf22_spi_write_reg(dev, RF22_REG_73_FREQUENCY_OFFSET1, 0);
    rf22_spi_write_reg(dev, RF22_REG_74_FREQUENCY_OFFSET2, 0);
    rf22_spi_write_reg(dev, RF22_REG_75_FREQUENCY_BAND_SELECT, fbsel);
    rf22_spi_write_reg(dev, RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1,
                       (uint8_t)(fc >> 8));
    rf22_spi_write_reg(dev, RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0,
                       (uint8_t)(fc & 0xffu));
    rf22_spi_write_reg(dev, RF22_REG_2A_AFC_LIMITER, afclimiter);

    return (rf22_status_read(dev) & RF22_FREQERR) == 0;
}

//-----------------------------------------------------------------------------
bool rf22_set_fh_step_size(const rf22* const dev, uint8_t fhs)
{
    if (!dev)
    {
        return false;
    }

    rf22_spi_write_reg(dev, RF22_REG_7A_FREQUENCY_HOPPING_STEP_SIZE, fhs);
    return (rf22_status_read(dev) & RF22_FREQERR) == 0;
}

//-----------------------------------------------------------------------------
bool rf22_set_fh_channel(const rf22* const dev, uint8_t fhch)
{
    if (!dev)
    {
        return false;
    }

    rf22_spi_write_reg(dev, RF22_REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT, fhch);
    return (rf22_status_read(dev) & RF22_FREQERR) == 0;
}

//-----------------------------------------------------------------------------
void rf22_set_operating_mode(const rf22* const dev, uint8_t mode)
{
    rf22_spi_write_reg(dev, RF22_REG_07_OPERATING_MODE1, mode);
}

//-----------------------------------------------------------------------------
void rf22_set_mode_idle(rf22* const dev)
{
    if (!dev)
    {
        return;
    }

    if (dev->mode != RF22_MODE_IDLE)
    {
        rf22_set_operating_mode(dev, dev->idle_mode);
        dev->mode = RF22_MODE_IDLE;
    }
}

//-----------------------------------------------------------------------------
void rf22_set_mode_rx(rf22* const dev)
{
    if (!dev)
    {
        return;
    }

    if (dev->mode != RF22_MODE_RX)
    {
        rf22_set_operating_mode(dev, (uint8_t)(dev->idle_mode | RF22_RXON));
        dev->mode = RF22_MODE_RX;
    }
}

//-----------------------------------------------------------------------------
void rf22_set_mode_tx(rf22* const dev)
{
    if (!dev)
    {
        return;
    }

    if (dev->mode != RF22_MODE_TX)
    {
        rf22_set_operating_mode(dev, (uint8_t)(dev->idle_mode | RF22_TXON));
        dev->mode = RF22_MODE_TX;
        // If you do not clear the RX FIFO here, going to transmit mode in the
        // middle of a receive can corrupt the RX FIFO
        rf22_reset_rx_fifo(dev);
    }
}

//-----------------------------------------------------------------------------
rf22_mode rf22_get_mode(const rf22* const dev)
{
    if (!dev)
    {
        return RF22_MODE_IDLE;
    }

    return dev->mode;
}

//-----------------------------------------------------------------------------
void rf22_set_tx_power(const rf22* const dev, uint8_t power)
{
    rf22_spi_write_reg(dev, RF22_REG_6D_TX_POWER, power);
}

//-----------------------------------------------------------------------------
void rf22_set_modem_registers(const rf22* const dev,
                              const rf22_modem_config* const config)
{
    if (!dev || !config)
    {
        return;
    }

    rf22_spi_write_reg(dev, RF22_REG_1C_IF_FILTER_BANDWIDTH, config->reg_1c);
    rf22_spi_write_reg(dev, RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE,
                       config->reg_1f);
    rf22_spi_burst_write(dev, RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE,
                         &config->reg_20, 6);
    rf22_spi_burst_write(dev, RF22_REG_2C_OOK_COUNTER_VALUE_1,
                         &config->reg_2c, 3);
    rf22_spi_write_reg(dev, RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING,
                       config->reg_58);
    rf22_spi_write_reg(dev, RF22_REG_69_AGC_OVERRIDE1, config->reg_69);
    rf22_spi_burst_write(dev, RF22_REG_6E_TX_DATA_RATE1, &config->reg_6e, 5);
}

//-----------------------------------------------------------------------------
bool rf22_set_modem_config(const rf22* const dev,
                           rf22_modem_config_choice index)
{
    if (!dev)
    {
        return false;
    }
    // The original used > here, which allowed a read one past the table
    if (index >= RF22_MODEM_CONFIG_CHOICE_NUM)
    {
        return false;
    }

    rf22_set_modem_registers(dev, &MODEM_CONFIG_TABLE[index]);
    return true;
}

//-----------------------------------------------------------------------------
// REVISIT: top bit is in Header Control 2 0x33
void rf22_set_preamble_length(const rf22* const dev, uint8_t nibbles)
{
    rf22_spi_write_reg(dev, RF22_REG_34_PREAMBLE_LENGTH, nibbles);
}

//-----------------------------------------------------------------------------
// Caution: does not set the sync word length in Header Control 2 0x33
void rf22_set_sync_words(const rf22* const dev, const uint8_t* sync_words,
                         uint8_t len)
{
    if (len == 0 || len > 4)
    {
        return;
    }

    rf22_spi_burst_write(dev, RF22_REG_36_SYNC_WORD3, sync_words, len);
}

//-----------------------------------------------------------------------------
void rf22_set_promiscuous(const rf22* const dev, bool promiscuous)
{
    rf22_spi_write_reg(dev, RF22_REG_43_HEADER_ENABLE3,
                       promiscuous ? 0x00u : 0xffu);
}

//-----------------------------------------------------------------------------
bool rf22_available(rf22* const dev)
{
    if (!dev)
    {
        return false;
    }

    rf22_poll(dev);

    if (!dev->rx_buf_valid)
    {
        rf22_set_mode_rx(dev); // Make sure we are receiving
    }

    return dev->rx_buf_valid;
}

//-----------------------------------------------------------------------------
bool rf22_wait_available(rf22* const dev, uint32_t timeout_ms)
{
    uint32_t start;

    if (!dev)
    {
        return false;
    }

    start = rf22_get_ms_ticks();
    do
    {
        if (rf22_available(dev))
        {
            return true;
        }
    } while (!rf22_timed_out(start, timeout_ms));

    return false;
}

//-----------------------------------------------------------------------------
bool rf22_recv(rf22* const dev, uint8_t* buf, uint8_t* len)
{
    if (!dev || !buf || !len)
    {
        return false;
    }

    if (!rf22_available(dev))
    {
        return false;
    }

    if (*len > dev->buf_len)
    {
        *len = dev->buf_len;
    }
    memcpy(buf, dev->buf, (size_t)*len);
    rf22_clear_rx_buf(dev);

    return true;
}

//-----------------------------------------------------------------------------
bool rf22_send(rf22* const dev, const uint8_t* data, uint8_t len)
{
    if (!dev || !data || len == 0)
    {
        return false;
    }

    if (!rf22_wait_packet_sent(dev, RF22_TX_TIMEOUT_MS))
    {
        return false;
    }

    if (!rf22_fill_tx_buf(dev, data, len))
    {
        return false;
    }

    rf22_start_transmit(dev);
    return true;
}

//-----------------------------------------------------------------------------
bool rf22_wait_packet_sent(rf22* const dev, uint32_t timeout_ms)
{
    uint32_t start;

    if (!dev)
    {
        return false;
    }

    start = rf22_get_ms_ticks();
    while (dev->mode == RF22_MODE_TX)
    {
        rf22_poll(dev);
        if (dev->mode != RF22_MODE_TX)
        {
            break;
        }
        if (rf22_timed_out(start, timeout_ms))
        {
            return false;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------
void rf22_set_header_to(const rf22* const dev, uint8_t to)
{
    rf22_spi_write_reg(dev, RF22_REG_3A_TRANSMIT_HEADER3, to);
}

//-----------------------------------------------------------------------------
void rf22_set_header_from(const rf22* const dev, uint8_t from)
{
    rf22_spi_write_reg(dev, RF22_REG_3B_TRANSMIT_HEADER2, from);
}

//-----------------------------------------------------------------------------
void rf22_set_header_id(const rf22* const dev, uint8_t id)
{
    rf22_spi_write_reg(dev, RF22_REG_3C_TRANSMIT_HEADER1, id);
}

//-----------------------------------------------------------------------------
void rf22_set_header_flags(const rf22* const dev, uint8_t flags)
{
    rf22_spi_write_reg(dev, RF22_REG_3D_TRANSMIT_HEADER0, flags);
}

//-----------------------------------------------------------------------------
uint8_t rf22_header_to(const rf22* const dev)
{
    return rf22_spi_read_reg(dev, RF22_REG_47_RECEIVED_HEADER3);
}

//-----------------------------------------------------------------------------
uint8_t rf22_header_from(const rf22* const dev)
{
    return rf22_spi_read_reg(dev, RF22_REG_48_RECEIVED_HEADER2);
}

//-----------------------------------------------------------------------------
uint8_t rf22_header_id(const rf22* const dev)
{
    return rf22_spi_read_reg(dev, RF22_REG_49_RECEIVED_HEADER1);
}

//-----------------------------------------------------------------------------
uint8_t rf22_header_flags(const rf22* const dev)
{
    return rf22_spi_read_reg(dev, RF22_REG_4A_RECEIVED_HEADER0);
}

//-----------------------------------------------------------------------------
void rf22_handle_interrupt(rf22* const dev)
{
    uint8_t flags[2] = { 0, 0 };

    if (!dev)
    {
        return;
    }

    // Read the interrupt flags, which clears the interrupt
    rf22_spi_burst_read(dev, RF22_REG_03_INTERRUPT_STATUS1, flags, sizeof(flags));

    if ((flags[0] & RF22_IFFERROR) != 0)
    {
        rf22_reset_fifos(dev); // Clears the interrupt
        if (dev->mode == RF22_MODE_TX)
        {
            rf22_restart_transmit(dev);
        }
        else if (dev->mode == RF22_MODE_RX)
        {
            rf22_clear_rx_buf(dev);
            // Stop and restart Rx
            rf22_set_mode_idle(dev);
            rf22_set_mode_rx(dev);
        }
        else
        {
            // Nothing to recover in idle mode
        }
        // Stop handling the remaining interrupts, something went wrong here
        return;
    }

    // Caution, any delay here may cause a FIFO underflow or overflow
    if ((flags[0] & RF22_ITXFFAEM) != 0)
    {
        rf22_send_next_fragment(dev);
    }

    if ((flags[0] & RF22_IRXFFAFULL) != 0)
    {
        rf22_read_next_fragment(dev);
    }

    if ((flags[0] & RF22_IPKSENT) != 0)
    {
        dev->tx_good = (uint16_t)(dev->tx_good + 1u);
        dev->mode = RF22_MODE_IDLE;
    }

    if ((flags[0] & RF22_IPKVALID) != 0)
    {
        const uint8_t len = rf22_spi_read_reg(dev, RF22_REG_4B_RECEIVED_PACKET_LENGTH);

        // We may have already read one or more fragments. Get any remaining
        // unread octets, based on the expected length. First make sure we do
        // not overflow the buffer in the case of a stupid length or partial
        // bad receives.
        bool bad_length = (len < dev->buf_len);
#if RF22_MAX_MESSAGE_LEN < 255
        if (len > RF22_MAX_MESSAGE_LEN)
        {
            bad_length = true;
        }
#endif
        if (bad_length)
        {
            dev->rx_bad = (uint16_t)(dev->rx_bad + 1u);
            dev->mode = RF22_MODE_IDLE;
            rf22_clear_rx_buf(dev);
            return; // Receiver buffer overflow
        }

        if (len > dev->buf_len)
        {
            rf22_spi_burst_read(dev, RF22_REG_7F_FIFO_ACCESS,
                                dev->buf + dev->buf_len,
                                (uint8_t)(len - dev->buf_len));
        }
        dev->rx_good = (uint16_t)(dev->rx_good + 1u);
        dev->buf_len = len;
        dev->mode = RF22_MODE_IDLE;
        dev->rx_buf_valid = true;
    }

    if ((flags[0] & RF22_ICRCERROR) != 0)
    {
        dev->rx_bad = (uint16_t)(dev->rx_bad + 1u);
        rf22_clear_rx_buf(dev);
        rf22_reset_rx_fifo(dev);
        dev->mode = RF22_MODE_IDLE;
        rf22_set_mode_rx(dev); // Keep trying
    }

    if ((flags[1] & RF22_IPREAVAL) != 0)
    {
        dev->last_rssi = rf22_spi_read_reg(dev, RF22_REG_26_RSSI);
        rf22_clear_rx_buf(dev);
    }
}

//-----------------------------------------------------------------------------
void rf22_poll(rf22* const dev)
{
    if (!dev)
    {
        return;
    }

    // nIRQ is active low
    if (!rf22_get_irq_pin(dev))
    {
        rf22_handle_interrupt(dev);
    }
}

//-----------------------------------------------------------------------------
void rf22_clear_rx_buf(rf22* const dev)
{
    if (!dev)
    {
        return;
    }

    dev->buf_len = 0;
    dev->rx_buf_valid = false;
}

//-----------------------------------------------------------------------------
void rf22_clear_tx_buf(rf22* const dev)
{
    if (!dev)
    {
        return;
    }

    dev->buf_len = 0;
    dev->tx_buf_sent_index = 0;
}

//-----------------------------------------------------------------------------
void rf22_reset_fifos(const rf22* const dev)
{
    rf22_spi_write_reg(dev, RF22_REG_08_OPERATING_MODE2,
                       RF22_FFCLRRX | RF22_FFCLRTX);
    rf22_spi_write_reg(dev, RF22_REG_08_OPERATING_MODE2, 0);
}

//-----------------------------------------------------------------------------
void rf22_reset_rx_fifo(const rf22* const dev)
{
    rf22_spi_write_reg(dev, RF22_REG_08_OPERATING_MODE2, RF22_FFCLRRX);
    rf22_spi_write_reg(dev, RF22_REG_08_OPERATING_MODE2, 0);
}

//-----------------------------------------------------------------------------
void rf22_reset_tx_fifo(const rf22* const dev)
{
    rf22_spi_write_reg(dev, RF22_REG_08_OPERATING_MODE2, RF22_FFCLRTX);
    rf22_spi_write_reg(dev, RF22_REG_08_OPERATING_MODE2, 0);
}

//-----------------------------------------------------------------------------
static bool rf22_append_tx_buf(rf22* const dev, const uint8_t* data, uint8_t len)
{
    if (((uint16_t)dev->buf_len + (uint16_t)len) > RF22_MAX_MESSAGE_LEN)
    {
        return false;
    }

    memcpy(dev->buf + dev->buf_len, data, (size_t)len);
    dev->buf_len = (uint8_t)(dev->buf_len + len);

    return true;
}

//-----------------------------------------------------------------------------
static bool rf22_fill_tx_buf(rf22* const dev, const uint8_t* data, uint8_t len)
{
    rf22_clear_tx_buf(dev);

    if (len == 0)
    {
        return false;
    }

    return rf22_append_tx_buf(dev, data, len);
}

//-----------------------------------------------------------------------------
// Assumption: there is currently <= RF22_TXFFAEM_THRESHOLD octets in the Tx FIFO
static void rf22_send_next_fragment(rf22* const dev)
{
    if (dev->tx_buf_sent_index < dev->buf_len)
    {
        uint8_t len = (uint8_t)(dev->buf_len - dev->tx_buf_sent_index);

        // But do not send too much
        if (len > (RF22_FIFO_SIZE - RF22_TXFFAEM_THRESHOLD - 1))
        {
            len = (uint8_t)(RF22_FIFO_SIZE - RF22_TXFFAEM_THRESHOLD - 1);
        }
        rf22_spi_burst_write(dev, RF22_REG_7F_FIFO_ACCESS,
                             dev->buf + dev->tx_buf_sent_index, len);
        dev->tx_buf_sent_index = (uint8_t)(dev->tx_buf_sent_index + len);
    }
}

//-----------------------------------------------------------------------------
// Assumption: there are at least RF22_RXFFAFULL_THRESHOLD octets in the Rx FIFO.
// That means it should only be called after a RXFFAFULL interrupt.
static void rf22_read_next_fragment(rf22* const dev)
{
    if (((uint16_t)dev->buf_len + RF22_RXFFAFULL_THRESHOLD) > RF22_MAX_MESSAGE_LEN)
    {
        return; // Receiver overflow, should never occur
    }

    rf22_spi_burst_read(dev, RF22_REG_7F_FIFO_ACCESS, dev->buf + dev->buf_len,
                        RF22_RXFFAFULL_THRESHOLD);
    dev->buf_len = (uint8_t)(dev->buf_len + RF22_RXFFAFULL_THRESHOLD);
}

//-----------------------------------------------------------------------------
static void rf22_start_transmit(rf22* const dev)
{
    rf22_send_next_fragment(dev); // Actually the first fragment
    // Total length that will be sent
    rf22_spi_write_reg(dev, RF22_REG_3E_PACKET_LENGTH, dev->buf_len);
    rf22_set_mode_tx(dev); // Start the transmitter, turns off the receiver
}

//-----------------------------------------------------------------------------
// Restart the transmission of a packet that had a problem
static void rf22_restart_transmit(rf22* const dev)
{
    dev->mode = RF22_MODE_IDLE;
    dev->tx_buf_sent_index = 0;
    rf22_start_transmit(dev);
}
