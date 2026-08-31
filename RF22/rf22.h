// rf22.h
//
// Portable C driver for the HopeRF RFM22B / RFM23B (RF22B) ISM transceiver.
//
// Derived from the RF22 library:
//   Author: Mike McCauley (mikem@open.com.au)
//   Copyright (C) 2011 Mike McCauley
//   $Id: RF22.h,v 1.23 2013/02/06 21:33:56 mikem Exp mikem $
//
//   ported to mbed by Karl Zweimueller
//   modified by Lukasz Uszko luszko@op.pl
//
// This software is Copyright (C) 2011 Mike McCauley. Use is subject to license
// conditions. The main licensing options available are GPL V2 or Commercial.
// See LICENSE.txt for the full statement. This file is NOT MIT licensed.
//
// Restructured for the DevLibs portable-core/platform layout by luk6xff, 2020.

#ifndef __RF22_H__
#define __RF22_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "registers.h"

/**
 * @brief Maximum message length supported by this library.
 *
 * Limited by the single message length octet in the header. 255 is correct
 * even though the FIFO size in the RF22 is only 64 octets: the FIFO is refilled
 * during transmission and drained during reception.
 * Can be pre-defined to a smaller size (to save RAM) prior to including this
 * header.
 */
#ifndef RF22_MAX_MESSAGE_LEN
#define RF22_MAX_MESSAGE_LEN        255
#endif

/** Max number of octets the RF22 Rx and Tx FIFOs can hold */
#define RF22_FIFO_SIZE              64

/** These values we set for FIFO thresholds are the same as the POR values */
#define RF22_TXFFAEM_THRESHOLD      4
#define RF22_RXFFAFULL_THRESHOLD    55

/** This is the default node address */
#define RF22_DEFAULT_NODE_ADDRESS   0

/** This address in the TO address signifies a broadcast */
#define RF22_BROADCAST_ADDRESS      0xff

/** Number of registers passed to rf22_set_modem_registers() */
#define RF22_NUM_MODEM_CONFIG_REGS  18

/** Default bound for rf22_send() waiting on a previous transmission [ms] */
#define RF22_TX_TIMEOUT_MS          3000

/** Bound for the on-chip ADC conversion (nominally 305 us) [ms] */
#define RF22_ADC_TIMEOUT_MS         100

/**
 * @brief Mode the driver believes the RF22 is in
 */
typedef enum
{
    RF22_MODE_IDLE = 0,
    RF22_MODE_RX   = 1,
    RF22_MODE_TX   = 2,
} rf22_mode;

/**
 * @brief Register values for a set of modem configuration registers.
 *
 * Suitable values can be computed with the HopeRF register calculator:
 * http://www.hoperf.com/upload/rf/RF22B%2023B%2031B%2042B%2043B%20Register%20Settings_RevB1-v5.xls
 *
 * @note The fields must stay contiguous and in this order, they are written to
 *       the device with burst writes.
 */
typedef struct
{
    uint8_t reg_1c; // RF22_REG_1C_IF_FILTER_BANDWIDTH
    uint8_t reg_1f; // RF22_REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE
    uint8_t reg_20; // RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE
    uint8_t reg_21; // RF22_REG_21_CLOCK_RECOVERY_OFFSET2
    uint8_t reg_22; // RF22_REG_22_CLOCK_RECOVERY_OFFSET1
    uint8_t reg_23; // RF22_REG_23_CLOCK_RECOVERY_OFFSET0
    uint8_t reg_24; // RF22_REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1
    uint8_t reg_25; // RF22_REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0
    uint8_t reg_2c; // RF22_REG_2C_OOK_COUNTER_VALUE_1
    uint8_t reg_2d; // RF22_REG_2D_OOK_COUNTER_VALUE_2
    uint8_t reg_2e; // RF22_REG_2E_SLICER_PEAK_HOLD
    uint8_t reg_58; // RF22_REG_58_CHARGE_PUMP_CURRENT_TRIMMING
    uint8_t reg_69; // RF22_REG_69_AGC_OVERRIDE1
    uint8_t reg_6e; // RF22_REG_6E_TX_DATA_RATE1
    uint8_t reg_6f; // RF22_REG_6F_TX_DATA_RATE0
    uint8_t reg_70; // RF22_REG_70_MODULATION_CONTROL1
    uint8_t reg_71; // RF22_REG_71_MODULATION_CONTROL2
    uint8_t reg_72; // RF22_REG_72_FREQUENCY_DEVIATION
} rf22_modem_config;

/**
 * @brief Predefined modem configurations, indexes into the canned table.
 */
typedef enum
{
    RF22_UNMODULATED_CARRIER = 0, // Unmodulated carrier for testing
    RF22_FSK_PN9_RB2FD5,          // FSK, Rb = 2kbs, Fd = 5kHz, PN9 for testing

    RF22_FSK_RB2FD5,              // FSK,  Rb = 2kbs,    Fd = 5kHz
    RF22_FSK_RB2_4FD36,           // FSK,  Rb = 2.4kbs,  Fd = 36kHz
    RF22_FSK_RB4_8FD45,           // FSK,  Rb = 4.8kbs,  Fd = 45kHz
    RF22_FSK_RB9_6FD45,           // FSK,  Rb = 9.6kbs,  Fd = 45kHz
    RF22_FSK_RB19_2FD9_6,         // FSK,  Rb = 19.2kbs, Fd = 9.6kHz
    RF22_FSK_RB38_4FD19_6,        // FSK,  Rb = 38.4kbs, Fd = 19.6kHz
    RF22_FSK_RB57_6FD28_8,        // FSK,  Rb = 57.6kbs, Fd = 28.8kHz
    RF22_FSK_RB125FD125,          // FSK,  Rb = 125kbs,  Fd = 125kHz

    RF22_GFSK_RB2FD5,             // GFSK, Rb = 2kbs,    Fd = 5kHz
    RF22_GFSK_RB2_4FD36,          // GFSK, Rb = 2.4kbs,  Fd = 36kHz
    RF22_GFSK_RB4_8FD45,          // GFSK, Rb = 4.8kbs,  Fd = 45kHz
    RF22_GFSK_RB9_6FD45,          // GFSK, Rb = 9.6kbs,  Fd = 45kHz
    RF22_GFSK_RB19_2FD9_6,        // GFSK, Rb = 19.2kbs, Fd = 9.6kHz
    RF22_GFSK_RB38_4FD19_6,       // GFSK, Rb = 38.4kbs, Fd = 19.6kHz
    RF22_GFSK_RB57_6FD28_8,       // GFSK, Rb = 57.6kbs, Fd = 28.8kHz
    RF22_GFSK_RB125FD125,         // GFSK, Rb = 125kbs,  Fd = 125kHz

    RF22_OOK_RB1_2BW75,           // OOK,  Rb = 1.2kbs,  Rx Bandwidth = 75kHz
    RF22_OOK_RB2_4BW335,          // OOK,  Rb = 2.4kbs,  Rx Bandwidth = 335kHz
    RF22_OOK_RB4_8BW335,          // OOK,  Rb = 4.8kbs,  Rx Bandwidth = 335kHz
    RF22_OOK_RB9_6BW335,          // OOK,  Rb = 9.6kbs,  Rx Bandwidth = 335kHz
    RF22_OOK_RB19_2BW335,         // OOK,  Rb = 19.2kbs, Rx Bandwidth = 335kHz
    RF22_OOK_RB38_4BW335,         // OOK,  Rb = 38.4kbs, Rx Bandwidth = 335kHz
    RF22_OOK_RB40BW335,           // OOK,  Rb = 40kbs,   Rx Bandwidth = 335kHz

    RF22_MODEM_CONFIG_CHOICE_NUM  // Number of canned configurations
} rf22_modem_config_choice;

/**
 * @brief rf22 dev object, holds the whole driver state.
 *
 * Nothing is allocated dynamically, the shared Rx/Tx buffer lives here.
 */
typedef struct
{
    uint8_t  idle_mode;              // Register mask used when going idle
    uint8_t  device_type;            // Value read from RF22_REG_00_DEVICE_TYPE
    volatile rf22_mode mode;         // Mode the driver believes we are in
    volatile uint8_t buf_len;        // Valid octets in buf
    volatile uint8_t tx_buf_sent_index; // Octets of buf already pushed to FIFO
    volatile bool    rx_buf_valid;   // A complete message is waiting in buf
    volatile uint8_t last_rssi;      // RSSI of the last received preamble
    volatile uint16_t rx_bad;        // Count of bad receptions
    volatile uint16_t rx_good;       // Count of good receptions
    volatile uint16_t tx_good;       // Count of completed transmissions
    uint8_t  buf[RF22_MAX_MESSAGE_LEN]; // Shared Rx/Tx message buffer
    void*    platform_dev;           // Platform descriptor, set by rf22_<platform>_init()
} rf22;


//-----------------------------------------------------------------------------
// @brief Initialisation and reset
//-----------------------------------------------------------------------------

/**
 * @brief Initialise the driver and the radio module connected to it.
 *
 * Powers the module up, software resets it, verifies the device type register,
 * configures the standard packet format, sets the frequency to 434.0 MHz and
 * the modem to RF22_FSK_RB125FD125.
 *
 * @param dev rf22 device object, dev->platform_dev must already be set
 * @return True on success, false otherwise
 */
bool rf22_init(rf22* const dev);

/**
 * @brief Issue a software reset to the RF22. Blocks for 1 ms.
 *
 * @param dev rf22 device object
 */
void rf22_reset(const rf22* const dev);


//-----------------------------------------------------------------------------
// @brief Raw register access
//-----------------------------------------------------------------------------

/**
 * @brief Read a single register.
 *
 * @param dev rf22 device object
 * @param reg Register number, one of RF22_REG_*
 * @return The register value, 0 if dev is invalid
 */
uint8_t rf22_spi_read_reg(const rf22* const dev, uint8_t reg);

/**
 * @brief Write a single register.
 *
 * @param dev rf22 device object
 * @param reg Register number, one of RF22_REG_*
 * @param val Value to write
 */
void rf22_spi_write_reg(const rf22* const dev, uint8_t reg, uint8_t val);

/**
 * @brief Read a number of consecutive registers using burst read mode.
 *
 * @param      dev  rf22 device object
 * @param      reg  Register number of the first register, one of RF22_REG_*
 * @param[out] dest Buffer to fill, must be at least len octets
 * @param      len  Number of octets to read
 */
void rf22_spi_burst_read(const rf22* const dev, uint8_t reg, uint8_t* dest,
                         uint8_t len);

/**
 * @brief Write a number of consecutive registers using burst write mode.
 *
 * @param dev rf22 device object
 * @param reg Register number of the first register, one of RF22_REG_*
 * @param src Values to write, must be at least len octets
 * @param len Number of octets to write
 */
void rf22_spi_burst_write(const rf22* const dev, uint8_t reg,
                          const uint8_t* src, uint8_t len);


//-----------------------------------------------------------------------------
// @brief Device status and measurements
//-----------------------------------------------------------------------------

/**
 * @brief Read RF22_REG_02_DEVICE_STATUS.
 *
 * @param dev rf22 device object
 * @return The device status register value
 */
uint8_t rf22_status_read(const rf22* const dev);

/**
 * @brief Read RF22_REG_31_EZMAC_STATUS.
 *
 * @param dev rf22 device object
 * @return The EZMAC status register value
 */
uint8_t rf22_ezmac_status_read(const rf22* const dev);

/**
 * @brief Read the current RSSI from RF22_REG_26_RSSI.
 *
 * @param dev rf22 device object
 * @return The current RSSI value
 */
uint8_t rf22_rssi_read(const rf22* const dev);

/**
 * @brief Return the RSSI captured while the last message was being received.
 *
 * @param dev rf22 device object
 * @return The RSSI of the last received message
 */
uint8_t rf22_last_rssi(const rf22* const dev);

/**
 * @brief Read a value from the on-chip analog to digital converter.
 *
 * @param      dev     rf22 device object
 * @param      adcsel  ADC input selection, one of RF22_ADCSEL_*
 * @param      adcref  Reference voltage selection, one of RF22_ADCREF_*
 * @param      adcgain Amplifier gain selection
 * @param      adcoffs Amplifier offset (0 to 15)
 * @param[out] value   The analog value, 0 to 255
 * @return True on success, false on timeout or invalid arguments
 */
bool rf22_adc_read(const rf22* const dev, uint8_t adcsel, uint8_t adcref,
                   uint8_t adcgain, uint8_t adcoffs, uint8_t* value);

/**
 * @brief Read the on-chip temperature sensor.
 *
 * @param      dev     rf22 device object
 * @param      tsrange Temperature range selection, one of RF22_TSRANGE_*
 * @param      tvoffs  Signed offset added to the measured temperature value
 * @param[out] value   The measured temperature
 * @return True on success, false on timeout or invalid arguments
 */
bool rf22_temperature_read(const rf22* const dev, uint8_t tsrange,
                           uint8_t tvoffs, uint8_t* value);

/**
 * @brief Read the wakeup timer value.
 *
 * @param dev rf22 device object
 * @return The wakeup timer value
 */
uint16_t rf22_wut_read(const rf22* const dev);

/**
 * @brief Set the wakeup timer period registers.
 *
 * @param dev rf22 device object
 * @param wtm Wakeup timer mantissa value
 * @param wtr Wakeup timer exponent R value
 * @param wtd Wakeup timer exponent D value
 */
void rf22_set_wut_period(const rf22* const dev, uint16_t wtm, uint8_t wtr,
                         uint8_t wtd);


//-----------------------------------------------------------------------------
// @brief Radio configuration
//-----------------------------------------------------------------------------

/**
 * @brief Set the transmitter and receiver centre frequency.
 *
 * @param dev              rf22 device object
 * @param centre_mhz       Frequency in MHz, 240.0 to 960.0
 * @param afc_pull_in_mhz  AFC pull in range in MHz. 0.0 to 0.159375 below
 *                         480 MHz, 0.0 to 0.318750 above
 * @return True if the requested frequency and pull in range are in range
 */
bool rf22_set_frequency(const rf22* const dev, float centre_mhz,
                        float afc_pull_in_mhz);

/**
 * @brief Set the frequency hopping step size in 10 kHz increments.
 *
 * @param dev rf22 device object
 * @param fhs Frequency hopping step size
 * @return True if centre + (fhch * fhs) is within limits
 */
bool rf22_set_fh_step_size(const rf22* const dev, uint8_t fhs);

/**
 * @brief Set the frequency hopping channel.
 *
 * @param dev  rf22 device object
 * @param fhch The channel number
 * @return True if centre + (fhch * fhs) is within limits
 */
bool rf22_set_fh_channel(const rf22* const dev, uint8_t fhch);

/**
 * @brief Write RF22_REG_07_OPERATING_MODE1 directly.
 *
 * @param dev  rf22 device object
 * @param mode Mask of RF22_SWRES, RF22_ENLBD, RF22_ENWT, RF22_X32KSEL,
 *             RF22_TXON, RF22_RXON, RF22_PLLON, RF22_XTON
 */
void rf22_set_operating_mode(const rf22* const dev, uint8_t mode);

/**
 * @brief Go to idle mode, stopping the receiver or transmitter.
 *
 * @param dev rf22 device object
 */
void rf22_set_mode_idle(rf22* const dev);

/**
 * @brief Start the receiver.
 *
 * @param dev rf22 device object
 */
void rf22_set_mode_rx(rf22* const dev);

/**
 * @brief Start the transmitter.
 *
 * @param dev rf22 device object
 */
void rf22_set_mode_tx(rf22* const dev);

/**
 * @brief Return the mode the driver believes the radio is in.
 *
 * @param dev rf22 device object
 * @return One of RF22_MODE_*
 */
rf22_mode rf22_get_mode(const rf22* const dev);

/**
 * @brief Set the transmitter power output level.
 *
 * After rf22_init() the power is set to RF22_TXPOW_8DBM.
 *
 * @param dev   rf22 device object
 * @param power Transmitter power level, one of RF22_TXPOW_*
 */
void rf22_set_tx_power(const rf22* const dev, uint8_t power);

/**
 * @brief Write a full custom modem configuration.
 *
 * @param dev    rf22 device object
 * @param config Modem configuration register values
 */
void rf22_set_modem_registers(const rf22* const dev,
                              const rf22_modem_config* const config);

/**
 * @brief Select one of the predefined modem configurations.
 *
 * @param dev   rf22 device object
 * @param index The configuration choice
 * @return True if index is a valid choice
 */
bool rf22_set_modem_config(const rf22* const dev,
                           rf22_modem_config_choice index);

/**
 * @brief Set the preamble length in 4-bit nibbles. Default is 8.
 *
 * @param dev     rf22 device object
 * @param nibbles Preamble length in nibbles of 4 bits each
 */
void rf22_set_preamble_length(const rf22* const dev, uint8_t nibbles);

/**
 * @brief Set the sync words for transmit and receive. Default is {0x2d, 0xd4}.
 *
 * @param dev        rf22 device object
 * @param sync_words Array of sync words
 * @param len        Number of sync words, 1 to 4
 */
void rf22_set_sync_words(const rf22* const dev, const uint8_t* sync_words,
                         uint8_t len);

/**
 * @brief Accept messages with any TO address.
 *
 * @param dev         rf22 device object
 * @param promiscuous True to receive messages with any TO address
 */
void rf22_set_promiscuous(const rf22* const dev, bool promiscuous);


//-----------------------------------------------------------------------------
// @brief Sending and receiving
//-----------------------------------------------------------------------------

/**
 * @brief Start the receiver and check whether a received message is waiting.
 *
 * Polls the IRQ pin first, so no interrupts are required.
 *
 * @param dev rf22 device object
 * @return True if a complete valid message can be retrieved with rf22_recv()
 */
bool rf22_available(rf22* const dev);

/**
 * @brief Start the receiver and block until a message arrives or time runs out.
 *
 * @param dev        rf22 device object
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return True if a message is available
 */
bool rf22_wait_available(rf22* const dev, uint32_t timeout_ms);

/**
 * @brief Copy a received message out of the driver buffer.
 *
 * @param      dev rf22 device object
 * @param[out] buf Location to copy the received message to
 * @param[in,out] len On entry the space in buf, on exit the octets copied
 * @return True if a valid message was copied to buf
 */
bool rf22_recv(rf22* const dev, uint8_t* buf, uint8_t* len);

/**
 * @brief Load a message into the transmitter and start transmitting.
 *
 * Waits for any previous transmission to complete first. A length of 0 is not
 * permitted, it confuses the RF22.
 *
 * @param dev  rf22 device object
 * @param data Data to send
 * @param len  Number of octets to send, 1 to RF22_MAX_MESSAGE_LEN
 * @return True if the message was queued for transmission
 */
bool rf22_send(rf22* const dev, const uint8_t* data, uint8_t len);

/**
 * @brief Block until the current transmission finishes or time runs out.
 *
 * @param dev        rf22 device object
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return True if the radio left RF22_MODE_TX before the timeout
 */
bool rf22_wait_packet_sent(rf22* const dev, uint32_t timeout_ms);


//-----------------------------------------------------------------------------
// @brief Message headers
//-----------------------------------------------------------------------------

/**
 * @brief Set the TO header sent in all subsequent messages.
 *
 * @param dev rf22 device object
 * @param to  The new TO header value
 */
void rf22_set_header_to(const rf22* const dev, uint8_t to);

/**
 * @brief Set the FROM header sent in all subsequent messages.
 *
 * @param dev  rf22 device object
 * @param from The new FROM header value
 */
void rf22_set_header_from(const rf22* const dev, uint8_t from);

/**
 * @brief Set the ID header sent in all subsequent messages.
 *
 * @param dev rf22 device object
 * @param id  The new ID header value
 */
void rf22_set_header_id(const rf22* const dev, uint8_t id);

/**
 * @brief Set the FLAGS header sent in all subsequent messages.
 *
 * @param dev   rf22 device object
 * @param flags The new FLAGS header value
 */
void rf22_set_header_flags(const rf22* const dev, uint8_t flags);

/**
 * @brief Return the TO header of the last received message.
 *
 * @param dev rf22 device object
 * @return The TO header
 */
uint8_t rf22_header_to(const rf22* const dev);

/**
 * @brief Return the FROM header of the last received message.
 *
 * @param dev rf22 device object
 * @return The FROM header
 */
uint8_t rf22_header_from(const rf22* const dev);

/**
 * @brief Return the ID header of the last received message.
 *
 * @param dev rf22 device object
 * @return The ID header
 */
uint8_t rf22_header_id(const rf22* const dev);

/**
 * @brief Return the FLAGS header of the last received message.
 *
 * @param dev rf22 device object
 * @return The FLAGS header
 */
uint8_t rf22_header_flags(const rf22* const dev);


//-----------------------------------------------------------------------------
// @brief Interrupt handling and buffers
//-----------------------------------------------------------------------------

/**
 * @brief Service the RF22.
 *
 * Call this from your nIRQ interrupt service routine, or let rf22_poll() call
 * it for you. Do not mix both approaches on the same device object.
 *
 * @param dev rf22 device object
 */
void rf22_handle_interrupt(rf22* const dev);

/**
 * @brief Call rf22_handle_interrupt() if the nIRQ pin is asserted.
 *
 * nIRQ is active low. Call this often from your main loop when you do not want
 * to use hardware interrupts.
 *
 * @param dev rf22 device object
 */
void rf22_poll(rf22* const dev);

/**
 * @brief Discard the contents of the receive buffer.
 *
 * @param dev rf22 device object
 */
void rf22_clear_rx_buf(rf22* const dev);

/**
 * @brief Discard the contents of the transmit buffer.
 *
 * @param dev rf22 device object
 */
void rf22_clear_tx_buf(rf22* const dev);

/**
 * @brief Clear both RF22 FIFOs.
 *
 * @param dev rf22 device object
 */
void rf22_reset_fifos(const rf22* const dev);

/**
 * @brief Clear the RF22 Rx FIFO.
 *
 * @param dev rf22 device object
 */
void rf22_reset_rx_fifo(const rf22* const dev);

/**
 * @brief Clear the RF22 Tx FIFO.
 *
 * @param dev rf22 device object
 */
void rf22_reset_tx_fifo(const rf22* const dev);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Full duplex SPI transfer of len octets, mode 0, MSB first.
 *
 * The chip select line is driven separately with rf22_set_cs_pin(), so several
 * transfers can share one chip select assertion.
 *
 * @param      dev rf22 device object
 * @param      tx  Octets to send, NULL sends len zero octets
 * @param[out] rx  Buffer for the received octets, NULL discards them
 * @param      len Number of octets to transfer
 * @return True on success, false otherwise
 */
extern bool rf22_spi_transfer(const rf22* const dev, const uint8_t* tx,
                              uint8_t* rx, size_t len);

/**
 * @brief Drive the nSEL (chip select) pin.
 *
 * @param dev   rf22 device object
 * @param state True drives the pin high (deselected), false drives it low
 */
extern void rf22_set_cs_pin(const rf22* const dev, bool state);

/**
 * @brief Drive the SDN (shutdown) pin.
 *
 * SDN must be low in all modes except shutdown. Implementations may ignore this
 * call when the pin is tied low in hardware.
 *
 * @param dev   rf22 device object
 * @param state True drives the pin high (shutdown), false drives it low
 */
extern void rf22_set_sdn_pin(const rf22* const dev, bool state);

/**
 * @brief Read the nIRQ pin.
 *
 * @param dev rf22 device object
 * @return The raw pin level. nIRQ is active low, so false means asserted
 */
extern bool rf22_get_irq_pin(const rf22* const dev);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void rf22_delay_ms(uint32_t delay_ms);

/**
 * @brief Return a free running millisecond counter.
 *
 * Used to bound every wait loop in the driver. Wrap around is handled.
 *
 * @return Milliseconds since an arbitrary origin
 */
extern uint32_t rf22_get_ms_ticks(void);


#ifdef __cplusplus
}
#endif

#endif // __RF22_H__
