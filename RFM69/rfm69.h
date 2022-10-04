/**
 *  @brief:  Implementation of a RFM69 radio driver
 *           Supported devices: HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
 *           Code inspired by LowPowerLab library: https://github.com/LowPowerLab/RFM69
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */

#ifndef __RFM69_H__
#define __RFM69_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "registers.h"

/**
 * ============================================================================
 * @brief Public defines
 * ============================================================================
 */

#define RFM69_FIFO_SIZE         66 // Max number of bytes the RFM69 Rx and Tx FIFOs can store
#define RFM69_HEADER_LEN        4
#define RFM69_MAX_DATA_LEN      (RFM69_FIFO_SIZE - RFM69_HEADER_LEN) // To take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 5 bytes header overhead)

#define RFM69_CSMA_LIMIT        -85 // Upper RX signal sensitivity threshold in dBm for carrier sense access

// Available frequency bands
#define RFM69_315MHZ            31
#define RFM69_433MHZ            43
#define RFM69_868MHZ            86
#define RFM69_915MHZ            91

#define RFM69_COURSE_TEMP_COEF  -90 // Puts the temperature reading in the ballpark, user can fine tune the returned value
#define RFM69_BROADCAST_ADDR    0
#define RFM69_CSMA_LIMIT_MS     1000
#define RFM69_TX_LIMIT_MS       1000
#define RFM69_FXOSC             32000000.0
#define RFM69_FSTEP             61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// CTLbyte bits
#define RFM69_CTL_SENDACK       0x80
#define RFM69_CTL_REQACK        0x40
#define RFM69_CTL_REQACK_RSSI   0x20
#define RFM69_ACK_TIMEOUT       30 // 30ms roundtrip req for 61byte packets

// AES KEY LENGTH
#define RFM69_AES_KEY_LEN       16

/**
 * @brief RFM69 other global defines
 */
#define RFM69_PWR_ATC_RSSI_HIST               5
#define RFM69HW_MAX_POWER_LEVEL_DBM           20
#define RFM69HW_MIN_POWER_LEVEL_DBM           -2
#define RFM69W_MAX_POWER_LEVEL_DBM            13
#define RFM69W_MIN_POWER_LEVEL_DBM           -18


/**
 * @brief RFM69 default device setup
 */
#define RFM69_DEFAULT_NODE_ID               0x99
#define RFM69_DEFAULT_GATEWAY_ID            0x01
#define RFM69_DEFAULT_NETWORKID             0xFF
#define RFM69_DEFAULT_POWER_LEVEL_DBM       RFM69HW_MIN_POWER_LEVEL_DBM
#define RFM69_DEFAULT_IS_RFM69HW            true
#define RFM69_DEFAULT_FREQUENCY             RFM69_433MHZ
#define RFM69_DEFAULT_PROMISCUOUS_MODE      false
#define RFM69_DEFAULT_TRANSMIT_PERIOD       2000 // [ms]
#define RFM69_DEFAULT_ACKTIMEOUT            999  // [ms]
#define RFM69_DEFAULT_SEND_RETRIES_NUM      3
#define RFM69_DEFAULT_MAX_PAYLOAD_LENGTH    RFM69_MAX_DATA_LEN
#define RFM69_DEFAULT_ENCRYPT_ENABLE        false
#define RFM69_DEFAULT_ENCRYPTKEY            "abcdefghijklmnop" // Has to be same 16 characters/uint8_ts on gateway
#define RFM69_DEFAULT_PWR_ATC_TARGET_RSSI   -55


/**
 * ============================================================================
 * @brief Public datatypes declarations
 * ============================================================================
 */


/**
 * @brief RFM69 Device modes
 */
typedef enum
{
    RFM69_MODE_SLEEP   = 0,  // XTAL OFF
    RFM69_MODE_STANDBY = 1,  // XTAL ON
    RFM69_MODE_SYNTH   = 2,  // PLL ON
    RFM69_MODE_RX      = 3,  // RX MODE
    RFM69_MODE_TX      = 4,  // TX MODE
} rfm69_device_mode;

/**
 * @brief RFM69 Radio settings container
 */
typedef struct
{
    uint8_t  node_id;       // rfm node address
    uint8_t  network_id;    // rfm network address
    uint8_t  freq_band;
    int8_t   power_level_dbm;
    bool     promiscuous_mode;
    bool     is_rfm69hw;
    uint8_t  aes_encrypt_enable;
    uint8_t  aes_encrypt_key[RFM69_AES_KEY_LEN];
} rfm69_radio_settings;


/**
 * @brief RFM69 Power auto control container
 */
typedef struct
{
    bool enabled;
    int16_t target_rssi;            // If non-zero then this is the desired end point RSSI for our transmission
    uint8_t transmit_pwr_level;     // Saved power level in case we do auto power adjustment, this value gets dithered
    int8_t transmit_pwr_level_dbm;  // Saved power level dbm in case we do auto power adjustment, this value gets dithered
    int16_t ack_rssi;               // The RSSI our destination Ack'd back to us (if we enabled auto power control)
    bool ack_rssi_requested;        // Is ack_rssi requested by the sender
} rfm69_power_atc;

/**
 * @brief RFM69 Container describing received/sent data payload
 */
typedef struct
{
    uint8_t  data[RFM69_MAX_DATA_LEN+1]; // RX/TX payload buffer, including end of string NULL char
    uint8_t  data_len;
    uint16_t sender_id;
    uint16_t target_id;    // should match _address
    uint8_t  payload_len;
    uint8_t  ack_requested;
    uint8_t  ack_received; // should be polled immediately after sending a packet with ACK request
    int16_t  rssi;         // most accurate RSSI during reception (closest to the reception). RSSI of last packet.
} rfm69_payload_data;

/** RFM69 dev object **/
typedef struct
{
    rfm69_radio_settings settings;
    rfm69_payload_data payload_data;
    rfm69_device_mode mode;
    rfm69_power_atc pwr_atc;
    volatile bool have_data;
    void (*dio0_irq)(void*);    // dio0 pin rise callback
    void (*on_receive)(void*);  // On receive user callback
    void *platform_dev;         // Platform dependent data
} rfm69;

/**
 * ============================================================================
 * @brief Public functions prototypes
 * ============================================================================
 */

/**
 * @brief Initializes the radio
 *
 * @param[in] events Structure containing the driver callback functions
 */
bool rfm69_init(rfm69 *const dev);

/**
 * @brief Register on_receive callback
 */
void rfm69_register_on_receive(rfm69 *const dev, void(*callback)(void*));

/**
 * @brief Get the frequency (in Hz)
 *
 * @return Module frequency in Hz
 */
uint32_t rfm69_get_frequency(rfm69 *const dev);

/**
 * @brief Set the frequency (in Hz)
 *
 * @param[in] freq_hz - frequency in Hz
 */
void rfm69_set_frequency(rfm69 *const dev, uint32_t freq_hz);

/**
 * @brief Set new device mode
 *
 * @param[in] mode - available modes:
 *            RFM69_MODE_SLEEP, RFM69_MODE_STANDBY, RFM69_MODE_SYNTH,
 *            RFM69_MODE_RX, RFM69_MODE_TX
 */
void rfm69_set_mode(rfm69 *const dev, rfm69_device_mode mode);

/**
 * @brief Put device transceiver in sleep mode to save battery
 *
 * @note To wake or resume receiving just call rfm69_receive_done()
 */
void rfm69_sleep(rfm69 *const dev);

/**
 * @brief Set address of the device
 *
 * @param[in] addr - device node addr
 * @note Unused in packet mode
 */
void rfm69_set_address(rfm69 *const dev, uint8_t addr);

/**
 * @brief Set this node's network id
 * @param network_id network id address
 */
void rfm69_set_network(rfm69 *const dev, uint8_t network_id);

/**
 * @brief Set *transmit/TX* output power: 0=min, 31=max
 *        this results in a "weaker" transmitted signal, and directly results in a lower RSSI at the receiver
 *        the power configurations are explained in the SX1231H datasheet (Table 10 on p21; RegPaLevel p66): http://www.semtech.com/images/datasheet/sx1231h.pdf
 *        valid powerLevel parameter values are 0-31 and result in a directly proportional effect on the output/transmission power
 *        this function implements 2 modes as follows:
 *        - for RFM69W the range is from 0-31 [-18dBm to 13dBm] (PA0 only on RFIO pin)
 *        - for RFM69HW the range is from 0-31 [5dBm to 20dBm]  (PA1 & PA2 on PA_BOOST pin & high Power PA settings - see section 3.3.7 in datasheet, p22)
 */
void rfm69_set_power_level(rfm69 *const dev, uint8_t level); // reduce/increase transmit power level

/**
 * @brief Set the output power level in dBm.
 *        This function takes care of the different PA settings of the modules.
 *        Depending on the requested power output setting and the available module,
 *        PA0, PA1 or PA1+PA2 is enabled.
 *
 * @param dBm Output power in dBm
 * @return true on success, false otherwise.
 */
bool rfm69_set_power_level_dbm(rfm69 *const dev, int8_t dBm);

/**
 * @brief Set the bitrate in bits per second.
 *        After calling this function, the module is in standby mode.
 *
 * @param[in] bitrate Bitrate in bits per second
 */
void rfm69_set_bitrate(rfm69 *const dev, uint32_t bitrate);

/**
 * @brief Check if a dataframe can be send to the receiver node.
 *        During the test, there are checked following conditions:
 *        current device mode, payload len, and current channel activity (rssi)
 */
bool rfm69_can_send(rfm69 *const dev);

/**
 * @brief Send a frame to the receiver node.
 *
 * @param[in] to_address         Receiver node address
 * @param[in] buffer             Data to be sent
 * @param[in] buffer_size        Length of data buffer to be sent
 * @param[in] request_ack        Ask for ACK from receiver or not
 */
void rfm69_send(rfm69 *const dev, uint16_t to_address, const void* buffer,
                uint8_t buffer_size, bool request_ack);

/**
 * @brief Send a frame with retires number defines and wait for response for some time.
 *
 * @param[in] to_address         Receiver node address
 * @param[in] buffer             Data to be sent
 * @param[in] buffer_size        Length of data buffer to be sent
 * @param[in] retries            Retries number
 * @param[in] retry_wait_time_ms Retries wait time in [ms]
 */
bool rfm69_send_with_retry(rfm69 *const dev, uint16_t to_address,
                            const void* buffer, uint8_t buffer_size,
                            uint8_t retries, uint32_t retry_wait_time_ms);

/**
 * @brief Check if rx data is available ad switch to RX mode
 */
void rfm69_receive_begin(rfm69 *const dev);

/**
 * @brief Check if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
 * @retval Is data received
 */
bool rfm69_receive_done(rfm69 *const dev);

/**
 * @brief Should be polled immediately after RFM69 sending a packet with ACK request
 * @param[in] from_node_id  Node addres which ack came from
 */
bool rfm69_ack_received(rfm69 *const dev, uint16_t from_node_id);

/**
 * @brief Check whether an ACK was requested in the last received packet (non-broadcasted packet)
 * @retval Was ack requested
 */
bool rfm69_ack_requested(rfm69 *const dev);

/**
 * @brief Should be called immediately after reception in case RFM69Sender wants ACK
 */
void rfm69_send_ack(rfm69 *const dev);

/**
 * @brief Return received payload data from the fifo
 * @retval Received payload data pointer
 */
const rfm69_payload_data* rfm69_received_data(rfm69 *const dev);

/**
 * @brief Enables AES key encryption
 *
 * @param[in] key AES key
 * @param[in] key_length AES key length - @note It must be 16 bytes long!
 */
void rfm69_aes_encryption(rfm69 *const dev, const char* key, uint8_t key_length);

/**
 * @brief Read the received signal strength indicator (RSSI)
 * @note *current* signal strength indicator; e.g. RSSI < -90dBm says the frequency channel is free + ready to transmit
 *
 * @param[in] force_trigger
 *
 * @return  A current RSSI value
 */
int16_t rfm69_read_rssi(rfm69 *const dev, bool force_trigger);

/**
 * @brief Enables/Disables promiscous mode
 *
 * @param[in] enable - true: disable filtering to capture all frames on network
 * 					   false: enable node/broadcast filtering to capture only frames sent to this/broadcast address
 */
void rfm69_set_promiscuous_mode(rfm69 *const dev, bool enable);

/**
 * @brief Sets high power on RFM69 module
 * @note  For RFM69HW only: you must call rfm69_set_high_power(true) after initialize(), otherwise transmission won't work!
 *
 * @param[in] enable
 */
void rfm69_set_high_power(rfm69 *const dev, bool enable);

/**
 * @brief Enable/disable the power amplifier(s) of the RFM69 module.
 *        PA0 for regular devices is enabled and PA1 is used for high power devices (default).
 *
 * @note Use this function if you want to manually override the PA settings.
 * @note PA0 can only be used with regular devices (not the high power ones!)
 * @note PA1 and PA2 can only be used with high power devices (not the regular ones!)
 *
 * @param forcePA If this is 0, default values are used. Otherwise, PA settings are forced.
 *                0x01 for PA0, 0x02 for PA1, 0x04 for PA2, 0x08 for +20 dBm high power settings.
 */
void rfm69_set_pa_settings(rfm69 *const dev, uint8_t forcePA);

/**
 * @brief Sets high power regs on RFM69 module
 * @note  It has to be called after initialize() for RFM69HW
 *
 * @param[in] enable
 */
void rfm69_set_high_power_regs(rfm69 *const dev, bool enable);

/**
 * @brief Gets CMOS temperature (8bit)
 *
 * @param[in] cal_factor -
 */
uint8_t rfm69_read_temperature(rfm69 *const dev, uint8_t cal_factor);

/**
 * @brief Calibrates the internal RC oscillator for use in wide temperature variations.
 * @note  See datasheet section [4.3.5. RC Timer Accuracy]
 *
 */
void rfm69_rc_calibration(rfm69 *const dev);


/**
 * @brief Read a radio register at address
 *
 * @param[in] addr  Radio register address to be read data from
 *
 * @return  A read register value
 */
uint8_t rfm69_read_reg(rfm69 *const dev, uint8_t addr);

/**
 * @brief Write a radio register at address
 *
 * @param[in] addr   Radio register address
 * @param[in] val 	 A register value to be write to
 */
void rfm69_write_reg(rfm69 *const dev, uint8_t addr, uint8_t val);


// AUTO Power control functionality utilities //

/**
 * @brief Enable/disable auto Power control
 *
 * @param[in] target_rssi  The desired end point RSSI for our transmission
 */
void rfm69_enable_auto_power(rfm69 *const dev, int16_t target_rssi);

/**
 * @brief Method to retrieve the ack'd RSSI (if any)
 *
 * @return  An ack'd rssi
 */
int16_t rfm69_get_ack_rssi(rfm69 *const dev);


/**
 * ============================================================================
 * @brief Board/Platform relative (HW dependent) functions,
 *        Must be implemented in folder platform.
 * ============================================================================
 */

/**
 * @brief Initialize the radio I/Os pins interface
 */
extern void rfm69_io_init(rfm69 *const dev);

/**
 * @brief Deinitialize the radio I/Os pins interface.
 *
 * @note Useful when going in MCU lowpower modes
 */
extern void rfm69_io_deinit(rfm69 *const dev);

/**
 * @brief Initialize DIO IRQ handlers
 *
 * @param[in] irqHandlers Array containing the IRQ callback functions
 */
extern void rfm69_ioirq_init(rfm69 *const dev);

/**
 * @brief Reset the RFM69
 */
extern void rfm69_reset(rfm69 *const dev);

/**
 * @brief Write multiple radio registers starting at address
 *
 * @param[in] addr   First Radio register address
 * @param[in] buffer Buffer containing the new register's values
 * @param[in] size   Number of registers to be written
 */
extern void rfm69_write_buffer(rfm69 *const dev, uint8_t addr,
		const uint8_t* buffer, uint8_t size);

/**
 * @brief Read multiple radio registers starting at address
 *
 * @param[in] addr First Radio register address
 * @param[out] buffer Buffer where to copy the registers data
 * @param[in] size Number of registers to be read
 */
extern void rfm69_read_buffer(rfm69 *const dev, uint8_t addr,
                              uint8_t* buffer, uint8_t size);

/**
 * @brief A simple ms sleep
 */
extern void rfm69_delay_ms(int ms);

/**
 * @brief Read uptime miliseconds
 */
extern uint32_t rfm69_timer_read_ms();

/**
 * @brief Print all the registers
 */
extern void rfm69_print_all_regs(rfm69 *const dev);

#ifdef __cplusplus
}
#endif

#endif // __RFM69_H__
