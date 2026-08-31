/**
 *  @brief:   BTM222 Bluetooth SPP module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __BTM222_H__
#define __BTM222_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Factory default UART baudrate of the module */
#define BTM222_DEFAULT_BAUDRATE     19200

/**
 * Size of the receive ring buffer. It MUST stay a power of two, the buffer
 * holds BTM222_RX_BUF_SIZE - 1 bytes because one slot separates head and tail.
 */
#define BTM222_RX_BUF_SIZE          256u

/** Maximum length of a command assembled by the AT helpers, NUL included */
#define BTM222_CMD_MAX_LEN          64u

/** Default timeout of an AT command exchange [ms] */
#define BTM222_AT_TIMEOUT_MS        1000u

/** Timeout granted to the commands that reconfigure the module [ms] */
#define BTM222_AT_LONG_TIMEOUT_MS   5000u

/** Idle time inserted between two UART polls while waiting for a response [ms] */
#define BTM222_POLL_INTERVAL_MS     2u

/** Maximum number of bytes moved into the ring buffer by one btm222_poll() */
#define BTM222_POLL_MAX_BYTES       256u

/** Value passed to btm222_send_tagged_value() to transmit an error marker */
#define BTM222_INVALID_VALUE        (-999.0f)

/** Character appended by btm222_send_tagged_value() after the value */
#define BTM222_VALUE_TERMINATOR     'x'

/** Largest magnitude btm222_send_tagged_value() is able to render */
#define BTM222_VALUE_MAX            9999999.0f

/**
 * @brief Information string selector of the ATIn command
 */
typedef enum
{
    BTM222_INFO_FIRMWARE = 0, // ATI0
    BTM222_INFO_MODULE   = 1, // ATI1
    BTM222_INFO_HARDWARE = 2, // ATI2
    BTM222_INFO_REVISION = 3, // ATI3
    BTM222_INFO_VENDOR   = 4, // ATI4
} btm222_info_id;

/**
 * @brief UART parity accepted by btm222_set_uart_config()
 */
typedef enum
{
    BTM222_PARITY_NONE = 0,
    BTM222_PARITY_ODD  = 1,
    BTM222_PARITY_EVEN = 2,
} btm222_parity;

/**
 * @brief Lock free single producer / single consumer receive ring buffer
 */
typedef struct
{
    uint8_t           buf[BTM222_RX_BUF_SIZE];
    volatile uint16_t head;    // Advanced by the producer (ISR or poll loop)
    volatile uint16_t tail;    // Advanced by the consumer
    uint32_t          dropped; // Bytes lost because the buffer was full
} btm222_ringbuf;

/**
 * @brief btm222 dev object
 */
typedef struct
{
    uint32_t       baudrate;     // UART baudrate the module runs at
    btm222_ringbuf rx;           // Receive ring buffer
    void*          platform_dev; // Platform descriptor, set by btm222_<platform>_init()
} btm222;

/**
 * @brief Initialize the module driver and clear the receive ring buffer.
 *
 * @param dev btm222 device object
 * @return True on success, false otherwise
 */
bool btm222_init(btm222* const dev);

/**
 * @brief Push one received byte into the ring buffer.
 *
 * Meant to be called from an UART receive interrupt handler or from
 * btm222_poll(). The byte is dropped when the buffer is full.
 *
 * @param dev  btm222 device object
 * @param byte Received byte
 * @return True when the byte was stored, false when it was dropped
 */
bool btm222_rx_push(btm222* const dev, uint8_t byte);

/**
 * @brief Move every byte currently readable from the UART into the ring buffer.
 *
 * Applications driving the module without an interrupt handler have to call
 * this periodically. Bounded by BTM222_POLL_MAX_BYTES.
 *
 * @param dev btm222 device object
 * @return Number of bytes moved into the ring buffer
 */
size_t btm222_poll(btm222* const dev);

/**
 * @brief Number of bytes waiting in the receive ring buffer.
 *
 * @param dev btm222 device object
 * @return Number of buffered bytes, 0 when dev is NULL
 */
size_t btm222_rx_available(const btm222* const dev);

/**
 * @brief Take one byte out of the receive ring buffer.
 *
 * @param dev btm222 device object
 * @return Buffered byte (0..255) or -1 when the buffer is empty
 */
int btm222_read(btm222* const dev);

/**
 * @brief Discard every byte held in the receive ring buffer.
 *
 * @param dev btm222 device object
 */
void btm222_flush_rx(btm222* const dev);

/**
 * @brief Number of bytes dropped so far because the ring buffer was full.
 *
 * @param dev btm222 device object
 * @return Number of dropped bytes, 0 when dev is NULL
 */
uint32_t btm222_rx_dropped(const btm222* const dev);

/**
 * @brief Transmit raw bytes over the module UART.
 *
 * @param dev btm222 device object
 * @param buf Data to be written
 * @param len Number of bytes to be written
 * @return True on success, false otherwise
 */
bool btm222_write(btm222* const dev, const uint8_t* buf, size_t len);

/**
 * @brief Transmit a NUL terminated string over the module UART.
 *
 * @param dev btm222 device object
 * @param str String to be written, the NUL terminator is not transmitted
 * @return True on success, false otherwise
 */
bool btm222_write_string(btm222* const dev, const char* str);

/**
 * @brief Transmit a tagged measurement, e.g. "T23.75x".
 *
 * Passing BTM222_INVALID_VALUE transmits the "ERROR" marker instead of a
 * number. The value is rendered with two decimal places.
 *
 * @param dev   btm222 device object
 * @param tag   Single character prefix identifying the quantity
 * @param value Value to be transmitted or BTM222_INVALID_VALUE
 * @return True on success, false otherwise
 */
bool btm222_send_tagged_value(btm222* const dev, char tag, float value);

/**
 * @brief Send an AT command and collect the response line.
 *
 * The receive buffer is flushed first, the CR terminator is appended by the
 * driver. The call returns as soon as a complete response line has been
 * received or when `timeout_ms` elapses.
 *
 * @param      dev        btm222 device object
 * @param      cmd        Command without the trailing CR, e.g. "ATI0"
 * @param[out] resp       Buffer for the NUL terminated response, may be NULL
 * @param      resp_size  Size of `resp` in bytes, must be > 0 when resp is set
 * @param      timeout_ms Maximum time to wait for the response in milliseconds
 * @return True when a complete response line arrived before the timeout
 */
bool btm222_send_at_command(btm222* const dev, const char* cmd, char* resp,
                            size_t resp_size, uint32_t timeout_ms);

/**
 * @brief Collect a response line without sending anything first.
 *
 * @param      dev        btm222 device object
 * @param[out] resp       Buffer for the NUL terminated response, may be NULL
 * @param      resp_size  Size of `resp` in bytes, must be > 0 when resp is set
 * @param      timeout_ms Maximum time to wait for the response in milliseconds
 * @return True when a complete response line arrived before the timeout
 */
bool btm222_read_response(btm222* const dev, char* resp, size_t resp_size,
                          uint32_t timeout_ms);

/**
 * @brief Check that the module answers, sends a bare "AT".
 *
 * @param dev        btm222 device object
 * @param timeout_ms Maximum time to wait for the response in milliseconds
 * @return True when the module answered "OK"
 */
bool btm222_ping(btm222* const dev, uint32_t timeout_ms);

/**
 * @brief Read one of the module information strings (ATIn).
 *
 * @param      dev       btm222 device object
 * @param      info      Information string selector
 * @param[out] resp      Buffer for the NUL terminated response
 * @param      resp_size Size of `resp` in bytes
 * @return True on success, false otherwise
 */
bool btm222_get_info(btm222* const dev, btm222_info_id info, char* resp,
                     size_t resp_size);

/**
 * @brief Set the Bluetooth friendly name of the module (AT+BTLNM).
 *
 * @param dev  btm222 device object
 * @param name New friendly name, ASCII, without CR or LF
 * @return True on success, false otherwise
 */
bool btm222_set_name(btm222* const dev, const char* name);

/**
 * @brief Query the Bluetooth friendly name of the module (AT+BTLNM?).
 *
 * @param      dev       btm222 device object
 * @param[out] resp      Buffer for the NUL terminated response
 * @param      resp_size Size of `resp` in bytes
 * @return True on success, false otherwise
 */
bool btm222_get_name(btm222* const dev, char* resp, size_t resp_size);

/**
 * @brief Set the pairing PIN code of the module (AT+BTPIN).
 *
 * @param dev btm222 device object
 * @param pin New PIN code, digits only
 * @return True on success, false otherwise
 */
bool btm222_set_pin(btm222* const dev, const char* pin);

/**
 * @brief Query the pairing PIN code of the module (AT+BTPIN?).
 *
 * @param      dev       btm222 device object
 * @param[out] resp      Buffer for the NUL terminated response
 * @param      resp_size Size of `resp` in bytes
 * @return True on success, false otherwise
 */
bool btm222_get_pin(btm222* const dev, char* resp, size_t resp_size);

/**
 * @brief Reconfigure the module UART (AT+BTURT).
 *
 * The new settings are applied by the module immediately, the host UART has to
 * be reconfigured by the application afterwards.
 *
 * @param dev       btm222 device object
 * @param baudrate  New baudrate in bits per second
 * @param data_bits Number of data bits, 7 or 8
 * @param stop_bits Number of stop bits, 1 or 2
 * @param parity    Parity setting
 * @return True on success, false otherwise
 */
bool btm222_set_uart_config(btm222* const dev, uint32_t baudrate,
                            uint8_t data_bits, uint8_t stop_bits,
                            btm222_parity parity);

/**
 * @brief Make the module discoverable and connectable (AT+BTSCAN) or hide it
 *        again (AT+BTCANCEL).
 *
 * @param dev          btm222 device object
 * @param discoverable True to start the inquiry and page scans
 * @return True on success, false otherwise
 */
bool btm222_set_discoverable(btm222* const dev, bool discoverable);

/**
 * @brief Drop the current SPP connection (ATH).
 *
 * @param dev btm222 device object
 * @return True on success, false otherwise
 */
bool btm222_disconnect(btm222* const dev);

/**
 * @brief Restore the factory settings of the module (ATZ0).
 *
 * @param dev btm222 device object
 * @return True on success, false otherwise
 */
bool btm222_restore_defaults(btm222* const dev);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes to the module UART.
 *
 * @param dev btm222 device object
 * @param buf Data to be written
 * @param len Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool btm222_uart_write(const btm222* const dev, const uint8_t* buf,
                              size_t len);

/**
 * @brief Read a single byte from the module UART.
 *
 * @param dev btm222 device object
 * @return Received byte (0..255) or -1 when no data is available
 */
extern int btm222_uart_read_byte(const btm222* const dev);

/**
 * @brief Check whether at least one byte can be read from the module UART.
 *
 * @param dev btm222 device object
 * @return True when data is available
 */
extern bool btm222_uart_readable(const btm222* const dev);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void btm222_delay_ms(uint32_t delay_ms);

/**
 * @brief Free running millisecond counter, may wrap around.
 *
 * @return Number of milliseconds elapsed since an arbitrary point in time
 */
extern uint32_t btm222_get_ms_ticks(void);


#ifdef __cplusplus
}
#endif

#endif /* __BTM222_H__ */
