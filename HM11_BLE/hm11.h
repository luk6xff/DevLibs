/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __HM11_H__
#define __HM11_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Factory default UART baudrate of the module */
#define HM11_DEFAULT_BAUDRATE       9600

/**
 * Size of the receive ring buffer. It MUST stay a power of two, the buffer
 * holds HM11_RX_BUF_SIZE - 1 bytes because one slot separates head and tail.
 */
#define HM11_RX_BUF_SIZE            256u

/** Maximum length of a command assembled by the AT helpers, NUL included */
#define HM11_CMD_MAX_LEN            32u

/** Maximum length of a single answer token collected during a discovery scan */
#define HM11_TOKEN_MAX_LEN          48u

/** Default timeout of an AT command exchange [ms] */
#define HM11_AT_TIMEOUT_MS          500u

/** Timeout granted to the commands that reconfigure or restart the module [ms] */
#define HM11_AT_LONG_TIMEOUT_MS     2000u

/**
 * Timeout granted to a connection attempt [ms]. The module answers "OK+CONNF"
 * about 10 s after a failed attempt.
 */
#define HM11_CONNECT_TIMEOUT_MS     12000u

/** Timeout granted to a full device discovery scan [ms] */
#define HM11_DISCOVERY_TIMEOUT_MS   10000u

/**
 * Idle time on the UART that marks the end of an answer [ms]. The HM-11 does
 * not terminate its answers, they are delimited by the gap that follows.
 */
#define HM11_RESP_IDLE_MS           20u

/** Idle time inserted between two UART polls while waiting for an answer [ms] */
#define HM11_POLL_INTERVAL_MS       2u

/** Maximum number of bytes moved into the ring buffer by one hm11_poll() */
#define HM11_POLL_MAX_BYTES         256u

/** Number of characters of a BLE address in its AT command form, e.g. 0017EA090909 */
#define HM11_ADDR_LEN               12u

/** Maximum number of characters of a module name */
#define HM11_NAME_MAX_LEN           12u

/** Number of digits of the pairing PIN code */
#define HM11_PIN_LEN                6u

/** Largest PIN code accepted by hm11_set_pin() */
#define HM11_PIN_MAX                999999u

/** Number of devices reported by one discovery scan */
#define HM11_MAX_SCAN_RESULTS       6u

/** Number of wake up characters sent by hm11_wake_up() */
#define HM11_WAKE_UP_CHAR_COUNT     80u

/**
 * @brief Master / slave role of the module, AT+ROLE
 */
typedef enum
{
    HM11_ROLE_PERIPHERAL = 0, // Slave, advertises and waits to be connected
    HM11_ROLE_CENTRAL    = 1, // Master, scans and connects
} hm11_role;

/**
 * @brief UART baudrate index used by AT+BAUD
 */
typedef enum
{
    HM11_BAUD_9600   = 0,
    HM11_BAUD_19200  = 1,
    HM11_BAUD_38400  = 2,
    HM11_BAUD_57600  = 3,
    HM11_BAUD_115200 = 4,
    HM11_BAUD_4800   = 5,
    HM11_BAUD_2400   = 6,
    HM11_BAUD_1200   = 7,
    HM11_BAUD_230400 = 8,
} hm11_baudrate;

/**
 * @brief Transmit power level, AT+POWE
 */
typedef enum
{
    HM11_POWER_MINUS_23_DBM = 0,
    HM11_POWER_MINUS_6_DBM  = 1,
    HM11_POWER_0_DBM        = 2,
    HM11_POWER_6_DBM        = 3,
} hm11_power;

/**
 * @brief Advertising interval, AT+ADVI
 *
 * Values above HM11_ADV_INTERVAL_1285MS are rejected by the iOS scanner in
 * some situations, 1285 ms is the largest interval Apple recommends.
 */
typedef enum
{
    HM11_ADV_INTERVAL_100MS  = 0x0,
    HM11_ADV_INTERVAL_152MS  = 0x1,
    HM11_ADV_INTERVAL_211MS  = 0x2,
    HM11_ADV_INTERVAL_318MS  = 0x3,
    HM11_ADV_INTERVAL_417MS  = 0x4,
    HM11_ADV_INTERVAL_546MS  = 0x5,
    HM11_ADV_INTERVAL_760MS  = 0x6,
    HM11_ADV_INTERVAL_852MS  = 0x7,
    HM11_ADV_INTERVAL_1022MS = 0x8,
    HM11_ADV_INTERVAL_1285MS = 0x9,
    HM11_ADV_INTERVAL_2000MS = 0xA,
    HM11_ADV_INTERVAL_3000MS = 0xB,
    HM11_ADV_INTERVAL_4000MS = 0xC,
    HM11_ADV_INTERVAL_5000MS = 0xD,
    HM11_ADV_INTERVAL_6000MS = 0xE,
    HM11_ADV_INTERVAL_7000MS = 0xF,
} hm11_adv_interval;

/**
 * @brief Module work type, AT+IMME
 */
typedef enum
{
    HM11_WORK_TYPE_WAIT_FOR_START = 0, // Idle until AT+START is received
    HM11_WORK_TYPE_IMMEDIATE      = 1, // Start working right after power up
} hm11_work_type;

/**
 * @brief Connection notification switch, AT+NOTI
 */
typedef enum
{
    HM11_NOTIFY_OFF = 0,
    HM11_NOTIFY_ON  = 1, // "OK+CONN" / "OK+LOST" are pushed over the UART
} hm11_notify;

/**
 * @brief Authentication and bonding mode, AT+TYPE
 */
typedef enum
{
    HM11_BOND_NO_PIN        = 0, // No PIN code needed
    HM11_BOND_AUTH_NO_PIN   = 1, // Authentication without a PIN code
    HM11_BOND_AUTH_WITH_PIN = 2, // Authentication with a PIN code
    HM11_BOND_AUTH_AND_BOND = 3, // Authentication and bonding, firmware >= V524
} hm11_bond_mode;

/**
 * @brief Outcome of a connection request
 */
typedef enum
{
    HM11_CONN_ACCEPTED    = 0, // OK+CONNA, request accepted, connecting
    HM11_CONN_CONNECTING  = 1, // OK+CONNL, connecting to the last device
    HM11_CONN_ERROR       = 2, // OK+CONNE, connect error
    HM11_CONN_FAILED      = 3, // OK+CONNF, connect failed
    HM11_CONN_NO_ADDRESS  = 4, // OK+CONNN, no address stored
    HM11_CONN_ESTABLISHED = 5, // OK+CONN, link established, needs AT+NOTI1
    HM11_CONN_UNKNOWN     = 6, // No or unrecognized answer
} hm11_conn_status;

/**
 * @brief One entry of a device discovery scan
 */
typedef struct
{
    char addr[HM11_ADDR_LEN + 1u];     // Address of the peer, e.g. "0017EA090909"
    char name[HM11_NAME_MAX_LEN + 1u]; // Peer name, empty unless AT+SHOW1 is set
} hm11_scan_result;

/**
 * @brief Lock free single producer / single consumer receive ring buffer
 */
typedef struct
{
    uint8_t           buf[HM11_RX_BUF_SIZE];
    volatile uint16_t head;    // Advanced by the producer (ISR or poll loop)
    volatile uint16_t tail;    // Advanced by the consumer
    uint32_t          dropped; // Bytes lost because the buffer was full
} hm11_ringbuf;

/**
 * @brief hm11 dev object
 */
typedef struct
{
    uint32_t     baudrate;     // UART baudrate the module runs at
    hm11_ringbuf rx;           // Receive ring buffer
    void*        platform_dev; // Platform descriptor, set by hm11_<platform>_init()
} hm11;

/**
 * @brief Initialize the module driver and clear the receive ring buffer.
 *
 * @param dev hm11 device object
 * @return True on success, false otherwise
 */
bool hm11_init(hm11* const dev);

/**
 * @brief Push one received byte into the ring buffer.
 *
 * Meant to be called from an UART receive interrupt handler or from
 * hm11_poll(). The byte is dropped when the buffer is full.
 *
 * @param dev  hm11 device object
 * @param byte Received byte
 * @return True when the byte was stored, false when it was dropped
 */
bool hm11_rx_push(hm11* const dev, uint8_t byte);

/**
 * @brief Move every byte currently readable from the UART into the ring buffer.
 *
 * Applications driving the module without an interrupt handler have to call
 * this periodically. Bounded by HM11_POLL_MAX_BYTES.
 *
 * @param dev hm11 device object
 * @return Number of bytes moved into the ring buffer
 */
size_t hm11_poll(hm11* const dev);

/**
 * @brief Number of bytes waiting in the receive ring buffer.
 *
 * @param dev hm11 device object
 * @return Number of buffered bytes, 0 when dev is NULL
 */
size_t hm11_rx_available(const hm11* const dev);

/**
 * @brief Take one byte out of the receive ring buffer.
 *
 * @param dev hm11 device object
 * @return Buffered byte (0..255) or -1 when the buffer is empty
 */
int hm11_read(hm11* const dev);

/**
 * @brief Discard every byte held in the receive ring buffer.
 *
 * @param dev hm11 device object
 */
void hm11_flush_rx(hm11* const dev);

/**
 * @brief Number of bytes dropped so far because the ring buffer was full.
 *
 * @param dev hm11 device object
 * @return Number of dropped bytes, 0 when dev is NULL
 */
uint32_t hm11_rx_dropped(const hm11* const dev);

/**
 * @brief Transmit raw bytes over the module UART.
 *
 * @param dev hm11 device object
 * @param buf Data to be written
 * @param len Number of bytes to be written
 * @return True on success, false otherwise
 */
bool hm11_write(hm11* const dev, const uint8_t* buf, size_t len);

/**
 * @brief Transmit a NUL terminated string over the module UART.
 *
 * @param dev hm11 device object
 * @param str String to be written, the NUL terminator is not transmitted
 * @return True on success, false otherwise
 */
bool hm11_write_string(hm11* const dev, const char* str);

/**
 * @brief Collect an answer without sending anything first.
 *
 * The HM-11 does not terminate its answers, therefore the answer is considered
 * complete as soon as the UART has been idle for HM11_RESP_IDLE_MS. Bytes that
 * do not fit into `resp` are consumed and discarded, `resp` is always NUL
 * terminated when it is not NULL.
 *
 * @param      dev        hm11 device object
 * @param[out] resp       Buffer for the NUL terminated answer, may be NULL
 * @param      resp_size  Size of `resp` in bytes, must be > 0 when resp is set
 * @param      timeout_ms Maximum time to wait for the answer in milliseconds
 * @return True when at least one byte arrived before the timeout
 */
bool hm11_read_response(hm11* const dev, char* resp, size_t resp_size,
                        uint32_t timeout_ms);

/**
 * @brief Send an AT command and collect the answer.
 *
 * The receive buffer is flushed first. HM-11 commands carry no terminator, the
 * command string is transmitted as it is.
 *
 * @param      dev        hm11 device object
 * @param      cmd        Command to be sent, e.g. "AT+NAME?"
 * @param[out] resp       Buffer for the NUL terminated answer, may be NULL
 * @param      resp_size  Size of `resp` in bytes, must be > 0 when resp is set
 * @param      timeout_ms Maximum time to wait for the answer in milliseconds
 * @return True when at least one byte arrived before the timeout
 */
bool hm11_send_at_command(hm11* const dev, const char* cmd, char* resp,
                          size_t resp_size, uint32_t timeout_ms);

/**
 * @brief Check that the module answers, sends a bare "AT" (AT).
 *
 * @param dev        hm11 device object
 * @param timeout_ms Maximum time to wait for the answer in milliseconds
 * @return True when the module answered "OK"
 */
bool hm11_test(hm11* const dev, uint32_t timeout_ms);

/**
 * @brief Restart the module (AT+RESET).
 *
 * @param dev hm11 device object
 * @return True on success, false otherwise
 */
bool hm11_reset(hm11* const dev);

/**
 * @brief Restore the factory settings and restart the module (AT+RENEW).
 *
 * All settings including the name, the PIN and the baudrate are lost.
 *
 * @param dev hm11 device object
 * @return True on success, false otherwise
 */
bool hm11_restore_defaults(hm11* const dev);

/**
 * @brief Put the module into sleep mode (AT+SLEEP).
 *
 * Only supported in the peripheral role. Use hm11_wake_up() to leave it.
 *
 * @param dev hm11 device object
 * @return True on success, false otherwise
 */
bool hm11_sleep(hm11* const dev);

/**
 * @brief Wake the module up from sleep mode.
 *
 * HM11_WAKE_UP_CHAR_COUNT dummy characters are transmitted, the module answers
 * "OK+WAKE" once it is awake again.
 *
 * @param dev hm11 device object
 * @return True on success, false otherwise
 */
bool hm11_wake_up(hm11* const dev);

/**
 * @brief Start working now, only useful with HM11_WORK_TYPE_WAIT_FOR_START
 *        (AT+START).
 *
 * @param dev hm11 device object
 * @return True on success, false otherwise
 */
bool hm11_start(hm11* const dev);

/**
 * @brief Read the BLE address of the module (AT+ADDR?).
 *
 * @param      dev       hm11 device object
 * @param[out] addr      Buffer for the NUL terminated address
 * @param      addr_size Size of `addr`, at least HM11_ADDR_LEN + 1 bytes
 * @return True on success, false otherwise
 */
bool hm11_get_address(hm11* const dev, char* addr, size_t addr_size);

/**
 * @brief Read the firmware version of the module (AT+VERS?).
 *
 * @param      dev      hm11 device object
 * @param[out] ver      Buffer for the NUL terminated version string
 * @param      ver_size Size of `ver` in bytes
 * @return True on success, false otherwise
 */
bool hm11_get_version(hm11* const dev, char* ver, size_t ver_size);

/**
 * @brief Read the name of the module (AT+NAME?).
 *
 * @param      dev       hm11 device object
 * @param[out] name      Buffer for the NUL terminated name
 * @param      name_size Size of `name`, at least HM11_NAME_MAX_LEN + 1 bytes
 * @return True on success, false otherwise
 */
bool hm11_get_name(hm11* const dev, char* name, size_t name_size);

/**
 * @brief Set the name of the module (AT+NAME).
 *
 * @param dev  hm11 device object
 * @param name New name, 1..HM11_NAME_MAX_LEN printable ASCII characters
 * @return True on success, false otherwise
 */
bool hm11_set_name(hm11* const dev, const char* name);

/**
 * @brief Read the pairing PIN code of the module (AT+PASS?).
 *
 * @param      dev      hm11 device object
 * @param[out] pin      PIN code, 0..HM11_PIN_MAX
 * @return True on success, false otherwise
 */
bool hm11_get_pin(hm11* const dev, uint32_t* pin);

/**
 * @brief Set the pairing PIN code of the module (AT+PASS).
 *
 * @param dev hm11 device object
 * @param pin New PIN code, 0..HM11_PIN_MAX, transmitted as six digits
 * @return True on success, false otherwise
 */
bool hm11_set_pin(hm11* const dev, uint32_t pin);

/**
 * @brief Read the master / slave role of the module (AT+ROLE?).
 *
 * @param      dev  hm11 device object
 * @param[out] role Current role
 * @return True on success, false otherwise
 */
bool hm11_get_role(hm11* const dev, hm11_role* role);

/**
 * @brief Set the master / slave role of the module (AT+ROLE).
 *
 * @param dev  hm11 device object
 * @param role New role
 * @return True on success, false otherwise
 */
bool hm11_set_role(hm11* const dev, hm11_role role);

/**
 * @brief Read the UART baudrate of the module (AT+BAUD?).
 *
 * @param      dev  hm11 device object
 * @param[out] baud Current baudrate index
 * @return True on success, false otherwise
 */
bool hm11_get_baudrate(hm11* const dev, hm11_baudrate* baud);

/**
 * @brief Set the UART baudrate of the module (AT+BAUD).
 *
 * The module applies the new baudrate after a restart, the host UART has to be
 * reconfigured by the application. `dev->baudrate` is updated on success.
 *
 * @param dev  hm11 device object
 * @param baud New baudrate index
 * @return True on success, false otherwise
 */
bool hm11_set_baudrate(hm11* const dev, hm11_baudrate baud);

/**
 * @brief Convert a baudrate index into bits per second.
 *
 * @param baud Baudrate index
 * @return Baudrate in bits per second, 0 for an invalid index
 */
uint32_t hm11_baudrate_to_bps(hm11_baudrate baud);

/**
 * @brief Read the transmit power level of the module (AT+POWE?).
 *
 * @param      dev   hm11 device object
 * @param[out] power Current power level
 * @return True on success, false otherwise
 */
bool hm11_get_power(hm11* const dev, hm11_power* power);

/**
 * @brief Set the transmit power level of the module (AT+POWE).
 *
 * @param dev   hm11 device object
 * @param power New power level
 * @return True on success, false otherwise
 */
bool hm11_set_power(hm11* const dev, hm11_power power);

/**
 * @brief Read the advertising interval of the module (AT+ADVI?).
 *
 * @param      dev      hm11 device object
 * @param[out] interval Current advertising interval
 * @return True on success, false otherwise
 */
bool hm11_get_adv_interval(hm11* const dev, hm11_adv_interval* interval);

/**
 * @brief Set the advertising interval of the module (AT+ADVI).
 *
 * @param dev      hm11 device object
 * @param interval New advertising interval
 * @return True on success, false otherwise
 */
bool hm11_set_adv_interval(hm11* const dev, hm11_adv_interval interval);

/**
 * @brief Read the work type of the module (AT+IMME?).
 *
 * @param      dev  hm11 device object
 * @param[out] type Current work type
 * @return True on success, false otherwise
 */
bool hm11_get_work_type(hm11* const dev, hm11_work_type* type);

/**
 * @brief Set the work type of the module (AT+IMME).
 *
 * @param dev  hm11 device object
 * @param type New work type
 * @return True on success, false otherwise
 */
bool hm11_set_work_type(hm11* const dev, hm11_work_type type);

/**
 * @brief Read the connection notification switch (AT+NOTI?).
 *
 * @param      dev    hm11 device object
 * @param[out] notify Current setting
 * @return True on success, false otherwise
 */
bool hm11_get_notify(hm11* const dev, hm11_notify* notify);

/**
 * @brief Set the connection notification switch (AT+NOTI).
 *
 * @param dev    hm11 device object
 * @param notify New setting
 * @return True on success, false otherwise
 */
bool hm11_set_notify(hm11* const dev, hm11_notify notify);

/**
 * @brief Read the authentication and bonding mode (AT+TYPE?).
 *
 * @param      dev  hm11 device object
 * @param[out] mode Current bonding mode
 * @return True on success, false otherwise
 */
bool hm11_get_bond_mode(hm11* const dev, hm11_bond_mode* mode);

/**
 * @brief Set the authentication and bonding mode (AT+TYPE).
 *
 * @param dev  hm11 device object
 * @param mode New bonding mode
 * @return True on success, false otherwise
 */
bool hm11_set_bond_mode(hm11* const dev, hm11_bond_mode mode);

/**
 * @brief Read the 16 bit service UUID of the module (AT+UUID?).
 *
 * @param      dev  hm11 device object
 * @param[out] uuid Current service UUID, 0x0001..0xFFFE
 * @return True on success, false otherwise
 */
bool hm11_get_service_uuid(hm11* const dev, uint16_t* uuid);

/**
 * @brief Set the 16 bit service UUID of the module (AT+UUID).
 *
 * @param dev  hm11 device object
 * @param uuid New service UUID, 0x0001..0xFFFE
 * @return True on success, false otherwise
 */
bool hm11_set_service_uuid(hm11* const dev, uint16_t uuid);

/**
 * @brief Read the 16 bit characteristic UUID of the module (AT+CHAR?).
 *
 * @param      dev  hm11 device object
 * @param[out] uuid Current characteristic UUID, 0x0001..0xFFFE
 * @return True on success, false otherwise
 */
bool hm11_get_char_uuid(hm11* const dev, uint16_t* uuid);

/**
 * @brief Set the 16 bit characteristic UUID of the module (AT+CHAR).
 *
 * @param dev  hm11 device object
 * @param uuid New characteristic UUID, 0x0001..0xFFFE
 * @return True on success, false otherwise
 */
bool hm11_set_char_uuid(hm11* const dev, uint16_t uuid);

/**
 * @brief Run a device discovery scan (AT+DISC?).
 *
 * The module has to be a central working immediately, set HM11_ROLE_CENTRAL and
 * HM11_WORK_TYPE_IMMEDIATE beforehand. The peer names are only reported when
 * AT+SHOW1 has been set, otherwise the `name` fields stay empty.
 *
 * @param      dev         hm11 device object
 * @param[out] results     Array receiving the discovered devices
 * @param      max_results Number of entries of `results`
 * @param[out] found       Number of entries actually filled in, may be NULL
 * @param      timeout_ms  Maximum duration of the scan in milliseconds
 * @return True when the module reported the end of the scan
 */
bool hm11_discover(hm11* const dev, hm11_scan_result* results,
                   uint8_t max_results, uint8_t* found, uint32_t timeout_ms);

/**
 * @brief Connect to a device reported by the last discovery scan (AT+CONN).
 *
 * The command clears the discovery results held by the module.
 *
 * @param      dev        hm11 device object
 * @param      index      Index of the device, 0..HM11_MAX_SCAN_RESULTS - 1
 * @param[out] status     Outcome of the request, may be NULL
 * @param      timeout_ms Maximum time to wait for the answer in milliseconds
 * @return True when the request was accepted by the module
 */
bool hm11_connect_to_index(hm11* const dev, uint8_t index,
                           hm11_conn_status* status, uint32_t timeout_ms);

/**
 * @brief Connect to a given BLE address (AT+CON).
 *
 * @param      dev        hm11 device object
 * @param      address    Peer address, exactly HM11_ADDR_LEN hex characters
 * @param[out] status     Outcome of the request, may be NULL
 * @param      timeout_ms Maximum time to wait for the answer in milliseconds
 * @return True when the request was accepted by the module
 */
bool hm11_connect_to_address(hm11* const dev, const char* address,
                             hm11_conn_status* status, uint32_t timeout_ms);

/**
 * @brief Connect to the last device the module was connected to (AT+CONNL).
 *
 * @param      dev        hm11 device object
 * @param[out] status     Outcome of the request, may be NULL
 * @param      timeout_ms Maximum time to wait for the answer in milliseconds
 * @return True when the request was accepted by the module
 */
bool hm11_connect_to_last(hm11* const dev, hm11_conn_status* status,
                          uint32_t timeout_ms);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes to the module UART.
 *
 * @param dev hm11 device object
 * @param buf Data to be written
 * @param len Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool hm11_uart_write(const hm11* const dev, const uint8_t* buf,
                            size_t len);

/**
 * @brief Read a single byte from the module UART.
 *
 * @param dev hm11 device object
 * @return Received byte (0..255) or -1 when no data is available
 */
extern int hm11_uart_read_byte(const hm11* const dev);

/**
 * @brief Check whether at least one byte can be read from the module UART.
 *
 * @param dev hm11 device object
 * @return True when data is available
 */
extern bool hm11_uart_readable(const hm11* const dev);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void hm11_delay_ms(uint32_t delay_ms);

/**
 * @brief Free running millisecond counter, may wrap around.
 *
 * @return Number of milliseconds elapsed since an arbitrary point in time
 */
extern uint32_t hm11_get_ms_ticks(void);


#ifdef __cplusplus
}
#endif

#endif /* __HM11_H__ */
