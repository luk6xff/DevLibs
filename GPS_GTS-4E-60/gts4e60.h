/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __GTS4E60_H__
#define __GTS4E60_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Factory default UART baudrate of the module */
#define GTS4E60_DEFAULT_BAUDRATE        9600

/**
 * NMEA 0183 limits a sentence to 82 characters including the leading '$' and
 * the trailing CR LF. The line buffer only holds the payload placed between
 * '$' and CR, so 80 characters are always enough, a few spare bytes are added
 * to tolerate non conforming sentences.
 */
#define GTS4E60_LINE_BUF_SIZE           96

/** Maximum length of the body of a command sent with gts4e60_send_nmea_command() */
#define GTS4E60_CMD_MAX_LEN             80

/** Maximum number of bytes consumed by a single gts4e60_process() call */
#define GTS4E60_PROCESS_MAX_BYTES       512

/** Upper bound of the time a single gts4e60_process() call may spend [ms] */
#define GTS4E60_PROCESS_TIMEOUT_MS      50

/** Poll interval used by the blocking helpers [ms] */
#define GTS4E60_POLL_INTERVAL_MS        10

/** Conversion factor between knots and kilometres per hour */
#define GTS4E60_KNOTS_TO_KMH            1.852f

/**
 * @brief NMEA sentences understood by the parser
 */
typedef enum
{
    GTS4E60_SENTENCE_NONE = 0, // No sentence parsed yet
    GTS4E60_SENTENCE_GGA,      // Global positioning system fix data
    GTS4E60_SENTENCE_GSA,      // GNSS DOP and active satellites
    GTS4E60_SENTENCE_GSV,      // GNSS satellites in view
    GTS4E60_SENTENCE_RMC,      // Recommended minimum specific GNSS data
    GTS4E60_SENTENCE_UNSUPPORTED,
} gts4e60_sentence;

/**
 * @brief GGA fix quality indicator
 */
typedef enum
{
    GTS4E60_FIX_INVALID    = 0,
    GTS4E60_FIX_GPS        = 1,
    GTS4E60_FIX_DGPS       = 2,
    GTS4E60_FIX_PPS        = 3,
    GTS4E60_FIX_RTK        = 4,
    GTS4E60_FIX_FLOAT_RTK  = 5,
    GTS4E60_FIX_ESTIMATED  = 6,
    GTS4E60_FIX_MANUAL     = 7,
    GTS4E60_FIX_SIMULATION = 8,
} gts4e60_fix_quality;

/**
 * @brief GSA fix mode
 */
typedef enum
{
    GTS4E60_FIX_MODE_NONE = 1,
    GTS4E60_FIX_MODE_2D   = 2,
    GTS4E60_FIX_MODE_3D   = 3,
} gts4e60_fix_mode;

/**
 * @brief UTC time of the last valid fix
 */
typedef struct
{
    uint8_t  hours;        // 0..23
    uint8_t  minutes;      // 0..59
    uint8_t  seconds;      // 0..60 (60 during a leap second)
    uint16_t milliseconds; // 0..999
} gts4e60_utc_time;

/**
 * @brief UTC date of the last valid fix
 */
typedef struct
{
    uint8_t  day;   // 1..31
    uint8_t  month; // 1..12
    uint16_t year;  // Full year, the NMEA two digit year uses a 1980..2079 pivot
} gts4e60_date;

/**
 * @brief Navigation data decoded from the received NMEA sentences
 */
typedef struct
{
    gts4e60_utc_time    time;                   // UTC time, valid when time_valid is set
    gts4e60_date        date;                   // UTC date, valid when date_valid is set
    bool                time_valid;             // A valid UTC time has been received
    bool                date_valid;             // A valid UTC date has been received
    bool                position_valid;         // RMC status field reported 'A' (active)
    gts4e60_fix_quality fix_quality;            // GGA fix quality indicator
    gts4e60_fix_mode    fix_mode;               // GSA fix mode (none / 2D / 3D)
    uint8_t             satellites_used;        // Satellites used for the fix (GGA)
    uint8_t             satellites_in_view;     // Satellites in view (GSV)
    float               latitude;               // Decimal degrees, positive north
    float               longitude;              // Decimal degrees, positive east
    char                ns;                     // Raw latitude hemisphere, 'N' or 'S'
    char                ew;                     // Raw longitude hemisphere, 'E' or 'W'
    float               altitude;               // Altitude above mean sea level [m]
    char                altitude_units;         // Altitude unit reported by the module, normally 'M'
    float               geoid_separation;       // Geoid separation [m]
    float               hdop;                   // Horizontal dilution of precision
    float               pdop;                   // Position dilution of precision
    float               vdop;                   // Vertical dilution of precision
    float               speed_knots;            // Ground speed [kn]
    float               speed_kmh;              // Ground speed [km/h]
    float               course;                 // Course over ground [deg], 0..360
    float               magnetic_variation;     // Magnetic variation [deg], positive east
} gts4e60_data;

/**
 * @brief gts4e60 dev object
 */
typedef struct
{
    uint32_t         baudrate;                    // UART baudrate the module runs at
    gts4e60_data     data;                        // Last decoded navigation data
    gts4e60_sentence last_sentence;               // Type of the last accepted sentence
    char             line[GTS4E60_LINE_BUF_SIZE]; // Sentence assembly buffer
    size_t           line_len;                    // Number of bytes held in `line`
    bool             in_sentence;                 // A '$' has been seen, assembly in progress
    bool             overflow;                    // Current sentence did not fit into `line`
    uint32_t         sentences_parsed;            // Number of accepted sentences
    uint32_t         checksum_errors;             // Number of sentences rejected by the checksum
    void*            platform_dev;                // Platform descriptor, set by gts4e60_<platform>_init()
} gts4e60;

/**
 * @brief Initialize the module driver.
 *
 * Clears the parser state, wakes the module up and gives it time to boot.
 *
 * @param dev gts4e60 device object
 * @return True on success, false otherwise
 */
bool gts4e60_init(gts4e60* const dev);

/**
 * @brief Feed a single received byte into the sentence assembler.
 *
 * Safe to call from an UART receive interrupt handler.
 *
 * @param dev  gts4e60 device object
 * @param byte Byte received from the module
 * @return True when the byte completed a valid, successfully parsed sentence
 */
bool gts4e60_process_byte(gts4e60* const dev, uint8_t byte);

/**
 * @brief Consume every byte currently readable from the UART.
 *
 * Non blocking, bounded by GTS4E60_PROCESS_MAX_BYTES and
 * GTS4E60_PROCESS_TIMEOUT_MS. Call it periodically from the main loop.
 *
 * @param dev gts4e60 device object
 * @return True when at least one new valid sentence has been parsed
 */
bool gts4e60_process(gts4e60* const dev);

/**
 * @brief Block until a new valid sentence is parsed or the timeout expires.
 *
 * @param dev        gts4e60 device object
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return True when a new valid sentence was parsed, false on timeout
 */
bool gts4e60_wait_for_sentence(gts4e60* const dev, uint32_t timeout_ms);

/**
 * @brief Block until the module reports a valid position or the timeout expires.
 *
 * @param dev        gts4e60 device object
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return True when a fix is available, false on timeout
 */
bool gts4e60_wait_for_fix(gts4e60* const dev, uint32_t timeout_ms);

/**
 * @brief Tell whether the driver currently holds a usable position.
 *
 * @param dev gts4e60 device object
 * @return True when the last RMC status was valid and the GGA fix quality is not 0
 */
bool gts4e60_has_fix(const gts4e60* const dev);

/**
 * @brief Return a pointer to the decoded navigation data.
 *
 * @param dev gts4e60 device object
 * @return Pointer to the internal data block or NULL when dev is NULL
 */
const gts4e60_data* gts4e60_get_data(const gts4e60* const dev);

/**
 * @brief Return the type of the last accepted sentence.
 *
 * @param dev gts4e60 device object
 * @return Sentence type, GTS4E60_SENTENCE_NONE when dev is NULL
 */
gts4e60_sentence gts4e60_get_last_sentence(const gts4e60* const dev);

/**
 * @brief Send a NMEA command, framing and checksum are added by the driver.
 *
 * Example: passing "PMTK101" transmits "$PMTK101*32\r\n".
 *
 * @param dev  gts4e60 device object
 * @param body Sentence body without '$', '*', checksum and CR LF
 * @return True on success, false otherwise
 */
bool gts4e60_send_nmea_command(gts4e60* const dev, const char* body);

/**
 * @brief Drive the module shutdown pin.
 *
 * @param dev      gts4e60 device object
 * @param shutdown True to assert the shutdown line, false to run the module
 * @return True on success, false otherwise
 */
bool gts4e60_set_shutdown(gts4e60* const dev, bool shutdown);

/**
 * @brief Verify the XOR checksum of a NMEA sentence.
 *
 * Accepts the sentence with or without the leading '$'. Trailing CR LF
 * characters are ignored.
 *
 * @param sentence Sentence characters, does not have to be NUL terminated
 * @param len      Number of characters in `sentence`
 * @return True when the sentence carries a matching checksum
 */
bool gts4e60_verify_checksum(const char* sentence, size_t len);

/**
 * @brief Convert a raw NMEA coordinate to decimal degrees.
 *
 * @param nmea_coord Coordinate in the NMEA ddmm.mmmm / dddmm.mmmm format
 * @param nsew       Hemisphere character, 'N', 'S', 'E' or 'W'
 * @return Coordinate in decimal degrees, negative for the south / west hemisphere
 */
float gts4e60_nmea_to_decimal_degrees(float nmea_coord, char nsew);

/**
 * @brief Convert knots to kilometres per hour.
 *
 * @param knots Speed in knots
 * @return Speed in kilometres per hour
 */
float gts4e60_knots_to_kmh(float knots);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write bytes to the module UART.
 *
 * @param dev gts4e60 device object
 * @param buf Data to be written
 * @param len Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool gts4e60_uart_write(const gts4e60* const dev, const uint8_t* buf,
                               size_t len);

/**
 * @brief Read a single byte from the module UART.
 *
 * @param dev gts4e60 device object
 * @return Received byte (0..255) or -1 when no data is available
 */
extern int gts4e60_uart_read_byte(const gts4e60* const dev);

/**
 * @brief Check whether at least one byte can be read from the module UART.
 *
 * @param dev gts4e60 device object
 * @return True when data is available
 */
extern bool gts4e60_uart_readable(const gts4e60* const dev);

/**
 * @brief Drive the module shutdown line.
 *
 * Platforms without a wired shutdown line may implement this as a no-op that
 * returns true.
 *
 * @param dev   gts4e60 device object
 * @param state True to assert the shutdown line, false to release it
 * @return True on success, false otherwise
 */
extern bool gts4e60_set_shutdown_pin(const gts4e60* const dev, bool state);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void gts4e60_delay_ms(uint32_t delay_ms);

/**
 * @brief Free running millisecond counter, may wrap around.
 *
 * @return Number of milliseconds elapsed since an arbitrary point in time
 */
extern uint32_t gts4e60_get_ms_ticks(void);


#ifdef __cplusplus
}
#endif

#endif /* __GTS4E60_H__ */
