/**
 *  @brief:   GTS-4E-60 (Fibocom) GPS module library, NMEA 0183 parser
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "gts4e60.h"
#include <string.h>

// Number of characters of the "*CS" checksum suffix
#define GTS4E60_CHECKSUM_SUFFIX_LEN 3

// Length of the talker id plus the sentence formatter, e.g. "GPGGA"
#define GTS4E60_SENTENCE_ID_LEN     5

// Number of characters of the sentence formatter, e.g. "GGA"
#define GTS4E60_FORMATTER_LEN       3

// Boot time granted to the module by gts4e60_init()
#define GTS4E60_BOOT_DELAY_MS       1000

// Hard limits protecting the decimal parser against integer overflow
#define GTS4E60_MAX_INT_DIGITS      9
#define GTS4E60_MAX_FRAC_DIGITS     6

// Field indexes of the GGA sentence
#define GTS4E60_GGA_FIELD_TIME      1
#define GTS4E60_GGA_FIELD_LAT       2
#define GTS4E60_GGA_FIELD_NS        3
#define GTS4E60_GGA_FIELD_LON       4
#define GTS4E60_GGA_FIELD_EW        5
#define GTS4E60_GGA_FIELD_QUALITY   6
#define GTS4E60_GGA_FIELD_SATS      7
#define GTS4E60_GGA_FIELD_HDOP      8
#define GTS4E60_GGA_FIELD_ALT       9
#define GTS4E60_GGA_FIELD_ALT_UNIT  10
#define GTS4E60_GGA_FIELD_GEOID     11

// Field indexes of the RMC sentence
#define GTS4E60_RMC_FIELD_TIME      1
#define GTS4E60_RMC_FIELD_STATUS    2
#define GTS4E60_RMC_FIELD_LAT       3
#define GTS4E60_RMC_FIELD_NS        4
#define GTS4E60_RMC_FIELD_LON       5
#define GTS4E60_RMC_FIELD_EW        6
#define GTS4E60_RMC_FIELD_SPEED     7
#define GTS4E60_RMC_FIELD_COURSE    8
#define GTS4E60_RMC_FIELD_DATE      9
#define GTS4E60_RMC_FIELD_MAGVAR    10
#define GTS4E60_RMC_FIELD_MAGVAR_EW 11

// Field indexes of the GSA sentence
#define GTS4E60_GSA_FIELD_FIX_MODE  2
#define GTS4E60_GSA_FIELD_PDOP      15
#define GTS4E60_GSA_FIELD_HDOP      16
#define GTS4E60_GSA_FIELD_VDOP      17

// Field indexes of the GSV sentence
#define GTS4E60_GSV_FIELD_IN_VIEW   3

/**
 * @brief Decomposed fixed point number extracted from a NMEA field
 */
typedef struct
{
    bool     negative;
    uint32_t integer;
    uint32_t fraction;
    uint32_t fraction_scale; // 10^(number of stored fractional digits)
} gts4e60_decimal;

//-----------------------------------------------------------------------------
static int gts4e60_hex_value(char c)
{
    if (c >= '0' && c <= '9')
    {
        return (int)(c - '0');
    }
    if (c >= 'A' && c <= 'F')
    {
        return (int)(c - 'A') + 10;
    }
    if (c >= 'a' && c <= 'f')
    {
        return (int)(c - 'a') + 10;
    }
    return -1;
}

//-----------------------------------------------------------------------------
static char gts4e60_hex_digit(uint8_t nibble)
{
    static const char digits[] = "0123456789ABCDEF";
    return digits[nibble & 0x0Fu];
}

//-----------------------------------------------------------------------------
/**
 * @brief Locate a comma separated field inside a sentence payload.
 *
 * The payload is never modified, the field is returned as a pointer plus a
 * length so no NUL terminator has to be inserted into the shared buffer.
 */
static bool gts4e60_get_field(const char* payload, size_t payload_len,
                              uint8_t index, const char** field,
                              size_t* field_len)
{
    if (!payload || !field || !field_len)
    {
        return false;
    }

    size_t start = 0;
    uint8_t current = 0;

    for (size_t i = 0; i <= payload_len; ++i)
    {
        if (i == payload_len || payload[i] == ',')
        {
            if (current == index)
            {
                *field = &payload[start];
                *field_len = i - start;
                return true;
            }
            current++;
            start = i + 1u;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
static bool gts4e60_parse_decimal(const char* field, size_t len,
                                  gts4e60_decimal* out)
{
    if (!field || !out || len == 0)
    {
        return false;
    }

    out->negative = false;
    out->integer = 0;
    out->fraction = 0;
    out->fraction_scale = 1;

    size_t i = 0;
    if (field[0] == '-' || field[0] == '+')
    {
        out->negative = (field[0] == '-');
        i = 1;
    }

    size_t int_digits = 0;
    while (i < len && field[i] >= '0' && field[i] <= '9')
    {
        if (int_digits >= GTS4E60_MAX_INT_DIGITS)
        {
            return false;
        }
        out->integer = (out->integer * 10u) + (uint32_t)(field[i] - '0');
        int_digits++;
        i++;
    }

    size_t frac_digits = 0;
    if (i < len && field[i] == '.')
    {
        i++;
        while (i < len && field[i] >= '0' && field[i] <= '9')
        {
            // Extra digits are dropped, they carry no usable precision
            if (frac_digits < GTS4E60_MAX_FRAC_DIGITS)
            {
                out->fraction = (out->fraction * 10u) +
                                (uint32_t)(field[i] - '0');
                out->fraction_scale *= 10u;
                frac_digits++;
            }
            i++;
        }
    }

    // The whole field has to be consumed and hold at least one digit
    return (i == len) && ((int_digits + frac_digits) != 0);
}

//-----------------------------------------------------------------------------
static float gts4e60_decimal_to_float(const gts4e60_decimal* const value)
{
    const float result = (float)value->integer +
                         ((float)value->fraction / (float)value->fraction_scale);
    return value->negative ? -result : result;
}

//-----------------------------------------------------------------------------
/**
 * @brief Convert a parsed ddmm.mmmm / dddmm.mmmm field to decimal degrees.
 *
 * Degrees and minutes are separated before the conversion to floating point so
 * that the full resolution of the sentence survives in a 32 bit float.
 */
static float gts4e60_decimal_to_degrees(const gts4e60_decimal* const value,
                                        char hemisphere)
{
    const uint32_t degrees = value->integer / 100u;
    const uint32_t whole_minutes = value->integer % 100u;
    const float minutes = (float)whole_minutes +
                          ((float)value->fraction / (float)value->fraction_scale);

    float result = (float)degrees + (minutes / 60.0f);
    if (hemisphere == 'S' || hemisphere == 'W')
    {
        result = -result;
    }
    return result;
}

//-----------------------------------------------------------------------------
static bool gts4e60_parse_float_field(const char* field, size_t len,
                                      float* out)
{
    gts4e60_decimal value;
    if (!gts4e60_parse_decimal(field, len, &value))
    {
        return false;
    }
    *out = gts4e60_decimal_to_float(&value);
    return true;
}

//-----------------------------------------------------------------------------
static bool gts4e60_parse_uint_field(const char* field, size_t len,
                                     uint32_t* out)
{
    gts4e60_decimal value;
    if (!gts4e60_parse_decimal(field, len, &value) || value.negative ||
        value.fraction_scale != 1u)
    {
        return false;
    }
    *out = value.integer;
    return true;
}

//-----------------------------------------------------------------------------
static bool gts4e60_parse_coordinate(const char* payload, size_t payload_len,
                                     uint8_t value_index, uint8_t hemi_index,
                                     float* coordinate, char* hemisphere)
{
    const char* value_field = NULL;
    const char* hemi_field = NULL;
    size_t value_len = 0;
    size_t hemi_len = 0;

    if (!gts4e60_get_field(payload, payload_len, value_index, &value_field,
                           &value_len) ||
        !gts4e60_get_field(payload, payload_len, hemi_index, &hemi_field,
                           &hemi_len))
    {
        return false;
    }
    if (value_len == 0 || hemi_len != 1)
    {
        return false;
    }

    const char hemi = hemi_field[0];
    if (hemi != 'N' && hemi != 'S' && hemi != 'E' && hemi != 'W')
    {
        return false;
    }

    gts4e60_decimal value;
    if (!gts4e60_parse_decimal(value_field, value_len, &value) || value.negative)
    {
        return false;
    }

    *coordinate = gts4e60_decimal_to_degrees(&value, hemi);
    *hemisphere = hemi;
    return true;
}

//-----------------------------------------------------------------------------
static bool gts4e60_parse_time(const char* field, size_t len,
                               gts4e60_utc_time* const out)
{
    gts4e60_decimal value;
    if (!gts4e60_parse_decimal(field, len, &value) || value.negative)
    {
        return false;
    }

    const uint32_t hours = value.integer / 10000u;
    const uint32_t minutes = (value.integer / 100u) % 100u;
    const uint32_t seconds = value.integer % 100u;
    if (hours > 23u || minutes > 59u || seconds > 60u)
    {
        return false;
    }

    out->hours = (uint8_t)hours;
    out->minutes = (uint8_t)minutes;
    out->seconds = (uint8_t)seconds;
    out->milliseconds =
        (uint16_t)((value.fraction * 1000u) / value.fraction_scale);
    return true;
}

//-----------------------------------------------------------------------------
static bool gts4e60_parse_date(const char* field, size_t len,
                               gts4e60_date* const out)
{
    gts4e60_decimal value;
    if (len != 6 || !gts4e60_parse_decimal(field, len, &value) ||
        value.negative || value.fraction_scale != 1u)
    {
        return false;
    }

    const uint32_t day = value.integer / 10000u;
    const uint32_t month = (value.integer / 100u) % 100u;
    const uint32_t year = value.integer % 100u;
    if (day == 0u || day > 31u || month == 0u || month > 12u)
    {
        return false;
    }

    out->day = (uint8_t)day;
    out->month = (uint8_t)month;
    // NMEA carries a two digit year, mapped with the common 1980..2079 pivot
    out->year = (uint16_t)((year >= 80u) ? (1900u + year) : (2000u + year));
    return true;
}

//-----------------------------------------------------------------------------
/**
 * @brief Match the sentence formatter, the talker id (GP, GN, GL, ...) is
 *        deliberately ignored so the parser works with every constellation.
 */
static gts4e60_sentence gts4e60_classify(const char* payload,
                                         size_t payload_len)
{
    const char* field = NULL;
    size_t field_len = 0;

    if (!gts4e60_get_field(payload, payload_len, 0, &field, &field_len) ||
        field_len != GTS4E60_SENTENCE_ID_LEN)
    {
        return GTS4E60_SENTENCE_UNSUPPORTED;
    }

    const char* formatter = &field[GTS4E60_SENTENCE_ID_LEN -
                                   GTS4E60_FORMATTER_LEN];
    if (strncmp(formatter, "GGA", GTS4E60_FORMATTER_LEN) == 0)
    {
        return GTS4E60_SENTENCE_GGA;
    }
    if (strncmp(formatter, "RMC", GTS4E60_FORMATTER_LEN) == 0)
    {
        return GTS4E60_SENTENCE_RMC;
    }
    if (strncmp(formatter, "GSA", GTS4E60_FORMATTER_LEN) == 0)
    {
        return GTS4E60_SENTENCE_GSA;
    }
    if (strncmp(formatter, "GSV", GTS4E60_FORMATTER_LEN) == 0)
    {
        return GTS4E60_SENTENCE_GSV;
    }
    return GTS4E60_SENTENCE_UNSUPPORTED;
}

//-----------------------------------------------------------------------------
/**
 * @brief Read one optional float field, leaving the destination untouched when
 *        the field is missing or empty.
 */
static void gts4e60_take_float(const char* payload, size_t payload_len,
                               uint8_t index, float* dest)
{
    const char* field = NULL;
    size_t field_len = 0;

    if (gts4e60_get_field(payload, payload_len, index, &field, &field_len) &&
        field_len != 0)
    {
        (void)gts4e60_parse_float_field(field, field_len, dest);
    }
}

//-----------------------------------------------------------------------------
static void gts4e60_parse_gga(gts4e60* const dev, const char* payload,
                              size_t payload_len)
{
    gts4e60_data* const data = &dev->data;
    const char* field = NULL;
    size_t field_len = 0;

    if (gts4e60_get_field(payload, payload_len, GTS4E60_GGA_FIELD_TIME, &field,
                          &field_len) && field_len != 0)
    {
        data->time_valid = gts4e60_parse_time(field, field_len, &data->time);
    }

    (void)gts4e60_parse_coordinate(payload, payload_len, GTS4E60_GGA_FIELD_LAT,
                                   GTS4E60_GGA_FIELD_NS, &data->latitude,
                                   &data->ns);
    (void)gts4e60_parse_coordinate(payload, payload_len, GTS4E60_GGA_FIELD_LON,
                                   GTS4E60_GGA_FIELD_EW, &data->longitude,
                                   &data->ew);

    uint32_t value = 0;
    if (gts4e60_get_field(payload, payload_len, GTS4E60_GGA_FIELD_QUALITY,
                          &field, &field_len) && field_len != 0 &&
        gts4e60_parse_uint_field(field, field_len, &value) &&
        value <= (uint32_t)GTS4E60_FIX_SIMULATION)
    {
        data->fix_quality = (gts4e60_fix_quality)value;
    }

    if (gts4e60_get_field(payload, payload_len, GTS4E60_GGA_FIELD_SATS, &field,
                          &field_len) && field_len != 0 &&
        gts4e60_parse_uint_field(field, field_len, &value) && value <= 255u)
    {
        data->satellites_used = (uint8_t)value;
    }

    gts4e60_take_float(payload, payload_len, GTS4E60_GGA_FIELD_HDOP,
                       &data->hdop);
    gts4e60_take_float(payload, payload_len, GTS4E60_GGA_FIELD_ALT,
                       &data->altitude);
    gts4e60_take_float(payload, payload_len, GTS4E60_GGA_FIELD_GEOID,
                       &data->geoid_separation);

    if (gts4e60_get_field(payload, payload_len, GTS4E60_GGA_FIELD_ALT_UNIT,
                          &field, &field_len) && field_len == 1)
    {
        data->altitude_units = field[0];
    }
}

//-----------------------------------------------------------------------------
static void gts4e60_parse_rmc(gts4e60* const dev, const char* payload,
                              size_t payload_len)
{
    gts4e60_data* const data = &dev->data;
    const char* field = NULL;
    size_t field_len = 0;

    if (gts4e60_get_field(payload, payload_len, GTS4E60_RMC_FIELD_TIME, &field,
                          &field_len) && field_len != 0)
    {
        data->time_valid = gts4e60_parse_time(field, field_len, &data->time);
    }

    if (gts4e60_get_field(payload, payload_len, GTS4E60_RMC_FIELD_STATUS,
                          &field, &field_len) && field_len == 1)
    {
        data->position_valid = (field[0] == 'A');
    }

    (void)gts4e60_parse_coordinate(payload, payload_len, GTS4E60_RMC_FIELD_LAT,
                                   GTS4E60_RMC_FIELD_NS, &data->latitude,
                                   &data->ns);
    (void)gts4e60_parse_coordinate(payload, payload_len, GTS4E60_RMC_FIELD_LON,
                                   GTS4E60_RMC_FIELD_EW, &data->longitude,
                                   &data->ew);

    if (gts4e60_get_field(payload, payload_len, GTS4E60_RMC_FIELD_SPEED, &field,
                          &field_len) && field_len != 0 &&
        gts4e60_parse_float_field(field, field_len, &data->speed_knots))
    {
        data->speed_kmh = gts4e60_knots_to_kmh(data->speed_knots);
    }

    gts4e60_take_float(payload, payload_len, GTS4E60_RMC_FIELD_COURSE,
                       &data->course);

    if (gts4e60_get_field(payload, payload_len, GTS4E60_RMC_FIELD_DATE, &field,
                          &field_len) && field_len != 0)
    {
        data->date_valid = gts4e60_parse_date(field, field_len, &data->date);
    }

    float variation = 0.0f;
    if (gts4e60_get_field(payload, payload_len, GTS4E60_RMC_FIELD_MAGVAR,
                          &field, &field_len) && field_len != 0 &&
        gts4e60_parse_float_field(field, field_len, &variation))
    {
        if (gts4e60_get_field(payload, payload_len,
                              GTS4E60_RMC_FIELD_MAGVAR_EW, &field,
                              &field_len) && field_len == 1 &&
            field[0] == 'W')
        {
            variation = -variation;
        }
        data->magnetic_variation = variation;
    }
}

//-----------------------------------------------------------------------------
static void gts4e60_parse_gsa(gts4e60* const dev, const char* payload,
                              size_t payload_len)
{
    gts4e60_data* const data = &dev->data;
    const char* field = NULL;
    size_t field_len = 0;
    uint32_t value = 0;

    if (gts4e60_get_field(payload, payload_len, GTS4E60_GSA_FIELD_FIX_MODE,
                          &field, &field_len) && field_len != 0 &&
        gts4e60_parse_uint_field(field, field_len, &value) &&
        value >= (uint32_t)GTS4E60_FIX_MODE_NONE &&
        value <= (uint32_t)GTS4E60_FIX_MODE_3D)
    {
        data->fix_mode = (gts4e60_fix_mode)value;
    }

    gts4e60_take_float(payload, payload_len, GTS4E60_GSA_FIELD_PDOP,
                       &data->pdop);
    gts4e60_take_float(payload, payload_len, GTS4E60_GSA_FIELD_HDOP,
                       &data->hdop);
    gts4e60_take_float(payload, payload_len, GTS4E60_GSA_FIELD_VDOP,
                       &data->vdop);
}

//-----------------------------------------------------------------------------
static void gts4e60_parse_gsv(gts4e60* const dev, const char* payload,
                              size_t payload_len)
{
    const char* field = NULL;
    size_t field_len = 0;
    uint32_t value = 0;

    if (gts4e60_get_field(payload, payload_len, GTS4E60_GSV_FIELD_IN_VIEW,
                          &field, &field_len) && field_len != 0 &&
        gts4e60_parse_uint_field(field, field_len, &value) && value <= 255u)
    {
        dev->data.satellites_in_view = (uint8_t)value;
    }
}

//-----------------------------------------------------------------------------
/**
 * @brief Validate and decode the sentence currently held in the line buffer.
 */
static bool gts4e60_parse_line(gts4e60* const dev)
{
    if (dev->line_len <= GTS4E60_CHECKSUM_SUFFIX_LEN)
    {
        return false;
    }

    if (!gts4e60_verify_checksum(dev->line, dev->line_len))
    {
        dev->checksum_errors++;
        return false;
    }

    const size_t payload_len = dev->line_len - GTS4E60_CHECKSUM_SUFFIX_LEN;
    const gts4e60_sentence type = gts4e60_classify(dev->line, payload_len);

    switch (type)
    {
        case GTS4E60_SENTENCE_GGA:
            gts4e60_parse_gga(dev, dev->line, payload_len);
            break;
        case GTS4E60_SENTENCE_RMC:
            gts4e60_parse_rmc(dev, dev->line, payload_len);
            break;
        case GTS4E60_SENTENCE_GSA:
            gts4e60_parse_gsa(dev, dev->line, payload_len);
            break;
        case GTS4E60_SENTENCE_GSV:
            gts4e60_parse_gsv(dev, dev->line, payload_len);
            break;
        default:
            dev->last_sentence = GTS4E60_SENTENCE_UNSUPPORTED;
            return false;
    }

    dev->last_sentence = type;
    dev->sentences_parsed++;
    return true;
}

//-----------------------------------------------------------------------------
bool gts4e60_init(gts4e60* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (dev->baudrate == 0)
    {
        dev->baudrate = GTS4E60_DEFAULT_BAUDRATE;
    }

    memset(&dev->data, 0, sizeof(dev->data));
    memset(dev->line, 0, sizeof(dev->line));
    dev->data.ns = ' ';
    dev->data.ew = ' ';
    dev->data.altitude_units = ' ';
    dev->data.fix_quality = GTS4E60_FIX_INVALID;
    dev->data.fix_mode = GTS4E60_FIX_MODE_NONE;
    dev->last_sentence = GTS4E60_SENTENCE_NONE;
    dev->line_len = 0;
    dev->in_sentence = false;
    dev->overflow = false;
    dev->sentences_parsed = 0;
    dev->checksum_errors = 0;

    if (!gts4e60_set_shutdown_pin(dev, false))
    {
        return false;
    }
    gts4e60_delay_ms(GTS4E60_BOOT_DELAY_MS);
    return true;
}

//-----------------------------------------------------------------------------
bool gts4e60_process_byte(gts4e60* const dev, uint8_t byte)
{
    if (!dev)
    {
        return false;
    }

    const char c = (char)byte;

    if (c == '$')
    {
        // A new sentence always restarts the assembler, a truncated one is lost
        dev->in_sentence = true;
        dev->overflow = false;
        dev->line_len = 0;
        return false;
    }

    if (!dev->in_sentence)
    {
        return false;
    }

    if (c == '\r' || c == '\n')
    {
        const bool usable = !dev->overflow && (dev->line_len != 0);
        dev->in_sentence = false;
        if (!usable)
        {
            dev->line_len = 0;
            dev->overflow = false;
            return false;
        }
        dev->line[dev->line_len] = '\0';
        const bool parsed = gts4e60_parse_line(dev);
        dev->line_len = 0;
        return parsed;
    }

    // One byte is always reserved for the NUL terminator
    if (dev->line_len >= (GTS4E60_LINE_BUF_SIZE - 1u))
    {
        dev->overflow = true;
        return false;
    }

    dev->line[dev->line_len] = c;
    dev->line_len++;
    return false;
}

//-----------------------------------------------------------------------------
bool gts4e60_process(gts4e60* const dev)
{
    if (!dev)
    {
        return false;
    }

    bool parsed = false;
    const uint32_t start = gts4e60_get_ms_ticks();

    for (uint32_t i = 0; i < (uint32_t)GTS4E60_PROCESS_MAX_BYTES; ++i)
    {
        if (!gts4e60_uart_readable(dev))
        {
            break;
        }

        const int byte = gts4e60_uart_read_byte(dev);
        if (byte < 0)
        {
            break;
        }

        if (gts4e60_process_byte(dev, (uint8_t)byte))
        {
            parsed = true;
        }

        if ((gts4e60_get_ms_ticks() - start) >= GTS4E60_PROCESS_TIMEOUT_MS)
        {
            break;
        }
    }
    return parsed;
}

//-----------------------------------------------------------------------------
bool gts4e60_wait_for_sentence(gts4e60* const dev, uint32_t timeout_ms)
{
    if (!dev)
    {
        return false;
    }

    const uint32_t start = gts4e60_get_ms_ticks();
    do
    {
        if (gts4e60_process(dev))
        {
            return true;
        }
        gts4e60_delay_ms(GTS4E60_POLL_INTERVAL_MS);
    } while ((gts4e60_get_ms_ticks() - start) < timeout_ms);

    return false;
}

//-----------------------------------------------------------------------------
bool gts4e60_wait_for_fix(gts4e60* const dev, uint32_t timeout_ms)
{
    if (!dev)
    {
        return false;
    }

    const uint32_t start = gts4e60_get_ms_ticks();
    do
    {
        (void)gts4e60_process(dev);
        if (gts4e60_has_fix(dev))
        {
            return true;
        }
        gts4e60_delay_ms(GTS4E60_POLL_INTERVAL_MS);
    } while ((gts4e60_get_ms_ticks() - start) < timeout_ms);

    return false;
}

//-----------------------------------------------------------------------------
bool gts4e60_has_fix(const gts4e60* const dev)
{
    if (!dev)
    {
        return false;
    }
    return dev->data.position_valid &&
           (dev->data.fix_quality != GTS4E60_FIX_INVALID);
}

//-----------------------------------------------------------------------------
const gts4e60_data* gts4e60_get_data(const gts4e60* const dev)
{
    if (!dev)
    {
        return NULL;
    }
    return &dev->data;
}

//-----------------------------------------------------------------------------
gts4e60_sentence gts4e60_get_last_sentence(const gts4e60* const dev)
{
    if (!dev)
    {
        return GTS4E60_SENTENCE_NONE;
    }
    return dev->last_sentence;
}

//-----------------------------------------------------------------------------
bool gts4e60_send_nmea_command(gts4e60* const dev, const char* body)
{
    if (!dev || !body)
    {
        return false;
    }

    // "$" + body + "*" + 2 checksum digits + CR + LF
    char frame[GTS4E60_CMD_MAX_LEN + 6u];
    size_t len = 0;
    uint8_t checksum = 0;

    frame[len++] = '$';
    for (size_t i = 0; body[i] != '\0'; ++i)
    {
        if (i >= GTS4E60_CMD_MAX_LEN)
        {
            return false;
        }
        const char c = body[i];
        if (c == '$' || c == '*' || c == '\r' || c == '\n')
        {
            return false;
        }
        checksum = (uint8_t)(checksum ^ (uint8_t)c);
        frame[len++] = c;
    }

    if (len == 1u)
    {
        return false;
    }

    frame[len++] = '*';
    frame[len++] = gts4e60_hex_digit((uint8_t)(checksum >> 4));
    frame[len++] = gts4e60_hex_digit((uint8_t)(checksum & 0x0Fu));
    frame[len++] = '\r';
    frame[len++] = '\n';

    return gts4e60_uart_write(dev, (const uint8_t*)frame, len);
}

//-----------------------------------------------------------------------------
bool gts4e60_set_shutdown(gts4e60* const dev, bool shutdown)
{
    if (!dev)
    {
        return false;
    }
    return gts4e60_set_shutdown_pin(dev, shutdown);
}

//-----------------------------------------------------------------------------
bool gts4e60_verify_checksum(const char* sentence, size_t len)
{
    if (!sentence)
    {
        return false;
    }

    // Tolerate the framing characters the caller may have kept
    if (len != 0 && sentence[0] == '$')
    {
        sentence++;
        len--;
    }
    while (len != 0 &&
           (sentence[len - 1u] == '\r' || sentence[len - 1u] == '\n'))
    {
        len--;
    }

    if (len <= GTS4E60_CHECKSUM_SUFFIX_LEN)
    {
        return false;
    }

    const size_t payload_len = len - GTS4E60_CHECKSUM_SUFFIX_LEN;
    if (sentence[payload_len] != '*')
    {
        return false;
    }

    const int high = gts4e60_hex_value(sentence[payload_len + 1u]);
    const int low = gts4e60_hex_value(sentence[payload_len + 2u]);
    if (high < 0 || low < 0)
    {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < payload_len; ++i)
    {
        checksum = (uint8_t)(checksum ^ (uint8_t)sentence[i]);
    }

    return checksum == (uint8_t)((high << 4) | low);
}

//-----------------------------------------------------------------------------
float gts4e60_nmea_to_decimal_degrees(float nmea_coord, char nsew)
{
    const bool negative = (nmea_coord < 0.0f);
    const float magnitude = negative ? -nmea_coord : nmea_coord;

    const float degrees = (float)((int32_t)(magnitude / 100.0f));
    const float minutes = magnitude - (degrees * 100.0f);

    float result = degrees + (minutes / 60.0f);
    if (negative || nsew == 'S' || nsew == 'W')
    {
        result = -result;
    }
    return result;
}

//-----------------------------------------------------------------------------
float gts4e60_knots_to_kmh(float knots)
{
    return knots * GTS4E60_KNOTS_TO_KMH;
}

//-----------------------------------------------------------------------------
