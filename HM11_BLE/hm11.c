/**
 *  @brief:   HM-11 Bluetooth Low Energy 4.0 module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "hm11.h"
#include <string.h>

// Index mask of the receive ring buffer
#define HM11_RX_MASK            (HM11_RX_BUF_SIZE - 1u)

// Answer prefixes used by the module
#define HM11_RESP_OK            "OK"
#define HM11_RESP_GET           "OK+Get:"
#define HM11_RESP_SET           "OK+Set:"
#define HM11_RESP_NAME          "OK+NAME:"
#define HM11_RESP_ADDR          "OK+ADDR:"
#define HM11_RESP_CONN          "OK+CONN"
#define HM11_RESP_DISC_START    "OK+DISCS"
#define HM11_RESP_DISC_END      "OK+DISCE"
#define HM11_RESP_DISC_ITEM     "OK+DIS"
#define HM11_RESP_RESET         "OK+RESET"
#define HM11_RESP_RENEW         "OK+RENEW"
#define HM11_RESP_SLEEP         "OK+SLEEP"
#define HM11_RESP_WAKE          "OK+WAKE"
#define HM11_RESP_START         "OK+START"

// Number of characters of the "OK+Get:" / "OK+Set:" prefixes
#define HM11_RESP_KV_PREFIX_LEN 7u

// Number of characters of the "OK+NAME:" / "OK+ADDR:" prefixes
#define HM11_RESP_TAG_PREFIX_LEN 8u

// Prefix the module puts in front of a 16 bit UUID, e.g. "OK+Get:0xFFE0"
#define HM11_RESP_GET_HEX       "OK+Get:0x"
#define HM11_RESP_SET_HEX       "OK+Set:0x"
#define HM11_RESP_HEX_PREFIX_LEN 9u

// Number of nibbles of a 16 bit UUID
#define HM11_UUID_NIBBLES       4u

// Smallest and largest 16 bit UUID accepted by the module
#define HM11_UUID_MIN           0x0001u
#define HM11_UUID_MAX           0xFFFEu

// Lowest and highest ASCII code accepted inside a module name
#define HM11_ASCII_MIN          0x20
#define HM11_ASCII_MAX          0x7E

// Character transmitted by hm11_wake_up()
#define HM11_WAKE_UP_CHAR       'A'

// Size of the scratch buffer holding a short "OK+Xxx:v" answer
#define HM11_SHORT_RESP_LEN     16u

// Size of the scratch buffer holding an answer carrying a payload
#define HM11_LONG_RESP_LEN      32u

// The ring buffer index arithmetic relies on a power of two size
typedef char hm11_rx_size_must_be_a_power_of_two[
    ((HM11_RX_BUF_SIZE & (HM11_RX_BUF_SIZE - 1u)) == 0u) ? 1 : -1];

/**
 * @brief Bounded command string builder, replaces sprintf in the core
 */
typedef struct
{
    char   buf[HM11_CMD_MAX_LEN];
    size_t len;
    bool   ok; // Cleared as soon as the command no longer fits
} hm11_cmd;

/**
 * @brief State of the answer parser of hm11_discover()
 */
typedef struct
{
    hm11_scan_result* results;
    uint8_t           max_results;
    uint8_t           count;
} hm11_disc_ctx;

//-----------------------------------------------------------------------------
/**
 * @brief Append a string to a bounded, always NUL terminated buffer.
 *
 * @param         dst      Destination buffer
 * @param         dst_size Size of `dst` in bytes, NUL terminator included
 * @param[in,out] dst_len  Current length of `dst`, updated on return
 * @param         src      String to be appended
 * @return True when the whole string fitted, false when it was truncated
 */
static bool hm11_append(char* dst, size_t dst_size, size_t* const dst_len,
                        const char* src)
{
    if (!dst || !dst_len || !src || dst_size == 0u || *dst_len >= dst_size)
    {
        return false;
    }

    size_t len = *dst_len;
    bool complete = true;

    for (size_t i = 0; src[i] != '\0'; ++i)
    {
        if ((len + 1u) >= dst_size)
        {
            complete = false;
            break;
        }
        dst[len] = src[i];
        len++;
    }

    dst[len] = '\0';
    *dst_len = len;
    return complete;
}

//-----------------------------------------------------------------------------
/**
 * @brief Copy a string into a bounded, always NUL terminated buffer.
 */
static bool hm11_copy_string(char* dst, size_t dst_size, const char* src)
{
    size_t len = 0;

    if (!dst || dst_size == 0u)
    {
        return false;
    }
    dst[0] = '\0';
    return hm11_append(dst, dst_size, &len, src);
}

//-----------------------------------------------------------------------------
static bool hm11_starts_with(const char* str, const char* prefix)
{
    if (!str || !prefix)
    {
        return false;
    }
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

//-----------------------------------------------------------------------------
/**
 * @brief Parse `nibbles` hexadecimal characters, most significant one first.
 */
static bool hm11_parse_hex(const char* str, size_t nibbles, uint32_t* const out)
{
    uint32_t value = 0;

    if (!str || !out || nibbles == 0u || nibbles > 8u)
    {
        return false;
    }

    for (size_t i = 0; i < nibbles; ++i)
    {
        const char c = str[i];
        uint32_t digit;

        if (c >= '0' && c <= '9')
        {
            digit = (uint32_t)(c - '0');
        }
        else if (c >= 'A' && c <= 'F')
        {
            digit = (uint32_t)(c - 'A') + 10u;
        }
        else if (c >= 'a' && c <= 'f')
        {
            digit = (uint32_t)(c - 'a') + 10u;
        }
        else
        {
            return false;
        }
        value = (value << 4) | digit;
    }

    *out = value;
    return true;
}

//-----------------------------------------------------------------------------
/**
 * @brief Parse `digits` decimal characters, most significant one first.
 */
static bool hm11_parse_uint(const char* str, size_t digits, uint32_t* const out)
{
    uint32_t value = 0;

    if (!str || !out || digits == 0u || digits > 9u)
    {
        return false;
    }

    for (size_t i = 0; i < digits; ++i)
    {
        const char c = str[i];
        if (c < '0' || c > '9')
        {
            return false;
        }
        value = (value * 10u) + (uint32_t)(c - '0');
    }

    *out = value;
    return true;
}

//-----------------------------------------------------------------------------
static void hm11_cmd_reset(hm11_cmd* const cmd)
{
    cmd->buf[0] = '\0';
    cmd->len = 0;
    cmd->ok = true;
}

//-----------------------------------------------------------------------------
static void hm11_cmd_add_str(hm11_cmd* const cmd, const char* str)
{
    if (!cmd->ok)
    {
        return;
    }
    if (!hm11_append(cmd->buf, sizeof(cmd->buf), &cmd->len, str))
    {
        cmd->ok = false;
    }
}

//-----------------------------------------------------------------------------
/**
 * @brief Append `nibbles` uppercase hexadecimal characters of `value`.
 */
static void hm11_cmd_add_hex(hm11_cmd* const cmd, uint32_t value,
                             uint8_t nibbles)
{
    char text[9];

    if (!cmd->ok)
    {
        return;
    }
    if (nibbles == 0u || nibbles > 8u)
    {
        cmd->ok = false;
        return;
    }

    text[nibbles] = '\0';
    for (size_t i = nibbles; i > 0u; --i)
    {
        const uint32_t nibble = value & 0x0Fu;
        text[i - 1u] = (nibble < 10u) ? (char)('0' + (int)nibble)
                                      : (char)(('A' + (int)nibble) - 10);
        value >>= 4;
    }

    hm11_cmd_add_str(cmd, text);
}

//-----------------------------------------------------------------------------
/**
 * @brief Append `digits` decimal characters of `value`, zero padded.
 */
static void hm11_cmd_add_uint(hm11_cmd* const cmd, uint32_t value,
                              uint8_t digits)
{
    char text[11];

    if (!cmd->ok)
    {
        return;
    }
    if (digits == 0u || digits > 10u)
    {
        cmd->ok = false;
        return;
    }

    text[digits] = '\0';
    for (size_t i = digits; i > 0u; --i)
    {
        text[i - 1u] = (char)('0' + (int)(value % 10u));
        value /= 10u;
    }

    hm11_cmd_add_str(cmd, text);
}

//-----------------------------------------------------------------------------
/**
 * @brief Reject names that would break the AT command framing.
 */
static bool hm11_is_valid_name(const char* name)
{
    size_t len = 0;

    if (!name || name[0] == '\0')
    {
        return false;
    }

    for (len = 0; name[len] != '\0'; ++len)
    {
        const unsigned char c = (unsigned char)name[len];
        if (c < HM11_ASCII_MIN || c > HM11_ASCII_MAX)
        {
            return false;
        }
    }
    return len <= HM11_NAME_MAX_LEN;
}

//-----------------------------------------------------------------------------
/**
 * @brief Check that `addr` holds exactly HM11_ADDR_LEN hexadecimal characters.
 */
static bool hm11_is_valid_address(const char* addr)
{
    uint32_t discarded = 0;

    if (!addr || strlen(addr) != HM11_ADDR_LEN)
    {
        return false;
    }
    // Parsed in two halves, hm11_parse_hex() handles at most 8 nibbles
    return hm11_parse_hex(&addr[0], 6u, &discarded) &&
           hm11_parse_hex(&addr[6], 6u, &discarded);
}

//-----------------------------------------------------------------------------
/**
 * @brief Send a command and check that the answer starts with `prefix`.
 */
static bool hm11_send_expect(hm11* const dev, const hm11_cmd* const cmd,
                             const char* prefix, uint32_t timeout_ms)
{
    char resp[HM11_SHORT_RESP_LEN];

    if (!cmd->ok)
    {
        return false;
    }
    if (!hm11_send_at_command(dev, cmd->buf, resp, sizeof(resp), timeout_ms))
    {
        return false;
    }
    return hm11_starts_with(resp, prefix);
}

//-----------------------------------------------------------------------------
/**
 * @brief Send "AT+<name><nibble>" and check the "OK+Set:" answer.
 */
static bool hm11_set_nibble(hm11* const dev, const char* cmd_base,
                            uint32_t value, uint32_t timeout_ms)
{
    hm11_cmd cmd;

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, cmd_base);
    hm11_cmd_add_hex(&cmd, value, 1u);
    return hm11_send_expect(dev, &cmd, HM11_RESP_SET, timeout_ms);
}

//-----------------------------------------------------------------------------
/**
 * @brief Send "AT+<name>?" and parse the single nibble of the "OK+Get:" answer.
 */
static bool hm11_get_nibble(hm11* const dev, const char* cmd, uint32_t max_value,
                            uint32_t* const out)
{
    char resp[HM11_SHORT_RESP_LEN];
    uint32_t value = 0;

    if (!out)
    {
        return false;
    }
    if (!hm11_send_at_command(dev, cmd, resp, sizeof(resp), HM11_AT_TIMEOUT_MS))
    {
        return false;
    }
    if (!hm11_starts_with(resp, HM11_RESP_GET))
    {
        return false;
    }
    if (!hm11_parse_hex(&resp[HM11_RESP_KV_PREFIX_LEN], 1u, &value))
    {
        return false;
    }
    if (value > max_value)
    {
        return false;
    }

    *out = value;
    return true;
}

//-----------------------------------------------------------------------------
/**
 * @brief Send a command and copy the payload following a "OK+XXXX:" prefix.
 */
static bool hm11_get_tagged_string(hm11* const dev, const char* cmd,
                                   const char* prefix, char* out,
                                   size_t out_size)
{
    char resp[HM11_LONG_RESP_LEN];

    if (!out || out_size == 0u)
    {
        return false;
    }
    out[0] = '\0';

    if (!hm11_send_at_command(dev, cmd, resp, sizeof(resp), HM11_AT_TIMEOUT_MS))
    {
        return false;
    }
    if (!hm11_starts_with(resp, prefix))
    {
        return false;
    }
    return hm11_copy_string(out, out_size, &resp[strlen(prefix)]);
}

//-----------------------------------------------------------------------------
/**
 * @brief Send "AT+<name>?" and parse the 16 bit UUID it answers with.
 */
static bool hm11_get_uuid(hm11* const dev, const char* cmd, uint16_t* const uuid)
{
    char resp[HM11_SHORT_RESP_LEN];
    uint32_t value = 0;

    if (!uuid)
    {
        return false;
    }
    if (!hm11_send_at_command(dev, cmd, resp, sizeof(resp), HM11_AT_TIMEOUT_MS))
    {
        return false;
    }
    if (!hm11_starts_with(resp, HM11_RESP_GET_HEX))
    {
        return false;
    }
    if (!hm11_parse_hex(&resp[HM11_RESP_HEX_PREFIX_LEN], HM11_UUID_NIBBLES,
                        &value))
    {
        return false;
    }

    *uuid = (uint16_t)value;
    return true;
}

//-----------------------------------------------------------------------------
/**
 * @brief Send "AT+<name>0xNNNN" and check the "OK+Set:0x" answer.
 */
static bool hm11_set_uuid(hm11* const dev, const char* cmd_base, uint16_t uuid)
{
    hm11_cmd cmd;

    if (uuid < HM11_UUID_MIN || uuid > HM11_UUID_MAX)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, cmd_base);
    hm11_cmd_add_str(&cmd, "0x");
    hm11_cmd_add_hex(&cmd, (uint32_t)uuid, (uint8_t)HM11_UUID_NIBBLES);
    return hm11_send_expect(dev, &cmd, HM11_RESP_SET_HEX,
                            HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
/**
 * @brief Translate the character following "OK+CONN" into a status code.
 */
static hm11_conn_status hm11_parse_conn_status(const char* resp)
{
    if (!hm11_starts_with(resp, HM11_RESP_CONN))
    {
        return HM11_CONN_UNKNOWN;
    }

    switch (resp[strlen(HM11_RESP_CONN)])
    {
        case '\0': return HM11_CONN_ESTABLISHED;
        case 'A':  return HM11_CONN_ACCEPTED;
        case 'L':  return HM11_CONN_CONNECTING;
        case 'E':  return HM11_CONN_ERROR;
        case 'F':  return HM11_CONN_FAILED;
        case 'N':  return HM11_CONN_NO_ADDRESS;
        default:   return HM11_CONN_UNKNOWN;
    }
}

//-----------------------------------------------------------------------------
/**
 * @brief Send a connection request and report its outcome.
 */
static bool hm11_request_connection(hm11* const dev, const hm11_cmd* const cmd,
                                    hm11_conn_status* const status,
                                    uint32_t timeout_ms)
{
    char resp[HM11_LONG_RESP_LEN];
    hm11_conn_status result = HM11_CONN_UNKNOWN;

    if (status)
    {
        *status = HM11_CONN_UNKNOWN;
    }
    if (!cmd->ok)
    {
        return false;
    }
    if (!hm11_send_at_command(dev, cmd->buf, resp, sizeof(resp), timeout_ms))
    {
        return false;
    }

    result = hm11_parse_conn_status(resp);
    if (status)
    {
        *status = result;
    }
    return (result == HM11_CONN_ACCEPTED) || (result == HM11_CONN_CONNECTING) ||
           (result == HM11_CONN_ESTABLISHED);
}

//-----------------------------------------------------------------------------
/**
 * @brief Feed one complete answer of a discovery scan into the result array.
 */
static void hm11_disc_handle_token(hm11_disc_ctx* const ctx, const char* token)
{
    const size_t len = strlen(token);

    if (hm11_starts_with(token, HM11_RESP_DISC_START))
    {
        ctx->count = 0;
        return;
    }

    // "OK+DISC:<addr>" on the V5xx firmwares, "OK+DIS0:<addr>" on the newer ones
    if (len > HM11_RESP_TAG_PREFIX_LEN &&
        hm11_starts_with(token, HM11_RESP_DISC_ITEM) &&
        token[HM11_RESP_TAG_PREFIX_LEN - 1u] == ':')
    {
        if (ctx->count < ctx->max_results)
        {
            hm11_scan_result* const entry = &ctx->results[ctx->count];
            (void)hm11_copy_string(entry->addr, sizeof(entry->addr),
                                   &token[HM11_RESP_TAG_PREFIX_LEN]);
            entry->name[0] = '\0';
            ctx->count++;
        }
        return;
    }

    if (hm11_starts_with(token, HM11_RESP_NAME) && ctx->count > 0u)
    {
        hm11_scan_result* const entry = &ctx->results[ctx->count - 1u];
        (void)hm11_copy_string(entry->name, sizeof(entry->name),
                               &token[HM11_RESP_TAG_PREFIX_LEN]);
    }
}

//-----------------------------------------------------------------------------
bool hm11_init(hm11* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (dev->baudrate == 0)
    {
        dev->baudrate = HM11_DEFAULT_BAUDRATE;
    }

    dev->rx.head = 0;
    dev->rx.tail = 0;
    dev->rx.dropped = 0;
    memset(dev->rx.buf, 0, sizeof(dev->rx.buf));
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_rx_push(hm11* const dev, uint8_t byte)
{
    if (!dev)
    {
        return false;
    }

    const uint16_t head = dev->rx.head;
    const uint16_t next = (uint16_t)((head + 1u) & HM11_RX_MASK);

    if (next == dev->rx.tail)
    {
        dev->rx.dropped++;
        return false;
    }

    dev->rx.buf[head] = byte;
    dev->rx.head = next;
    return true;
}

//-----------------------------------------------------------------------------
size_t hm11_poll(hm11* const dev)
{
    if (!dev)
    {
        return 0;
    }

    size_t count = 0;
    while (count < HM11_POLL_MAX_BYTES && hm11_uart_readable(dev))
    {
        const int byte = hm11_uart_read_byte(dev);
        if (byte < 0)
        {
            break;
        }
        if (!hm11_rx_push(dev, (uint8_t)byte))
        {
            break;
        }
        count++;
    }
    return count;
}

//-----------------------------------------------------------------------------
size_t hm11_rx_available(const hm11* const dev)
{
    if (!dev)
    {
        return 0;
    }
    return (size_t)((uint16_t)(dev->rx.head - dev->rx.tail) & HM11_RX_MASK);
}

//-----------------------------------------------------------------------------
int hm11_read(hm11* const dev)
{
    if (!dev)
    {
        return -1;
    }

    const uint16_t tail = dev->rx.tail;
    if (tail == dev->rx.head)
    {
        return -1;
    }

    const uint8_t byte = dev->rx.buf[tail];
    dev->rx.tail = (uint16_t)((tail + 1u) & HM11_RX_MASK);
    return (int)byte;
}

//-----------------------------------------------------------------------------
void hm11_flush_rx(hm11* const dev)
{
    if (!dev)
    {
        return;
    }
    dev->rx.tail = dev->rx.head;
}

//-----------------------------------------------------------------------------
uint32_t hm11_rx_dropped(const hm11* const dev)
{
    if (!dev)
    {
        return 0;
    }
    return dev->rx.dropped;
}

//-----------------------------------------------------------------------------
bool hm11_write(hm11* const dev, const uint8_t* buf, size_t len)
{
    if (!dev || !buf)
    {
        return false;
    }
    if (len == 0)
    {
        return true;
    }
    return hm11_uart_write(dev, buf, len);
}

//-----------------------------------------------------------------------------
bool hm11_write_string(hm11* const dev, const char* str)
{
    if (!dev || !str)
    {
        return false;
    }
    return hm11_write(dev, (const uint8_t*)str, strlen(str));
}

//-----------------------------------------------------------------------------
bool hm11_read_response(hm11* const dev, char* resp, size_t resp_size,
                        uint32_t timeout_ms)
{
    if (!dev || (resp && resp_size == 0u))
    {
        return false;
    }

    size_t written = 0;
    bool got_payload = false;
    const uint32_t start = hm11_get_ms_ticks();
    uint32_t last_rx = start;

    for (;;)
    {
        (void)hm11_poll(dev);

        const int byte = hm11_read(dev);
        if (byte >= 0)
        {
            const char c = (char)byte;
            got_payload = true;
            last_rx = hm11_get_ms_ticks();
            if (c != '\r' && c != '\n')
            {
                // Bytes beyond the caller buffer are consumed and discarded
                if (resp && ((written + 1u) < resp_size))
                {
                    resp[written] = c;
                    written++;
                }
            }
            continue;
        }

        const uint32_t now = hm11_get_ms_ticks();
        if (got_payload && ((now - last_rx) >= HM11_RESP_IDLE_MS))
        {
            break;
        }
        if ((now - start) >= timeout_ms)
        {
            break;
        }
        hm11_delay_ms(HM11_POLL_INTERVAL_MS);
    }

    if (resp)
    {
        resp[written] = '\0';
    }
    return got_payload;
}

//-----------------------------------------------------------------------------
bool hm11_send_at_command(hm11* const dev, const char* cmd, char* resp,
                          size_t resp_size, uint32_t timeout_ms)
{
    if (!dev || !cmd || (resp && resp_size == 0u))
    {
        return false;
    }

    hm11_flush_rx(dev);

    // The HM-11 expects the bare command, no CR or LF terminator
    if (!hm11_write_string(dev, cmd))
    {
        return false;
    }

    return hm11_read_response(dev, resp, resp_size, timeout_ms);
}

//-----------------------------------------------------------------------------
bool hm11_test(hm11* const dev, uint32_t timeout_ms)
{
    hm11_cmd cmd;

    if (!dev)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT");
    return hm11_send_expect(dev, &cmd, HM11_RESP_OK, timeout_ms);
}

//-----------------------------------------------------------------------------
bool hm11_reset(hm11* const dev)
{
    hm11_cmd cmd;

    if (!dev)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+RESET");
    return hm11_send_expect(dev, &cmd, HM11_RESP_RESET, HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_restore_defaults(hm11* const dev)
{
    hm11_cmd cmd;

    if (!dev)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+RENEW");
    if (!hm11_send_expect(dev, &cmd, HM11_RESP_RENEW, HM11_AT_LONG_TIMEOUT_MS))
    {
        return false;
    }

    dev->baudrate = HM11_DEFAULT_BAUDRATE;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_sleep(hm11* const dev)
{
    hm11_cmd cmd;

    if (!dev)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+SLEEP");
    return hm11_send_expect(dev, &cmd, HM11_RESP_SLEEP, HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_wake_up(hm11* const dev)
{
    uint8_t burst[HM11_WAKE_UP_CHAR_COUNT];
    char resp[HM11_SHORT_RESP_LEN];

    if (!dev)
    {
        return false;
    }

    memset(burst, (int)HM11_WAKE_UP_CHAR, sizeof(burst));
    hm11_flush_rx(dev);
    if (!hm11_write(dev, burst, sizeof(burst)))
    {
        return false;
    }

    if (!hm11_read_response(dev, resp, sizeof(resp), HM11_AT_LONG_TIMEOUT_MS))
    {
        return false;
    }
    return hm11_starts_with(resp, HM11_RESP_WAKE);
}

//-----------------------------------------------------------------------------
bool hm11_start(hm11* const dev)
{
    hm11_cmd cmd;

    if (!dev)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+START");
    return hm11_send_expect(dev, &cmd, HM11_RESP_START, HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_address(hm11* const dev, char* addr, size_t addr_size)
{
    if (!dev || !addr || addr_size < (HM11_ADDR_LEN + 1u))
    {
        return false;
    }
    return hm11_get_tagged_string(dev, "AT+ADDR?", HM11_RESP_ADDR, addr,
                                  addr_size);
}

//-----------------------------------------------------------------------------
bool hm11_get_version(hm11* const dev, char* ver, size_t ver_size)
{
    char resp[HM11_LONG_RESP_LEN];

    if (!dev || !ver || ver_size == 0u)
    {
        return false;
    }
    ver[0] = '\0';

    // Answered as a plain string, e.g. "HMSoft V540", without any prefix
    if (!hm11_send_at_command(dev, "AT+VERS?", resp, sizeof(resp),
                              HM11_AT_TIMEOUT_MS))
    {
        return false;
    }
    return hm11_copy_string(ver, ver_size, resp);
}

//-----------------------------------------------------------------------------
bool hm11_get_name(hm11* const dev, char* name, size_t name_size)
{
    if (!dev || !name || name_size < (HM11_NAME_MAX_LEN + 1u))
    {
        return false;
    }
    return hm11_get_tagged_string(dev, "AT+NAME?", HM11_RESP_NAME, name,
                                  name_size);
}

//-----------------------------------------------------------------------------
bool hm11_set_name(hm11* const dev, const char* name)
{
    hm11_cmd cmd;

    if (!dev || !hm11_is_valid_name(name))
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+NAME");
    hm11_cmd_add_str(&cmd, name);
    return hm11_send_expect(dev, &cmd, HM11_RESP_SET, HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_pin(hm11* const dev, uint32_t* pin)
{
    char resp[HM11_SHORT_RESP_LEN];

    if (!dev || !pin)
    {
        return false;
    }
    if (!hm11_send_at_command(dev, "AT+PASS?", resp, sizeof(resp),
                              HM11_AT_TIMEOUT_MS))
    {
        return false;
    }
    if (!hm11_starts_with(resp, HM11_RESP_GET))
    {
        return false;
    }
    return hm11_parse_uint(&resp[HM11_RESP_KV_PREFIX_LEN], HM11_PIN_LEN, pin);
}

//-----------------------------------------------------------------------------
bool hm11_set_pin(hm11* const dev, uint32_t pin)
{
    hm11_cmd cmd;

    if (!dev || pin > HM11_PIN_MAX)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+PASS");
    hm11_cmd_add_uint(&cmd, pin, (uint8_t)HM11_PIN_LEN);
    return hm11_send_expect(dev, &cmd, HM11_RESP_SET, HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_role(hm11* const dev, hm11_role* role)
{
    uint32_t value = 0;

    if (!dev || !role)
    {
        return false;
    }
    if (!hm11_get_nibble(dev, "AT+ROLE?", (uint32_t)HM11_ROLE_CENTRAL, &value))
    {
        return false;
    }

    *role = (hm11_role)value;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_set_role(hm11* const dev, hm11_role role)
{
    if (!dev || role > HM11_ROLE_CENTRAL)
    {
        return false;
    }
    return hm11_set_nibble(dev, "AT+ROLE", (uint32_t)role,
                           HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_baudrate(hm11* const dev, hm11_baudrate* baud)
{
    uint32_t value = 0;

    if (!dev || !baud)
    {
        return false;
    }
    if (!hm11_get_nibble(dev, "AT+BAUD?", (uint32_t)HM11_BAUD_230400, &value))
    {
        return false;
    }

    *baud = (hm11_baudrate)value;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_set_baudrate(hm11* const dev, hm11_baudrate baud)
{
    const uint32_t bps = hm11_baudrate_to_bps(baud);

    if (!dev || bps == 0u)
    {
        return false;
    }
    if (!hm11_set_nibble(dev, "AT+BAUD", (uint32_t)baud,
                         HM11_AT_LONG_TIMEOUT_MS))
    {
        return false;
    }

    dev->baudrate = bps;
    return true;
}

//-----------------------------------------------------------------------------
uint32_t hm11_baudrate_to_bps(hm11_baudrate baud)
{
    switch (baud)
    {
        case HM11_BAUD_9600:   return 9600u;
        case HM11_BAUD_19200:  return 19200u;
        case HM11_BAUD_38400:  return 38400u;
        case HM11_BAUD_57600:  return 57600u;
        case HM11_BAUD_115200: return 115200u;
        case HM11_BAUD_4800:   return 4800u;
        case HM11_BAUD_2400:   return 2400u;
        case HM11_BAUD_1200:   return 1200u;
        case HM11_BAUD_230400: return 230400u;
        default:               return 0u;
    }
}

//-----------------------------------------------------------------------------
bool hm11_get_power(hm11* const dev, hm11_power* power)
{
    uint32_t value = 0;

    if (!dev || !power)
    {
        return false;
    }
    if (!hm11_get_nibble(dev, "AT+POWE?", (uint32_t)HM11_POWER_6_DBM, &value))
    {
        return false;
    }

    *power = (hm11_power)value;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_set_power(hm11* const dev, hm11_power power)
{
    if (!dev || power > HM11_POWER_6_DBM)
    {
        return false;
    }
    return hm11_set_nibble(dev, "AT+POWE", (uint32_t)power, HM11_AT_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_adv_interval(hm11* const dev, hm11_adv_interval* interval)
{
    uint32_t value = 0;

    if (!dev || !interval)
    {
        return false;
    }
    if (!hm11_get_nibble(dev, "AT+ADVI?", (uint32_t)HM11_ADV_INTERVAL_7000MS,
                         &value))
    {
        return false;
    }

    *interval = (hm11_adv_interval)value;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_set_adv_interval(hm11* const dev, hm11_adv_interval interval)
{
    if (!dev || interval > HM11_ADV_INTERVAL_7000MS)
    {
        return false;
    }
    return hm11_set_nibble(dev, "AT+ADVI", (uint32_t)interval,
                           HM11_AT_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_work_type(hm11* const dev, hm11_work_type* type)
{
    uint32_t value = 0;

    if (!dev || !type)
    {
        return false;
    }
    if (!hm11_get_nibble(dev, "AT+IMME?", (uint32_t)HM11_WORK_TYPE_IMMEDIATE,
                         &value))
    {
        return false;
    }

    *type = (hm11_work_type)value;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_set_work_type(hm11* const dev, hm11_work_type type)
{
    if (!dev || type > HM11_WORK_TYPE_IMMEDIATE)
    {
        return false;
    }
    return hm11_set_nibble(dev, "AT+IMME", (uint32_t)type, HM11_AT_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_notify(hm11* const dev, hm11_notify* notify)
{
    uint32_t value = 0;

    if (!dev || !notify)
    {
        return false;
    }
    if (!hm11_get_nibble(dev, "AT+NOTI?", (uint32_t)HM11_NOTIFY_ON, &value))
    {
        return false;
    }

    *notify = (hm11_notify)value;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_set_notify(hm11* const dev, hm11_notify notify)
{
    if (!dev || notify > HM11_NOTIFY_ON)
    {
        return false;
    }
    return hm11_set_nibble(dev, "AT+NOTI", (uint32_t)notify,
                           HM11_AT_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_bond_mode(hm11* const dev, hm11_bond_mode* mode)
{
    uint32_t value = 0;

    if (!dev || !mode)
    {
        return false;
    }
    if (!hm11_get_nibble(dev, "AT+TYPE?", (uint32_t)HM11_BOND_AUTH_AND_BOND,
                         &value))
    {
        return false;
    }

    *mode = (hm11_bond_mode)value;
    return true;
}

//-----------------------------------------------------------------------------
bool hm11_set_bond_mode(hm11* const dev, hm11_bond_mode mode)
{
    if (!dev || mode > HM11_BOND_AUTH_AND_BOND)
    {
        return false;
    }
    return hm11_set_nibble(dev, "AT+TYPE", (uint32_t)mode,
                           HM11_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool hm11_get_service_uuid(hm11* const dev, uint16_t* uuid)
{
    if (!dev)
    {
        return false;
    }
    return hm11_get_uuid(dev, "AT+UUID?", uuid);
}

//-----------------------------------------------------------------------------
bool hm11_set_service_uuid(hm11* const dev, uint16_t uuid)
{
    if (!dev)
    {
        return false;
    }
    return hm11_set_uuid(dev, "AT+UUID", uuid);
}

//-----------------------------------------------------------------------------
bool hm11_get_char_uuid(hm11* const dev, uint16_t* uuid)
{
    if (!dev)
    {
        return false;
    }
    return hm11_get_uuid(dev, "AT+CHAR?", uuid);
}

//-----------------------------------------------------------------------------
bool hm11_set_char_uuid(hm11* const dev, uint16_t uuid)
{
    if (!dev)
    {
        return false;
    }
    return hm11_set_uuid(dev, "AT+CHAR", uuid);
}

//-----------------------------------------------------------------------------
bool hm11_discover(hm11* const dev, hm11_scan_result* results,
                   uint8_t max_results, uint8_t* found, uint32_t timeout_ms)
{
    hm11_disc_ctx ctx;
    char token[HM11_TOKEN_MAX_LEN];
    size_t token_len = 0;
    char prev1 = '\0';
    char prev2 = '\0';
    bool complete = false;

    if (!dev || !results || max_results == 0u)
    {
        return false;
    }

    ctx.results = results;
    ctx.max_results = max_results;
    ctx.count = 0;
    if (found)
    {
        *found = 0;
    }

    token[0] = '\0';
    hm11_flush_rx(dev);
    if (!hm11_write_string(dev, "AT+DISC?"))
    {
        return false;
    }

    const uint32_t start = hm11_get_ms_ticks();
    while (!complete && ((hm11_get_ms_ticks() - start) < timeout_ms))
    {
        (void)hm11_poll(dev);

        const int byte = hm11_read(dev);
        if (byte < 0)
        {
            hm11_delay_ms(HM11_POLL_INTERVAL_MS);
            continue;
        }

        const char c = (char)byte;
        if (c == '\r' || c == '\n')
        {
            prev1 = prev2;
            prev2 = c;
            continue;
        }

        // Every answer starts with "OK+", that sequence delimits the tokens
        if (prev1 == 'O' && prev2 == 'K' && c == '+')
        {
            if (token_len >= 2u && token[token_len - 1u] == 'K' &&
                token[token_len - 2u] == 'O')
            {
                token_len -= 2u;
                token[token_len] = '\0';
            }
            hm11_disc_handle_token(&ctx, token);

            token[0] = '\0';
            token_len = 0;
            (void)hm11_append(token, sizeof(token), &token_len, "OK+");
        }
        else if ((token_len + 1u) < sizeof(token))
        {
            token[token_len] = c;
            token_len++;
            token[token_len] = '\0';
        }

        if (strcmp(token, HM11_RESP_DISC_END) == 0)
        {
            complete = true;
        }

        prev1 = prev2;
        prev2 = c;
    }

    if (found)
    {
        *found = ctx.count;
    }
    return complete;
}

//-----------------------------------------------------------------------------
bool hm11_connect_to_index(hm11* const dev, uint8_t index,
                           hm11_conn_status* status, uint32_t timeout_ms)
{
    hm11_cmd cmd;

    if (!dev || index >= HM11_MAX_SCAN_RESULTS)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+CONN");
    hm11_cmd_add_uint(&cmd, (uint32_t)index, 1u);
    return hm11_request_connection(dev, &cmd, status, timeout_ms);
}

//-----------------------------------------------------------------------------
bool hm11_connect_to_address(hm11* const dev, const char* address,
                             hm11_conn_status* status, uint32_t timeout_ms)
{
    hm11_cmd cmd;

    if (!dev || !hm11_is_valid_address(address))
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+CON");
    hm11_cmd_add_str(&cmd, address);
    return hm11_request_connection(dev, &cmd, status, timeout_ms);
}

//-----------------------------------------------------------------------------
bool hm11_connect_to_last(hm11* const dev, hm11_conn_status* status,
                          uint32_t timeout_ms)
{
    hm11_cmd cmd;

    if (!dev)
    {
        return false;
    }

    hm11_cmd_reset(&cmd);
    hm11_cmd_add_str(&cmd, "AT+CONNL");
    return hm11_request_connection(dev, &cmd, status, timeout_ms);
}

//-----------------------------------------------------------------------------
