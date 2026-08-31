/**
 *  @brief:   BTM222 Bluetooth SPP module library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "btm222.h"
#include <string.h>

// Index mask of the receive ring buffer
#define BTM222_RX_MASK          (BTM222_RX_BUF_SIZE - 1u)

// Marker transmitted instead of a number for BTM222_INVALID_VALUE
#define BTM222_ERROR_MARKER     "ERROR"

// Response returned by the module when a command succeeded
#define BTM222_RESPONSE_OK      "OK"

// Lowest and highest ASCII code accepted inside a name or a PIN
#define BTM222_ASCII_MIN        0x20
#define BTM222_ASCII_MAX        0x7E

// The ring buffer index arithmetic relies on a power of two size
typedef char btm222_rx_size_must_be_a_power_of_two[
    ((BTM222_RX_BUF_SIZE & (BTM222_RX_BUF_SIZE - 1u)) == 0u) ? 1 : -1];

/**
 * @brief Bounded command string builder, replaces sprintf in the core
 */
typedef struct
{
    char   buf[BTM222_CMD_MAX_LEN];
    size_t len;
    bool   ok; // Cleared as soon as the command no longer fits
} btm222_cmd;

//-----------------------------------------------------------------------------
static void btm222_cmd_reset(btm222_cmd* const cmd)
{
    cmd->buf[0] = '\0';
    cmd->len = 0;
    cmd->ok = true;
}

//-----------------------------------------------------------------------------
static void btm222_cmd_add_char(btm222_cmd* const cmd, char c)
{
    if (!cmd->ok)
    {
        return;
    }
    // One byte is always reserved for the NUL terminator
    if ((cmd->len + 1u) >= BTM222_CMD_MAX_LEN)
    {
        cmd->ok = false;
        return;
    }
    cmd->buf[cmd->len] = c;
    cmd->len++;
    cmd->buf[cmd->len] = '\0';
}

//-----------------------------------------------------------------------------
static void btm222_cmd_add_str(btm222_cmd* const cmd, const char* str)
{
    if (!cmd->ok)
    {
        return;
    }
    if (!str)
    {
        cmd->ok = false;
        return;
    }
    for (size_t i = 0; str[i] != '\0'; ++i)
    {
        btm222_cmd_add_char(cmd, str[i]);
    }
}

//-----------------------------------------------------------------------------
static void btm222_cmd_add_uint(btm222_cmd* const cmd, uint32_t value)
{
    char digits[10];
    size_t count = 0;

    do
    {
        digits[count] = (char)('0' + (int)(value % 10u));
        count++;
        value /= 10u;
    } while (value != 0u && count < sizeof(digits));

    while (count != 0u)
    {
        count--;
        btm222_cmd_add_char(cmd, digits[count]);
    }
}

//-----------------------------------------------------------------------------
/**
 * @brief Render a value with two decimal places into the command builder.
 */
static bool btm222_cmd_add_fixed2(btm222_cmd* const cmd, float value)
{
    bool negative = false;

    if (value != value) // NaN
    {
        return false;
    }
    if (value < 0.0f)
    {
        negative = true;
        value = -value;
    }
    if (value > BTM222_VALUE_MAX)
    {
        return false;
    }

    const uint32_t scaled = (uint32_t)((value * 100.0f) + 0.5f);
    const uint32_t integer = scaled / 100u;
    const uint32_t fraction = scaled % 100u;

    if (negative)
    {
        btm222_cmd_add_char(cmd, '-');
    }
    btm222_cmd_add_uint(cmd, integer);
    btm222_cmd_add_char(cmd, '.');
    btm222_cmd_add_char(cmd, (char)('0' + (int)(fraction / 10u)));
    btm222_cmd_add_char(cmd, (char)('0' + (int)(fraction % 10u)));
    return cmd->ok;
}

//-----------------------------------------------------------------------------
/**
 * @brief Reject arguments that would break the AT command framing.
 */
static bool btm222_is_printable(const char* str)
{
    if (!str || str[0] == '\0')
    {
        return false;
    }
    for (size_t i = 0; str[i] != '\0'; ++i)
    {
        const unsigned char c = (unsigned char)str[i];
        if (c < BTM222_ASCII_MIN || c > BTM222_ASCII_MAX || c == '"' ||
            c == ',')
        {
            return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
static bool btm222_send_simple(btm222* const dev, const btm222_cmd* const cmd,
                               uint32_t timeout_ms)
{
    if (!cmd->ok)
    {
        return false;
    }

    char resp[8];
    if (!btm222_send_at_command(dev, cmd->buf, resp, sizeof(resp), timeout_ms))
    {
        return false;
    }
    return strncmp(resp, BTM222_RESPONSE_OK, sizeof(BTM222_RESPONSE_OK) - 1u) ==
           0;
}

//-----------------------------------------------------------------------------
bool btm222_init(btm222* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (dev->baudrate == 0)
    {
        dev->baudrate = BTM222_DEFAULT_BAUDRATE;
    }

    dev->rx.head = 0;
    dev->rx.tail = 0;
    dev->rx.dropped = 0;
    memset(dev->rx.buf, 0, sizeof(dev->rx.buf));
    return true;
}

//-----------------------------------------------------------------------------
bool btm222_rx_push(btm222* const dev, uint8_t byte)
{
    if (!dev)
    {
        return false;
    }

    const uint16_t head = dev->rx.head;
    const uint16_t next = (uint16_t)((head + 1u) & BTM222_RX_MASK);

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
size_t btm222_poll(btm222* const dev)
{
    if (!dev)
    {
        return 0;
    }

    size_t count = 0;
    while (count < BTM222_POLL_MAX_BYTES && btm222_uart_readable(dev))
    {
        const int byte = btm222_uart_read_byte(dev);
        if (byte < 0)
        {
            break;
        }
        if (!btm222_rx_push(dev, (uint8_t)byte))
        {
            break;
        }
        count++;
    }
    return count;
}

//-----------------------------------------------------------------------------
size_t btm222_rx_available(const btm222* const dev)
{
    if (!dev)
    {
        return 0;
    }
    return (size_t)((uint16_t)(dev->rx.head - dev->rx.tail) & BTM222_RX_MASK);
}

//-----------------------------------------------------------------------------
int btm222_read(btm222* const dev)
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
    dev->rx.tail = (uint16_t)((tail + 1u) & BTM222_RX_MASK);
    return (int)byte;
}

//-----------------------------------------------------------------------------
void btm222_flush_rx(btm222* const dev)
{
    if (!dev)
    {
        return;
    }
    dev->rx.tail = dev->rx.head;
}

//-----------------------------------------------------------------------------
uint32_t btm222_rx_dropped(const btm222* const dev)
{
    if (!dev)
    {
        return 0;
    }
    return dev->rx.dropped;
}

//-----------------------------------------------------------------------------
bool btm222_write(btm222* const dev, const uint8_t* buf, size_t len)
{
    if (!dev || !buf)
    {
        return false;
    }
    if (len == 0)
    {
        return true;
    }
    return btm222_uart_write(dev, buf, len);
}

//-----------------------------------------------------------------------------
bool btm222_write_string(btm222* const dev, const char* str)
{
    if (!dev || !str)
    {
        return false;
    }
    return btm222_write(dev, (const uint8_t*)str, strlen(str));
}

//-----------------------------------------------------------------------------
bool btm222_send_tagged_value(btm222* const dev, char tag, float value)
{
    if (!dev)
    {
        return false;
    }

    btm222_cmd cmd;
    btm222_cmd_reset(&cmd);
    btm222_cmd_add_char(&cmd, tag);

    if (value == BTM222_INVALID_VALUE)
    {
        btm222_cmd_add_str(&cmd, BTM222_ERROR_MARKER);
    }
    else if (!btm222_cmd_add_fixed2(&cmd, value))
    {
        return false;
    }

    btm222_cmd_add_char(&cmd, BTM222_VALUE_TERMINATOR);
    if (!cmd.ok)
    {
        return false;
    }
    return btm222_write(dev, (const uint8_t*)cmd.buf, cmd.len);
}

//-----------------------------------------------------------------------------
bool btm222_read_response(btm222* const dev, char* resp, size_t resp_size,
                          uint32_t timeout_ms)
{
    if (!dev || (resp && resp_size == 0))
    {
        return false;
    }

    size_t written = 0;
    bool got_payload = false;
    bool complete = false;
    const uint32_t start = btm222_get_ms_ticks();

    while (!complete && ((btm222_get_ms_ticks() - start) < timeout_ms))
    {
        (void)btm222_poll(dev);

        int byte = btm222_read(dev);
        if (byte < 0)
        {
            btm222_delay_ms(BTM222_POLL_INTERVAL_MS);
            continue;
        }

        while (byte >= 0)
        {
            const char c = (char)byte;
            if (c == '\n')
            {
                // Leading empty lines are skipped, the first real one ends the response
                if (got_payload)
                {
                    complete = true;
                    break;
                }
            }
            else if (c != '\r')
            {
                got_payload = true;
                if (resp && ((written + 1u) < resp_size))
                {
                    resp[written] = c;
                    written++;
                }
            }
            byte = btm222_read(dev);
        }
    }

    if (resp)
    {
        resp[written] = '\0';
    }
    return complete;
}

//-----------------------------------------------------------------------------
bool btm222_send_at_command(btm222* const dev, const char* cmd, char* resp,
                            size_t resp_size, uint32_t timeout_ms)
{
    if (!dev || !cmd || (resp && resp_size == 0))
    {
        return false;
    }

    btm222_flush_rx(dev);

    if (!btm222_write_string(dev, cmd))
    {
        return false;
    }

    static const uint8_t terminator = (uint8_t)'\r';
    if (!btm222_write(dev, &terminator, 1))
    {
        return false;
    }

    return btm222_read_response(dev, resp, resp_size, timeout_ms);
}

//-----------------------------------------------------------------------------
bool btm222_ping(btm222* const dev, uint32_t timeout_ms)
{
    btm222_cmd cmd;
    if (!dev)
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, "AT");
    return btm222_send_simple(dev, &cmd, timeout_ms);
}

//-----------------------------------------------------------------------------
bool btm222_get_info(btm222* const dev, btm222_info_id info, char* resp,
                     size_t resp_size)
{
    btm222_cmd cmd;
    if (!dev || !resp || resp_size == 0)
    {
        return false;
    }
    if (info > BTM222_INFO_VENDOR)
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, "ATI");
    btm222_cmd_add_uint(&cmd, (uint32_t)info);
    if (!cmd.ok)
    {
        return false;
    }

    return btm222_send_at_command(dev, cmd.buf, resp, resp_size,
                                  BTM222_AT_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool btm222_set_name(btm222* const dev, const char* name)
{
    btm222_cmd cmd;
    if (!dev || !btm222_is_printable(name))
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, "AT+BTLNM=\"");
    btm222_cmd_add_str(&cmd, name);
    btm222_cmd_add_char(&cmd, '"');
    return btm222_send_simple(dev, &cmd, BTM222_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool btm222_get_name(btm222* const dev, char* resp, size_t resp_size)
{
    if (!dev || !resp || resp_size == 0)
    {
        return false;
    }
    return btm222_send_at_command(dev, "AT+BTLNM?", resp, resp_size,
                                  BTM222_AT_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool btm222_set_pin(btm222* const dev, const char* pin)
{
    btm222_cmd cmd;
    if (!dev || !btm222_is_printable(pin))
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, "AT+BTPIN=\"");
    btm222_cmd_add_str(&cmd, pin);
    btm222_cmd_add_char(&cmd, '"');
    return btm222_send_simple(dev, &cmd, BTM222_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool btm222_get_pin(btm222* const dev, char* resp, size_t resp_size)
{
    if (!dev || !resp || resp_size == 0)
    {
        return false;
    }
    return btm222_send_at_command(dev, "AT+BTPIN?", resp, resp_size,
                                  BTM222_AT_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool btm222_set_uart_config(btm222* const dev, uint32_t baudrate,
                            uint8_t data_bits, uint8_t stop_bits,
                            btm222_parity parity)
{
    btm222_cmd cmd;
    if (!dev || baudrate == 0)
    {
        return false;
    }
    if ((data_bits != 7 && data_bits != 8) ||
        (stop_bits != 1 && stop_bits != 2) || parity > BTM222_PARITY_EVEN)
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, "AT+BTURT=");
    btm222_cmd_add_uint(&cmd, baudrate);
    btm222_cmd_add_char(&cmd, ',');
    btm222_cmd_add_uint(&cmd, (uint32_t)data_bits);
    btm222_cmd_add_char(&cmd, ',');
    btm222_cmd_add_uint(&cmd, (uint32_t)stop_bits);
    btm222_cmd_add_char(&cmd, ',');
    btm222_cmd_add_uint(&cmd, (uint32_t)parity);

    if (!btm222_send_simple(dev, &cmd, BTM222_AT_LONG_TIMEOUT_MS))
    {
        return false;
    }

    dev->baudrate = baudrate;
    return true;
}

//-----------------------------------------------------------------------------
bool btm222_set_discoverable(btm222* const dev, bool discoverable)
{
    btm222_cmd cmd;
    if (!dev)
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, discoverable ? "AT+BTSCAN" : "AT+BTCANCEL");
    return btm222_send_simple(dev, &cmd, BTM222_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool btm222_disconnect(btm222* const dev)
{
    btm222_cmd cmd;
    if (!dev)
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, "ATH");
    return btm222_send_simple(dev, &cmd, BTM222_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
bool btm222_restore_defaults(btm222* const dev)
{
    btm222_cmd cmd;
    if (!dev)
    {
        return false;
    }

    btm222_cmd_reset(&cmd);
    btm222_cmd_add_str(&cmd, "ATZ0");
    return btm222_send_simple(dev, &cmd, BTM222_AT_LONG_TIMEOUT_MS);
}

//-----------------------------------------------------------------------------
