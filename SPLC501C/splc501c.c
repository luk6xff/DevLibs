/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "splc501c.h"
#include <string.h>

// Number of glyphs stored in the built-in font
#define SPLC501C_FONT_CHARS (SPLC501C_FONT_LAST_CHAR - SPLC501C_FONT_FIRST_CHAR + 1)

// Reset pulse and power-up timings taken from the datasheet
#define SPLC501C_RESET_LOW_MS   10
#define SPLC501C_RESET_HIGH_MS  10
#define SPLC501C_POWER_UP_MS    50

/**
 * 5x7 font, ASCII 32..127. One glyph is five column bytes, the least
 * significant bit of a byte is the topmost pixel of the column.
 */
static const uint8_t splc501c_font5x7[SPLC501C_FONT_CHARS * SPLC501C_FONT_WIDTH] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, // (space)
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x08, 0x2A, 0x1C, 0x2A, 0x08, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x30, 0x30, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x01, 0x01, // F
    0x3E, 0x41, 0x41, 0x51, 0x32, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x04, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x7F, 0x20, 0x18, 0x20, 0x7F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x03, 0x04, 0x78, 0x04, 0x03, // Y
    0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x00, 0x7F, 0x41, 0x41, // [
    0x02, 0x04, 0x08, 0x10, 0x20, // backslash
    0x41, 0x41, 0x7F, 0x00, 0x00, // ]
    0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x01, 0x02, 0x04, 0x00, // `
    0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x08, 0x14, 0x54, 0x54, 0x3C, // g
    0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x20, 0x40, 0x44, 0x3D, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, // k
    0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x7C, 0x14, 0x14, 0x14, 0x08, // p
    0x08, 0x14, 0x14, 0x18, 0x7C, // q
    0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x0C, 0x50, 0x50, 0x50, 0x3C, // y
    0x44, 0x64, 0x54, 0x4C, 0x44, // z
    0x00, 0x08, 0x36, 0x41, 0x00, // {
    0x00, 0x00, 0x7F, 0x00, 0x00, // |
    0x00, 0x41, 0x36, 0x08, 0x00, // }
    0x08, 0x08, 0x2A, 0x1C, 0x08, // right arrow
    0x08, 0x1C, 0x2A, 0x08, 0x08, // left arrow
};

//-----------------------------------------------------------------------------
static bool splc501c_write_cmd(const splc501c* const dev, uint8_t cmd)
{
    splc501c_set_pin(dev, SPLC501C_PIN_A0, false);
    splc501c_set_pin(dev, SPLC501C_PIN_CS, false);
    const bool ok = splc501c_write_bytes(dev, &cmd, 1);
    splc501c_set_pin(dev, SPLC501C_PIN_CS, true);
    return ok;
}

//-----------------------------------------------------------------------------
static bool splc501c_write_cmds(const splc501c* const dev, const uint8_t* cmds,
                                size_t len)
{
    splc501c_set_pin(dev, SPLC501C_PIN_A0, false);
    splc501c_set_pin(dev, SPLC501C_PIN_CS, false);
    const bool ok = splc501c_write_bytes(dev, cmds, len);
    splc501c_set_pin(dev, SPLC501C_PIN_CS, true);
    return ok;
}

//-----------------------------------------------------------------------------
static bool splc501c_write_data(const splc501c* const dev, const uint8_t* buf,
                                size_t len)
{
    splc501c_set_pin(dev, SPLC501C_PIN_A0, true);
    splc501c_set_pin(dev, SPLC501C_PIN_CS, false);
    const bool ok = splc501c_write_bytes(dev, buf, len);
    splc501c_set_pin(dev, SPLC501C_PIN_CS, true);
    return ok;
}

//-----------------------------------------------------------------------------
/**
 * With the segment driver reversed the visible window moves to the far end of
 * the 132 column wide controller RAM, so the first column has to be skewed.
 */
static uint8_t splc501c_column_offset(const splc501c* const dev)
{
    return (dev->orientation == SPLC501C_ORIENTATION_ROTATED_180)
               ? (uint8_t)SPLC501C_RAM_COLUMN_SKEW
               : (uint8_t)0;
}

//-----------------------------------------------------------------------------
static bool splc501c_goto_page(const splc501c* const dev, uint8_t page)
{
    const uint8_t column = splc501c_column_offset(dev);
    const uint8_t cmds[3] = {
        (uint8_t)(SPLC501C_CMD_PAGE_ADDRESS | (page & 0x0Fu)),
        (uint8_t)(SPLC501C_CMD_COLUMN_ADDRESS_HI | ((column >> 4) & 0x0Fu)),
        (uint8_t)(SPLC501C_CMD_COLUMN_ADDRESS_LO | (column & 0x0Fu)),
    };
    return splc501c_write_cmds(dev, cmds, sizeof(cmds));
}

//-----------------------------------------------------------------------------
static bool splc501c_apply_orientation(const splc501c* const dev)
{
    const bool rotated = (dev->orientation == SPLC501C_ORIENTATION_ROTATED_180);
    const uint8_t cmds[2] = {
        rotated ? (uint8_t)SPLC501C_CMD_ADC_REVERSE
                : (uint8_t)SPLC501C_CMD_ADC_NORMAL,
        rotated ? (uint8_t)SPLC501C_CMD_COM_NORMAL
                : (uint8_t)SPLC501C_CMD_COM_REVERSE,
    };
    return splc501c_write_cmds(dev, cmds, sizeof(cmds));
}

//-----------------------------------------------------------------------------
static bool splc501c_startup_sequence(splc501c* const dev)
{
    if (!splc501c_apply_orientation(dev))
    {
        return false;
    }

    const uint8_t power[3] = {
        (uint8_t)SPLC501C_CMD_BIAS_1_9,
        (uint8_t)(SPLC501C_CMD_VOLTAGE_RATIO | SPLC501C_VOLTAGE_RATIO_DEFAULT),
        (uint8_t)SPLC501C_CMD_POWER_ON,
    };
    if (!splc501c_write_cmds(dev, power, sizeof(power)))
    {
        return false;
    }
    splc501c_delay_ms(SPLC501C_POWER_UP_MS);

    if (!splc501c_set_contrast(dev, dev->contrast))
    {
        return false;
    }

    const uint8_t display[3] = {
        (uint8_t)SPLC501C_CMD_DISPLAY_ALL_OFF,
        (uint8_t)(dev->inverted ? SPLC501C_CMD_DISPLAY_REVERSE
                                : SPLC501C_CMD_DISPLAY_NORMAL),
        (uint8_t)(SPLC501C_CMD_START_LINE | 0u),
    };
    if (!splc501c_write_cmds(dev, display, sizeof(display)))
    {
        return false;
    }

    return splc501c_set_display_on(dev, true);
}

//-----------------------------------------------------------------------------
bool splc501c_init(splc501c* const dev)
{
    if (!dev)
    {
        return false;
    }

    memset(dev->buf, 0, sizeof(dev->buf));
    dev->cursor_x = 0;
    dev->cursor_y = 0;
    dev->text_color = SPLC501C_COLOR_ON;
    dev->contrast = SPLC501C_CONTRAST_DEFAULT;
    dev->inverted = false;
    dev->display_on = false;
    if (dev->orientation != SPLC501C_ORIENTATION_ROTATED_180)
    {
        dev->orientation = SPLC501C_ORIENTATION_NORMAL;
    }

    return splc501c_reset(dev);
}

//-----------------------------------------------------------------------------
bool splc501c_reset(splc501c* const dev)
{
    if (!dev)
    {
        return false;
    }

    splc501c_set_pin(dev, SPLC501C_PIN_CS, true);
    splc501c_set_pin(dev, SPLC501C_PIN_A0, false);
    splc501c_set_pin(dev, SPLC501C_PIN_RESET, false);
    splc501c_delay_ms(SPLC501C_RESET_LOW_MS);
    splc501c_set_pin(dev, SPLC501C_PIN_RESET, true);
    splc501c_delay_ms(SPLC501C_RESET_HIGH_MS);

    if (!splc501c_startup_sequence(dev))
    {
        return false;
    }

    return splc501c_update(dev);
}

//-----------------------------------------------------------------------------
bool splc501c_set_display_on(splc501c* const dev, bool on)
{
    if (!dev)
    {
        return false;
    }

    const uint8_t cmd = on ? (uint8_t)SPLC501C_CMD_DISPLAY_ON
                           : (uint8_t)SPLC501C_CMD_DISPLAY_OFF;
    if (!splc501c_write_cmd(dev, cmd))
    {
        return false;
    }

    dev->display_on = on;
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_set_contrast(splc501c* const dev, uint8_t contrast)
{
    if (!dev)
    {
        return false;
    }

    if (contrast > SPLC501C_CONTRAST_MAX)
    {
        contrast = (uint8_t)SPLC501C_CONTRAST_MAX;
    }

    const uint8_t cmds[2] = {(uint8_t)SPLC501C_CMD_VOLUME_MODE, contrast};
    if (!splc501c_write_cmds(dev, cmds, sizeof(cmds)))
    {
        return false;
    }

    dev->contrast = contrast;
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_set_inverted(splc501c* const dev, bool inverted)
{
    if (!dev)
    {
        return false;
    }

    const uint8_t cmd = inverted ? (uint8_t)SPLC501C_CMD_DISPLAY_REVERSE
                                 : (uint8_t)SPLC501C_CMD_DISPLAY_NORMAL;
    if (!splc501c_write_cmd(dev, cmd))
    {
        return false;
    }

    dev->inverted = inverted;
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_set_orientation(splc501c* const dev,
                              splc501c_orientation orientation)
{
    if (!dev)
    {
        return false;
    }

    dev->orientation = orientation;
    if (!splc501c_apply_orientation(dev))
    {
        return false;
    }

    return splc501c_update(dev);
}

//-----------------------------------------------------------------------------
bool splc501c_update(splc501c* const dev)
{
    if (!dev)
    {
        return false;
    }

    for (uint8_t page = 0; page < (uint8_t)SPLC501C_PAGES; page++)
    {
        if (!splc501c_goto_page(dev, page))
        {
            return false;
        }
        if (!splc501c_write_data(dev, &dev->buf[(size_t)page * SPLC501C_WIDTH],
                                 (size_t)SPLC501C_WIDTH))
        {
            return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_clear(splc501c* const dev, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }

    memset(dev->buf, (color == SPLC501C_COLOR_ON) ? 0xFF : 0x00,
           sizeof(dev->buf));
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_pixel(splc501c* const dev, int16_t x, int16_t y,
                         splc501c_color color)
{
    if (!dev)
    {
        return false;
    }
    if (x < 0 || x >= SPLC501C_WIDTH || y < 0 || y >= SPLC501C_HEIGHT)
    {
        return false;
    }

    const size_t idx = ((size_t)(y / SPLC501C_PIXELS_PER_PAGE) * SPLC501C_WIDTH)
                       + (size_t)x;
    const uint8_t mask = (uint8_t)(1u << ((unsigned)y % SPLC501C_PIXELS_PER_PAGE));

    switch (color)
    {
        case SPLC501C_COLOR_ON:
            dev->buf[idx] |= mask;
            break;
        case SPLC501C_COLOR_INVERSE:
            dev->buf[idx] ^= mask;
            break;
        case SPLC501C_COLOR_OFF:
        default:
            dev->buf[idx] &= (uint8_t)~mask;
            break;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_get_pixel(const splc501c* const dev, int16_t x, int16_t y)
{
    if (!dev)
    {
        return false;
    }
    if (x < 0 || x >= SPLC501C_WIDTH || y < 0 || y >= SPLC501C_HEIGHT)
    {
        return false;
    }

    const size_t idx = ((size_t)(y / SPLC501C_PIXELS_PER_PAGE) * SPLC501C_WIDTH)
                       + (size_t)x;
    const uint8_t mask = (uint8_t)(1u << ((unsigned)y % SPLC501C_PIXELS_PER_PAGE));
    return (dev->buf[idx] & mask) != 0u;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_hline(splc501c* const dev, int16_t x, int16_t y,
                         int16_t len, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }

    int32_t x0 = x;
    int32_t count = len;
    if (count < 0)
    {
        count = -count;
        x0 -= count - 1;
    }
    if (count == 0 || y < 0 || y >= SPLC501C_HEIGHT)
    {
        return true;
    }

    int32_t x1 = x0 + count - 1;
    if (x1 < 0 || x0 >= SPLC501C_WIDTH)
    {
        return true;
    }
    if (x0 < 0)
    {
        x0 = 0;
    }
    if (x1 > (SPLC501C_WIDTH - 1))
    {
        x1 = SPLC501C_WIDTH - 1;
    }

    for (int32_t i = x0; i <= x1; i++)
    {
        (void)splc501c_draw_pixel(dev, (int16_t)i, y, color);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_vline(splc501c* const dev, int16_t x, int16_t y,
                         int16_t len, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }

    int32_t y0 = y;
    int32_t count = len;
    if (count < 0)
    {
        count = -count;
        y0 -= count - 1;
    }
    if (count == 0 || x < 0 || x >= SPLC501C_WIDTH)
    {
        return true;
    }

    int32_t y1 = y0 + count - 1;
    if (y1 < 0 || y0 >= SPLC501C_HEIGHT)
    {
        return true;
    }
    if (y0 < 0)
    {
        y0 = 0;
    }
    if (y1 > (SPLC501C_HEIGHT - 1))
    {
        y1 = SPLC501C_HEIGHT - 1;
    }

    for (int32_t i = y0; i <= y1; i++)
    {
        (void)splc501c_draw_pixel(dev, x, (int16_t)i, color);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_line(splc501c* const dev, int16_t x0, int16_t y0,
                        int16_t x1, int16_t y1, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }

    if (y0 == y1)
    {
        const int16_t left = (x0 < x1) ? x0 : x1;
        const int16_t len = (int16_t)(((x0 < x1) ? (x1 - x0) : (x0 - x1)) + 1);
        return splc501c_draw_hline(dev, left, y0, len, color);
    }
    if (x0 == x1)
    {
        const int16_t top = (y0 < y1) ? y0 : y1;
        const int16_t len = (int16_t)(((y0 < y1) ? (y1 - y0) : (y0 - y1)) + 1);
        return splc501c_draw_vline(dev, x0, top, len, color);
    }

    // Plain Bresenham, out of range points are dropped by splc501c_draw_pixel()
    int32_t cx = x0;
    int32_t cy = y0;
    const int32_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    const int32_t dy = (y1 > y0) ? (y0 - y1) : (y1 - y0); // negative
    const int32_t sx = (x0 < x1) ? 1 : -1;
    const int32_t sy = (y0 < y1) ? 1 : -1;
    int32_t err = dx + dy;

    for (;;)
    {
        (void)splc501c_draw_pixel(dev, (int16_t)cx, (int16_t)cy, color);
        if (cx == x1 && cy == y1)
        {
            break;
        }
        const int32_t err2 = 2 * err;
        if (err2 >= dy)
        {
            err += dy;
            cx += sx;
        }
        if (err2 <= dx)
        {
            err += dx;
            cy += sy;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_rect(splc501c* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }
    if (width <= 0 || height <= 0)
    {
        return true;
    }

    const int16_t right = (int16_t)(x + width - 1);
    const int16_t bottom = (int16_t)(y + height - 1);

    (void)splc501c_draw_hline(dev, x, y, width, color);
    (void)splc501c_draw_hline(dev, x, bottom, width, color);
    (void)splc501c_draw_vline(dev, x, y, height, color);
    (void)splc501c_draw_vline(dev, right, y, height, color);
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_fill_rect(splc501c* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }
    if (width <= 0 || height <= 0)
    {
        return true;
    }

    for (int32_t row = 0; row < height; row++)
    {
        (void)splc501c_draw_hline(dev, x, (int16_t)(y + row), width, color);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_circle(splc501c* const dev, int16_t cx, int16_t cy,
                          int16_t radius, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }
    if (radius < 0)
    {
        return true;
    }

    int32_t x = radius;
    int32_t y = 0;
    int32_t err = 1 - radius;

    while (x >= y)
    {
        (void)splc501c_draw_pixel(dev, (int16_t)(cx + x), (int16_t)(cy + y), color);
        (void)splc501c_draw_pixel(dev, (int16_t)(cx - x), (int16_t)(cy + y), color);
        (void)splc501c_draw_pixel(dev, (int16_t)(cx + x), (int16_t)(cy - y), color);
        (void)splc501c_draw_pixel(dev, (int16_t)(cx - x), (int16_t)(cy - y), color);
        (void)splc501c_draw_pixel(dev, (int16_t)(cx + y), (int16_t)(cy + x), color);
        (void)splc501c_draw_pixel(dev, (int16_t)(cx - y), (int16_t)(cy + x), color);
        (void)splc501c_draw_pixel(dev, (int16_t)(cx + y), (int16_t)(cy - x), color);
        (void)splc501c_draw_pixel(dev, (int16_t)(cx - y), (int16_t)(cy - x), color);

        y++;
        if (err < 0)
        {
            err += (2 * y) + 1;
        }
        else
        {
            x--;
            err += (2 * (y - x)) + 1;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_fill_circle(splc501c* const dev, int16_t cx, int16_t cy,
                          int16_t radius, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }
    if (radius < 0)
    {
        return true;
    }

    // One span per line, so SPLC501C_COLOR_INVERSE cannot cancel itself out
    const int32_t r2 = (int32_t)radius * (int32_t)radius;
    for (int32_t dy = -radius; dy <= radius; dy++)
    {
        int32_t dx = 0;
        while ((((dx + 1) * (dx + 1)) + (dy * dy)) <= r2)
        {
            dx++;
        }
        (void)splc501c_draw_hline(dev, (int16_t)(cx - dx), (int16_t)(cy + dy),
                                  (int16_t)((2 * dx) + 1), color);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_char(splc501c* const dev, char c, int16_t x, int16_t y,
                        splc501c_color color)
{
    if (!dev)
    {
        return false;
    }

    // The font only holds ASCII 32..127, everything else becomes a space
    unsigned code = (unsigned)(unsigned char)c;
    if (code < SPLC501C_FONT_FIRST_CHAR || code > SPLC501C_FONT_LAST_CHAR)
    {
        code = SPLC501C_FONT_FIRST_CHAR;
    }

    const size_t glyph = (size_t)(code - SPLC501C_FONT_FIRST_CHAR)
                         * SPLC501C_FONT_WIDTH;

    // Only the lit pixels are touched, the background stays as it is
    for (int32_t col = 0; col < SPLC501C_FONT_WIDTH; col++)
    {
        const uint8_t bits = splc501c_font5x7[glyph + (size_t)col];
        for (int32_t row = 0; row < SPLC501C_FONT_HEIGHT; row++)
        {
            if (((bits >> (unsigned)row) & 0x01u) != 0u)
            {
                (void)splc501c_draw_pixel(dev, (int16_t)(x + col),
                                          (int16_t)(y + row), color);
            }
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_string(splc501c* const dev, const char* str, int16_t x,
                          int16_t y, splc501c_color color)
{
    if (!dev || !str)
    {
        return false;
    }

    int32_t pos = x;
    while (*str != '\0')
    {
        // Stop once the glyph would start past the right edge of the panel
        if (pos >= SPLC501C_WIDTH)
        {
            break;
        }
        (void)splc501c_draw_char(dev, *str, (int16_t)pos, y, color);
        pos += SPLC501C_CHAR_WIDTH;
        str++;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_set_cursor(splc501c* const dev, int16_t x, int16_t y)
{
    if (!dev)
    {
        return false;
    }

    dev->cursor_x = x;
    dev->cursor_y = y;
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_set_text_color(splc501c* const dev, splc501c_color color)
{
    if (!dev)
    {
        return false;
    }

    dev->text_color = color;
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_print(splc501c* const dev, const char* str)
{
    if (!dev || !str)
    {
        return false;
    }

    while (*str != '\0')
    {
        if (*str == '\n')
        {
            dev->cursor_x = 0;
            dev->cursor_y = (int16_t)(dev->cursor_y + SPLC501C_PIXELS_PER_PAGE);
        }
        else if (*str != '\r')
        {
            if ((dev->cursor_x + SPLC501C_CHAR_WIDTH) > SPLC501C_WIDTH)
            {
                dev->cursor_x = 0;
                dev->cursor_y = (int16_t)(dev->cursor_y + SPLC501C_PIXELS_PER_PAGE);
            }
            (void)splc501c_draw_char(dev, *str, dev->cursor_x, dev->cursor_y,
                                     dev->text_color);
            dev->cursor_x = (int16_t)(dev->cursor_x + SPLC501C_CHAR_WIDTH);
        }
        str++;
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_bitmap(splc501c* const dev, int16_t x, int16_t y,
                          int16_t width, int16_t height,
                          const uint8_t* bitmap, splc501c_color color)
{
    if (!dev || !bitmap)
    {
        return false;
    }
    if (width <= 0 || height <= 0)
    {
        return true;
    }

    const size_t row_bytes = ((size_t)width + 7u) / 8u;

    for (int32_t row = 0; row < height; row++)
    {
        const int32_t py = y + row;
        if (py < 0 || py >= SPLC501C_HEIGHT)
        {
            continue;
        }
        for (int32_t col = 0; col < width; col++)
        {
            const int32_t px = x + col;
            if (px < 0 || px >= SPLC501C_WIDTH)
            {
                continue;
            }
            const uint8_t byte = bitmap[((size_t)row * row_bytes)
                                        + ((size_t)col / 8u)];
            if (((byte >> (7u - ((unsigned)col % 8u))) & 0x01u) != 0u)
            {
                (void)splc501c_draw_pixel(dev, (int16_t)px, (int16_t)py, color);
            }
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
bool splc501c_draw_image(splc501c* const dev, const uint8_t* image,
                         size_t img_size)
{
    if (!dev || !image)
    {
        return false;
    }

    const size_t len = (img_size < sizeof(dev->buf)) ? img_size
                                                     : sizeof(dev->buf);
    memcpy(dev->buf, image, len);
    return true;
}

//-----------------------------------------------------------------------------
