/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "spfd5408.h"
#include <string.h>

// Header of a UTFT font: glyph width, glyph height, first code, glyph count
#define SPFD5408_FONT_HEADER_SIZE   4

// Power-up and reset timings taken from the datasheet
#define SPFD5408_RESET_LOW_MS       10
#define SPFD5408_RESET_HIGH_MS      120
#define SPFD5408_OSC_SETTLE_MS      500
#define SPFD5408_POWER_SETTLE_MS    200
#define SPFD5408_POWER_STEP_MS      50
#define SPFD5408_SHORT_DELAY_MS     10
#define SPFD5408_DISPLAY_ON_MS      100

// Entry mode: 65k colours, horizontal address counter, both counters increment
#define SPFD5408_ENTRY_MODE_DEFAULT 0x1030

// POWER_CTRL7 holds VCOMH in the lower six bits
#define SPFD5408_POWER_CTRL7_MASK   0x003F

/**
 * Built-in 8x8 font in the UTFT layout, covers ASCII 32..127.
 */
static const uint8_t spfd5408_font8x8[] =
{
    0x08, 0x08, 0x20, 0x60, // xSize, ySize, first char, number of chars
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // (space)
    0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x20, 0x00, // !
    0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, // "
    0x50, 0x50, 0xF8, 0x50, 0xF8, 0x50, 0x50, 0x00, // #
    0x20, 0x78, 0xA0, 0x70, 0x28, 0xF0, 0x20, 0x00, // $
    0xC0, 0xC8, 0x10, 0x20, 0x40, 0x98, 0x18, 0x00, // %
    0x60, 0x90, 0xA0, 0x40, 0xA8, 0x90, 0x68, 0x00, // &
    0x60, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, // '
    0x10, 0x20, 0x40, 0x40, 0x40, 0x20, 0x10, 0x00, // (
    0x40, 0x20, 0x10, 0x10, 0x10, 0x20, 0x40, 0x00, // )
    0x00, 0x50, 0x20, 0xF8, 0x20, 0x50, 0x00, 0x00, // *
    0x00, 0x20, 0x20, 0xF8, 0x20, 0x20, 0x00, 0x00, // +
    0x00, 0x00, 0x00, 0x00, 0x60, 0x20, 0x40, 0x00, // ,
    0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, // -
    0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x00, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, // /
    0x70, 0x88, 0x98, 0xA8, 0xC8, 0x88, 0x70, 0x00, // 0
    0x20, 0x60, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00, // 1
    0x70, 0x88, 0x08, 0x10, 0x20, 0x40, 0xF8, 0x00, // 2
    0xF8, 0x10, 0x20, 0x10, 0x08, 0x88, 0x70, 0x00, // 3
    0x10, 0x30, 0x50, 0x90, 0xF8, 0x10, 0x10, 0x00, // 4
    0xF8, 0x80, 0xF0, 0x08, 0x08, 0x88, 0x70, 0x00, // 5
    0x30, 0x40, 0x80, 0xF0, 0x88, 0x88, 0x70, 0x00, // 6
    0xF8, 0x08, 0x10, 0x20, 0x40, 0x40, 0x40, 0x00, // 7
    0x70, 0x88, 0x88, 0x70, 0x88, 0x88, 0x70, 0x00, // 8
    0x70, 0x88, 0x88, 0x78, 0x08, 0x10, 0x60, 0x00, // 9
    0x00, 0x60, 0x60, 0x00, 0x60, 0x60, 0x00, 0x00, // :
    0x00, 0x60, 0x60, 0x00, 0x60, 0x20, 0x40, 0x00, // ;
    0x08, 0x10, 0x20, 0x40, 0x20, 0x10, 0x08, 0x00, // <
    0x00, 0x00, 0xF8, 0x00, 0xF8, 0x00, 0x00, 0x00, // =
    0x80, 0x40, 0x20, 0x10, 0x20, 0x40, 0x80, 0x00, // >
    0x70, 0x88, 0x08, 0x10, 0x20, 0x00, 0x20, 0x00, // ?
    0x70, 0x88, 0x08, 0x68, 0xA8, 0xA8, 0x70, 0x00, // @
    0x70, 0x88, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x00, // A
    0xF0, 0x88, 0x88, 0xF0, 0x88, 0x88, 0xF0, 0x00, // B
    0x70, 0x88, 0x80, 0x80, 0x80, 0x88, 0x70, 0x00, // C
    0xE0, 0x90, 0x88, 0x88, 0x88, 0x90, 0xE0, 0x00, // D
    0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0xF8, 0x00, // E
    0xF8, 0x80, 0x80, 0xE0, 0x80, 0x80, 0x80, 0x00, // F
    0x70, 0x88, 0x80, 0x80, 0x98, 0x88, 0x70, 0x00, // G
    0x88, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x88, 0x00, // H
    0x70, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00, // I
    0x38, 0x10, 0x10, 0x10, 0x10, 0x90, 0x60, 0x00, // J
    0x88, 0x90, 0xA0, 0xC0, 0xA0, 0x90, 0x88, 0x00, // K
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xF8, 0x00, // L
    0x88, 0xD8, 0xA8, 0x88, 0x88, 0x88, 0x88, 0x00, // M
    0x88, 0x88, 0xC8, 0xA8, 0x98, 0x88, 0x88, 0x00, // N
    0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, // O
    0xF0, 0x88, 0x88, 0xF0, 0x80, 0x80, 0x80, 0x00, // P
    0x70, 0x88, 0x88, 0x88, 0xA8, 0x90, 0x68, 0x00, // Q
    0xF0, 0x88, 0x88, 0xF0, 0xA0, 0x90, 0x88, 0x00, // R
    0x78, 0x80, 0x80, 0x70, 0x08, 0x08, 0xF0, 0x00, // S
    0xF8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, // T
    0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, // U
    0x88, 0x88, 0x88, 0x88, 0x88, 0x50, 0x20, 0x00, // V
    0x88, 0x88, 0x88, 0xA8, 0xA8, 0xD8, 0x88, 0x00, // W
    0x88, 0x88, 0x50, 0x20, 0x50, 0x88, 0x88, 0x00, // X
    0x88, 0x88, 0x50, 0x20, 0x20, 0x20, 0x20, 0x00, // Y
    0xF8, 0x08, 0x10, 0x20, 0x40, 0x80, 0xF8, 0x00, // Z
    0x38, 0x20, 0x20, 0x20, 0x20, 0x20, 0x38, 0x00, // [
    0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, // backslash
    0xE0, 0x20, 0x20, 0x20, 0x20, 0x20, 0xE0, 0x00, // ]
    0x20, 0x50, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, // ^
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, // _
    0x40, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, // `
    0x00, 0x00, 0x70, 0x08, 0x78, 0x88, 0x78, 0x00, // a
    0x80, 0x80, 0xB0, 0xC8, 0x88, 0x88, 0xF0, 0x00, // b
    0x00, 0x00, 0x70, 0x80, 0x80, 0x88, 0x70, 0x00, // c
    0x08, 0x08, 0x68, 0x98, 0x88, 0x88, 0x78, 0x00, // d
    0x00, 0x00, 0x70, 0x88, 0xF8, 0x80, 0x70, 0x00, // e
    0x30, 0x48, 0x40, 0xE0, 0x40, 0x40, 0x40, 0x00, // f
    0x00, 0x00, 0x78, 0x88, 0x78, 0x08, 0x30, 0x00, // g
    0x80, 0x80, 0xB0, 0xC8, 0x88, 0x88, 0x88, 0x00, // h
    0x20, 0x00, 0x60, 0x20, 0x20, 0x20, 0x70, 0x00, // i
    0x10, 0x00, 0x30, 0x10, 0x10, 0x90, 0x60, 0x00, // j
    0x40, 0x40, 0x48, 0x50, 0x60, 0x50, 0x48, 0x00, // k
    0x60, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00, // l
    0x00, 0x00, 0xD0, 0xA8, 0xA8, 0x88, 0x88, 0x00, // m
    0x00, 0x00, 0xB0, 0xC8, 0x88, 0x88, 0x88, 0x00, // n
    0x00, 0x00, 0x70, 0x88, 0x88, 0x88, 0x70, 0x00, // o
    0x00, 0x00, 0xF0, 0x88, 0xF0, 0x80, 0x80, 0x00, // p
    0x00, 0x00, 0x68, 0x98, 0x78, 0x08, 0x08, 0x00, // q
    0x00, 0x00, 0xB0, 0xC8, 0x80, 0x80, 0x80, 0x00, // r
    0x00, 0x00, 0x70, 0x80, 0x70, 0x08, 0xF0, 0x00, // s
    0x40, 0x40, 0xE0, 0x40, 0x40, 0x48, 0x30, 0x00, // t
    0x00, 0x00, 0x88, 0x88, 0x88, 0x98, 0x68, 0x00, // u
    0x00, 0x00, 0x88, 0x88, 0x88, 0x50, 0x20, 0x00, // v
    0x00, 0x00, 0x88, 0x88, 0xA8, 0xA8, 0x50, 0x00, // w
    0x00, 0x00, 0x88, 0x50, 0x20, 0x50, 0x88, 0x00, // x
    0x00, 0x00, 0x88, 0x88, 0x78, 0x08, 0x70, 0x00, // y
    0x00, 0x00, 0xF8, 0x10, 0x20, 0x40, 0xF8, 0x00, // z
    0x10, 0x20, 0x20, 0x40, 0x20, 0x20, 0x10, 0x00, // {
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, // |
    0x40, 0x20, 0x20, 0x10, 0x20, 0x20, 0x40, 0x00, // }
    0x00, 0x20, 0x10, 0xF8, 0x10, 0x20, 0x00, 0x00, // right arrow
    0x00, 0x20, 0x40, 0xF8, 0x40, 0x20, 0x00, 0x00, // left arrow
};

/**
 * @brief Rectangle already clipped to the visible screen area
 */
typedef struct
{
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
} spfd5408_rect;

//-----------------------------------------------------------------------------
static void spfd5408_select(const spfd5408* const dev, bool selected)
{
    // CS is active low
    spfd5408_set_pin(dev, SPFD5408_PIN_CS, !selected);
}

//-----------------------------------------------------------------------------
static void spfd5408_bus_strobe(const spfd5408* const dev, uint16_t value)
{
    spfd5408_set_pin(dev, SPFD5408_PIN_WR, false);
    spfd5408_write_bus(dev, value);
    spfd5408_set_pin(dev, SPFD5408_PIN_WR, true);
}

//-----------------------------------------------------------------------------
static void spfd5408_send_cmd(const spfd5408* const dev, uint16_t index)
{
    spfd5408_set_pin(dev, SPFD5408_PIN_RS, false);
    spfd5408_bus_strobe(dev, index);
}

//-----------------------------------------------------------------------------
static void spfd5408_send_data(const spfd5408* const dev, uint16_t data)
{
    spfd5408_set_pin(dev, SPFD5408_PIN_RS, true);
    spfd5408_bus_strobe(dev, data);
}

//-----------------------------------------------------------------------------
static void spfd5408_write_reg(const spfd5408* const dev, uint16_t reg,
                               uint16_t value)
{
    spfd5408_select(dev, true);
    spfd5408_send_cmd(dev, reg);
    spfd5408_send_data(dev, value);
    spfd5408_select(dev, false);
}

//-----------------------------------------------------------------------------
static void spfd5408_swap(int32_t* a, int32_t* b)
{
    const int32_t tmp = *a;
    *a = *b;
    *b = tmp;
}

//-----------------------------------------------------------------------------
/**
 * Normalise and clip a rectangle given in user coordinates. Returns false when
 * nothing of it is visible, which is what keeps every primitive on screen.
 */
static bool spfd5408_clip_rect(const spfd5408* const dev, int32_t x1,
                               int32_t y1, int32_t x2, int32_t y2,
                               spfd5408_rect* const out)
{
    const int32_t w = spfd5408_get_width(dev);
    const int32_t h = spfd5408_get_height(dev);

    if (x1 > x2)
    {
        spfd5408_swap(&x1, &x2);
    }
    if (y1 > y2)
    {
        spfd5408_swap(&y1, &y2);
    }

    if (x2 < 0 || y2 < 0 || x1 >= w || y1 >= h)
    {
        return false;
    }

    out->x1 = (int16_t)((x1 < 0) ? 0 : x1);
    out->y1 = (int16_t)((y1 < 0) ? 0 : y1);
    out->x2 = (int16_t)((x2 >= w) ? (w - 1) : x2);
    out->y2 = (int16_t)((y2 >= h) ? (h - 1) : y2);
    return true;
}

//-----------------------------------------------------------------------------
/**
 * Program the GRAM window and leave the controller waiting for pixel data.
 * The caller has to assert CS before and release it after the burst.
 */
static void spfd5408_prepare_window(const spfd5408* const dev,
                                    const spfd5408_rect* const r)
{
    uint16_t px1;
    uint16_t py1;
    uint16_t px2;
    uint16_t py2;

    if (dev->orientation == SPFD5408_ORIENTATION_LANDSCAPE)
    {
        // The panel is used sideways: user x runs along the panel y axis
        px1 = (uint16_t)r->y1;
        px2 = (uint16_t)r->y2;
        py1 = (uint16_t)(SPFD5408_PANEL_MAX_Y - r->x2);
        py2 = (uint16_t)(SPFD5408_PANEL_MAX_Y - r->x1);
    }
    else
    {
        px1 = (uint16_t)r->x1;
        px2 = (uint16_t)r->x2;
        py1 = (uint16_t)r->y1;
        py2 = (uint16_t)r->y2;
    }

    spfd5408_send_cmd(dev, SPFD5408_REG_WINDOW_X_START);
    spfd5408_send_data(dev, px1);
    spfd5408_send_cmd(dev, SPFD5408_REG_WINDOW_X_END);
    spfd5408_send_data(dev, px2);
    spfd5408_send_cmd(dev, SPFD5408_REG_WINDOW_Y_START);
    spfd5408_send_data(dev, py1);
    spfd5408_send_cmd(dev, SPFD5408_REG_WINDOW_Y_END);
    spfd5408_send_data(dev, py2);
    spfd5408_send_cmd(dev, SPFD5408_REG_GRAM_ADDR_X);
    spfd5408_send_data(dev, px1);
    spfd5408_send_cmd(dev, SPFD5408_REG_GRAM_ADDR_Y);
    spfd5408_send_data(dev, py1);
    spfd5408_send_cmd(dev, SPFD5408_REG_GRAM_WRITE);
}

//-----------------------------------------------------------------------------
static bool spfd5408_fill_area(const spfd5408* const dev,
                               const spfd5408_rect* const r, uint16_t color)
{
    const uint32_t count = (uint32_t)((r->x2 - r->x1) + 1)
                           * (uint32_t)((r->y2 - r->y1) + 1);

    spfd5408_select(dev, true);
    spfd5408_prepare_window(dev, r);
    for (uint32_t i = 0; i < count; i++)
    {
        spfd5408_send_data(dev, color);
    }
    spfd5408_select(dev, false);
    return true;
}

//-----------------------------------------------------------------------------
static bool spfd5408_load_font(spfd5408* const dev, const uint8_t* font)
{
    const uint8_t* const data = (font != NULL) ? font : spfd5408_font8x8;
    const uint8_t x_size = data[0];
    const uint8_t y_size = data[1];
    const uint8_t num_chars = data[3];

    if (x_size == 0u || (x_size % 8u) != 0u || y_size == 0u || num_chars == 0u)
    {
        return false;
    }

    dev->font.data = data;
    dev->font.x_size = x_size;
    dev->font.y_size = y_size;
    dev->font.offset = data[2];
    dev->font.num_chars = num_chars;
    return true;
}

//-----------------------------------------------------------------------------
static void spfd5408_run_init_sequence(const spfd5408* const dev)
{
    spfd5408_write_reg(dev, SPFD5408_REG_INTERNAL_TIMING, 0x8000);
    spfd5408_delay_ms(SPFD5408_OSC_SETTLE_MS);
    spfd5408_write_reg(dev, SPFD5408_REG_START_OSC, 0x0001);
    spfd5408_write_reg(dev, SPFD5408_REG_DRIVER_OUTPUT_CTRL, 0x0100);
    spfd5408_write_reg(dev, SPFD5408_REG_LCD_DRIVING_WAVE_CTRL, 0x0700);
    spfd5408_write_reg(dev, SPFD5408_REG_ENTRY_MODE, SPFD5408_ENTRY_MODE_DEFAULT);
    spfd5408_write_reg(dev, SPFD5408_REG_RESIZE_CTRL, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_DISPLAY_CTRL2, 0x0202);
    spfd5408_write_reg(dev, SPFD5408_REG_DISPLAY_CTRL3, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_DISPLAY_CTRL4, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_EXT_IF_CTRL1, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_FRAME_MARKER_POS, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_EXT_IF_CTRL2, 0x0000);
    spfd5408_delay_ms(SPFD5408_POWER_SETTLE_MS);

    // Power on sequence
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL1, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL2, 0x0007);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL3, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL4, 0x0000);
    spfd5408_delay_ms(SPFD5408_POWER_STEP_MS);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL1, 0x17B0);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL2, 0x0007);
    spfd5408_delay_ms(SPFD5408_SHORT_DELAY_MS);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL3, 0x013A);
    spfd5408_delay_ms(SPFD5408_SHORT_DELAY_MS);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL4, 0x1A00);
    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL7,
                       (uint16_t)(dev->contrast & SPFD5408_POWER_CTRL7_MASK));
    spfd5408_delay_ms(SPFD5408_SHORT_DELAY_MS);

    // Gamma control
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL1, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL2, 0x0505);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL3, 0x0004);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL4, 0x0006);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL5, 0x0707);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL6, 0x0105);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL7, 0x0002);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL8, 0x0707);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL9, 0x0704);
    spfd5408_write_reg(dev, SPFD5408_REG_GAMMA_CTRL10, 0x0807);

    // Full screen GRAM window
    spfd5408_write_reg(dev, SPFD5408_REG_WINDOW_X_START, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_WINDOW_X_END, SPFD5408_PANEL_MAX_X);
    spfd5408_write_reg(dev, SPFD5408_REG_WINDOW_Y_START, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_WINDOW_Y_END, SPFD5408_PANEL_MAX_Y);
    spfd5408_write_reg(dev, SPFD5408_REG_GATE_SCAN_CTRL1, 0x2700);
    spfd5408_write_reg(dev, SPFD5408_REG_GATE_SCAN_CTRL2, 0x0001);
    spfd5408_write_reg(dev, SPFD5408_REG_VERTICAL_SCROLL, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_GRAM_ADDR_Y, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_GRAM_ADDR_X, 0x0000);

    // Partial display control, both windows disabled
    spfd5408_write_reg(dev, SPFD5408_REG_PARTIAL_POS1, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_PARTIAL_START1, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_PARTIAL_END1, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_PARTIAL_POS2, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_PARTIAL_START2, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_PARTIAL_END2, 0x0000);

    // Panel interface control
    spfd5408_write_reg(dev, SPFD5408_REG_PANEL_IF_CTRL1, 0x0010);
    spfd5408_write_reg(dev, SPFD5408_REG_PANEL_IF_CTRL2, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_PANEL_IF_CTRL3, 0x0003);
    spfd5408_write_reg(dev, SPFD5408_REG_PANEL_IF_CTRL4, 0x0110);
    spfd5408_write_reg(dev, SPFD5408_REG_PANEL_IF_CTRL5, 0x0000);
    spfd5408_write_reg(dev, SPFD5408_REG_PANEL_IF_CTRL6, 0x0000);

    spfd5408_delay_ms(SPFD5408_DISPLAY_ON_MS);
    spfd5408_write_reg(dev, SPFD5408_REG_DISPLAY_CTRL1,
                       SPFD5408_DISPLAY_CTRL1_ON);
    spfd5408_delay_ms(SPFD5408_DISPLAY_ON_MS);
}

//-----------------------------------------------------------------------------
bool spfd5408_init(spfd5408* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (dev->orientation != SPFD5408_ORIENTATION_LANDSCAPE)
    {
        dev->orientation = SPFD5408_ORIENTATION_PORTRAIT;
    }
    if (dev->fg_color == 0u && dev->bg_color == 0u)
    {
        dev->fg_color = SPFD5408_COLOR_WHITE;
        dev->bg_color = SPFD5408_COLOR_BLACK;
    }
    dev->contrast = SPFD5408_CONTRAST_DEFAULT;
    dev->display_on = false;

    if (!spfd5408_load_font(dev, NULL))
    {
        return false;
    }

    return spfd5408_reset(dev);
}

//-----------------------------------------------------------------------------
bool spfd5408_reset(spfd5408* const dev)
{
    if (!dev)
    {
        return false;
    }

    spfd5408_set_pin(dev, SPFD5408_PIN_CS, true);
    spfd5408_set_pin(dev, SPFD5408_PIN_RD, true);
    spfd5408_set_pin(dev, SPFD5408_PIN_WR, true);
    spfd5408_set_pin(dev, SPFD5408_PIN_RS, true);

    spfd5408_set_pin(dev, SPFD5408_PIN_RESET, false);
    spfd5408_delay_ms(SPFD5408_RESET_LOW_MS);
    spfd5408_set_pin(dev, SPFD5408_PIN_RESET, true);
    spfd5408_delay_ms(SPFD5408_RESET_HIGH_MS);

    spfd5408_run_init_sequence(dev);
    dev->display_on = true;

    return spfd5408_clear_screen(dev);
}

//-----------------------------------------------------------------------------
bool spfd5408_set_display_on(spfd5408* const dev, bool on)
{
    if (!dev)
    {
        return false;
    }

    spfd5408_write_reg(dev, SPFD5408_REG_DISPLAY_CTRL1,
                       on ? (uint16_t)SPFD5408_DISPLAY_CTRL1_ON
                          : (uint16_t)SPFD5408_DISPLAY_CTRL1_OFF);
    dev->display_on = on;
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_set_contrast(spfd5408* const dev, uint8_t contrast)
{
    if (!dev)
    {
        return false;
    }

    if (contrast > SPFD5408_CONTRAST_MAX)
    {
        contrast = (uint8_t)SPFD5408_CONTRAST_MAX;
    }

    spfd5408_write_reg(dev, SPFD5408_REG_POWER_CTRL7, (uint16_t)contrast);
    dev->contrast = contrast;
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_set_orientation(spfd5408* const dev,
                              spfd5408_orientation orientation)
{
    if (!dev)
    {
        return false;
    }

    dev->orientation = orientation;
    return true;
}

//-----------------------------------------------------------------------------
int16_t spfd5408_get_width(const spfd5408* const dev)
{
    if (!dev)
    {
        return 0;
    }
    return (dev->orientation == SPFD5408_ORIENTATION_LANDSCAPE)
               ? (int16_t)SPFD5408_PANEL_HEIGHT
               : (int16_t)SPFD5408_PANEL_WIDTH;
}

//-----------------------------------------------------------------------------
int16_t spfd5408_get_height(const spfd5408* const dev)
{
    if (!dev)
    {
        return 0;
    }
    return (dev->orientation == SPFD5408_ORIENTATION_LANDSCAPE)
               ? (int16_t)SPFD5408_PANEL_WIDTH
               : (int16_t)SPFD5408_PANEL_HEIGHT;
}

//-----------------------------------------------------------------------------
bool spfd5408_set_colors(spfd5408* const dev, uint16_t fg, uint16_t bg)
{
    if (!dev)
    {
        return false;
    }

    dev->fg_color = fg;
    dev->bg_color = bg;
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_clear_screen(spfd5408* const dev)
{
    if (!dev)
    {
        return false;
    }
    return spfd5408_fill_screen(dev, dev->bg_color);
}

//-----------------------------------------------------------------------------
bool spfd5408_fill_screen(spfd5408* const dev, uint16_t color)
{
    if (!dev)
    {
        return false;
    }

    const spfd5408_rect r = {
        0, 0,
        (int16_t)(spfd5408_get_width(dev) - 1),
        (int16_t)(spfd5408_get_height(dev) - 1),
    };
    return spfd5408_fill_area(dev, &r, color);
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_pixel(spfd5408* const dev, int16_t x, int16_t y,
                         uint16_t color)
{
    if (!dev)
    {
        return false;
    }

    spfd5408_rect r;
    if (!spfd5408_clip_rect(dev, x, y, x, y, &r))
    {
        return false;
    }

    spfd5408_select(dev, true);
    spfd5408_prepare_window(dev, &r);
    spfd5408_send_data(dev, color);
    spfd5408_select(dev, false);
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_hline(spfd5408* const dev, int16_t x, int16_t y,
                         int16_t len, uint16_t color)
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
    if (count == 0)
    {
        return true;
    }

    spfd5408_rect r;
    if (!spfd5408_clip_rect(dev, x0, y, x0 + count - 1, y, &r))
    {
        return true;
    }
    return spfd5408_fill_area(dev, &r, color);
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_vline(spfd5408* const dev, int16_t x, int16_t y,
                         int16_t len, uint16_t color)
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
    if (count == 0)
    {
        return true;
    }

    spfd5408_rect r;
    if (!spfd5408_clip_rect(dev, x, y0, x, y0 + count - 1, &r))
    {
        return true;
    }
    return spfd5408_fill_area(dev, &r, color);
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_line(spfd5408* const dev, int16_t x0, int16_t y0,
                        int16_t x1, int16_t y1, uint16_t color)
{
    if (!dev)
    {
        return false;
    }

    if (y0 == y1)
    {
        const int16_t left = (x0 < x1) ? x0 : x1;
        const int16_t len = (int16_t)(((x0 < x1) ? (x1 - x0) : (x0 - x1)) + 1);
        return spfd5408_draw_hline(dev, left, y0, len, color);
    }
    if (x0 == x1)
    {
        const int16_t top = (y0 < y1) ? y0 : y1;
        const int16_t len = (int16_t)(((y0 < y1) ? (y1 - y0) : (y0 - y1)) + 1);
        return spfd5408_draw_vline(dev, x0, top, len, color);
    }

    // Plain Bresenham, out of range points are dropped by spfd5408_draw_pixel()
    int32_t cx = x0;
    int32_t cy = y0;
    const int32_t dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    const int32_t dy = (y1 > y0) ? (y0 - y1) : (y1 - y0); // negative
    const int32_t sx = (x0 < x1) ? 1 : -1;
    const int32_t sy = (y0 < y1) ? 1 : -1;
    int32_t err = dx + dy;

    for (;;)
    {
        (void)spfd5408_draw_pixel(dev, (int16_t)cx, (int16_t)cy, color);
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
bool spfd5408_draw_rect(spfd5408* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, uint16_t color)
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

    (void)spfd5408_draw_hline(dev, x, y, width, color);
    (void)spfd5408_draw_hline(dev, x, bottom, width, color);
    (void)spfd5408_draw_vline(dev, x, y, height, color);
    (void)spfd5408_draw_vline(dev, right, y, height, color);
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_fill_rect(spfd5408* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, uint16_t color)
{
    if (!dev)
    {
        return false;
    }
    if (width <= 0 || height <= 0)
    {
        return true;
    }

    spfd5408_rect r;
    if (!spfd5408_clip_rect(dev, x, y, x + width - 1, y + height - 1, &r))
    {
        return true;
    }
    return spfd5408_fill_area(dev, &r, color);
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_round_rect(spfd5408* const dev, int16_t x, int16_t y,
                              int16_t width, int16_t height, uint16_t color)
{
    if (!dev)
    {
        return false;
    }
    if (width < 5 || height < 5)
    {
        return spfd5408_draw_rect(dev, x, y, width, height, color);
    }

    const int16_t right = (int16_t)(x + width - 1);
    const int16_t bottom = (int16_t)(y + height - 1);

    (void)spfd5408_draw_pixel(dev, (int16_t)(x + 1), (int16_t)(y + 1), color);
    (void)spfd5408_draw_pixel(dev, (int16_t)(right - 1), (int16_t)(y + 1), color);
    (void)spfd5408_draw_pixel(dev, (int16_t)(x + 1), (int16_t)(bottom - 1), color);
    (void)spfd5408_draw_pixel(dev, (int16_t)(right - 1), (int16_t)(bottom - 1),
                              color);

    (void)spfd5408_draw_hline(dev, (int16_t)(x + 2), y, (int16_t)(width - 4), color);
    (void)spfd5408_draw_hline(dev, (int16_t)(x + 2), bottom, (int16_t)(width - 4),
                              color);
    (void)spfd5408_draw_vline(dev, x, (int16_t)(y + 2), (int16_t)(height - 4), color);
    (void)spfd5408_draw_vline(dev, right, (int16_t)(y + 2), (int16_t)(height - 4),
                              color);
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_fill_round_rect(spfd5408* const dev, int16_t x, int16_t y,
                              int16_t width, int16_t height, uint16_t color)
{
    if (!dev)
    {
        return false;
    }
    if (width < 5 || height < 5)
    {
        return spfd5408_fill_rect(dev, x, y, width, height, color);
    }

    const int32_t bottom = y + height - 1;

    for (int32_t i = 0; i <= ((height - 1) / 2); i++)
    {
        int32_t start;
        int32_t len;
        if (i == 0)
        {
            start = x + 2;
            len = width - 4;
        }
        else if (i == 1)
        {
            start = x + 1;
            len = width - 2;
        }
        else
        {
            start = x;
            len = width;
        }

        (void)spfd5408_draw_hline(dev, (int16_t)start, (int16_t)(y + i),
                                  (int16_t)len, color);
        (void)spfd5408_draw_hline(dev, (int16_t)start, (int16_t)(bottom - i),
                                  (int16_t)len, color);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_circle(spfd5408* const dev, int16_t cx, int16_t cy,
                          int16_t radius, uint16_t color)
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
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx + x), (int16_t)(cy + y), color);
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx - x), (int16_t)(cy + y), color);
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx + x), (int16_t)(cy - y), color);
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx - x), (int16_t)(cy - y), color);
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx + y), (int16_t)(cy + x), color);
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx - y), (int16_t)(cy + x), color);
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx + y), (int16_t)(cy - x), color);
        (void)spfd5408_draw_pixel(dev, (int16_t)(cx - y), (int16_t)(cy - x), color);

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
bool spfd5408_fill_circle(spfd5408* const dev, int16_t cx, int16_t cy,
                          int16_t radius, uint16_t color)
{
    if (!dev)
    {
        return false;
    }
    if (radius < 0)
    {
        return true;
    }

    // One span per line, the old implementation drew every line twice
    const int32_t r2 = (int32_t)radius * (int32_t)radius;
    for (int32_t dy = -radius; dy <= radius; dy++)
    {
        int32_t dx = 0;
        while ((((dx + 1) * (dx + 1)) + (dy * dy)) <= r2)
        {
            dx++;
        }
        (void)spfd5408_draw_hline(dev, (int16_t)(cx - dx), (int16_t)(cy + dy),
                                  (int16_t)((2 * dx) + 1), color);
    }
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_set_font(spfd5408* const dev, const uint8_t* font)
{
    if (!dev)
    {
        return false;
    }
    return spfd5408_load_font(dev, font);
}

//-----------------------------------------------------------------------------
uint8_t spfd5408_get_font_width(const spfd5408* const dev)
{
    return dev ? dev->font.x_size : (uint8_t)0;
}

//-----------------------------------------------------------------------------
uint8_t spfd5408_get_font_height(const spfd5408* const dev)
{
    return dev ? dev->font.y_size : (uint8_t)0;
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_char(spfd5408* const dev, char c, int16_t x, int16_t y,
                        uint16_t color)
{
    if (!dev || !dev->font.data)
    {
        return false;
    }

    // Characters the font does not cover fall back to its very first glyph
    unsigned code = (unsigned)(unsigned char)c;
    const unsigned first = dev->font.offset;
    const unsigned last = first + dev->font.num_chars - 1u;
    if (code < first || code > last)
    {
        code = first;
    }

    const size_t row_bytes = (size_t)dev->font.x_size / 8u;
    const size_t glyph = SPFD5408_FONT_HEADER_SIZE
                         + ((size_t)(code - first) * row_bytes
                            * (size_t)dev->font.y_size);

    for (int32_t row = 0; row < dev->font.y_size; row++)
    {
        for (size_t byte_idx = 0; byte_idx < row_bytes; byte_idx++)
        {
            const uint8_t bits =
                dev->font.data[glyph + ((size_t)row * row_bytes) + byte_idx];
            for (int32_t bit = 0; bit < 8; bit++)
            {
                const int32_t px = x + (int32_t)(byte_idx * 8u) + bit;
                const bool on = ((bits >> (7 - (unsigned)bit)) & 0x01u) != 0u;
                (void)spfd5408_draw_pixel(dev, (int16_t)px, (int16_t)(y + row),
                                          on ? color : dev->bg_color);
            }
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_string(spfd5408* const dev, const char* str, int16_t x,
                          int16_t y, uint16_t color)
{
    if (!dev || !str || !dev->font.data)
    {
        return false;
    }

    const int32_t width = spfd5408_get_width(dev);
    const int32_t x_size = dev->font.x_size;
    const int32_t len = (int32_t)strlen(str);

    int32_t px;
    switch (x)
    {
        case SPFD5408_X_LEFT:
            px = 0;
            break;
        case SPFD5408_X_CENTER:
            px = (width - (len * x_size)) / 2;
            break;
        case SPFD5408_X_RIGHT:
            px = width - (len * x_size);
            break;
        default:
            px = x;
            break;
    }

    for (int32_t i = 0; i < len; i++)
    {
        if (px >= width)
        {
            break;
        }
        (void)spfd5408_draw_char(dev, str[i], (int16_t)px, y, color);
        px += x_size;
    }
    return true;
}

//-----------------------------------------------------------------------------
/**
 * Left pad a NUL terminated string in place, silently ignoring requests that
 * would not fit into the buffer.
 */
static void spfd5408_pad_left(char* buf, size_t buf_size, uint8_t length,
                              char filler)
{
    const size_t len = strlen(buf);
    if (length <= len || (size_t)length >= buf_size)
    {
        return;
    }

    const size_t pad = (size_t)length - len;
    memmove(&buf[pad], buf, len + 1u);
    memset(buf, filler, pad);
}

//-----------------------------------------------------------------------------
static void spfd5408_int_to_str(char* buf, size_t buf_size, int32_t num)
{
    char tmp[12];
    size_t n = 0;

    // Negate in the unsigned domain so INT32_MIN does not overflow
    const bool neg = (num < 0);
    uint32_t mag = neg ? (0u - (uint32_t)num) : (uint32_t)num;

    do
    {
        tmp[n++] = (char)('0' + (char)(mag % 10u));
        mag /= 10u;
    } while (mag != 0u && n < sizeof(tmp));

    size_t pos = 0;
    if (neg && (pos + 1u) < buf_size)
    {
        buf[pos++] = '-';
    }
    while (n > 0u && (pos + 1u) < buf_size)
    {
        buf[pos++] = tmp[--n];
    }
    buf[pos] = '\0';
}

//-----------------------------------------------------------------------------
static bool spfd5408_float_to_str(char* buf, size_t buf_size, float num,
                                  uint8_t dec, char divider)
{
    uint32_t scale = 1u;
    for (uint8_t i = 0; i < dec; i++)
    {
        scale *= 10u;
    }

    const bool neg = (num < 0.0f);
    const float mag = neg ? -num : num;

    // Also rejects NaN, for which every comparison is false
    if (!(mag < (2000000000.0f / (float)scale)))
    {
        return false;
    }

    const uint32_t scaled = (uint32_t)((mag * (float)scale) + 0.5f);
    uint32_t whole = scaled / scale;
    const uint32_t frac = scaled % scale;

    char tmp[12];
    size_t n = 0;
    do
    {
        tmp[n++] = (char)('0' + (char)(whole % 10u));
        whole /= 10u;
    } while (whole != 0u && n < sizeof(tmp));

    size_t pos = 0;
    if (neg && (pos + 1u) < buf_size)
    {
        buf[pos++] = '-';
    }
    while (n > 0u && (pos + 1u) < buf_size)
    {
        buf[pos++] = tmp[--n];
    }
    if ((pos + 1u) < buf_size)
    {
        buf[pos++] = divider;
    }

    uint32_t divisor = scale / 10u;
    while (divisor != 0u && (pos + 1u) < buf_size)
    {
        buf[pos++] = (char)('0' + (char)((frac / divisor) % 10u));
        divisor /= 10u;
    }
    buf[pos] = '\0';
    return true;
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_number(spfd5408* const dev, int32_t num, int16_t x,
                          int16_t y, uint8_t length, char filler,
                          uint16_t color)
{
    if (!dev)
    {
        return false;
    }

    char buf[SPFD5408_NUM_BUF_SIZE];
    spfd5408_int_to_str(buf, sizeof(buf), num);
    spfd5408_pad_left(buf, sizeof(buf), length, filler);
    return spfd5408_draw_string(dev, buf, x, y, color);
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_float(spfd5408* const dev, float num, uint8_t dec,
                         int16_t x, int16_t y, char divider, uint8_t length,
                         char filler, uint16_t color)
{
    if (!dev)
    {
        return false;
    }

    if (dec < SPFD5408_FLOAT_DEC_MIN)
    {
        dec = (uint8_t)SPFD5408_FLOAT_DEC_MIN;
    }
    else if (dec > SPFD5408_FLOAT_DEC_MAX)
    {
        dec = (uint8_t)SPFD5408_FLOAT_DEC_MAX;
    }

    char buf[SPFD5408_NUM_BUF_SIZE];
    if (!spfd5408_float_to_str(buf, sizeof(buf), num, dec, divider))
    {
        return false;
    }
    spfd5408_pad_left(buf, sizeof(buf), length, filler);
    return spfd5408_draw_string(dev, buf, x, y, color);
}

//-----------------------------------------------------------------------------
bool spfd5408_draw_bitmap(spfd5408* const dev, int16_t x, int16_t y,
                          int16_t width, int16_t height,
                          const uint16_t* bitmap, uint8_t scale)
{
    if (!dev || !bitmap)
    {
        return false;
    }
    if (width <= 0 || height <= 0)
    {
        return true;
    }

    const int32_t factor = (scale < 1u) ? 1 : (int32_t)scale;
    const int32_t dst_w = (int32_t)width * factor;
    const int32_t dst_h = (int32_t)height * factor;

    for (int32_t row = 0; row < dst_h; row++)
    {
        const int32_t dst_y = y + row;
        spfd5408_rect r;
        if (!spfd5408_clip_rect(dev, x, dst_y, x + dst_w - 1, dst_y, &r))
        {
            continue;
        }

        const int32_t src_row = row / factor;
        const bool landscape =
            (dev->orientation == SPFD5408_ORIENTATION_LANDSCAPE);

        spfd5408_select(dev, true);
        spfd5408_prepare_window(dev, &r);
        // Sideways the GRAM address counter walks the row backwards
        for (int32_t i = 0; i <= (r.x2 - r.x1); i++)
        {
            const int32_t dst_x = landscape ? (r.x2 - i) : (r.x1 + i);
            const int32_t src_col = (dst_x - x) / factor;
            spfd5408_send_data(dev,
                               bitmap[((size_t)src_row * (size_t)width)
                                      + (size_t)src_col]);
        }
        spfd5408_select(dev, false);
    }
    return true;
}

//-----------------------------------------------------------------------------
