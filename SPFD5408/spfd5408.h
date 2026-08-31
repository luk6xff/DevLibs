/**
 *  @brief:   SPFD5408 240x320 TFT LCD controller library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPFD5408_H__
#define __SPFD5408_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Panel geometry, given for the native (portrait) orientation
 */
#define SPFD5408_PANEL_WIDTH        240
#define SPFD5408_PANEL_HEIGHT       320
#define SPFD5408_PANEL_MAX_X        (SPFD5408_PANEL_WIDTH - 1)
#define SPFD5408_PANEL_MAX_Y        (SPFD5408_PANEL_HEIGHT - 1)

/**
 * @brief Basic RGB565 colours
 */
#define SPFD5408_COLOR_BLACK        0x0000
#define SPFD5408_COLOR_WHITE        0xFFFF
#define SPFD5408_COLOR_RED          0xF800
#define SPFD5408_COLOR_GREEN        0x07E0
#define SPFD5408_COLOR_BLUE         0x001F
#define SPFD5408_COLOR_YELLOW       0xFFE0
#define SPFD5408_COLOR_CYAN         0x07FF
#define SPFD5408_COLOR_MAGENTA      0xF81F
#define SPFD5408_COLOR_BRIGHT_RED   0xF810
#define SPFD5408_COLOR_GRAY1        0x8410
#define SPFD5408_COLOR_GRAY2        0x4208

/**
 * @brief Controller register map
 */
#define SPFD5408_REG_START_OSC              0x00
#define SPFD5408_REG_DRIVER_OUTPUT_CTRL     0x01
#define SPFD5408_REG_LCD_DRIVING_WAVE_CTRL  0x02
#define SPFD5408_REG_ENTRY_MODE             0x03
#define SPFD5408_REG_RESIZE_CTRL            0x04
#define SPFD5408_REG_DISPLAY_CTRL1          0x07
#define SPFD5408_REG_DISPLAY_CTRL2          0x08
#define SPFD5408_REG_DISPLAY_CTRL3          0x09
#define SPFD5408_REG_DISPLAY_CTRL4          0x0A
#define SPFD5408_REG_EXT_IF_CTRL1           0x0C
#define SPFD5408_REG_FRAME_MARKER_POS       0x0D
#define SPFD5408_REG_EXT_IF_CTRL2           0x0F
#define SPFD5408_REG_POWER_CTRL1            0x10
#define SPFD5408_REG_POWER_CTRL2            0x11
#define SPFD5408_REG_POWER_CTRL3            0x12
#define SPFD5408_REG_POWER_CTRL4            0x13
#define SPFD5408_REG_GRAM_ADDR_X            0x20
#define SPFD5408_REG_GRAM_ADDR_Y            0x21
#define SPFD5408_REG_GRAM_WRITE             0x22
#define SPFD5408_REG_POWER_CTRL7            0x29
#define SPFD5408_REG_GAMMA_CTRL1            0x30
#define SPFD5408_REG_GAMMA_CTRL2            0x31
#define SPFD5408_REG_GAMMA_CTRL3            0x32
#define SPFD5408_REG_GAMMA_CTRL4            0x35
#define SPFD5408_REG_GAMMA_CTRL5            0x36
#define SPFD5408_REG_GAMMA_CTRL6            0x37
#define SPFD5408_REG_GAMMA_CTRL7            0x38
#define SPFD5408_REG_GAMMA_CTRL8            0x39
#define SPFD5408_REG_GAMMA_CTRL9            0x3C
#define SPFD5408_REG_GAMMA_CTRL10           0x3D
#define SPFD5408_REG_WINDOW_X_START         0x50
#define SPFD5408_REG_WINDOW_X_END           0x51
#define SPFD5408_REG_WINDOW_Y_START         0x52
#define SPFD5408_REG_WINDOW_Y_END           0x53
#define SPFD5408_REG_GATE_SCAN_CTRL1        0x60
#define SPFD5408_REG_GATE_SCAN_CTRL2        0x61
#define SPFD5408_REG_VERTICAL_SCROLL        0x6A
#define SPFD5408_REG_PARTIAL_POS1           0x80
#define SPFD5408_REG_PARTIAL_START1         0x81
#define SPFD5408_REG_PARTIAL_END1           0x82
#define SPFD5408_REG_PARTIAL_POS2           0x83
#define SPFD5408_REG_PARTIAL_START2         0x84
#define SPFD5408_REG_PARTIAL_END2           0x85
#define SPFD5408_REG_PANEL_IF_CTRL1         0x90
#define SPFD5408_REG_PANEL_IF_CTRL2         0x92
#define SPFD5408_REG_PANEL_IF_CTRL3         0x93
#define SPFD5408_REG_PANEL_IF_CTRL4         0x95
#define SPFD5408_REG_PANEL_IF_CTRL5         0x97
#define SPFD5408_REG_PANEL_IF_CTRL6         0x98
#define SPFD5408_REG_INTERNAL_TIMING        0xE5

/** DISPLAY_CTRL1 values used to switch the panel on and off */
#define SPFD5408_DISPLAY_CTRL1_ON           0x0173
#define SPFD5408_DISPLAY_CTRL1_OFF          0x0000

/** Contrast is mapped onto POWER_CTRL7 (VCOMH), the register accepts 0..0x3F */
#define SPFD5408_CONTRAST_MAX               0x3F
#define SPFD5408_CONTRAST_DEFAULT           0x0C

/** Text drawn by spfd5408_draw_string() can be aligned instead of positioned */
#define SPFD5408_X_LEFT                     ((int16_t)-1)
#define SPFD5408_X_CENTER                   ((int16_t)-2)
#define SPFD5408_X_RIGHT                    ((int16_t)-3)

/** Number of decimal places accepted by spfd5408_draw_float() */
#define SPFD5408_FLOAT_DEC_MIN              1
#define SPFD5408_FLOAT_DEC_MAX              5

/** Longest string produced by the built-in number formatting */
#define SPFD5408_NUM_BUF_SIZE               32

/**
 * @brief Screen orientation
 */
typedef enum
{
    SPFD5408_ORIENTATION_PORTRAIT  = 0, // 240 x 320
    SPFD5408_ORIENTATION_LANDSCAPE = 1, // 320 x 240
} spfd5408_orientation;

/**
 * @brief Control lines driven by the platform layer
 */
typedef enum
{
    SPFD5408_PIN_RS    = 0, // Register (low) / data (high) select
    SPFD5408_PIN_WR    = 1, // Active low write strobe
    SPFD5408_PIN_RD    = 2, // Active low read strobe, kept high by this library
    SPFD5408_PIN_CS    = 3, // Active low chip select
    SPFD5408_PIN_RESET = 4, // Active low hardware reset
} spfd5408_pin;

/**
 * @brief Font descriptor
 *
 * The font data is the well known UTFT layout, four header bytes
 * (glyph width, glyph height, code of the first glyph, number of glyphs)
 * followed by the glyphs themselves. One glyph row takes (width / 8) bytes and
 * the most significant bit of a byte is the leftmost pixel.
 */
typedef struct
{
    const uint8_t* data; // Glyph data including the four header bytes
    uint8_t x_size;      // Glyph width in pixels, always a multiple of 8
    uint8_t y_size;      // Glyph height in pixels
    uint8_t offset;      // Character code of the first glyph
    uint8_t num_chars;   // Number of glyphs stored in the font
} spfd5408_font;

/**
 * @brief spfd5408 dev object
 */
typedef struct
{
    void* platform_dev;               // Platform descriptor, set by spfd5408_<platform>_init()
    spfd5408_orientation orientation; // Current screen orientation
    spfd5408_font font;               // Currently selected font
    uint16_t fg_color;                // Colour used when no explicit colour is given
    uint16_t bg_color;                // Colour used to clear the screen
    uint8_t contrast;                 // Cached VCOMH value 0..63
    bool display_on;                  // True when the panel is switched on
} spfd5408;

/**
 * @brief Initialize the controller, select the built-in font and clear the
 *        screen with the background colour.
 *
 * @param dev spfd5408 device object
 * @return True on success, false otherwise
 */
bool spfd5408_init(spfd5408* const dev);

/**
 * @brief Pulse the RESET line and re-run the whole power-up register sequence.
 *
 * @param dev spfd5408 device object
 * @return True on success, false otherwise
 */
bool spfd5408_reset(spfd5408* const dev);

/**
 * @brief Turn the panel on or off, the GRAM content is preserved.
 *
 * @param dev spfd5408 device object
 * @param on  True to turn the display on
 * @return True on success, false otherwise
 */
bool spfd5408_set_display_on(spfd5408* const dev, bool on);

/**
 * @brief Set the VCOMH voltage, which is what this panel offers as contrast.
 *
 * @param dev      spfd5408 device object
 * @param contrast Contrast, clamped to SPFD5408_CONTRAST_MAX
 * @return True on success, false otherwise
 */
bool spfd5408_set_contrast(spfd5408* const dev, uint8_t contrast);

/**
 * @brief Select the screen orientation.
 *
 * @param dev         spfd5408 device object
 * @param orientation Orientation to be applied
 * @return True on success, false otherwise
 */
bool spfd5408_set_orientation(spfd5408* const dev,
                              spfd5408_orientation orientation);

/**
 * @brief Screen width in the current orientation.
 *
 * @param dev spfd5408 device object
 * @return Width in pixels, 0 when dev is NULL
 */
int16_t spfd5408_get_width(const spfd5408* const dev);

/**
 * @brief Screen height in the current orientation.
 *
 * @param dev spfd5408 device object
 * @return Height in pixels, 0 when dev is NULL
 */
int16_t spfd5408_get_height(const spfd5408* const dev);

/**
 * @brief Select the foreground and background colours.
 *
 * @param dev spfd5408 device object
 * @param fg  Foreground colour in RGB565
 * @param bg  Background colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_set_colors(spfd5408* const dev, uint16_t fg, uint16_t bg);

/**
 * @brief Fill the whole screen with the background colour.
 *
 * @param dev spfd5408 device object
 * @return True on success, false otherwise
 */
bool spfd5408_clear_screen(spfd5408* const dev);

/**
 * @brief Fill the whole screen with a given colour.
 *
 * @param dev   spfd5408 device object
 * @param color Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_fill_screen(spfd5408* const dev, uint16_t color);

/**
 * @brief Draw a single pixel, coordinates outside the screen are discarded.
 *
 * @param dev   spfd5408 device object
 * @param x     X coordinate
 * @param y     Y coordinate
 * @param color Colour in RGB565
 * @return True when the pixel was inside the screen, false otherwise
 */
bool spfd5408_draw_pixel(spfd5408* const dev, int16_t x, int16_t y,
                         uint16_t color);

/**
 * @brief Draw a horizontal line, clipped to the screen.
 *
 * @param dev   spfd5408 device object
 * @param x     X coordinate of the left end
 * @param y     Y coordinate
 * @param len   Length in pixels, negative values extend to the left
 * @param color Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_hline(spfd5408* const dev, int16_t x, int16_t y,
                         int16_t len, uint16_t color);

/**
 * @brief Draw a vertical line, clipped to the screen.
 *
 * @param dev   spfd5408 device object
 * @param x     X coordinate
 * @param y     Y coordinate of the top end
 * @param len   Length in pixels, negative values extend upwards
 * @param color Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_vline(spfd5408* const dev, int16_t x, int16_t y,
                         int16_t len, uint16_t color);

/**
 * @brief Draw a line between two points, clipped to the screen.
 *
 * @param dev   spfd5408 device object
 * @param x0    X coordinate of the first point
 * @param y0    Y coordinate of the first point
 * @param x1    X coordinate of the second point
 * @param y1    Y coordinate of the second point
 * @param color Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_line(spfd5408* const dev, int16_t x0, int16_t y0,
                        int16_t x1, int16_t y1, uint16_t color);

/**
 * @brief Draw a rectangle outline, clipped to the screen.
 *
 * @param dev    spfd5408 device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Rectangle width in pixels
 * @param height Rectangle height in pixels
 * @param color  Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_rect(spfd5408* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, uint16_t color);

/**
 * @brief Draw a filled rectangle, clipped to the screen.
 *
 * @param dev    spfd5408 device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Rectangle width in pixels
 * @param height Rectangle height in pixels
 * @param color  Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_fill_rect(spfd5408* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, uint16_t color);

/**
 * @brief Draw a rectangle outline with rounded corners, clipped to the screen.
 *
 * @param dev    spfd5408 device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Rectangle width in pixels, at least 5
 * @param height Rectangle height in pixels, at least 5
 * @param color  Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_round_rect(spfd5408* const dev, int16_t x, int16_t y,
                              int16_t width, int16_t height, uint16_t color);

/**
 * @brief Draw a filled rectangle with rounded corners, clipped to the screen.
 *
 * @param dev    spfd5408 device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Rectangle width in pixels, at least 5
 * @param height Rectangle height in pixels, at least 5
 * @param color  Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_fill_round_rect(spfd5408* const dev, int16_t x, int16_t y,
                              int16_t width, int16_t height, uint16_t color);

/**
 * @brief Draw a circle outline, clipped to the screen.
 *
 * @param dev    spfd5408 device object
 * @param cx     X coordinate of the centre
 * @param cy     Y coordinate of the centre
 * @param radius Radius in pixels
 * @param color  Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_circle(spfd5408* const dev, int16_t cx, int16_t cy,
                          int16_t radius, uint16_t color);

/**
 * @brief Draw a filled circle, clipped to the screen.
 *
 * @param dev    spfd5408 device object
 * @param cx     X coordinate of the centre
 * @param cy     Y coordinate of the centre
 * @param radius Radius in pixels
 * @param color  Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_fill_circle(spfd5408* const dev, int16_t cx, int16_t cy,
                          int16_t radius, uint16_t color);

/**
 * @brief Select the font used for text rendering.
 *
 * @param dev  spfd5408 device object
 * @param font Font data in the UTFT layout, NULL restores the built-in font
 * @return True on success, false when the header of the font is not usable
 */
bool spfd5408_set_font(spfd5408* const dev, const uint8_t* font);

/**
 * @brief Glyph width of the currently selected font.
 *
 * @param dev spfd5408 device object
 * @return Width in pixels, 0 when dev is NULL
 */
uint8_t spfd5408_get_font_width(const spfd5408* const dev);

/**
 * @brief Glyph height of the currently selected font.
 *
 * @param dev spfd5408 device object
 * @return Height in pixels, 0 when dev is NULL
 */
uint8_t spfd5408_get_font_height(const spfd5408* const dev);

/**
 * @brief Draw a single character of the currently selected font.
 *
 * The glyph is drawn opaque, the cleared pixels are filled with the background
 * colour of the device.
 *
 * @param dev   spfd5408 device object
 * @param c     Character to be drawn
 * @param x     X coordinate of the top left corner of the glyph
 * @param y     Y coordinate of the top left corner of the glyph
 * @param color Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_char(spfd5408* const dev, char c, int16_t x, int16_t y,
                        uint16_t color);

/**
 * @brief Draw a NUL terminated string.
 *
 * @param dev   spfd5408 device object
 * @param str   String to be drawn
 * @param x     X coordinate, or SPFD5408_X_LEFT / _CENTER / _RIGHT
 * @param y     Y coordinate of the top left corner of the first glyph
 * @param color Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_string(spfd5408* const dev, const char* str, int16_t x,
                          int16_t y, uint16_t color);

/**
 * @brief Draw a signed integer.
 *
 * @param dev    spfd5408 device object
 * @param num    Number to be drawn
 * @param x      X coordinate, or SPFD5408_X_LEFT / _CENTER / _RIGHT
 * @param y      Y coordinate of the top left corner of the first glyph
 * @param length Minimum number of characters, shorter results are left padded
 * @param filler Character used for the left padding
 * @param color  Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_number(spfd5408* const dev, int32_t num, int16_t x,
                          int16_t y, uint8_t length, char filler,
                          uint16_t color);

/**
 * @brief Draw a floating point number with a fixed number of decimal places.
 *
 * @param dev     spfd5408 device object
 * @param num     Number to be drawn
 * @param dec     Number of decimal places, clamped to 1..5
 * @param x       X coordinate, or SPFD5408_X_LEFT / _CENTER / _RIGHT
 * @param y       Y coordinate of the top left corner of the first glyph
 * @param divider Character used as the decimal separator
 * @param length  Minimum number of characters, shorter results are left padded
 * @param filler  Character used for the left padding
 * @param color   Colour in RGB565
 * @return True on success, false otherwise
 */
bool spfd5408_draw_float(spfd5408* const dev, float num, uint8_t dec,
                         int16_t x, int16_t y, char divider, uint8_t length,
                         char filler, uint16_t color);

/**
 * @brief Blit an RGB565 bitmap, optionally scaled up by an integer factor.
 *
 * Parts falling outside the screen are clipped away.
 *
 * @param dev    spfd5408 device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Bitmap width in pixels
 * @param height Bitmap height in pixels
 * @param bitmap Bitmap data, width * height RGB565 values
 * @param scale  Integer magnification, values below 1 are treated as 1
 * @return True on success, false otherwise
 */
bool spfd5408_draw_bitmap(spfd5408* const dev, int16_t x, int16_t y,
                          int16_t width, int16_t height,
                          const uint16_t* bitmap, uint8_t scale);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Put a 16-bit word on the parallel data bus.
 *
 * The library drives the controller in 16-bit 8080 mode and generates the
 * write strobe itself, this function only has to update the data lines.
 *
 * @param dev   spfd5408 device object
 * @param value Value to be put on the bus
 */
extern void spfd5408_write_bus(const spfd5408* const dev, uint16_t value);

/**
 * @brief Drive one of the display control lines.
 *
 * @param dev   spfd5408 device object
 * @param pin   Pin to be driven
 * @param state True drives the pin high, false drives it low
 */
extern void spfd5408_set_pin(const spfd5408* const dev, spfd5408_pin pin,
                             bool state);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void spfd5408_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __SPFD5408_H__ */
