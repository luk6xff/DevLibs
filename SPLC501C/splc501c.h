/**
 *  @brief:   SPLC501C 128x64 monochrome graphic LCD library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __SPLC501C_H__
#define __SPLC501C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Panel geometry
 */
#define SPLC501C_WIDTH              128
#define SPLC501C_HEIGHT             64
#define SPLC501C_PIXELS_PER_PAGE    8
#define SPLC501C_PAGES              (SPLC501C_HEIGHT / SPLC501C_PIXELS_PER_PAGE)
#define SPLC501C_FRAMEBUF_SIZE      (SPLC501C_WIDTH * SPLC501C_PAGES)

/** The controller RAM is 132 columns wide while the panel shows only 128 of them */
#define SPLC501C_RAM_WIDTH          132
#define SPLC501C_RAM_COLUMN_SKEW    (SPLC501C_RAM_WIDTH - SPLC501C_WIDTH)

/**
 * @brief Built-in 5x7 font, covers ASCII 32..127
 */
#define SPLC501C_FONT_WIDTH         5
#define SPLC501C_FONT_HEIGHT        7
#define SPLC501C_FONT_FIRST_CHAR    32
#define SPLC501C_FONT_LAST_CHAR     127
#define SPLC501C_FONT_CHAR_SPACING  1
#define SPLC501C_CHAR_WIDTH         (SPLC501C_FONT_WIDTH + SPLC501C_FONT_CHAR_SPACING)

/**
 * @brief Controller command set
 */
#define SPLC501C_CMD_DISPLAY_ON         0xAF
#define SPLC501C_CMD_DISPLAY_OFF        0xAE
#define SPLC501C_CMD_START_LINE         0x40  // OR-ed with the line number 0..63
#define SPLC501C_CMD_PAGE_ADDRESS       0xB0  // OR-ed with the page number 0..7
#define SPLC501C_CMD_COLUMN_ADDRESS_HI  0x10  // OR-ed with the upper column nibble
#define SPLC501C_CMD_COLUMN_ADDRESS_LO  0x00  // OR-ed with the lower column nibble
#define SPLC501C_CMD_ADC_NORMAL         0xA0
#define SPLC501C_CMD_ADC_REVERSE        0xA1
#define SPLC501C_CMD_DISPLAY_NORMAL     0xA6
#define SPLC501C_CMD_DISPLAY_REVERSE    0xA7
#define SPLC501C_CMD_DISPLAY_ALL_ON     0xA5
#define SPLC501C_CMD_DISPLAY_ALL_OFF    0xA4
#define SPLC501C_CMD_BIAS_1_9           0xA2
#define SPLC501C_CMD_BIAS_1_5           0xA3
#define SPLC501C_CMD_RMW_START          0xE0
#define SPLC501C_CMD_RMW_END            0xEE
#define SPLC501C_CMD_SOFT_RESET         0xE2
#define SPLC501C_CMD_COM_NORMAL         0xC0  // COM0 -> COM63
#define SPLC501C_CMD_COM_REVERSE        0xC8  // COM63 -> COM0
#define SPLC501C_CMD_POWER_ON           0x2F  // Booster, regulator and follower all enabled
#define SPLC501C_CMD_VOLTAGE_RATIO      0x20  // OR-ed with the resistor ratio 0..7
#define SPLC501C_CMD_VOLUME_MODE        0x81  // Followed by the contrast value 0..63
#define SPLC501C_CMD_PAGE_BLINKING      0xD5  // Followed by the page mask

/** Contrast (electronic volume) limits */
#define SPLC501C_CONTRAST_MIN           0x00
#define SPLC501C_CONTRAST_MAX           0x3F
#define SPLC501C_CONTRAST_DEFAULT       0x14

/** Regulator resistor ratio applied on init, 0..7 */
#define SPLC501C_VOLTAGE_RATIO_DEFAULT  4

/**
 * @brief Pixel colours of a monochrome panel
 */
typedef enum
{
    SPLC501C_COLOR_OFF     = 0, // Pixel cleared (background)
    SPLC501C_COLOR_ON      = 1, // Pixel set (foreground)
    SPLC501C_COLOR_INVERSE = 2, // Pixel toggled
} splc501c_color;

/**
 * @brief Panel mounting orientation
 */
typedef enum
{
    SPLC501C_ORIENTATION_NORMAL      = 0, // Origin in the top left corner
    SPLC501C_ORIENTATION_ROTATED_180 = 1, // Panel rotated by 180 degrees
} splc501c_orientation;

/**
 * @brief Control lines driven by the platform layer
 */
typedef enum
{
    SPLC501C_PIN_A0    = 0, // Data (high) / command (low) select, sometimes named RS
    SPLC501C_PIN_RESET = 1, // Active low hardware reset
    SPLC501C_PIN_CS    = 2, // Active low chip select
} splc501c_pin;

/**
 * @brief splc501c dev object
 */
typedef struct
{
    void* platform_dev;                  // Platform descriptor, set by splc501c_<platform>_init()
    uint8_t buf[SPLC501C_FRAMEBUF_SIZE]; // Page organised framebuffer, 1 bit per pixel
    int16_t cursor_x;                    // Text cursor, x coordinate in pixels
    int16_t cursor_y;                    // Text cursor, y coordinate in pixels
    splc501c_orientation orientation;    // Current panel orientation
    splc501c_color text_color;           // Colour used by splc501c_print()
    uint8_t contrast;                    // Cached contrast value 0..63
    bool inverted;                       // True when the panel shows an inverted image
    bool display_on;                     // True when the panel is powered on
} splc501c;

/**
 * @brief Initialize the display and clear the framebuffer.
 *
 * @param dev splc501c device object
 * @return True on success, false otherwise
 */
bool splc501c_init(splc501c* const dev);

/**
 * @brief Pulse the RESET line and re-apply the power-up command sequence.
 *
 * @param dev splc501c device object
 * @return True on success, false otherwise
 */
bool splc501c_reset(splc501c* const dev);

/**
 * @brief Turn the panel on or off, the framebuffer content is preserved.
 *
 * @param dev splc501c device object
 * @param on  True to turn the display on
 * @return True on success, false otherwise
 */
bool splc501c_set_display_on(splc501c* const dev, bool on);

/**
 * @brief Set the electronic volume (contrast).
 *
 * @param dev      splc501c device object
 * @param contrast Contrast, clamped to SPLC501C_CONTRAST_MAX
 * @return True on success, false otherwise
 */
bool splc501c_set_contrast(splc501c* const dev, uint8_t contrast);

/**
 * @brief Show the framebuffer normally or with all the pixels inverted.
 *
 * @param dev      splc501c device object
 * @param inverted True to invert the panel
 * @return True on success, false otherwise
 */
bool splc501c_set_inverted(splc501c* const dev, bool inverted);

/**
 * @brief Select the panel orientation.
 *
 * @param dev         splc501c device object
 * @param orientation Orientation to be applied
 * @return True on success, false otherwise
 */
bool splc501c_set_orientation(splc501c* const dev,
                              splc501c_orientation orientation);

/**
 * @brief Push the whole framebuffer to the panel.
 *
 * All the drawing primitives only touch the framebuffer, nothing shows up on
 * the panel until this function is called.
 *
 * @param dev splc501c device object
 * @return True on success, false otherwise
 */
bool splc501c_update(splc501c* const dev);

/**
 * @brief Fill the framebuffer with a single colour.
 *
 * @param dev   splc501c device object
 * @param color SPLC501C_COLOR_ON fills the framebuffer, anything else clears it
 * @return True on success, false otherwise
 */
bool splc501c_clear(splc501c* const dev, splc501c_color color);

/**
 * @brief Draw a single pixel, coordinates outside the panel are discarded.
 *
 * @param dev   splc501c device object
 * @param x     X coordinate
 * @param y     Y coordinate
 * @param color Pixel colour
 * @return True when the pixel was inside the panel, false otherwise
 */
bool splc501c_draw_pixel(splc501c* const dev, int16_t x, int16_t y,
                         splc501c_color color);

/**
 * @brief Read a pixel back from the framebuffer.
 *
 * @param dev splc501c device object
 * @param x   X coordinate
 * @param y   Y coordinate
 * @return True when the pixel is set, false when it is cleared or out of range
 */
bool splc501c_get_pixel(const splc501c* const dev, int16_t x, int16_t y);

/**
 * @brief Draw a horizontal line, clipped to the panel.
 *
 * @param dev   splc501c device object
 * @param x     X coordinate of the left end
 * @param y     Y coordinate
 * @param len   Length in pixels, negative values extend to the left
 * @param color Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_draw_hline(splc501c* const dev, int16_t x, int16_t y,
                         int16_t len, splc501c_color color);

/**
 * @brief Draw a vertical line, clipped to the panel.
 *
 * @param dev   splc501c device object
 * @param x     X coordinate
 * @param y     Y coordinate of the top end
 * @param len   Length in pixels, negative values extend upwards
 * @param color Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_draw_vline(splc501c* const dev, int16_t x, int16_t y,
                         int16_t len, splc501c_color color);

/**
 * @brief Draw a line between two points, clipped to the panel.
 *
 * @param dev   splc501c device object
 * @param x0    X coordinate of the first point
 * @param y0    Y coordinate of the first point
 * @param x1    X coordinate of the second point
 * @param y1    Y coordinate of the second point
 * @param color Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_draw_line(splc501c* const dev, int16_t x0, int16_t y0,
                        int16_t x1, int16_t y1, splc501c_color color);

/**
 * @brief Draw a rectangle outline, clipped to the panel.
 *
 * @param dev    splc501c device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Rectangle width in pixels
 * @param height Rectangle height in pixels
 * @param color  Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_draw_rect(splc501c* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, splc501c_color color);

/**
 * @brief Draw a filled rectangle, clipped to the panel.
 *
 * @param dev    splc501c device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Rectangle width in pixels
 * @param height Rectangle height in pixels
 * @param color  Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_fill_rect(splc501c* const dev, int16_t x, int16_t y,
                        int16_t width, int16_t height, splc501c_color color);

/**
 * @brief Draw a circle outline, clipped to the panel.
 *
 * @param dev    splc501c device object
 * @param cx     X coordinate of the centre
 * @param cy     Y coordinate of the centre
 * @param radius Radius in pixels
 * @param color  Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_draw_circle(splc501c* const dev, int16_t cx, int16_t cy,
                          int16_t radius, splc501c_color color);

/**
 * @brief Draw a filled circle, clipped to the panel.
 *
 * @param dev    splc501c device object
 * @param cx     X coordinate of the centre
 * @param cy     Y coordinate of the centre
 * @param radius Radius in pixels
 * @param color  Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_fill_circle(splc501c* const dev, int16_t cx, int16_t cy,
                          int16_t radius, splc501c_color color);

/**
 * @brief Draw a single character of the built-in 5x7 font.
 *
 * Only the lit pixels of the glyph are touched, the background is left as it
 * is. Characters outside ASCII 32..127 are rendered as a space.
 *
 * @param dev   splc501c device object
 * @param c     Character to be drawn
 * @param x     X coordinate of the top left corner of the glyph
 * @param y     Y coordinate of the top left corner of the glyph
 * @param color Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_draw_char(splc501c* const dev, char c, int16_t x, int16_t y,
                        splc501c_color color);

/**
 * @brief Draw a NUL terminated string using the built-in 5x7 font.
 *
 * @param dev   splc501c device object
 * @param str   String to be drawn
 * @param x     X coordinate of the top left corner of the first glyph
 * @param y     Y coordinate of the top left corner of the first glyph
 * @param color Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_draw_string(splc501c* const dev, const char* str, int16_t x,
                          int16_t y, splc501c_color color);

/**
 * @brief Move the text cursor used by splc501c_print().
 *
 * @param dev splc501c device object
 * @param x   X coordinate in pixels
 * @param y   Y coordinate in pixels
 * @return True on success, false otherwise
 */
bool splc501c_set_cursor(splc501c* const dev, int16_t x, int16_t y);

/**
 * @brief Select the colour used by splc501c_print().
 *
 * @param dev   splc501c device object
 * @param color Pixel colour
 * @return True on success, false otherwise
 */
bool splc501c_set_text_color(splc501c* const dev, splc501c_color color);

/**
 * @brief Draw a string at the current cursor and advance the cursor.
 *
 * A newline moves the cursor to the beginning of the next text line, the
 * cursor also wraps automatically at the right edge of the panel.
 *
 * @param dev splc501c device object
 * @param str String to be printed
 * @return True on success, false otherwise
 */
bool splc501c_print(splc501c* const dev, const char* str);

/**
 * @brief Blit a 1 bit per pixel, row organised bitmap into the framebuffer.
 *
 * Every row starts on a byte boundary and the most significant bit of a byte
 * is the leftmost pixel. Parts falling outside the panel are clipped away.
 *
 * @param dev    splc501c device object
 * @param x      X coordinate of the top left corner
 * @param y      Y coordinate of the top left corner
 * @param width  Bitmap width in pixels
 * @param height Bitmap height in pixels
 * @param bitmap Bitmap data, ((width + 7) / 8) * height bytes
 * @param color  Colour used for the bits that are set
 * @return True on success, false otherwise
 */
bool splc501c_draw_bitmap(splc501c* const dev, int16_t x, int16_t y,
                          int16_t width, int16_t height,
                          const uint8_t* bitmap, splc501c_color color);

/**
 * @brief Copy a full screen, page organised image into the framebuffer.
 *
 * This is the layout the panel itself uses, one byte holds eight vertically
 * stacked pixels and the pages follow each other from top to bottom.
 *
 * @param dev      splc501c device object
 * @param image    Image data
 * @param img_size Number of bytes, at most SPLC501C_FRAMEBUF_SIZE are copied
 * @return True on success, false otherwise
 */
bool splc501c_draw_image(splc501c* const dev, const uint8_t* image,
                         size_t img_size);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Write raw bytes into the splc501c device.
 *
 * The A0 and CS lines are already driven by the library when this is called.
 *
 * @param dev splc501c device object
 * @param buf Data to be written
 * @param len Number of bytes to be written
 * @return True on success, false otherwise
 */
extern bool splc501c_write_bytes(const splc501c* const dev,
                                 const uint8_t* buf, size_t len);

/**
 * @brief Drive one of the display control lines.
 *
 * @param dev   splc501c device object
 * @param pin   Pin to be driven
 * @param state True drives the pin high, false drives it low
 */
extern void splc501c_set_pin(const splc501c* const dev, splc501c_pin pin,
                             bool state);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void splc501c_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __SPLC501C_H__ */
