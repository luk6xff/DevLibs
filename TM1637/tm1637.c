#include "tm1637.h"

//------------------------------------------------------------------------------
// @brief GLOBAL VARIABLES
//------------------------------------------------------------------------------
//Mask for blending out and restoring Icons
const char MASK_ICON_GRID[] = {
                                LO(S7_ICON_GR1),
                                LO(S7_ICON_GR2),
                                LO(S7_ICON_GR3),
                                LO(S7_ICON_GR4),
                                LO(S7_ICON_GR5),
                                LO(S7_ICON_GR6)
                              };

// ASCII Font definition mapping table for transmission to TM1637
const uint16_t TM1637_FONT[] =
{
    TM1637_CHAR_SPC, //32 0x20, Space
    TM1637_CHAR_EXC,
    TM1637_CHAR_QTE,
    TM1637_CHAR_HSH,
    TM1637_CHAR_DLR,
    TM1637_CHAR_PCT,
    TM1637_CHAR_AMP,
    TM1637_CHAR_ACC,
    TM1637_CHAR_LBR,
    TM1637_CHAR_RBR,
    TM1637_CHAR_MLT,
    TM1637_CHAR_PLS,
    TM1637_CHAR_CMA,
    TM1637_CHAR_MIN,
    TM1637_CHAR_DPT,
    TM1637_CHAR_RS,
    TM1637_CHAR_0,   //48 0x30
    TM1637_CHAR_1,
    TM1637_CHAR_2,
    TM1637_CHAR_3,
    TM1637_CHAR_4,
    TM1637_CHAR_5,
    TM1637_CHAR_6,
    TM1637_CHAR_7,
    TM1637_CHAR_8,
    TM1637_CHAR_9,
    TM1637_CHAR_COL, //58 0x3A
    TM1637_CHAR_SCL,
    TM1637_CHAR_LT,
    TM1637_CHAR_EQ,
    TM1637_CHAR_GT,
    TM1637_CHAR_QM,
    TM1637_CHAR_AT,  //64 0x40
    TM1637_CHAR_A,   //65 0x41, A
    TM1637_CHAR_B,
    TM1637_CHAR_C,
    TM1637_CHAR_D,
    TM1637_CHAR_E,
    TM1637_CHAR_F,
    TM1637_CHAR_G,
    TM1637_CHAR_H,
    TM1637_CHAR_I,
    TM1637_CHAR_J,
    TM1637_CHAR_K,
    TM1637_CHAR_L,
    TM1637_CHAR_M,
    TM1637_CHAR_N,
    TM1637_CHAR_O,
    TM1637_CHAR_P,
    TM1637_CHAR_Q,
    TM1637_CHAR_R,
    TM1637_CHAR_S,
    TM1637_CHAR_T,
    TM1637_CHAR_U,
    TM1637_CHAR_V,
    TM1637_CHAR_W,
    TM1637_CHAR_X,
    TM1637_CHAR_Y,
    TM1637_CHAR_Z,   //90 0x5A, Z
    TM1637_CHAR_SBL, //91 0x5B
    TM1637_CHAR_LS,
    TM1637_CHAR_SBR,
    TM1637_CHAR_PWR,
    TM1637_CHAR_UDS,
    TM1637_CHAR_ACC,
    TM1637_CHAR_a,   //97 0x61, A replacing a
    TM1637_CHAR_b,
    TM1637_CHAR_c,
    TM1637_CHAR_d,
    TM1637_CHAR_e,
    TM1637_CHAR_f,
    TM1637_CHAR_g,
    TM1637_CHAR_h,
    TM1637_CHAR_i,
    TM1637_CHAR_j,
    TM1637_CHAR_k,
    TM1637_CHAR_l,
    TM1637_CHAR_m,
    TM1637_CHAR_n,
    TM1637_CHAR_o,
    TM1637_CHAR_p,
    TM1637_CHAR_q,
    TM1637_CHAR_r,
    TM1637_CHAR_s,
    TM1637_CHAR_t,
    TM1637_CHAR_u,
    TM1637_CHAR_v,
    TM1637_CHAR_w,
    TM1637_CHAR_x,
    TM1637_CHAR_y,
    TM1637_CHAR_z,
    TM1637_CHAR_CBL, // 123 0x7B
    TM1637_CHAR_OR,
    TM1637_CHAR_CBR,
    TM1637_CHAR_TLD,
    TM1637_CHAR_DEL  // 127
};

//------------------------------------------------------------------------------
// @brief PRIVATE FUNCTIONS
//------------------------------------------------------------------------------
/**
 * @brief Generate Start condition for TM1637
 * @param  none
 * @return none
 */
static void _start(tm1637* const dev);

/**
 * @brief Generate Stop condition for TM1637
 * @param  none
 * @return none
 */
static void _stop(tm1637* const dev);

/**
 * @brief Send byte to TM1637
 * @param  int data
 * @return none
 */
static void _write(tm1637* const dev, uint8_t data);

/**
 * @brief  Read byte from TM1637
 * @return read byte
 */
static uint8_t _read(tm1637* const dev);

/**
 * @brief Write command and parameter to TM1637
 * @param  int cmd Command byte
 * @param  int data Parameters for command
 * @return none
 */
static void _write_cmd(tm1637* const dev, uint8_t cmd, uint8_t data);

/**
 * @brief  Write Display datablock to TM1637
 * @param  tm1637_display_data data Array of TM1637_DISPLAY_MEM (=4) bytes for displaydata
 * @param  length number bytes to write (valid range 0..(TM1637_DISPLAY_NR_DIGITS * TM1637_BYTES_PER_GRID_NUM) (=4), when starting at address 0)
 * @param  pos address display memory location to write bytes (default = 0)
 * @return none
 */
void _write_data(tm1637* const dev, const tm1637_display_data data,
                 uint8_t data_len, uint8_t pos);


//------------------------------------------------------------------------------
// @brief FUNCTIONS DEFINITIONS
//------------------------------------------------------------------------------
void tm1637_init(tm1637* const dev)
{
    if (!dev)
    {
        return;
    }

    // Init Serial bus
    tm1637_set_dio_mode(dev, TM1637_DIO_OUTPUT);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    tm1637_set_dio(dev, TM1637_PIN_HIGH);
    tm1637_set_clk(dev, TM1637_PIN_HIGH);

    // Init controller
    dev->column = 0;
    dev->columns_num = TM1637_DISPLAY_NR_DIGITS;
    dev->display_on_off = TM1637_DSP_ON;
    dev->brightness = TM1637_BRT_DEF;
    dev->ud_chars[0] = TM1637_CHAR_DGR;
    tm1637_clear(dev);
    _write_cmd(dev, TM1637_DSP_CTRL_CMD, dev->display_on_off | dev->brightness);                                 // Display control cmd, display on/off, brightness
    _write_cmd(dev, TM1637_DATA_SET_CMD, TM1637_DATA_WR | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, write data
}

//------------------------------------------------------------------------------
void tm1637_clear(tm1637* const dev)
{
    _start(dev);

    _write(dev, TM1637_ADDR_SET_CMD | 0x00); // Address set cmd, 0
    for (int i=0; i<TM1637_DISPLAY_MEM; i++)
    {
        dev->display_buffer[i] = 0x00;
        _write(dev, dev->display_buffer[i]); // data
    }
    _stop(dev);
    dev->column = 0;
}

//------------------------------------------------------------------------------
bool tm1637_get_keys(tm1637* const dev, tm1637_key_data *keydata)
{
    _start(dev);

    // Enable Key Read mode
    _write(dev, TM1637_DATA_SET_CMD | TM1637_KEY_RD | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, read data

    // Read keys
    // Bitpattern S0 S1 S2 K1 K2 1 1 1
    *keydata = _read(dev);

    _stop(dev);

    // Restore Data Write mode
    _write_cmd(dev, TM1637_DATA_SET_CMD, TM1637_DATA_WR | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, write data

    return (*keydata != TM1637_SW_NONE);
}


//------------------------------------------------------------------------------
void tm1637_set_brightness(tm1637* const dev, uint8_t brightness)
{
    dev->brightness = brightness & TM1637_BRT_MSK; // mask invalid bits

    _write_cmd(dev, TM1637_DSP_CTRL_CMD, dev->display_on_off | dev->brightness );  // Display control cmd, display on/off, brightness
}


//------------------------------------------------------------------------------
void tm1637_set_display(tm1637* const dev, bool on)
{
    if (on)
    {
        dev->display_on_off = TM1637_DSP_ON;
    }
    else
    {
        dev->display_on_off = TM1637_DSP_OFF;
    }

    _write_cmd(dev, TM1637_DSP_CTRL_CMD, dev->display_on_off | dev->brightness );  // Display control cmd, display on/off, brightness
}

//------------------------------------------------------------------------------
void tm1637_locate(tm1637* const dev, int column)
{
    // Sanity check
    if (column < 0)
    {
        column = 0;
    }
    if (column > (dev->columns_num - 1))
    {
        column = dev->columns_num - 1;
    }
    dev->column = column;
}

//------------------------------------------------------------------------------
void tm1637_set_icon(tm1637* const dev, tm1637_icon icon)
{
    int addr, icn;

    icn =  icon  & 0xFFFF;
    addr = (icon >> 24) & 0xFF;
    addr = (addr - 1);

    //Save char...and set bits for icon to write
    dev->display_buffer[addr] = dev->display_buffer[addr] | LO(icn);
    _write_data(dev, dev->display_buffer, TM1637_BYTES_PER_GRID, addr);
}


//------------------------------------------------------------------------------
void tm1637_clear_icon(tm1637* const dev, tm1637_icon icon)
{
    int addr, icn;

    icn =  icon  & 0xFFFF;
    addr = (icon >> 24) & 0xFF;
    addr = (addr - 1);

    //Save char...and clr bits for icon to write
    dev->display_buffer[addr] = dev->display_buffer[addr] & ~LO(icn);
    _write_data(dev, dev->display_buffer, TM1637_BYTES_PER_GRID, addr);
}


//------------------------------------------------------------------------------
void tm1637_set_udc(tm1637* const dev, uint8_t udc_idx, int udc_data)
{
    //Sanity check
    if (udc_idx > (TM1637_DISPLAY_NR_UDC-1))
    {
        return;
    }
    // Mask out Icon bits?
    dev->ud_chars[udc_idx] = LO(udc_data);
}


//------------------------------------------------------------------------------
int columns(const tm1637* const dev)
{
    return dev->columns_num;
}


//------------------------------------------------------------------------------
void tm1637_print(tm1637* const dev, const uint8_t* data, size_t data_len)
{
    for (size_t i = 0; i < data_len; i++)
    {
        uint8_t addr;
        bool write_char = false;
        uint8_t font    = 0x00;

        if ((data[i] == 0x00) || (data[i] == '\n') || (data[i] == '\r'))
        {
            // No character to write
            write_char = false;

            // Update Cursor
            dev->column = 0;
        }
        else if ((data[i] == '.') || (data[i] == ','))
        {
            // No character to write
            write_char = false;
            font = S7_DP; // placeholder for all DPs

            // Check to see that DP can be shown for current column
            if (dev->column > 0)
            {
                // Translate between dev->column and dev->display_buffer entries
                // Add DP to bitpattern of digit left of current column.
                addr = (dev->column - 1);
                // Save icons...and set bits for decimal point to write
                dev->display_buffer[addr] = dev->display_buffer[addr] | font;
                _write_data(dev, dev->display_buffer, TM1637_BYTES_PER_GRID, addr);
                // No Cursor Update here
            }
        }
        else if (data[i] < TM1637_DISPLAY_NR_UDC) // 1...8
        {
            //Character to write
            write_char = true;
            font = dev->ud_chars[data[i]-1];
        }

        // Encode all the ASCII characters
        else if ((data[i] >= TM1637_FONT_START) && (data[i] <= TM1637_FONT_END))
        {
            // Character to write
            write_char = true;
            font = TM1637_FONT[data[i] - TM1637_FONT_START];
        }

        if (write_char)
        {
            // Character to write
            // Translate between dev->column and displaybuffer entries
            addr = dev->column;

            // Save icons...and set bits for character to write
            dev->display_buffer[addr] = (dev->display_buffer[addr] & MASK_ICON_GRID[dev->column]) | font;

            _write_data(dev, dev->display_buffer, TM1637_BYTES_PER_GRID, addr);

            // Update Cursor
            dev->column++;
            if (dev->column > (TM1637_DISPLAY_NR_DIGITS - 1))
            {
                dev->column = 0;
            }
        }
    }
}

//------------------------------------------------------------------------------
// @brief PRIVATE FUNCTIONS DEFINITIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void _start(tm1637* const dev)
{
    tm1637_set_dio(dev, TM1637_PIN_LOW);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);
    tm1637_set_clk(dev, TM1637_PIN_LOW);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);
}


//------------------------------------------------------------------------------
void _stop(tm1637* const dev)
{
    tm1637_set_dio(dev, TM1637_PIN_LOW);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);
    tm1637_set_clk(dev, TM1637_PIN_HIGH);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);
    tm1637_set_dio(dev, TM1637_PIN_HIGH);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);
}


//------------------------------------------------------------------------------
void _write(tm1637* const dev, uint8_t data)
{
    //bool ack = false; // not used
    for (int bit=0; bit<8; bit++)
    {
        //The TM1637 expects LSB first
        if (((data >> bit) & 0x01) == 0x01)
        {
            tm1637_set_dio(dev, TM1637_PIN_HIGH);
        }
        else
        {
            tm1637_set_dio(dev, TM1637_PIN_LOW);
        }
        tm1637_delay_us(dev, TM1637_BIT_DELAY);
        tm1637_set_clk(dev, TM1637_PIN_HIGH);
        tm1637_delay_us(dev, TM1637_BIT_DELAY);
        tm1637_set_clk(dev, TM1637_PIN_LOW );
        tm1637_delay_us(dev, TM1637_BIT_DELAY);
    }

    tm1637_set_dio(dev, TM1637_PIN_HIGH);

    // Prepare DIO to read data
    tm1637_set_dio_mode(dev, TM1637_DIO_INPUT);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    // dummy Ack
    tm1637_set_clk(dev, TM1637_PIN_HIGH);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);
    //if (tm1637_get_dio(dev) == TM1637_PIN_HIGH)
    //{
    //    ack = true;
    //}
    tm1637_set_clk(dev, TM1637_PIN_LOW);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    // Return DIO to output mode
    tm1637_set_dio_mode(dev, TM1637_DIO_OUTPUT);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    tm1637_set_dio(dev, TM1637_PIN_HIGH); //idle
    //return ack;
}

//------------------------------------------------------------------------------
uint8_t _read(tm1637* const dev)
{
    char keycode = 0;

    // Prepare DIO to read data
    tm1637_set_dio_mode(dev, TM1637_DIO_INPUT);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    for (int bit=0; bit<8; bit++)
    {
        // The TM1637 sends bitpattern: S0 S1 S2 K1 K2 1 1 1
        // Data is shifted out by the TM1637 on the falling edge of CLK
        // Observe sufficient delay to allow the Open Drain DIO to rise to H levels
        // Prepare to read next bit, LSB (ie S0) first.
        // The code below flips bits for easier matching with datasheet
        keycode = keycode << 1;

        tm1637_set_clk(dev, TM1637_PIN_HIGH);
        tm1637_delay_us(dev, TM1637_BIT_DELAY);

        // Read next bit
        if (tm1637_get_dio(dev) == TM1637_PIN_HIGH)
        {
            keycode |= 0x01;
        }

        tm1637_set_clk(dev, TM1637_PIN_LOW );
        tm1637_delay_us(dev, 10); // Delay to allow for slow risetime
    }

    // Return DIO to output mode
    tm1637_set_dio_mode(dev, TM1637_DIO_OUTPUT);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    // Dummy Ack
    tm1637_set_dio(dev, TM1637_PIN_LOW); // Ack
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    tm1637_set_clk(dev, TM1637_PIN_HIGH);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);
    tm1637_set_clk(dev, TM1637_PIN_LOW);
    tm1637_delay_us(dev, TM1637_BIT_DELAY);

    tm1637_set_dio(dev, TM1637_PIN_HIGH); // Idle

    return keycode;
}

//------------------------------------------------------------------------------
void _write_cmd(tm1637* const dev, uint8_t cmd, uint8_t data)
{
    _start(dev);

    _write(dev, (cmd & TM1637_CMD_MSK) | (data & ~TM1637_CMD_MSK));

    _stop(dev);
}

//------------------------------------------------------------------------------
void _write_data(tm1637* const dev, const tm1637_display_data data,
                 uint8_t data_len, uint8_t pos)
{
    _start(dev);

    // Sanity check
    pos &= TM1637_ADDR_MSK;

    if ((data_len + pos) > TM1637_DISPLAY_MEM)
    {
        data_len = (TM1637_DISPLAY_MEM - pos);
    }

    _write(dev, TM1637_ADDR_SET_CMD | pos); // Set Address

    for (int idx = 0; idx < data_len; idx++)
    {
        _write(dev, (uint8_t)data[pos + idx]); // data
    }

    _stop(dev);
}

//------------------------------------------------------------------------------

