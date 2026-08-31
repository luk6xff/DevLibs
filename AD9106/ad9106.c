/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#include "ad9106.h"

// Length of the hardware reset pulse
#define AD9106_RESET_PULSE_MS       2u

// Time given to the device to reload its default register values
#define AD9106_RESET_WAIT_MS        10u

// Scratch value written into DAC1_CST to prove the SPI link is working,
// the register keeps the DAC code in bits [15:4]
#define AD9106_PRESENCE_TEST_VALUE  0xA5A0u

// Largest signed value which fits into the 12-bit DAC data fields
#define AD9106_DAC_CODE_MIN         (-2048)
#define AD9106_DAC_CODE_MAX         2047
#define AD9106_DAC_CODE_MASK        0x0FFFu

// START_ADDRx/STOP_ADDRx keep the SRAM word address in bits [15:4]
#define AD9106_SRAM_ADDR_SHIFT      4u

// Field widths used by the setters
#define AD9106_ANALOG_GAIN_MAX      127u
#define AD9106_GAIN_RANGE_MAX       3u
#define AD9106_RSET_MAX             31u
#define AD9106_SAW_STEP_MAX         63u
#define AD9106_TIMEBASE_FIELD_MAX   15u
#define AD9106_CAL_CLK_DIV_MAX      7u

// Bounded wait for the calibration to finish
#define AD9106_CAL_POLL_STEP_MS     1u
#define AD9106_CAL_POLL_RETRIES     500u

// Number of bits of a register, used to bound the bit field helpers
#define AD9106_REG_BITS             16u

// Per channel register lookup tables, indexed with (channel - 1)
static const uint16_t ad9106_dof_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_DAC1DOF, AD9106_REG_DAC2DOF,
    AD9106_REG_DAC3DOF, AD9106_REG_DAC4DOF,
};

static const uint16_t ad9106_cst_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_DAC1_CST, AD9106_REG_DAC2_CST,
    AD9106_REG_DAC3_CST, AD9106_REG_DAC4_CST,
};

static const uint16_t ad9106_dgain_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_DAC1_DGAIN, AD9106_REG_DAC2_DGAIN,
    AD9106_REG_DAC3_DGAIN, AD9106_REG_DAC4_DGAIN,
};

static const uint16_t ad9106_again_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_DAC1AGAIN, AD9106_REG_DAC2AGAIN,
    AD9106_REG_DAC3AGAIN, AD9106_REG_DAC4AGAIN,
};

static const uint16_t ad9106_rset_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_DAC1RSET, AD9106_REG_DAC2RSET,
    AD9106_REG_DAC3RSET, AD9106_REG_DAC4RSET,
};

static const uint16_t ad9106_dds_pw_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_DDS1_PW, AD9106_REG_DDS2_PW,
    AD9106_REG_DDS3_PW, AD9106_REG_DDS4_PW,
};

static const uint16_t ad9106_dds_cyc_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_DDS_CYC1, AD9106_REG_DDS_CYC2,
    AD9106_REG_DDS_CYC3, AD9106_REG_DDS_CYC4,
};

static const uint16_t ad9106_start_dly_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_START_DLY1, AD9106_REG_START_DLY2,
    AD9106_REG_START_DLY3, AD9106_REG_START_DLY4,
};

static const uint16_t ad9106_start_addr_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_START_ADDR1, AD9106_REG_START_ADDR2,
    AD9106_REG_START_ADDR3, AD9106_REG_START_ADDR4,
};

static const uint16_t ad9106_stop_addr_regs[AD9106_CHANNEL_COUNT] = {
    AD9106_REG_STOP_ADDR1, AD9106_REG_STOP_ADDR2,
    AD9106_REG_STOP_ADDR3, AD9106_REG_STOP_ADDR4,
};

//-----------------------------------------------------------------------------
static bool ad9106_channel_valid(ad9106_channel ch)
{
    return (ch >= AD9106_CHANNEL_1) && (ch <= AD9106_CHANNEL_4);
}

//-----------------------------------------------------------------------------
static size_t ad9106_channel_idx(ad9106_channel ch)
{
    return (size_t)((int)ch - 1);
}

//-----------------------------------------------------------------------------
static uint8_t ad9106_mask_shift(uint16_t mask)
{
    uint8_t shift = 0;

    while ((shift < AD9106_REG_BITS) && (((mask >> shift) & 1u) == 0u))
    {
        shift++;
    }
    return shift;
}

//-----------------------------------------------------------------------------
static bool ad9106_spi_frame(const ad9106* const dev, bool read,
                             uint16_t reg_addr, uint16_t value, uint16_t* out)
{
    const uint16_t addr = (uint16_t)(reg_addr & AD9106_REG_ADDR_MASK);
    uint8_t tx[AD9106_SPI_FRAME_SIZE] = {0};
    uint8_t rx[AD9106_SPI_FRAME_SIZE] = {0};

    // Instruction word: R/W flag in the MSB followed by the 15-bit address
    tx[0] = (uint8_t)(addr >> 8);
    if (read)
    {
        tx[0] |= (uint8_t)AD9106_SPI_READ_BIT;
    }
    tx[1] = (uint8_t)(addr & 0x00FFu);
    if (!read)
    {
        tx[2] = (uint8_t)(value >> 8);
        tx[3] = (uint8_t)(value & 0x00FFu);
    }

    ad9106_set_pin(dev, AD9106_PIN_CS, false);
    const bool res = ad9106_spi_transfer(dev, tx, rx, sizeof(tx));
    ad9106_set_pin(dev, AD9106_PIN_CS, true);

    if (!res)
    {
        return false;
    }

    if (out)
    {
        // The device shifts the register content out during the data word
        *out = (uint16_t)(((uint16_t)rx[2] << 8) | (uint16_t)rx[3]);
    }
    return true;
}

//-----------------------------------------------------------------------------
static bool ad9106_check_presence(const ad9106* const dev)
{
    uint16_t readback = 0;

    if (!ad9106_write_reg(dev, AD9106_REG_DAC1_CST, AD9106_PRESENCE_TEST_VALUE))
    {
        return false;
    }
    if (!ad9106_read_reg(dev, AD9106_REG_DAC1_CST, &readback))
    {
        return false;
    }
    if (readback != AD9106_PRESENCE_TEST_VALUE)
    {
        return false;
    }

    // Restore the reset default, the value is not applied until RAMUPDATE
    return ad9106_write_reg(dev, AD9106_REG_DAC1_CST, 0x0000);
}

//-----------------------------------------------------------------------------
bool ad9106_init(ad9106* const dev)
{
    if (!dev)
    {
        return false;
    }

    if (dev->dac_clk_hz == 0)
    {
        dev->dac_clk_hz = AD9106_DEFAULT_DAC_CLK_HZ;
    }

    // Idle levels, both the chip select and the trigger inputs are active low
    ad9106_set_pin(dev, AD9106_PIN_CS, true);
    ad9106_set_pin(dev, AD9106_PIN_TRIGGER, true);

    // Hardware reset pulse, ignored by platforms without a RESET line
    ad9106_set_pin(dev, AD9106_PIN_RESET, false);
    ad9106_delay_ms(AD9106_RESET_PULSE_MS);
    ad9106_set_pin(dev, AD9106_PIN_RESET, true);
    ad9106_delay_ms(AD9106_RESET_WAIT_MS);

    if (!ad9106_reset(dev))
    {
        return false;
    }

    return ad9106_check_presence(dev);
}

//-----------------------------------------------------------------------------
bool ad9106_reset(const ad9106* const dev)
{
    if (!dev)
    {
        return false;
    }

    // The mirror bits [15:10] have to mirror bits [5:0] at all times
    if (!ad9106_write_reg(dev, AD9106_REG_SPICONFIG,
                          AD9106_SPICONFIG_RESET | AD9106_SPICONFIG_RESETM))
    {
        return false;
    }
    ad9106_delay_ms(AD9106_RESET_WAIT_MS);

    // The reset bits are not self clearing
    return ad9106_write_reg(dev, AD9106_REG_SPICONFIG, 0x0000);
}

//-----------------------------------------------------------------------------
bool ad9106_write_reg(const ad9106* const dev, uint16_t reg_addr, uint16_t value)
{
    if (!dev)
    {
        return false;
    }
    return ad9106_spi_frame(dev, false, reg_addr, value, NULL);
}

//-----------------------------------------------------------------------------
bool ad9106_read_reg(const ad9106* const dev, uint16_t reg_addr, uint16_t* value)
{
    if (!dev || !value)
    {
        return false;
    }
    return ad9106_spi_frame(dev, true, reg_addr, 0, value);
}

//-----------------------------------------------------------------------------
bool ad9106_write_reg_field(const ad9106* const dev, uint16_t reg_addr,
                            uint16_t mask, uint16_t value)
{
    uint16_t reg = 0;

    if (!dev || (mask == 0))
    {
        return false;
    }

    if (!ad9106_read_reg(dev, reg_addr, &reg))
    {
        return false;
    }

    const uint8_t shift = ad9106_mask_shift(mask);
    const uint16_t field = (uint16_t)((uint16_t)(value << shift) & mask);
    reg = (uint16_t)((uint16_t)(reg & (uint16_t)(~mask)) | field);
    return ad9106_write_reg(dev, reg_addr, reg);
}

//-----------------------------------------------------------------------------
bool ad9106_read_reg_field(const ad9106* const dev, uint16_t reg_addr,
                           uint16_t mask, uint16_t* value)
{
    uint16_t reg = 0;

    if (!dev || !value || (mask == 0))
    {
        return false;
    }

    if (!ad9106_read_reg(dev, reg_addr, &reg))
    {
        return false;
    }

    *value = (uint16_t)((uint16_t)(reg & mask) >> ad9106_mask_shift(mask));
    return true;
}

//-----------------------------------------------------------------------------
bool ad9106_ram_update(const ad9106* const dev)
{
    // The bit is self clearing once the shadow registers have been transferred
    return ad9106_write_reg(dev, AD9106_REG_RAMUPDATE, AD9106_RAMUPDATE_UPDATE);
}

//-----------------------------------------------------------------------------
static bool ad9106_set_mem_access(const ad9106* const dev, uint16_t mode)
{
    // The pattern generator must be stopped while the SRAM is accessed
    if (!ad9106_write_reg(dev, AD9106_REG_PAT_STATUS, mode))
    {
        return false;
    }
    return ad9106_ram_update(dev);
}

//-----------------------------------------------------------------------------
bool ad9106_write_sram(const ad9106* const dev, uint16_t offset,
                       const uint16_t* data, size_t len)
{
    if (!dev || !data || (len == 0))
    {
        return false;
    }
    if ((offset >= AD9106_SRAM_SIZE) || (len > (AD9106_SRAM_SIZE - offset)))
    {
        return false;
    }

    if (!ad9106_set_mem_access(dev, AD9106_PAT_STATUS_MEM_ACCESS))
    {
        return false;
    }

    for (size_t i = 0; i < len; ++i)
    {
        const uint16_t addr =
            (uint16_t)(AD9106_REG_SRAMDATA + offset + (uint16_t)i);
        if (!ad9106_write_reg(dev, addr, data[i]))
        {
            (void)ad9106_set_mem_access(dev, 0);
            return false;
        }
    }

    return ad9106_set_mem_access(dev, 0);
}

//-----------------------------------------------------------------------------
bool ad9106_read_sram(const ad9106* const dev, uint16_t offset,
                      uint16_t* data, size_t len)
{
    if (!dev || !data || (len == 0))
    {
        return false;
    }
    if ((offset >= AD9106_SRAM_SIZE) || (len > (AD9106_SRAM_SIZE - offset)))
    {
        return false;
    }

    if (!ad9106_set_mem_access(dev, AD9106_PAT_STATUS_MEM_ACCESS |
                                    AD9106_PAT_STATUS_BUF_READ))
    {
        return false;
    }

    for (size_t i = 0; i < len; ++i)
    {
        const uint16_t addr =
            (uint16_t)(AD9106_REG_SRAMDATA + offset + (uint16_t)i);
        if (!ad9106_read_reg(dev, addr, &data[i]))
        {
            (void)ad9106_set_mem_access(dev, 0);
            return false;
        }
    }

    return ad9106_set_mem_access(dev, 0);
}

//-----------------------------------------------------------------------------
bool ad9106_set_wave_config(const ad9106* const dev, ad9106_channel ch,
                            ad9106_wave_sel wave,
                            ad9106_prestored_wave prestore)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }

    // Channels 1 and 2 share WAV2_1CONFIG, channels 3 and 4 WAV4_3CONFIG
    const bool low_reg = (ch == AD9106_CHANNEL_1) || (ch == AD9106_CHANNEL_2);
    const uint16_t reg = low_reg ? (uint16_t)AD9106_REG_WAV2_1CONFIG
                                 : (uint16_t)AD9106_REG_WAV4_3CONFIG;

    // Odd channels occupy the low byte of their register, even ones the high byte
    const bool low_byte = (ch == AD9106_CHANNEL_1) || (ch == AD9106_CHANNEL_3);
    const uint16_t prestore_mask = low_byte
        ? (uint16_t)AD9106_WAV2_1CONFIG_PRESTORE_SEL1
        : (uint16_t)AD9106_WAV2_1CONFIG_PRESTORE_SEL2;
    const uint16_t wave_mask = low_byte ? (uint16_t)AD9106_WAV2_1CONFIG_WAVE_SEL1
                                        : (uint16_t)AD9106_WAV2_1CONFIG_WAVE_SEL2;

    if (!ad9106_write_reg_field(dev, reg, prestore_mask, (uint16_t)prestore))
    {
        return false;
    }
    return ad9106_write_reg_field(dev, reg, wave_mask, (uint16_t)wave);
}

//-----------------------------------------------------------------------------
bool ad9106_set_sawtooth(const ad9106* const dev, ad9106_channel ch,
                         ad9106_saw_type type, uint8_t step)
{
    if (!dev || !ad9106_channel_valid(ch) || (step > AD9106_SAW_STEP_MAX))
    {
        return false;
    }

    const bool low_reg = (ch == AD9106_CHANNEL_1) || (ch == AD9106_CHANNEL_2);
    const uint16_t reg = low_reg ? (uint16_t)AD9106_REG_SAW2_1CONFIG
                                 : (uint16_t)AD9106_REG_SAW4_3CONFIG;

    const bool low_byte = (ch == AD9106_CHANNEL_1) || (ch == AD9106_CHANNEL_3);
    const uint16_t step_mask = low_byte ? (uint16_t)AD9106_SAW2_1CONFIG_SAW_STEP1
                                        : (uint16_t)AD9106_SAW2_1CONFIG_SAW_STEP2;
    const uint16_t type_mask = low_byte ? (uint16_t)AD9106_SAW2_1CONFIG_SAW_TYPE1
                                        : (uint16_t)AD9106_SAW2_1CONFIG_SAW_TYPE2;

    if (!ad9106_write_reg_field(dev, reg, step_mask, step))
    {
        return false;
    }
    return ad9106_write_reg_field(dev, reg, type_mask, (uint16_t)type);
}

//-----------------------------------------------------------------------------
bool ad9106_set_dds_tuning_word(const ad9106* const dev, uint32_t tw)
{
    if (!dev || (tw > AD9106_DDS_TW_MAX))
    {
        return false;
    }

    // DDSTW[23:8] lives in DDS_TW32, DDSTW[7:0] in the high byte of DDS_TW1
    if (!ad9106_write_reg(dev, AD9106_REG_DDS_TW32, (uint16_t)(tw >> 8)))
    {
        return false;
    }
    return ad9106_write_reg(dev, AD9106_REG_DDS_TW1,
                            (uint16_t)((tw & 0x000000FFu) << 8));
}

//-----------------------------------------------------------------------------
bool ad9106_set_dds_freq(const ad9106* const dev, uint32_t freq_hz)
{
    if (!dev || (dev->dac_clk_hz == 0))
    {
        return false;
    }

    // Nyquist limit of the DDS
    if (freq_hz > (dev->dac_clk_hz / 2u))
    {
        return false;
    }

    const uint64_t tw = ((uint64_t)freq_hz << AD9106_DDS_TW_BITS) /
                        (uint64_t)dev->dac_clk_hz;
    return ad9106_set_dds_tuning_word(dev, (uint32_t)tw);
}

//-----------------------------------------------------------------------------
bool ad9106_set_dds_phase(const ad9106* const dev, ad9106_channel ch,
                          uint16_t phase)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }
    return ad9106_write_reg(dev, ad9106_dds_pw_regs[ad9106_channel_idx(ch)],
                            phase);
}

//-----------------------------------------------------------------------------
bool ad9106_set_dds_cycles(const ad9106* const dev, ad9106_channel ch,
                           uint16_t cycles)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }
    return ad9106_write_reg(dev, ad9106_dds_cyc_regs[ad9106_channel_idx(ch)],
                            cycles);
}

//-----------------------------------------------------------------------------
static bool ad9106_set_dac_code(const ad9106* const dev, uint16_t reg_addr,
                                uint16_t mask, int16_t code)
{
    if ((code < AD9106_DAC_CODE_MIN) || (code > AD9106_DAC_CODE_MAX))
    {
        return false;
    }

    const uint16_t raw = (uint16_t)((uint16_t)code & AD9106_DAC_CODE_MASK);
    return ad9106_write_reg_field(dev, reg_addr, mask, raw);
}

//-----------------------------------------------------------------------------
bool ad9106_set_digital_gain(const ad9106* const dev, ad9106_channel ch,
                             int16_t gain)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }
    return ad9106_set_dac_code(dev, ad9106_dgain_regs[ad9106_channel_idx(ch)],
                               AD9106_DACX_DGAIN_DIG_GAIN, gain);
}

//-----------------------------------------------------------------------------
bool ad9106_set_digital_offset(const ad9106* const dev, ad9106_channel ch,
                               int16_t offset)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }
    return ad9106_set_dac_code(dev, ad9106_dof_regs[ad9106_channel_idx(ch)],
                               AD9106_DACXDOF_DIG_OFFSET, offset);
}

//-----------------------------------------------------------------------------
bool ad9106_set_constant_value(const ad9106* const dev, ad9106_channel ch,
                               int16_t value)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }
    return ad9106_set_dac_code(dev, ad9106_cst_regs[ad9106_channel_idx(ch)],
                               AD9106_DACX_CST_CONST, value);
}

//-----------------------------------------------------------------------------
bool ad9106_set_analog_gain(const ad9106* const dev, ad9106_channel ch,
                            uint8_t gain)
{
    if (!dev || !ad9106_channel_valid(ch) || (gain > AD9106_ANALOG_GAIN_MAX))
    {
        return false;
    }
    return ad9106_write_reg_field(dev, ad9106_again_regs[ad9106_channel_idx(ch)],
                                  AD9106_DACXAGAIN_GAIN, gain);
}

//-----------------------------------------------------------------------------
bool ad9106_set_gain_range(const ad9106* const dev, ad9106_channel ch,
                           uint8_t range)
{
    if (!dev || !ad9106_channel_valid(ch) || (range > AD9106_GAIN_RANGE_MAX))
    {
        return false;
    }

    // Two bits per channel, DAC1 occupies bits [1:0]
    const uint16_t mask = (uint16_t)(AD9106_DACXRANGE_DAC1_GAIN_RNG
                                     << (2u * ad9106_channel_idx(ch)));
    return ad9106_write_reg_field(dev, AD9106_REG_DACXRANGE, mask, range);
}

//-----------------------------------------------------------------------------
bool ad9106_set_rset(const ad9106* const dev, ad9106_channel ch, bool use_spi,
                     uint8_t rset)
{
    if (!dev || !ad9106_channel_valid(ch) || (rset > AD9106_RSET_MAX))
    {
        return false;
    }

    const uint16_t reg_addr = ad9106_rset_regs[ad9106_channel_idx(ch)];
    uint16_t reg = (uint16_t)(rset & AD9106_DACXRSET_RSET);
    if (use_spi)
    {
        reg |= (uint16_t)AD9106_DACXRSET_EN;
    }
    return ad9106_write_reg(dev, reg_addr, reg);
}

//-----------------------------------------------------------------------------
bool ad9106_set_dac_sleep(const ad9106* const dev, ad9106_channel ch, bool sleep)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }

    // DAC1_SLEEP is bit 3, the following channels use the bits below it
    const uint16_t mask = (uint16_t)(AD9106_POWERCONFIG_DAC1_SLEEP
                                     >> ad9106_channel_idx(ch));
    return ad9106_write_reg_field(dev, AD9106_REG_POWERCONFIG, mask,
                                  sleep ? 1u : 0u);
}

//-----------------------------------------------------------------------------
bool ad9106_config_sram_playback(const ad9106* const dev, ad9106_channel ch,
                                 uint16_t start_addr, uint16_t stop_addr,
                                 uint16_t start_delay)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }
    if ((start_addr >= AD9106_SRAM_SIZE) || (stop_addr >= AD9106_SRAM_SIZE) ||
        (stop_addr < start_addr))
    {
        return false;
    }

    const size_t idx = ad9106_channel_idx(ch);

    // Both address fields hold the word address in bits [15:4]
    const uint16_t start_raw =
        (uint16_t)((uint16_t)(start_addr << AD9106_SRAM_ADDR_SHIFT) &
                   AD9106_START_ADDRX_ADDR);
    const uint16_t stop_raw =
        (uint16_t)((uint16_t)(stop_addr << AD9106_SRAM_ADDR_SHIFT) &
                   AD9106_STOP_ADDRX_ADDR);

    if (!ad9106_write_reg(dev, ad9106_start_addr_regs[idx], start_raw))
    {
        return false;
    }
    if (!ad9106_write_reg(dev, ad9106_stop_addr_regs[idx], stop_raw))
    {
        return false;
    }
    return ad9106_write_reg(dev, ad9106_start_dly_regs[idx], start_delay);
}

//-----------------------------------------------------------------------------
bool ad9106_set_pattern_period(const ad9106* const dev, uint16_t period)
{
    return ad9106_write_reg(dev, AD9106_REG_PAT_PERIOD, period);
}

//-----------------------------------------------------------------------------
bool ad9106_set_pattern_delay(const ad9106* const dev, uint16_t delay)
{
    return ad9106_write_reg(dev, AD9106_REG_PATTERN_DLY, delay);
}

//-----------------------------------------------------------------------------
bool ad9106_set_pattern_timebase(const ad9106* const dev, uint8_t hold,
                                 uint8_t period_base, uint8_t start_delay_base)
{
    if (!dev || (hold > AD9106_TIMEBASE_FIELD_MAX) ||
        (period_base > AD9106_TIMEBASE_FIELD_MAX) ||
        (start_delay_base > AD9106_TIMEBASE_FIELD_MAX))
    {
        return false;
    }

    const uint16_t reg = (uint16_t)(((uint16_t)hold << 8) |
                                    (uint16_t)((uint16_t)period_base << 4) |
                                    (uint16_t)start_delay_base);
    return ad9106_write_reg(dev, AD9106_REG_PAT_TIMEBASE, reg);
}

//-----------------------------------------------------------------------------
bool ad9106_set_pattern_repeat_mode(const ad9106* const dev, bool finite)
{
    return ad9106_write_reg_field(dev, AD9106_REG_PAT_TYPE,
                                  AD9106_PAT_TYPE_PATTERN_RPT, finite ? 1u : 0u);
}

//-----------------------------------------------------------------------------
bool ad9106_set_pattern_repeat_cycles(const ad9106* const dev,
                                      ad9106_channel ch, uint8_t cycles)
{
    if (!dev || !ad9106_channel_valid(ch))
    {
        return false;
    }

    const bool low_reg = (ch == AD9106_CHANNEL_1) || (ch == AD9106_CHANNEL_2);
    const uint16_t reg = low_reg ? (uint16_t)AD9106_REG_DAC2_1PATX
                                 : (uint16_t)AD9106_REG_DAC4_3PATX;

    // Odd channels occupy the low byte of their register, even ones the high byte
    const bool low_byte = (ch == AD9106_CHANNEL_1) || (ch == AD9106_CHANNEL_3);
    const uint16_t mask = low_byte ? (uint16_t)AD9106_DAC2_1PATX_DAC1_RPT_CYCLE
                                   : (uint16_t)AD9106_DAC2_1PATX_DAC2_RPT_CYCLE;
    return ad9106_write_reg_field(dev, reg, mask, cycles);
}

//-----------------------------------------------------------------------------
bool ad9106_start_pattern(const ad9106* const dev)
{
    if (!ad9106_write_reg_field(dev, AD9106_REG_PAT_STATUS,
                                AD9106_PAT_STATUS_RUN, 1u))
    {
        return false;
    }
    return ad9106_ram_update(dev);
}

//-----------------------------------------------------------------------------
bool ad9106_stop_pattern(const ad9106* const dev)
{
    if (!ad9106_write_reg_field(dev, AD9106_REG_PAT_STATUS,
                                AD9106_PAT_STATUS_RUN, 0u))
    {
        return false;
    }
    return ad9106_ram_update(dev);
}

//-----------------------------------------------------------------------------
bool ad9106_get_pattern_status(const ad9106* const dev, uint16_t* status)
{
    return ad9106_read_reg(dev, AD9106_REG_PAT_STATUS, status);
}

//-----------------------------------------------------------------------------
void ad9106_set_trigger(const ad9106* const dev, bool active)
{
    if (!dev)
    {
        return;
    }
    ad9106_set_pin(dev, AD9106_PIN_TRIGGER, !active);
}

//-----------------------------------------------------------------------------
bool ad9106_set_trigger_delay_mode(const ad9106* const dev, bool enable)
{
    return ad9106_write_reg_field(dev, AD9106_REG_TRIG_TW_SEL,
                                  AD9106_TRIG_TW_SEL_TRIG_DELAY_EN,
                                  enable ? 1u : 0u);
}

//-----------------------------------------------------------------------------
bool ad9106_calibrate(const ad9106* const dev, uint8_t clk_div)
{
    if (!dev || (clk_div > AD9106_CAL_CLK_DIV_MAX))
    {
        return false;
    }

    // Enable the calibration clock and enter the calibration mode
    const uint16_t cal_cfg = (uint16_t)(AD9106_CALCONFIG_CAL_MODE_EN |
                                        AD9106_CALCONFIG_CAL_CLK_EN |
                                        (uint16_t)(clk_div &
                                                   AD9106_CALCONFIG_CAL_CLK_DIV));
    if (!ad9106_write_reg(dev, AD9106_REG_CALCONFIG, cal_cfg))
    {
        return false;
    }
    if (!ad9106_write_reg(dev, AD9106_REG_COMPOFFSET,
                          AD9106_COMPOFFSET_START_CAL))
    {
        return false;
    }
    if (!ad9106_ram_update(dev))
    {
        return false;
    }

    // Bounded wait for the CAL_FIN flag
    bool finished = false;
    for (uint32_t retry = 0; retry < AD9106_CAL_POLL_RETRIES; ++retry)
    {
        uint16_t reg = 0;
        if (!ad9106_read_reg(dev, AD9106_REG_COMPOFFSET, &reg))
        {
            return false;
        }
        if ((reg & AD9106_COMPOFFSET_CAL_FIN) != 0)
        {
            finished = true;
            break;
        }
        ad9106_delay_ms(AD9106_CAL_POLL_STEP_MS);
    }

    // Leave the calibration mode no matter what the outcome was
    const bool stopped = ad9106_write_reg(dev, AD9106_REG_COMPOFFSET, 0x0000) &&
                         ad9106_write_reg_field(dev, AD9106_REG_CALCONFIG,
                                                AD9106_CALCONFIG_CAL_MODE_EN, 0u);
    return finished && stopped;
}

//-----------------------------------------------------------------------------
bool ad9106_get_error_flags(const ad9106* const dev, uint16_t* flags)
{
    return ad9106_read_reg(dev, AD9106_REG_CFG_ERROR, flags);
}

//-----------------------------------------------------------------------------
bool ad9106_clear_error_flags(const ad9106* const dev)
{
    return ad9106_write_reg(dev, AD9106_REG_CFG_ERROR, AD9106_CFG_ERROR_CLEAR);
}

//-----------------------------------------------------------------------------
