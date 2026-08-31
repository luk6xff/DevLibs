/**
 *  @brief:   AD9106 quad, low power, 12-bit, 180 MSPS DAC and waveform generator library
 *  @author:  luk6xff
 *  @email:   lukasz.uszko@gmail.com
 *  @date:    2020-09-02
 *  @license: MIT
 */

#ifndef __AD9106_H__
#define __AD9106_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Number of DAC output channels */
#define AD9106_CHANNEL_COUNT      4

/** Size of the pattern SRAM in 16-bit words (0x6000 ... 0x6FFF) */
#define AD9106_SRAM_SIZE          4096u

/** Number of bits of the DDS phase accumulator and of the tuning word */
#define AD9106_DDS_TW_BITS        24u

/** Largest DDS tuning word */
#define AD9106_DDS_TW_MAX         0x00FFFFFFu

/** DAC sample clock assumed when `dac_clk_hz` is left at 0 [Hz] */
#define AD9106_DEFAULT_DAC_CLK_HZ 180000000u

/** Length of a single SPI access: 1 instruction word + 1 data word */
#define AD9106_SPI_FRAME_SIZE     4u

/** Read/write flag, set in the MSB of the first instruction byte */
#define AD9106_SPI_READ_BIT       0x80u

/** Register addresses are 15 bits wide */
#define AD9106_REG_ADDR_MASK      0x7FFFu

/**
 * @brief Device register map
 */
typedef enum
{
    AD9106_REG_SPICONFIG      = 0x0000,
    AD9106_REG_POWERCONFIG    = 0x0001,
    AD9106_REG_CLOCKCONFIG    = 0x0002,
    AD9106_REG_REFADJ         = 0x0003,
    AD9106_REG_DAC4AGAIN      = 0x0004,
    AD9106_REG_DAC3AGAIN      = 0x0005,
    AD9106_REG_DAC2AGAIN      = 0x0006,
    AD9106_REG_DAC1AGAIN      = 0x0007,
    AD9106_REG_DACXRANGE      = 0x0008,
    AD9106_REG_DAC4RSET       = 0x0009,
    AD9106_REG_DAC3RSET       = 0x000A,
    AD9106_REG_DAC2RSET       = 0x000B,
    AD9106_REG_DAC1RSET       = 0x000C,
    AD9106_REG_CALCONFIG      = 0x000D,
    AD9106_REG_COMPOFFSET     = 0x000E,
    AD9106_REG_RAMUPDATE      = 0x001D,
    AD9106_REG_PAT_STATUS     = 0x001E,
    AD9106_REG_PAT_TYPE       = 0x001F,
    AD9106_REG_PATTERN_DLY    = 0x0020,
    AD9106_REG_DAC4DOF        = 0x0022,
    AD9106_REG_DAC3DOF        = 0x0023,
    AD9106_REG_DAC2DOF        = 0x0024,
    AD9106_REG_DAC1DOF        = 0x0025,
    AD9106_REG_WAV4_3CONFIG   = 0x0026,
    AD9106_REG_WAV2_1CONFIG   = 0x0027,
    AD9106_REG_PAT_TIMEBASE   = 0x0028,
    AD9106_REG_PAT_PERIOD     = 0x0029,
    AD9106_REG_DAC4_3PATX     = 0x002A,
    AD9106_REG_DAC2_1PATX     = 0x002B,
    AD9106_REG_DOUT_START_DLY = 0x002C,
    AD9106_REG_DOUT_CONFIG    = 0x002D,
    AD9106_REG_DAC4_CST       = 0x002E,
    AD9106_REG_DAC3_CST       = 0x002F,
    AD9106_REG_DAC2_CST       = 0x0030,
    AD9106_REG_DAC1_CST       = 0x0031,
    AD9106_REG_DAC4_DGAIN     = 0x0032,
    AD9106_REG_DAC3_DGAIN     = 0x0033,
    AD9106_REG_DAC2_DGAIN     = 0x0034,
    AD9106_REG_DAC1_DGAIN     = 0x0035,
    AD9106_REG_SAW4_3CONFIG   = 0x0036,
    AD9106_REG_SAW2_1CONFIG   = 0x0037,
    AD9106_REG_DDS_TW32       = 0x003E,
    AD9106_REG_DDS_TW1        = 0x003F,
    AD9106_REG_DDS4_PW        = 0x0040,
    AD9106_REG_DDS3_PW        = 0x0041,
    AD9106_REG_DDS2_PW        = 0x0042,
    AD9106_REG_DDS1_PW        = 0x0043,
    AD9106_REG_TRIG_TW_SEL    = 0x0044,
    AD9106_REG_DDSX_CONFIG    = 0x0045,
    AD9106_REG_TW_RAM_CONFIG  = 0x0047,
    AD9106_REG_START_DLY4     = 0x0050,
    AD9106_REG_START_ADDR4    = 0x0051,
    AD9106_REG_STOP_ADDR4     = 0x0052,
    AD9106_REG_DDS_CYC4       = 0x0053,
    AD9106_REG_START_DLY3     = 0x0054,
    AD9106_REG_START_ADDR3    = 0x0055,
    AD9106_REG_STOP_ADDR3     = 0x0056,
    AD9106_REG_DDS_CYC3       = 0x0057,
    AD9106_REG_START_DLY2     = 0x0058,
    AD9106_REG_START_ADDR2    = 0x0059,
    AD9106_REG_STOP_ADDR2     = 0x005A,
    AD9106_REG_DDS_CYC2       = 0x005B,
    AD9106_REG_START_DLY1     = 0x005C,
    AD9106_REG_START_ADDR1    = 0x005D,
    AD9106_REG_STOP_ADDR1     = 0x005E,
    AD9106_REG_DDS_CYC1       = 0x005F,
    AD9106_REG_CFG_ERROR      = 0x0060,
    AD9106_REG_SRAMDATA       = 0x6000, // 0x6000 ... 0x6FFF
} ad9106_reg;

//-----------------------------------------------------------------------------
// Register bit masks
//-----------------------------------------------------------------------------

// SPICONFIG (0x0000)
// NOTE: SPICONFIG[15:10] must always mirror SPICONFIG[5:0] so that the SPI
// operation can be recovered when the LSBFIRST bit is set incorrectly.
#define AD9106_SPICONFIG_LSBFIRST         0x8000u
#define AD9106_SPICONFIG_SPI3WIRE         0x4000u
#define AD9106_SPICONFIG_RESET            0x2000u
#define AD9106_SPICONFIG_DOUBLESPI        0x1000u
#define AD9106_SPICONFIG_SPI_DRV          0x0800u
#define AD9106_SPICONFIG_DOUT_EN          0x0400u
#define AD9106_SPICONFIG_DOUT_ENM         0x0020u
#define AD9106_SPICONFIG_SPI_DRVM         0x0010u
#define AD9106_SPICONFIG_DOUBLESPIM       0x0008u
#define AD9106_SPICONFIG_RESETM           0x0004u
#define AD9106_SPICONFIG_SPI3WIREM        0x0002u
#define AD9106_SPICONFIG_LSBFIRSTM        0x0001u

// POWERCONFIG (0x0001)
#define AD9106_POWERCONFIG_CLK_LDO_STAT   0x0800u // Read only
#define AD9106_POWERCONFIG_DIG1_LDO_STAT  0x0400u // Read only
#define AD9106_POWERCONFIG_DIG2_LDO_STAT  0x0200u // Read only
#define AD9106_POWERCONFIG_PDN_LDO_CLK    0x0100u
#define AD9106_POWERCONFIG_PDN_LDO_DIG1   0x0080u
#define AD9106_POWERCONFIG_PDN_LDO_DIG2   0x0040u
#define AD9106_POWERCONFIG_REF_PDN        0x0020u
#define AD9106_POWERCONFIG_REF_EXT        0x0010u
#define AD9106_POWERCONFIG_DAC1_SLEEP     0x0008u
#define AD9106_POWERCONFIG_DAC2_SLEEP     0x0004u
#define AD9106_POWERCONFIG_DAC3_SLEEP     0x0002u
#define AD9106_POWERCONFIG_DAC4_SLEEP     0x0001u

// CLOCKCONFIG (0x0002)
#define AD9106_CLOCKCONFIG_DIS_CLK1       0x0800u
#define AD9106_CLOCKCONFIG_DIS_CLK2       0x0400u
#define AD9106_CLOCKCONFIG_DIS_CLK3       0x0200u
#define AD9106_CLOCKCONFIG_DIS_CLK4       0x0100u
#define AD9106_CLOCKCONFIG_DIS_DCLK       0x0080u
#define AD9106_CLOCKCONFIG_CLK_SLEEP      0x0040u
#define AD9106_CLOCKCONFIG_CLK_PDN        0x0020u
#define AD9106_CLOCKCONFIG_EPS            0x0010u
#define AD9106_CLOCKCONFIG_DAC1_INV_CLK   0x0008u
#define AD9106_CLOCKCONFIG_DAC2_INV_CLK   0x0004u
#define AD9106_CLOCKCONFIG_DAC3_INV_CLK   0x0002u
#define AD9106_CLOCKCONFIG_DAC4_INV_CLK   0x0001u

// REFADJ (0x0003)
#define AD9106_REFADJ_BGDR                0x003Fu

// DACxAGAIN (0x0004 ... 0x0007)
#define AD9106_DACXAGAIN_GAIN_CAL         0x7F00u // Read only
#define AD9106_DACXAGAIN_GAIN             0x007Fu

// DACxRANGE (0x0008)
#define AD9106_DACXRANGE_DAC4_GAIN_RNG    0x00C0u
#define AD9106_DACXRANGE_DAC3_GAIN_RNG    0x0030u
#define AD9106_DACXRANGE_DAC2_GAIN_RNG    0x000Cu
#define AD9106_DACXRANGE_DAC1_GAIN_RNG    0x0003u

// DACxRSET (0x0009 ... 0x000C)
#define AD9106_DACXRSET_EN                0x8000u
#define AD9106_DACXRSET_CAL               0x1F00u // Read only
#define AD9106_DACXRSET_RSET              0x001Fu

// CALCONFIG (0x000D)
#define AD9106_CALCONFIG_COMP_OFFSET_OF   0x4000u // Read only
#define AD9106_CALCONFIG_COMP_OFFSET_UF   0x2000u // Read only
#define AD9106_CALCONFIG_RSET_CAL_OF      0x1000u // Read only
#define AD9106_CALCONFIG_RSET_CAL_UF      0x0800u // Read only
#define AD9106_CALCONFIG_GAIN_CAL_OF      0x0400u // Read only
#define AD9106_CALCONFIG_GAIN_CAL_UF      0x0200u // Read only
#define AD9106_CALCONFIG_CAL_RESET        0x0100u
#define AD9106_CALCONFIG_CAL_MODE         0x0080u // Read only
#define AD9106_CALCONFIG_CAL_MODE_EN      0x0040u
#define AD9106_CALCONFIG_COMP_CAL_RNG     0x0030u
#define AD9106_CALCONFIG_CAL_CLK_EN       0x0008u
#define AD9106_CALCONFIG_CAL_CLK_DIV      0x0007u

// COMPOFFSET (0x000E)
#define AD9106_COMPOFFSET_CAL             0x7F00u // Read only
#define AD9106_COMPOFFSET_CAL_FIN         0x0002u // Read only
#define AD9106_COMPOFFSET_START_CAL       0x0001u

// RAMUPDATE (0x001D)
#define AD9106_RAMUPDATE_UPDATE           0x0001u

// PAT_STATUS (0x001E)
#define AD9106_PAT_STATUS_BUF_READ        0x0008u
#define AD9106_PAT_STATUS_MEM_ACCESS      0x0004u
#define AD9106_PAT_STATUS_PATTERN         0x0002u // Read only
#define AD9106_PAT_STATUS_RUN             0x0001u

// PAT_TYPE (0x001F)
#define AD9106_PAT_TYPE_PATTERN_RPT       0x0001u

// PATTERN_DLY (0x0020)
#define AD9106_PATTERN_DLY_DELAY          0xFFFFu

// DACxDOF (0x0022 ... 0x0025)
#define AD9106_DACXDOF_DIG_OFFSET         0xFFF0u

// WAV4_3CONFIG (0x0026)
#define AD9106_WAV4_3CONFIG_PRESTORE_SEL4 0x3000u
#define AD9106_WAV4_3CONFIG_WAVE_SEL4     0x0300u
#define AD9106_WAV4_3CONFIG_PRESTORE_SEL3 0x0030u
#define AD9106_WAV4_3CONFIG_WAVE_SEL3     0x0003u

// WAV2_1CONFIG (0x0027)
#define AD9106_WAV2_1CONFIG_PRESTORE_SEL2 0x3000u
#define AD9106_WAV2_1CONFIG_MASK_DAC4     0x0800u
#define AD9106_WAV2_1CONFIG_CH2_ADD       0x0400u
#define AD9106_WAV2_1CONFIG_WAVE_SEL2     0x0300u
#define AD9106_WAV2_1CONFIG_PRESTORE_SEL1 0x0030u
#define AD9106_WAV2_1CONFIG_MASK_DAC3     0x0008u
#define AD9106_WAV2_1CONFIG_CH1_ADD       0x0004u
#define AD9106_WAV2_1CONFIG_WAVE_SEL1     0x0003u

// PAT_TIMEBASE (0x0028)
#define AD9106_PAT_TIMEBASE_HOLD          0x0F00u
#define AD9106_PAT_TIMEBASE_PERIOD_BASE   0x00F0u
#define AD9106_PAT_TIMEBASE_START_DLY_BASE 0x000Fu

// PAT_PERIOD (0x0029)
#define AD9106_PAT_PERIOD_PATTERN_PERIOD  0xFFFFu

// DAC4_3PATx (0x002A)
#define AD9106_DAC4_3PATX_DAC4_RPT_CYCLE  0xFF00u
#define AD9106_DAC4_3PATX_DAC3_RPT_CYCLE  0x00FFu

// DAC2_1PATx (0x002B)
#define AD9106_DAC2_1PATX_DAC2_RPT_CYCLE  0xFF00u
#define AD9106_DAC2_1PATX_DAC1_RPT_CYCLE  0x00FFu

// DOUT_START_DLY (0x002C)
#define AD9106_DOUT_START_DLY_START       0xFFFFu

// DOUT_CONFIG (0x002D)
#define AD9106_DOUT_CONFIG_DOUT_VAL       0x0020u
#define AD9106_DOUT_CONFIG_DOUT_MODE      0x0010u
#define AD9106_DOUT_CONFIG_DOUT_STOP      0x000Fu

// DACx_CST (0x002E ... 0x0031)
#define AD9106_DACX_CST_CONST             0xFFF0u

// DACx_DGAIN (0x0032 ... 0x0035)
#define AD9106_DACX_DGAIN_DIG_GAIN        0xFFF0u

// SAW4_3CONFIG (0x0036)
#define AD9106_SAW4_3CONFIG_SAW_STEP4     0xFC00u
#define AD9106_SAW4_3CONFIG_SAW_TYPE4     0x0300u
#define AD9106_SAW4_3CONFIG_SAW_STEP3     0x00FCu
#define AD9106_SAW4_3CONFIG_SAW_TYPE3     0x0003u

// SAW2_1CONFIG (0x0037)
#define AD9106_SAW2_1CONFIG_SAW_STEP2     0xFC00u
#define AD9106_SAW2_1CONFIG_SAW_TYPE2     0x0300u
#define AD9106_SAW2_1CONFIG_SAW_STEP1     0x00FCu
#define AD9106_SAW2_1CONFIG_SAW_TYPE1     0x0003u

// DDS_TW32 (0x003E) / DDS_TW1 (0x003F)
#define AD9106_DDS_TW32_DDSTW_MSB         0xFFFFu // DDSTW[23:8]
#define AD9106_DDS_TW1_DDSTW_LSB          0xFF00u // DDSTW[7:0]

// DDSx_PW (0x0040 ... 0x0043)
#define AD9106_DDSX_PW_PHASE              0xFFFFu

// TRIG_TW_SEL (0x0044)
#define AD9106_TRIG_TW_SEL_TRIG_DELAY_EN  0x0002u

// DDSx_CONFIG (0x0045)
#define AD9106_DDSX_CONFIG_DDS_COS_EN4    0x8000u
#define AD9106_DDSX_CONFIG_DDS_MSB_EN4    0x4000u
#define AD9106_DDSX_CONFIG_DDS_COS_EN3    0x0800u
#define AD9106_DDSX_CONFIG_DDS_MSB_EN3    0x0400u
#define AD9106_DDSX_CONFIG_PHASE_MEM_EN3  0x0200u
#define AD9106_DDSX_CONFIG_DDS_COS_EN2    0x0080u
#define AD9106_DDSX_CONFIG_DDS_MSB_EN2    0x0040u
#define AD9106_DDSX_CONFIG_DDS_MSB_EN1    0x0004u
#define AD9106_DDSX_CONFIG_TW_MEM_EN      0x0001u

// TW_RAM_CONFIG (0x0047)
#define AD9106_TW_RAM_CONFIG_TW_MEM_SHIFT 0x000Fu

// START_DLYx / START_ADDRx / STOP_ADDRx / DDS_CYCx (0x0050 ... 0x005F)
#define AD9106_START_DLYX_DELAY           0xFFFFu
#define AD9106_START_ADDRX_ADDR           0xFFF0u
#define AD9106_STOP_ADDRX_ADDR            0xFFF0u
#define AD9106_DDS_CYCX_CYCLES            0xFFFFu

// CFG_ERROR (0x0060)
#define AD9106_CFG_ERROR_CLEAR            0x8000u
#define AD9106_CFG_ERROR_ERRORS           0x7FC0u
#define AD9106_CFG_ERROR_DOUT_START_LG    0x0020u // Read only
#define AD9106_CFG_ERROR_PAT_DLY_SHORT    0x0010u // Read only
#define AD9106_CFG_ERROR_DOUT_START_SHORT 0x0008u // Read only
#define AD9106_CFG_ERROR_PERIOD_SHORT     0x0004u // Read only
#define AD9106_CFG_ERROR_ODD_ADDR         0x0002u // Read only
#define AD9106_CFG_ERROR_MEM_READ         0x0001u // Read only

//-----------------------------------------------------------------------------
// Driver types
//-----------------------------------------------------------------------------

/**
 * @brief DAC output channel
 */
typedef enum
{
    AD9106_CHANNEL_1 = 1,
    AD9106_CHANNEL_2 = 2,
    AD9106_CHANNEL_3 = 3,
    AD9106_CHANNEL_4 = 4,
} ad9106_channel;

/**
 * @brief Prestored waveform generator, PRESTORE_SELx field
 */
typedef enum
{
    AD9106_PRESTORE_CONSTANT      = 0, // DACx_CST constant value
    AD9106_PRESTORE_SAWTOOTH      = 1, // Sawtooth/triangle generator
    AD9106_PRESTORE_PSEUDO_RANDOM = 2, // Pseudorandom noise generator
    AD9106_PRESTORE_DDS           = 3, // DDS sine/cosine generator
} ad9106_prestored_wave;

/**
 * @brief Waveform source, WAVE_SELx field
 */
typedef enum
{
    AD9106_WAVE_SRAM               = 0, // SRAM pattern only
    AD9106_WAVE_PRESTORED          = 1, // Prestored waveform only
    AD9106_WAVE_PRESTORED_X_SRAM   = 2, // Prestored waveform multiplied by the SRAM pattern
    AD9106_WAVE_PRESTORED_PLUS_DLY = 3, // Start delay, prestored waveform and SRAM pattern
} ad9106_wave_sel;

/**
 * @brief Sawtooth generator shape, SAW_TYPEx field
 */
typedef enum
{
    AD9106_SAW_RAMP_UP   = 0,
    AD9106_SAW_RAMP_DOWN = 1,
    AD9106_SAW_TRIANGLE  = 2,
    AD9106_SAW_ZERO      = 3, // No wave, output stays at zero
} ad9106_saw_type;

/**
 * @brief Control lines driven by the platform layer
 */
typedef enum
{
    AD9106_PIN_CS = 0, // SPI chip select, active low
    AD9106_PIN_RESET,  // Hardware reset, active low, optional
    AD9106_PIN_TRIGGER, // Pattern trigger, active low
} ad9106_pin;

/**
 * @brief ad9106 dev object
 */
typedef struct
{
    uint32_t dac_clk_hz; // DAC sample clock in Hz, 0 selects AD9106_DEFAULT_DAC_CLK_HZ
    void* platform_dev;  // Platform descriptor, assigned by ad9106_<platform>_init()
} ad9106;

//-----------------------------------------------------------------------------
// API
//-----------------------------------------------------------------------------

/**
 * @brief Reset the device and check that the SPI link is working.
 *
 * @param dev ad9106 device object
 * @return True on success, false otherwise
 */
bool ad9106_init(ad9106* const dev);

/**
 * @brief Execute a software reset, all registers but SPICONFIG are reloaded
 *        with their default values.
 *
 * @param dev ad9106 device object
 * @return True on success, false otherwise
 */
bool ad9106_reset(const ad9106* const dev);

/**
 * @brief Write a 16-bit device register.
 *
 * @param dev      ad9106 device object
 * @param reg_addr Register address, see ad9106_reg
 * @param value    Value to be written
 * @return True on success, false otherwise
 */
bool ad9106_write_reg(const ad9106* const dev, uint16_t reg_addr, uint16_t value);

/**
 * @brief Read a 16-bit device register.
 *
 * @param      dev      ad9106 device object
 * @param      reg_addr Register address, see ad9106_reg
 * @param[out] value    Value read from the device
 * @return True on success, false otherwise
 */
bool ad9106_read_reg(const ad9106* const dev, uint16_t reg_addr, uint16_t* value);

/**
 * @brief Read, modify and write back a single register bit field.
 *
 * @param dev      ad9106 device object
 * @param reg_addr Register address, see ad9106_reg
 * @param mask     Bit field mask, must not be 0
 * @param value    Unshifted field value
 * @return True on success, false otherwise
 */
bool ad9106_write_reg_field(const ad9106* const dev, uint16_t reg_addr,
                            uint16_t mask, uint16_t value);

/**
 * @brief Read a single register bit field, shifted down to bit 0.
 *
 * @param      dev      ad9106 device object
 * @param      reg_addr Register address, see ad9106_reg
 * @param      mask     Bit field mask, must not be 0
 * @param[out] value    Unshifted field value
 * @return True on success, false otherwise
 */
bool ad9106_read_reg_field(const ad9106* const dev, uint16_t reg_addr,
                           uint16_t mask, uint16_t* value);

/**
 * @brief Transfer the shadow register content into the active registers.
 *
 * Most of the pattern and waveform registers only take effect after this call.
 *
 * @param dev ad9106 device object
 * @return True on success, false otherwise
 */
bool ad9106_ram_update(const ad9106* const dev);

/**
 * @brief Write a pattern into the device SRAM.
 *
 * The pattern generator is stopped for the duration of the transfer. The DAC
 * takes the 12 most significant bits of every word, so a sample has to be
 * placed in bits [15:4].
 *
 * @param dev    ad9106 device object
 * @param offset Word offset within the SRAM (0 ... AD9106_SRAM_SIZE-1)
 * @param data   Words to be written
 * @param len    Number of words to be written
 * @return True on success, false otherwise
 */
bool ad9106_write_sram(const ad9106* const dev, uint16_t offset,
                       const uint16_t* data, size_t len);

/**
 * @brief Read a pattern back from the device SRAM.
 *
 * @param      dev    ad9106 device object
 * @param      offset Word offset within the SRAM (0 ... AD9106_SRAM_SIZE-1)
 * @param[out] data   Buffer filled with the words read
 * @param      len    Number of words to read
 * @return True on success, false otherwise
 */
bool ad9106_read_sram(const ad9106* const dev, uint16_t offset,
                      uint16_t* data, size_t len);

/**
 * @brief Select the waveform source and the prestored generator of a channel.
 *
 * @param dev      ad9106 device object
 * @param ch       DAC channel
 * @param wave     Waveform source
 * @param prestore Prestored waveform generator
 * @return True on success, false otherwise
 */
bool ad9106_set_wave_config(const ad9106* const dev, ad9106_channel ch,
                            ad9106_wave_sel wave,
                            ad9106_prestored_wave prestore);

/**
 * @brief Configure the sawtooth generator of a channel.
 *
 * @param dev  ad9106 device object
 * @param ch   DAC channel
 * @param type Sawtooth shape
 * @param step Number of samples per step (0 ... 63)
 * @return True on success, false otherwise
 */
bool ad9106_set_sawtooth(const ad9106* const dev, ad9106_channel ch,
                         ad9106_saw_type type, uint8_t step);

/**
 * @brief Set the raw 24-bit DDS tuning word shared by all channels.
 *
 * @param dev ad9106 device object
 * @param tw  Tuning word (0 ... AD9106_DDS_TW_MAX)
 * @return True on success, false otherwise
 */
bool ad9106_set_dds_tuning_word(const ad9106* const dev, uint32_t tw);

/**
 * @brief Set the DDS output frequency.
 *
 * The tuning word is computed from `dev->dac_clk_hz`, so that field has to
 * describe the clock actually fed into the DACCLK pins.
 *
 * @param dev     ad9106 device object
 * @param freq_hz Requested frequency in Hz, must be below dac_clk_hz/2
 * @return True on success, false otherwise
 */
bool ad9106_set_dds_freq(const ad9106* const dev, uint32_t freq_hz);

/**
 * @brief Set the DDS phase offset of a channel.
 *
 * @param dev   ad9106 device object
 * @param ch    DAC channel
 * @param phase Phase offset, full scale (0xFFFF) corresponds to 360 degrees
 * @return True on success, false otherwise
 */
bool ad9106_set_dds_phase(const ad9106* const dev, ad9106_channel ch,
                          uint16_t phase);

/**
 * @brief Set the number of DDS sine wave cycles played by a channel.
 *
 * @param dev    ad9106 device object
 * @param ch     DAC channel
 * @param cycles Number of cycles
 * @return True on success, false otherwise
 */
bool ad9106_set_dds_cycles(const ad9106* const dev, ad9106_channel ch,
                           uint16_t cycles);

/**
 * @brief Set the digital gain of a channel.
 *
 * @param dev  ad9106 device object
 * @param ch   DAC channel
 * @param gain Signed 12-bit gain (-2048 ... 2047), 0x0800 equals a gain of 1.0
 * @return True on success, false otherwise
 */
bool ad9106_set_digital_gain(const ad9106* const dev, ad9106_channel ch,
                             int16_t gain);

/**
 * @brief Set the digital offset added to the output of a channel.
 *
 * @param dev    ad9106 device object
 * @param ch     DAC channel
 * @param offset Signed 12-bit offset (-2048 ... 2047)
 * @return True on success, false otherwise
 */
bool ad9106_set_digital_offset(const ad9106* const dev, ad9106_channel ch,
                               int16_t offset);

/**
 * @brief Set the constant output value of a channel.
 *
 * Used when the channel is configured with AD9106_PRESTORE_CONSTANT.
 *
 * @param dev   ad9106 device object
 * @param ch    DAC channel
 * @param value Signed 12-bit DAC code (-2048 ... 2047)
 * @return True on success, false otherwise
 */
bool ad9106_set_constant_value(const ad9106* const dev, ad9106_channel ch,
                               int16_t value);

/**
 * @brief Set the analog full scale gain of a channel.
 *
 * @param dev  ad9106 device object
 * @param ch   DAC channel
 * @param gain 7-bit gain code (0 ... 127)
 * @return True on success, false otherwise
 */
bool ad9106_set_analog_gain(const ad9106* const dev, ad9106_channel ch,
                            uint8_t gain);

/**
 * @brief Select the analog full scale current range of a channel.
 *
 * @param dev   ad9106 device object
 * @param ch    DAC channel
 * @param range 2-bit range code (0 ... 3)
 * @return True on success, false otherwise
 */
bool ad9106_set_gain_range(const ad9106* const dev, ad9106_channel ch,
                           uint8_t range);

/**
 * @brief Configure the internal RSET resistor of a channel.
 *
 * @param dev     ad9106 device object
 * @param ch      DAC channel
 * @param use_spi True selects the value given here, false the calibrated one
 * @param rset    5-bit RSET code (0 ... 31)
 * @return True on success, false otherwise
 */
bool ad9106_set_rset(const ad9106* const dev, ad9106_channel ch, bool use_spi,
                     uint8_t rset);

/**
 * @brief Enable or disable the output current of a channel.
 *
 * @param dev   ad9106 device object
 * @param ch    DAC channel
 * @param sleep True disables the output current of the channel
 * @return True on success, false otherwise
 */
bool ad9106_set_dac_sleep(const ad9106* const dev, ad9106_channel ch,
                          bool sleep);

/**
 * @brief Configure the SRAM region played back by a channel.
 *
 * @param dev         ad9106 device object
 * @param ch          DAC channel
 * @param start_addr  First SRAM word played (0 ... AD9106_SRAM_SIZE-1)
 * @param stop_addr   Last SRAM word played (0 ... AD9106_SRAM_SIZE-1)
 * @param start_delay Delay before the pattern starts, in DAC clock cycles
 * @return True on success, false otherwise
 */
bool ad9106_config_sram_playback(const ad9106* const dev, ad9106_channel ch,
                                 uint16_t start_addr, uint16_t stop_addr,
                                 uint16_t start_delay);

/**
 * @brief Set the pattern period.
 *
 * @param dev    ad9106 device object
 * @param period Pattern period expressed in PAT_TIMEBASE units
 * @return True on success, false otherwise
 */
bool ad9106_set_pattern_period(const ad9106* const dev, uint16_t period);

/**
 * @brief Set the delay between the trigger and the first pattern sample.
 *
 * @param dev   ad9106 device object
 * @param delay Pattern delay in DAC clock cycles
 * @return True on success, false otherwise
 */
bool ad9106_set_pattern_delay(const ad9106* const dev, uint16_t delay);

/**
 * @brief Set the pattern generator time base.
 *
 * @param dev             ad9106 device object
 * @param hold            HOLD field (0 ... 15)
 * @param period_base     PAT_PERIOD_BASE field (0 ... 15)
 * @param start_delay_base START_DELAY_BASE field (0 ... 15)
 * @return True on success, false otherwise
 */
bool ad9106_set_pattern_timebase(const ad9106* const dev, uint8_t hold,
                                 uint8_t period_base, uint8_t start_delay_base);

/**
 * @brief Select whether a pattern repeats a finite number of times.
 *
 * @param dev    ad9106 device object
 * @param finite True repeats the pattern DACx_REPEAT_CYCLE times,
 *               false repeats it continuously
 * @return True on success, false otherwise
 */
bool ad9106_set_pattern_repeat_mode(const ad9106* const dev, bool finite);

/**
 * @brief Set how many times the pattern of a channel is repeated.
 *
 * @param dev    ad9106 device object
 * @param ch     DAC channel
 * @param cycles Number of repetitions (0 ... 255)
 * @return True on success, false otherwise
 */
bool ad9106_set_pattern_repeat_cycles(const ad9106* const dev,
                                      ad9106_channel ch, uint8_t cycles);

/**
 * @brief Start the pattern generator.
 *
 * @param dev ad9106 device object
 * @return True on success, false otherwise
 */
bool ad9106_start_pattern(const ad9106* const dev);

/**
 * @brief Stop the pattern generator.
 *
 * @param dev ad9106 device object
 * @return True on success, false otherwise
 */
bool ad9106_stop_pattern(const ad9106* const dev);

/**
 * @brief Read the PAT_STATUS register.
 *
 * @param      dev    ad9106 device object
 * @param[out] status Raw PAT_STATUS content
 * @return True on success, false otherwise
 */
bool ad9106_get_pattern_status(const ad9106* const dev, uint16_t* status);

/**
 * @brief Drive the TRIGGER line.
 *
 * The TRIGGER input is active low, so `active == true` drives the pin low.
 *
 * @param dev    ad9106 device object
 * @param active True asserts the trigger, false releases it
 */
void ad9106_set_trigger(const ad9106* const dev, bool active);

/**
 * @brief Enable the trigger delay mode.
 *
 * @param dev    ad9106 device object
 * @param enable True applies the start delay only to the first pattern
 * @return True on success, false otherwise
 */
bool ad9106_set_trigger_delay_mode(const ad9106* const dev, bool enable);

/**
 * @brief Run the internal gain, RSET and comparator offset calibration.
 *
 * The call blocks until the CAL_FIN flag is set or the calibration times out.
 *
 * @param dev     ad9106 device object
 * @param clk_div Calibration clock divider code (0 ... 7)
 * @return True on success, false otherwise
 */
bool ad9106_calibrate(const ad9106* const dev, uint8_t clk_div);

/**
 * @brief Read the configuration error flags.
 *
 * @param      dev   ad9106 device object
 * @param[out] flags Raw CFG_ERROR content
 * @return True on success, false otherwise
 */
bool ad9106_get_error_flags(const ad9106* const dev, uint16_t* flags);

/**
 * @brief Clear all configuration error flags.
 *
 * @param dev ad9106 device object
 * @return True on success, false otherwise
 */
bool ad9106_clear_error_flags(const ad9106* const dev);


//-----------------------------------------------------------------------------
// @brief HW dependent functions - must be defined for each platform
//-----------------------------------------------------------------------------

/**
 * @brief Full duplex SPI transfer, SPI mode 0 (CPOL=0, CPHA=0), MSB first.
 *
 * The chip select line is driven by the core with ad9106_set_pin().
 *
 * @param dev ad9106 device object
 * @param tx  Bytes to be shifted out
 * @param rx  Buffer filled with the bytes shifted in, same length as `tx`
 * @param len Number of bytes to transfer
 * @return True on success, false otherwise
 */
extern bool ad9106_spi_transfer(const ad9106* const dev, const uint8_t* tx,
                                uint8_t* rx, size_t len);

/**
 * @brief Drive one of the device control lines.
 *
 * Platforms which do not wire a given line simply ignore the call.
 *
 * @param dev   ad9106 device object
 * @param pin   Control line to be driven
 * @param state True drives the line high, false drives it low
 */
extern void ad9106_set_pin(const ad9106* const dev, ad9106_pin pin, bool state);

/**
 * @brief Block for a given number of milliseconds.
 *
 * @param delay_ms Delay in milliseconds
 */
extern void ad9106_delay_ms(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __AD9106_H__ */
