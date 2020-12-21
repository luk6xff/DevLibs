/**
 *  @brief:  LoRa library for SEMTECH SX127x devices
 *  @author: luk6xff based on the code: https://github.com/sandeepmistry/arduino-LoRa
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2020-11-22
 */

#include "lora.h"
#include "common.h"

// registers
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define REG_OCP                     0x0b
#define REG_LNA                     0x0c
#define REG_FIFO_ADDR_PTR           0x0d
#define REG_FIFO_TX_BASE_ADDR       0x0e
#define REG_FIFO_RX_BASE_ADDR       0x0f
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI_VALUE          0x1a
#define REG_RSSI_VALUE              0x1b
#define REG_MODEM_CONFIG_1          0x1d
#define REG_MODEM_CONFIG_2          0x1e
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_MODEM_CONFIG_3          0x26
#define REG_FREQ_ERROR_MSB          0x28
#define REG_FREQ_ERROR_MID          0x29
#define REG_FREQ_ERROR_LSB          0x2a
#define REG_RSSI_WIDEBAND           0x2c
#define REG_DETECTION_OPTIMIZE      0x31
#define REG_INVERTIQ                0x33
#define REG_DETECTION_THRESHOLD     0x37
#define REG_SYNC_WORD               0x39
#define REG_INVERTIQ2               0x3b
#define REG_DIO_MAPPING_1           0x40
#define REG_VERSION                 0x42
#define REG_PA_DAC                  0x4d

// modes
#define MODE_LONG_RANGE_MODE        0x80
#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RX_CONTINUOUS          0x05
#define MODE_RX_SINGLE              0x06

// PA config
#define PA_BOOST                    0x80

// IRQ masks
#define IRQ_TX_DONE_MASK            0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK  0x20
#define IRQ_RX_DONE_MASK            0x40

#define RF_MID_BAND_THRESHOLD       525E6
#define RSSI_OFFSET_HF_PORT         157
#define RSSI_OFFSET_LF_PORT         164

#define PA_OUTPUT_RFO_PIN           0
#define PA_OUTPUT_PA_BOOST_PIN      1


/**
 * ============================================================================
 * @brief Private functions prototypes
 * ============================================================================
 */

/**
 * @brief DIO 0 IRQ callback
 */
static ISR_PREFIX void on_dio0_irq(void *ctx);



/**
 * ============================================================================
 * @brief Public functions definitions
 * ============================================================================
 */

//-----------------------------------------------------------------------------
bool lora_init(lora *const dev)
{
    // Register Hardware DIO IRQ callback for dio0 - dio5
    dev->dio_irq = on_dio0_irq;

    // Perform Reset
    lora_reset(dev);

    // Initialize IO interfaces
    lora_io_init(dev);
    lora_ioirq_init(dev);

    // check version
    uint8_t version = lora_read_reg(dev, REG_VERSION);
    if (version != 0x12)
    {
        return false;
    }

    // put in sleep mode
    lora_sleep(dev);

    // set frequency
    lora_set_frequency(dev, dev->frequency);

    // set base addresses
    lora_write_reg(dev, REG_FIFO_TX_BASE_ADDR, 0);
    lora_write_reg(dev, REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    lora_write_reg(dev, REG_LNA, lora_read_reg(dev, REG_LNA) | 0x03);

    // set auto AGC
    lora_write_reg(dev, REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    lora_set_tx_power(dev, 17, PA_OUTPUT_PA_BOOST_PIN);

    // put in standby mode
    lora_idle(dev);

    return true;
}

//-----------------------------------------------------------------------------
void lora_end(lora *const dev)
{
    // put in sleep mode
    lora_sleep(dev);
    // Perform Reset
    lora_reset(dev);

    // Deinitialize IO interfaces
    lora_io_deinit(dev);
    lora_ioirq_deinit(dev);
}

//-----------------------------------------------------------------------------
int lora_begin_packet(lora *const dev, int implicitHeader)
{
    if (lora_is_transmitting(dev))
    {
        return 0;
    }

    // Put in standby mode
    lora_idle(dev);

    if (implicitHeader)
    {
        lora_implicit_header_mode(dev);
    }
    else
    {
        lora_explicit_header_mode(dev);
    }

    // reset FIFO address and paload length
    lora_write_reg(dev, REG_FIFO_ADDR_PTR, 0);
    lora_write_reg(dev, REG_PAYLOAD_LENGTH, 0);

    return 1;
}

//-----------------------------------------------------------------------------
int lora_end_packet(lora *const dev, bool async)
{
    if ((async) && (dev->on_tx_done))
    {
        lora_write_reg(dev, REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
    }

    // put in TX mode
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if (!async)
    {
        // wait for TX done
        const uint32_t start = lora_timer_read_ms();
        do
        {
            if ((lora_timer_read_ms() - start) > 5000)
            {
                break;
            }
        } while ((lora_read_reg(dev, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);
        // clear IRQ's
        lora_write_reg(dev, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 1;
}

//-----------------------------------------------------------------------------
bool lora_is_transmitting(lora *const dev)
{
    if ((lora_read_reg(dev, REG_OP_MODE) & MODE_TX) == MODE_TX)
    {
        return true;
    }

    if (lora_read_reg(dev, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
    {
        // clear IRQ's
        lora_write_reg(dev, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return false;
}

//-----------------------------------------------------------------------------
int lora_parse_packet(lora *const dev, int size)
{
    int packetLength = 0;
    int irqFlags = lora_read_reg(dev, REG_IRQ_FLAGS);

    if (size > 0)
    {
        lora_implicit_header_mode(dev);

        lora_write_reg(dev, REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
    {
        lora_explicit_header_mode(dev);
    }

    // clear IRQ's
    lora_write_reg(dev, REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        // received a packet
        dev->packet_index = 0;

        // read packet length
        if (dev->implicit_header_mode)
        {
            packetLength = lora_read_reg(dev, REG_PAYLOAD_LENGTH);
        }
        else
        {
            packetLength = lora_read_reg(dev, REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        lora_write_reg(dev, REG_FIFO_ADDR_PTR, lora_read_reg(dev, REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        lora_idle(dev);
    }
    else if (lora_read_reg(dev, REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
        // not currently in RX mode

        // reset FIFO address
        lora_write_reg(dev, REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

//-----------------------------------------------------------------------------
int lora_packet_rssi(lora *const dev)
{
    return (lora_read_reg(dev, REG_PKT_RSSI_VALUE) - (dev->frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

//-----------------------------------------------------------------------------
float lora_packet_snr(lora *const dev)
{
    return ((int8_t)lora_read_reg(dev, REG_PKT_SNR_VALUE)) * 0.25;
}

//-----------------------------------------------------------------------------
long lora_packet_frequency_error(lora *const dev)
{
    int32_t freqError = 0;
    freqError = (int32_t)(lora_read_reg(dev, REG_FREQ_ERROR_MSB) & 0x07);
    freqError <<= 8L;
    freqError += (int32_t)(lora_read_reg(dev, REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += (int32_t)(lora_read_reg(dev, REG_FREQ_ERROR_LSB));

    if (lora_read_reg(dev, REG_FREQ_ERROR_MSB) & 0x80)  // Sign bit is on
    {
        freqError -= 524288; // B1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = (((float)freqError * (1L << 24)) / fXtal) * (lora_get_signal_bandwidth(dev) / 500000.0f); // p. 37

    return (long)fError;
}

//-----------------------------------------------------------------------------
int lora_rssi(lora *const dev)
{
    return (lora_read_reg(dev, REG_RSSI_VALUE) - (dev->frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

//-----------------------------------------------------------------------------
uint32_t lora_write(lora *const dev, uint8_t byte)
{
    return lora_write_data(dev, &byte, sizeof(byte));
}

//-----------------------------------------------------------------------------
uint32_t lora_write_data(lora *const dev, const uint8_t *buffer, uint32_t size)
{
    int currentLength = lora_read_reg(dev, REG_PAYLOAD_LENGTH);

    // check size
    if ((currentLength + size) > LORA_MAX_PKT_LEN)
    {
        size = LORA_MAX_PKT_LEN - currentLength;
    }

    // write data
    // for (uint32_t i = 0; i < size; i++)
    // {
    //     lora_write_reg(dev, REG_FIFO, buffer[i]);
    // }
    lora_write_buffer(dev, REG_FIFO, buffer, size);

    // update length
    lora_write_reg(dev, REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

//-----------------------------------------------------------------------------
int lora_available(lora *const dev)
{
    return (lora_read_reg(dev, REG_RX_NB_BYTES) - dev->packet_index);
}

//-----------------------------------------------------------------------------
int lora_read(lora *const dev)
{
    if (!lora_available(dev))
    {
        return -1;
    }

    dev->packet_index++;

    return lora_read_reg(dev, REG_FIFO);
}

//-----------------------------------------------------------------------------
int lora_peek(lora *const dev)
{
    if (!lora_available(dev))
    {
        return -1;
    }

    // store current FIFO address
    int currentAddress = lora_read_reg(dev, REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = lora_read_reg(dev, REG_FIFO);

    // restore FIFO address
    lora_write_reg(dev, REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

//-----------------------------------------------------------------------------
void lora_flush(lora *const dev)
{
}

//-----------------------------------------------------------------------------
void lora_on_receive(lora *const dev, void(*callback)(void*, int))
{
    dev->on_receive = callback;
}

//-----------------------------------------------------------------------------
void lora_on_tx_done(lora *const dev, void(*callback)(void*))
{
    dev->on_tx_done = callback;
}

//-----------------------------------------------------------------------------
void lora_receive(lora *const dev, int size)
{
    lora_write_reg(dev, REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

    if (size > 0)
    {
        lora_implicit_header_mode(dev);
        lora_write_reg(dev, REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
    {
        lora_explicit_header_mode(dev);
    }

    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

//-----------------------------------------------------------------------------
void lora_idle(lora *const dev)
{
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

//-----------------------------------------------------------------------------
void lora_sleep(lora *const dev)
{
    lora_write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

//-----------------------------------------------------------------------------
void lora_set_tx_power(lora *const dev, int level, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin)
    {
        // RFO
        if (level < 0)
        {
            level = 0;
        }
        else if (level > 14)
        {
            level = 14;
        }

        lora_write_reg(dev, REG_PA_CONFIG, 0x70 | level);
    }
    else
    {
        // PA BOOST
        if (level > 17)
        {
            if (level > 20)
            {
                level = 20;
            }

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            lora_write_reg(dev, REG_PA_DAC, 0x87);
            lora_set_ocp(dev, 140);
        }
        else
        {
            if (level < 2)
            {
                level = 2;
            }
            //Default value PA_HF/LF or +17dBm
            lora_write_reg(dev, REG_PA_DAC, 0x84);
            lora_set_ocp(dev, 100);
        }

        lora_write_reg(dev, REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

//-----------------------------------------------------------------------------
void lora_set_frequency(lora *const dev, long frequency)
{
    dev->frequency = frequency;

    uint64_t frf = ((uint64_t)dev->frequency << 19) / 32000000;

    lora_write_reg(dev, REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(dev, REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(dev, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

//-----------------------------------------------------------------------------
int lora_get_spreading_factor(lora *const dev)
{
    return lora_read_reg(dev, REG_MODEM_CONFIG_2) >> 4;
}

//-----------------------------------------------------------------------------
void lora_set_spreading_factor(lora *const dev, int sf)
{
    if (sf < 6)
    {
        sf = 6;
    } else if (sf > 12)
    {
        sf = 12;
    }

    if (sf == 6)
    {
        lora_write_reg(dev, REG_DETECTION_OPTIMIZE, 0xc5);
        lora_write_reg(dev, REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        lora_write_reg(dev, REG_DETECTION_OPTIMIZE, 0xc3);
        lora_write_reg(dev, REG_DETECTION_THRESHOLD, 0x0a);
    }

    lora_write_reg(dev, REG_MODEM_CONFIG_2, (lora_read_reg(dev, REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    lora_set_ldo_flag(dev);
}

//-----------------------------------------------------------------------------
long lora_get_signal_bandwidth(lora *const dev)
{
    uint8_t bw = (lora_read_reg(dev, REG_MODEM_CONFIG_1) >> 4);

    switch (bw)
    {
        case 0: return 7.8E3;
        case 1: return 10.4E3;
        case 2: return 15.6E3;
        case 3: return 20.8E3;
        case 4: return 31.25E3;
        case 5: return 41.7E3;
        case 6: return 62.5E3;
        case 7: return 125E3;
        case 8: return 250E3;
        case 9: return 500E3;
    }

    return -1;
}

//-----------------------------------------------------------------------------
void lora_set_signal_bandwidth(lora *const dev, long sbw)
{
    int bw;

    if (sbw <= 7.8E3)
    {
        bw = 0;
    }
    else if (sbw <= 10.4E3)
    {
        bw = 1;
    }
    else if (sbw <= 15.6E3)
    {
        bw = 2;
    }
    else if (sbw <= 20.8E3)
    {
        bw = 3;
    }
    else if (sbw <= 31.25E3)
    {
        bw = 4;
    }
    else if (sbw <= 41.7E3)
    {
        bw = 5;
    }
    else if (sbw <= 62.5E3)
    {
        bw = 6;
    }
    else if (sbw <= 125E3)
    {
        bw = 7;
    }
    else if (sbw <= 250E3)
    {
        bw = 8;
    }
    else /*if (sbw <= 250E3)*/
    {
        bw = 9;
    }

    lora_write_reg(dev, REG_MODEM_CONFIG_1, (lora_read_reg(dev, REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    lora_set_ldo_flag(dev);
}

//-----------------------------------------------------------------------------
void lora_set_ldo_flag(lora *const dev)
{
    // Section 4.1.1.5
    long symbolDuration = 1000 / ( lora_get_signal_bandwidth(dev) / (1L << lora_get_spreading_factor(dev)) ) ;

    // Section 4.1.1.6
    bool ldoOn = symbolDuration > 16;

    uint8_t config3 = lora_read_reg(dev, REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    lora_write_reg(dev, REG_MODEM_CONFIG_3, config3);
}

//-----------------------------------------------------------------------------
void lora_set_coding_rate4(lora *const dev, int denominator)
{
    if (denominator < 5)
    {
        denominator = 5;
    }
    else if (denominator > 8)
    {
        denominator = 8;
    }

    int cr = denominator - 4;
    lora_write_reg(dev, REG_MODEM_CONFIG_1, (lora_read_reg(dev, REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

//-----------------------------------------------------------------------------
void lora_set_preamble_length(lora *const dev, long length)
{
    lora_write_reg(dev, REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    lora_write_reg(dev, REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

//-----------------------------------------------------------------------------
void lora_set_sync_word(lora *const dev, int sw)
{
    lora_write_reg(dev, REG_SYNC_WORD, sw);
}

//-----------------------------------------------------------------------------
void lora_enable_crc(lora *const dev)
{
    lora_write_reg(dev, REG_MODEM_CONFIG_2, lora_read_reg(dev, REG_MODEM_CONFIG_2) | 0x04);
}

//-----------------------------------------------------------------------------
void lora_disable_crc(lora *const dev)
{
    lora_write_reg(dev, REG_MODEM_CONFIG_2, lora_read_reg(dev, REG_MODEM_CONFIG_2) & 0xfb);
}

//-----------------------------------------------------------------------------
void lora_enable_invert_IQ(lora *const dev)
{
    lora_write_reg(dev, REG_INVERTIQ,  0x66);
    lora_write_reg(dev, REG_INVERTIQ2, 0x19);
}

//-----------------------------------------------------------------------------
void lora_disable_invert_IQ(lora *const dev)
{
    lora_write_reg(dev, REG_INVERTIQ,  0x27);
    lora_write_reg(dev, REG_INVERTIQ2, 0x1d);
}

//-----------------------------------------------------------------------------
void lora_set_ocp(lora *const dev, uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120)
    {
        ocpTrim = (mA - 45) / 5;
    }
    else if (mA <= 240)
    {
        ocpTrim = (mA + 30) / 10;
    }

    lora_write_reg(dev, REG_OCP, 0x20 | (0x1F & ocpTrim));
}

//-----------------------------------------------------------------------------
void lora_set_gain(lora *const dev, uint8_t gain)
{
    // check allowed range
    if (gain > 6)
    {
        gain = 6;
    }

    // set to standby
    lora_idle(dev);

    // set gain
    if (gain == 0)
    {
        // if gain = 0, enable AGC
        lora_write_reg(dev, REG_MODEM_CONFIG_3, 0x04);
    }
    else
    {
        // disable AGC
        lora_write_reg(dev, REG_MODEM_CONFIG_3, 0x00);

        // clear Gain and set LNA boost
        lora_write_reg(dev, REG_LNA, 0x03);

        // set gain
        lora_write_reg(dev, REG_LNA, lora_read_reg(dev, REG_LNA) | (gain << 5));
    }
}

//-----------------------------------------------------------------------------
uint8_t lora_random(lora *const dev)
{
    return lora_read_reg(dev, REG_RSSI_WIDEBAND);
}

//-----------------------------------------------------------------------------
void lora_explicit_header_mode(lora *const dev)
{
    dev->implicit_header_mode = 0;
    lora_write_reg(dev, REG_MODEM_CONFIG_1, lora_read_reg(dev, REG_MODEM_CONFIG_1) & 0xfe);
}

//-----------------------------------------------------------------------------
void lora_implicit_header_mode(lora *const dev)
{
    dev->implicit_header_mode = 1;
    lora_write_reg(dev, REG_MODEM_CONFIG_1, lora_read_reg(dev, REG_MODEM_CONFIG_1) | 0x01);
}

//-----------------------------------------------------------------------------
ISR_PREFIX void lora_handle_dio0_rise(lora *const dev)
{
    int irqFlags = lora_read_reg(dev, REG_IRQ_FLAGS);
    // Clear IRQ's
    lora_write_reg(dev, REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        if ((irqFlags & IRQ_RX_DONE_MASK) != 0)
        {
            // Received a packet
            dev->packet_index = 0;

            // read packet length
            int packetLength = dev->implicit_header_mode ? lora_read_reg(dev, REG_PAYLOAD_LENGTH) : lora_read_reg(dev, REG_RX_NB_BYTES);

            // set FIFO address to current RX address
            lora_write_reg(dev, REG_FIFO_ADDR_PTR, lora_read_reg(dev, REG_FIFO_RX_CURRENT_ADDR));

            if (dev->on_receive)
            {
                dev->on_receive(dev, packetLength);
            }
        }
    }
    else if ((irqFlags & IRQ_TX_DONE_MASK) != 0)
    {
        if (dev->on_tx_done)
        {
            dev->on_tx_done(dev);
        }
    }
}

//-----------------------------------------------------------------------------
void lora_write_reg(lora *const dev, uint8_t addr, uint8_t data)
{
    lora_write_buffer(dev, addr, &data, 1);
}

//-----------------------------------------------------------------------------
uint8_t lora_read_reg(lora *const dev, uint8_t addr)
{
    uint8_t data;
    lora_read_buffer(dev, addr, &data, 1);
    return data;
}

//-----------------------------------------------------------------------------
ISR_PREFIX void on_dio0_irq(void *ctx)
{
    lora *const dev = (lora*)ctx;
    // Disable interrupts and switch into idle mode
    lora_handle_dio0_rise(dev);
}
