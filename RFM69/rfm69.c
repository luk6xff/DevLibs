/**
 *  @brief:  Implementation of a RFM69 radio driver
 *           Supported devices: HopeRF RFM69W/RFM69HW/RFM69CW/RFM69HCW, Semtech SX1231/1231H
 *           Code inspired by LowPowerLab library: https://github.com/LowPowerLab/RFM69
 *  @author: luk6xff
 *  @email:  lukasz.uszko@gmail.com
 *  @date:   2019-11-15
 */
#include "registers.h"
#include "rfm69.h"
#include "common.h"

#include <string.h>
/**
 * =============================================================================
 * @brief Private global constants
 * =============================================================================
 */
//------------------------------------------------------------------------------

/**
 * =============================================================================
 * @brief Private functions prototypes
 * =============================================================================
 */
/**
 * @brief Send the full protocol based data frame to the receiver module
 *
 * @param[in] to_address         Receiver node address
 * @param[in] buffer             Data to be sent
 * @param[in] buffer_size        Length of data buffer to be sent
 * @param[in] request_ack        Ask for ACK from receiver or not
 * @param[in] send_ack           Message is an ACK one
 */
static void rfm69_send_frame(rfm69 *const dev, uint16_t to_address,
                             const void* buffer, uint8_t buffer_size,
                             bool request_ack, bool send_ack);

/**
 * @brief Interrupt handler - gets called when a packet is received
 */
static void rfm69_interrupt_handler(rfm69 *const dev);

/**
 * @brief Interrupt mapped to dio0 pin
 */
static void rfm69_isr0(void* ctx);

/**
 * @brief Clear FIFO and flags of RFM69 module.
 */
static void rfm69_clear_fifo(rfm69 *const dev);

/**
 * @brief Update value of dev->transmit_pwr_level_dbm.
 *
 * @param[in] increase - increment transmit_pwr_level_dbm value, decrement otherwise.
 */
static void rfm69_update_pwr_atc_level(rfm69 *const dev, bool increase);

/**
 * =============================================================================
 * @brief Private variables
 * =============================================================================
 */

/**
 * =============================================================================
 * @brief Public functions definitions
 * =============================================================================
 */
//------------------------------------------------------------------------------
bool rfm69_init(rfm69 *const dev)
{
    dev->dio0_irq = rfm69_isr0; // Register Hardware DIO IRQ callback for dio0
    dev->mode = RFM69_MODE_STANDBY;

    const uint8_t defualt_regs_config[][2] =
    {
        /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
        /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // Packet mode, FSK, no shaping
        /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // 55kbps
        /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555}, // 55kbps
        /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000},       // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
        /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

        /* 0x07 */ { REG_FRFMSB, (uint8_t) (dev->settings.freq_band==RFM69_315MHZ ? RF_FRFMSB_315 : (dev->settings.freq_band==RFM69_433MHZ ? RF_FRFMSB_433 : (dev->settings.freq_band==RFM69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
        /* 0x08 */ { REG_FRFMID, (uint8_t) (dev->settings.freq_band==RFM69_315MHZ ? RF_FRFMID_315 : (dev->settings.freq_band==RFM69_433MHZ ? RF_FRFMID_433 : (dev->settings.freq_band==RFM69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
        /* 0x09 */ { REG_FRFLSB, (uint8_t) (dev->settings.freq_band==RFM69_315MHZ ? RF_FRFLSB_315 : (dev->settings.freq_band==RFM69_433MHZ ? RF_FRFLSB_433 : (dev->settings.freq_band==RFM69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

        // looks like PA1 and PA2 are not implemented on RFM69W/CW, hence the max output power is 13dBm
        // +17dBm and +20dBm are possible on RFM69HW
        // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
        // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
        // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
        ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
        ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

        // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
        /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
        //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
        /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
        /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
        /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
        /* 0x29 */ { REG_RSSITHRESH, 200 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
        ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
        /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
        /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
        /* 0x30 */ { REG_SYNCVALUE2, dev->settings.network_id }, // NETWORK ID
        /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_WHITENING | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
        /* 0x38 */ { REG_PAYLOADLENGTH, RFM69_FIFO_SIZE }, // in variable length mode: the max frame size, not used in TX
        ///* 0x39 */ { REG_NODEADRS, node_id }, // turned off because we're not using address filtering
        /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
        /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
				  // {REG_AFCFEI, RF_AFCFEI_AFCAUTO_ON | RF_AFCFEI_AFCAUTOCLEAR_ON},
		//{REG_AFCBW, 0x8A},
        {255, 0}
    };

    // Initialize IO interfaces
    rfm69_io_init(dev);
    // Reset module
    rfm69_reset(dev);

    uint32_t start = rfm69_timer_read_ms();
    uint8_t timeout = 50;
    do
    {
        rfm69_write_reg(dev, REG_SYNCVALUE1, 0xAA);
    } while (rfm69_read_reg(dev, REG_SYNCVALUE1) != 0xaa && rfm69_timer_read_ms()-start < timeout);

    start = rfm69_timer_read_ms();
    do
    {
        rfm69_write_reg(dev, REG_SYNCVALUE1, 0x55);
    } while (rfm69_read_reg(dev, REG_SYNCVALUE1) != 0x55 && rfm69_timer_read_ms()-start < timeout);

    for (uint8_t i = 0; defualt_regs_config[i][0] != 255; i++)
    {
        rfm69_write_reg(dev, defualt_regs_config[i][0], defualt_regs_config[i][1]);
    }

    // Set PA and OCP settings according to RF module (normal/high power)
    rfm69_set_pa_settings(dev, false);

    // Encryption is persistent between resets and can trip you up during debugging.
    // Disable it during initialization so we always start from a known state.
    rfm69_aes_encryption(dev, 0, 0);

    // Set power level
    rfm69_set_power_level_dbm(dev, dev->settings.power_level_dbm);

    rfm69_set_mode(dev, RFM69_MODE_STANDBY);
    start = rfm69_timer_read_ms();
    while (((rfm69_read_reg(dev, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && rfm69_timer_read_ms()-start < timeout); // wait for ModeReady
    if (rfm69_timer_read_ms()-start >= timeout)
    {
        return false;
    }

    // Set interrupts
    rfm69_ioirq_init(dev);
    return true;
}

//------------------------------------------------------------------------------
void rfm69_register_on_receive(rfm69 *const dev, void(*callback)(void*))
{
    dev->on_receive = callback;
}

//------------------------------------------------------------------------------
uint32_t rfm69_get_frequency(rfm69 *const dev)
{
    return RFM69_FSTEP * (((uint32_t) rfm69_read_reg(dev, REG_FRFMSB) << 16) + ((uint16_t) rfm69_read_reg(dev, REG_FRFMID) << 8) + rfm69_read_reg(dev, REG_FRFLSB));
}

//------------------------------------------------------------------------------
void rfm69_set_frequency(rfm69 *const dev, uint32_t freq_hz)
{
    rfm69_device_mode last_mode = dev->mode;
    if (last_mode == RFM69_MODE_TX)
    {
        rfm69_set_mode(dev, RFM69_MODE_RX);
    }
    freq_hz /= RFM69_FSTEP; // divide down by FSTEP to get FRF
    rfm69_write_reg(dev, REG_FRFMSB, freq_hz >> 16);
    rfm69_write_reg(dev, REG_FRFMID, freq_hz >> 8);
    rfm69_write_reg(dev, REG_FRFLSB, freq_hz);
    if (last_mode == RFM69_MODE_RX)
    {
        rfm69_set_mode(dev, RFM69_MODE_SYNTH);
    }
    rfm69_set_mode(dev, last_mode);
}

//------------------------------------------------------------------------------
void rfm69_set_mode(rfm69 *const dev, rfm69_device_mode new_mode)
{
    if (new_mode == dev->mode)
    {
        return;
    }

    // Get new power level value
    const int8_t pwr_level = dev->pwr_atc.enabled ? dev->pwr_atc.transmit_pwr_level_dbm : dev->settings.power_level_dbm;

    switch (new_mode)
    {
        case RFM69_MODE_TX:
        {
            // Set power level
            rfm69_set_power_level_dbm(dev, pwr_level);
            // Send actual data here
            rfm69_write_reg(dev, REG_OPMODE, (rfm69_read_reg(dev, REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
            break;
        }
        case RFM69_MODE_RX:
        {
            rfm69_write_reg(dev, REG_OPMODE, (rfm69_read_reg(dev, REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
            rfm69_set_power_level_dbm(dev, pwr_level);
            break;
        }
        case RFM69_MODE_SYNTH:
        {
            rfm69_write_reg(dev, REG_OPMODE, (rfm69_read_reg(dev, REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        }
        case RFM69_MODE_STANDBY:
        {
            rfm69_write_reg(dev, REG_OPMODE, (rfm69_read_reg(dev, REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        }
        case RFM69_MODE_SLEEP:
        {
            rfm69_write_reg(dev, REG_OPMODE, (rfm69_read_reg(dev, REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        }
        default:
        {
            return;
        }
    }
    // We are using packet mode, so this check is not really needed
    // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
    while (dev->mode == RFM69_MODE_SLEEP && (rfm69_read_reg(dev, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

    dev->mode = new_mode;
}

//------------------------------------------------------------------------------
void rfm69_sleep(rfm69 *const dev)
{
    rfm69_set_mode(dev, RFM69_MODE_SLEEP);
}

//------------------------------------------------------------------------------
void rfm69_set_address(rfm69 *const dev, uint8_t addr)
{
    dev->settings.node_id = addr;
    rfm69_write_reg(dev, REG_NODEADRS, dev->settings.node_id);
}

//------------------------------------------------------------------------------
void rfm69_set_network(rfm69 *const dev, uint8_t network_id)
{
    dev->settings.network_id = network_id;
    rfm69_write_reg(dev, REG_SYNCVALUE2, dev->settings.network_id);
}

//------------------------------------------------------------------------------
bool rfm69_set_power_level_dbm(rfm69 *const dev, int8_t dBm)
{
    /**
     * Output power of module is from -18 dBm to +13 dBm
     * in "low" power devices (RFM69CW), -2 dBm to +20 dBm in high power devices (RFM69HW)
     */
    if (dBm < -18 || dBm > 20)
    {
        return false;
    }


    if (false == dev->settings.is_rfm69hw && dBm > 13)
    {
        return false;
    }

    if (true == dev->settings.is_rfm69hw && dBm < -2)
    {
        return false;
    }

    if (false == dev->settings.is_rfm69hw)
    {
        // only PA0 can be used
        dev->pwr_atc.transmit_pwr_level = dBm + 18;

        // enable PA0 only
        rfm69_write_reg(dev, REG_PALEVEL, 0x80 | dev->pwr_atc.transmit_pwr_level);
    }
    else
    {
        if (dBm >= -2 && dBm <= 13)
        {
            // use PA1 on pin PA_BOOST
        	dev->pwr_atc.transmit_pwr_level = dBm + 18;

            // enable PA1 only
            rfm69_write_reg(dev, REG_PALEVEL, 0x40 | dev->pwr_atc.transmit_pwr_level);

            // disable high power settings
            rfm69_set_high_power_regs(dev, false);
        }
        else if (dBm > 13 && dBm <= 17)
        {
            // use PA1 and PA2 combined on pin PA_BOOST
        	dev->pwr_atc.transmit_pwr_level = dBm + 14;

            // enable PA1+PA2
            rfm69_write_reg(dev, REG_PALEVEL, 0x60 | dev->pwr_atc.transmit_pwr_level);

            // disable high power settings
            rfm69_set_high_power_regs(dev, false);
        }
        else
        {
            // output power from 18 dBm to 20 dBm, use PA1+PA2 with high power settings
        	dev->pwr_atc.transmit_pwr_level= dBm + 11;

            // enable PA1+PA2
            rfm69_write_reg(dev, REG_PALEVEL, 0x60 | dev->pwr_atc.transmit_pwr_level);

            // enable high power settings
            rfm69_set_high_power_regs(dev, true);
        }
    }

    return true;
}

//------------------------------------------------------------------------------
void rfm69_set_bitrate(rfm69 *const dev, uint32_t bitrate)
{
    if (dev->mode == RFM69_MODE_RX || dev->mode == RFM69_MODE_TX) // if signal stronger than -100dBm is detected assume channel activity
    {
        rfm69_set_mode(dev, RFM69_MODE_STANDBY);
        if (dev->pwr_atc.enabled)
        {
            rfm69_set_power_level_dbm(dev, dev->pwr_atc.transmit_pwr_level_dbm);
        }
    }
    bitrate = RFM69_FXOSC / bitrate;
    rfm69_write_reg(dev, REG_BITRATEMSB, bitrate >> 8);
    rfm69_write_reg(dev, REG_BITRATELSB, bitrate);
}

//------------------------------------------------------------------------------
bool rfm69_can_send(rfm69 *const dev)
{
    if (dev->mode == RFM69_MODE_RX && dev->payload_data.payload_len == 0 && rfm69_read_rssi(dev, false) < RFM69_CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
    {
        rfm69_set_mode(dev, RFM69_MODE_STANDBY);
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
void rfm69_send(rfm69 *const dev, uint16_t to_address, const void* buffer, uint8_t buffer_size, bool request_ack)
{
    rfm69_write_reg(dev, REG_PACKETCONFIG2, (rfm69_read_reg(dev, REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    uint32_t now = rfm69_timer_read_ms();
    while (!rfm69_can_send(dev) && rfm69_timer_read_ms() - now < RFM69_CSMA_LIMIT_MS)
    {
        rfm69_receive_done(dev);
    }
    rfm69_send_frame(dev, to_address, buffer, buffer_size, request_ack, false);
}

//------------------------------------------------------------------------------
bool rfm69_send_with_retry(rfm69 *const dev, uint16_t to_address, const void* buffer, uint8_t buffer_size, uint8_t retries, uint32_t retry_wait_time_ms)
{
    uint32_t sent_time;
    for (uint8_t i = 0; i <= retries; i++)
    {
        rfm69_send(dev, to_address, buffer, buffer_size, true);
        sent_time = rfm69_timer_read_ms();
        while ((rfm69_timer_read_ms()-sent_time) < retry_wait_time_ms)
        {
            if (rfm69_ack_received(dev, to_address))
            {
                return true;
            }
        }

        // Sending failed, increase power transmit level if pwr_atc enabled
        rfm69_update_pwr_atc_level(dev, true);
    }
    return false;
}

//------------------------------------------------------------------------------
void rfm69_receive_begin(rfm69 *const dev)
{
    // Clear payload data
    memset(&dev->payload_data, 0, sizeof(dev->payload_data));

    if (rfm69_read_reg(dev, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    {
        rfm69_write_reg(dev, REG_PACKETCONFIG2, (rfm69_read_reg(dev, REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    }
    rfm69_write_reg(dev, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
    rfm69_set_mode(dev, RFM69_MODE_RX);
}

//------------------------------------------------------------------------------
bool rfm69_receive_done(rfm69 *const dev)
{
    if (dev->have_data)
    {
        dev->have_data = false;
        rfm69_interrupt_handler(dev);
    }
    // Previous frame still not handled
    if (dev->mode == RFM69_MODE_RX && dev->payload_data.payload_len > 0)
    {
        rfm69_set_mode(dev, RFM69_MODE_STANDBY); // disable interrupts
        return true;
    }
    else if (dev->mode == RFM69_MODE_RX) // already in RX no payload yet
    {
        return false;
    }
    rfm69_receive_begin(dev);
    return false;
}

//------------------------------------------------------------------------------
bool rfm69_ack_received(rfm69 *const dev, uint16_t from_node_id)
{
    if (rfm69_receive_done(dev))
    {
        return (dev->payload_data.sender_id == from_node_id || from_node_id == RFM69_BROADCAST_ADDR)
                && dev->payload_data.ack_received;
    }
    return false;
}

//------------------------------------------------------------------------------
bool rfm69_ack_requested(rfm69 *const dev)
{
    return dev->payload_data.ack_requested && (dev->payload_data.target_id != RFM69_BROADCAST_ADDR);
}

//------------------------------------------------------------------------------
void rfm69_send_ack(rfm69 *const dev)
{
    dev->payload_data.ack_requested = false;   // Added to make sure we don't end up in a timing race and infinite loop RFM69 sending Acks
    uint16_t sender_id = dev->payload_data.sender_id;
    int16_t rssi = dev->payload_data.rssi; // Save payload received RSSI value
    rfm69_write_reg(dev, REG_PACKETCONFIG2, (rfm69_read_reg(dev, REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // Avoid RX deadlocks
    uint32_t now = rfm69_timer_read_ms();
    while (!rfm69_can_send(dev) && rfm69_timer_read_ms() - now < RFM69_CSMA_LIMIT_MS)
    {
        rfm69_receive_done(dev);
    }
    dev->payload_data.sender_id = sender_id;    // Restore SenderID after it gets wiped out by rfm69_receive_done()
    // pwr_atc, send rssi on ack if requested
    if (dev->pwr_atc.ack_rssi_requested)
    {
        rfm69_send_frame(dev, sender_id, &rssi, sizeof(rssi), dev->payload_data.ack_requested, true);
    }
    else
    {
        rfm69_send_frame(dev, sender_id, NULL, 0, dev->payload_data.ack_requested, true);
    }

    dev->payload_data.rssi = rssi; // Restore payload RSSI
}

//------------------------------------------------------------------------------
const rfm69_payload_data* rfm69_received_data(rfm69 *const dev)
{
    return &(dev->payload_data);
}

//------------------------------------------------------------------------------
void rfm69_aes_encryption(rfm69 *const dev, const char* key, uint8_t key_length)
{
    rfm69_set_mode(dev, RFM69_MODE_STANDBY);
    if ((key != 0) && (key_length == 16))
    {
        rfm69_write_buffer(dev, REG_AESKEY1, (const uint8_t*)key, key_length);
        rfm69_write_reg(dev, REG_PACKETCONFIG2, (rfm69_read_reg(dev, REG_PACKETCONFIG2) & 0xFE) | RF_PACKET2_AES_ON);
    }
    else
    {
        rfm69_write_reg(dev, REG_PACKETCONFIG2, (rfm69_read_reg(dev, REG_PACKETCONFIG2) & 0xFE) | RF_PACKET2_AES_OFF);
    }
}

//------------------------------------------------------------------------------
int16_t rfm69_read_rssi(rfm69 *const dev, bool force_trigger)
{
    int16_t rssi = 0;

    if (force_trigger)
    {
        // RSSI trigger not needed if DAGC is in continuous mode
        rfm69_write_reg(dev, REG_RSSICONFIG, RF_RSSI_START);
        while ((rfm69_read_reg(dev, REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
    }
    rssi = -rfm69_read_reg(dev, REG_RSSIVALUE);
    rssi >>= 1; // divide by 2

    return rssi;
}


//------------------------------------------------------------------------------
void rfm69_set_promiscuous_mode(rfm69 *const dev, bool enable)
{
    dev->settings.promiscuous_mode = enable;
    //rfm69_write_reg(dev, REG_PACKETCONFIG1, (rfm69_read_reg(REG_PACKETCONFIG1) & 0xF9) | (enable ? RF_PACKET1_ADRSFILTERING_OFF : RF_PACKET1_ADRSFILTERING_NODEBROADCAST));
}

//------------------------------------------------------------------------------
void rfm69_set_pa_settings(rfm69 *const dev, uint8_t forcePA)
{
    // Disable OCP for high power devices, enable otherwise
    rfm69_write_reg(dev, REG_OCP, 0x0A | (dev->settings.is_rfm69hw ? 0x00 : 0x10));

    if (0 == forcePA)
    {
        if (true == dev->settings.is_rfm69hw)
        {
            // enable PA1 only
            rfm69_write_reg(dev, REG_PALEVEL, (rfm69_read_reg(dev, REG_PALEVEL) & 0x1F) | 0x40);
        }
        else
        {
            // enable PA0 only
            rfm69_write_reg(dev, REG_PALEVEL, (rfm69_read_reg(dev, REG_PALEVEL) & 0x1F) | 0x80);
        }
    }
    else
    {
        // PA settings forced
        uint8_t pa = 0;

        if (forcePA & 0x01)
        {
            pa |= 0x80;
        }

        if (forcePA & 0x02)
        {
            pa |= 0x40;
        }

        if (forcePA & 0x04)
        {
            pa |= 0x20;
        }

        // check if high power settings are forced
        bool high_pwr_settings = (forcePA & 0x08) ? true : false;
        rfm69_set_high_power_regs(dev, high_pwr_settings);
        rfm69_write_reg(dev, REG_PALEVEL, (rfm69_read_reg(dev, REG_PALEVEL) & 0x1F) | pa);
    }
}



//------------------------------------------------------------------------------
void rfm69_set_high_power_regs(rfm69 *const dev, bool enable)
{
    // Enabling only works if this is a high power device
    if (true == enable && false == dev->settings.is_rfm69hw)
    {
        enable = false;
    }

    rfm69_write_reg(dev, REG_TESTPA1, enable ? 0x5D : 0x55);
    rfm69_write_reg(dev, REG_TESTPA2, enable ? 0x7C : 0x70);
}

//------------------------------------------------------------------------------
uint8_t rfm69_read_temperature(rfm69 *const dev, uint8_t cal_factor)
{
    rfm69_set_mode(dev, RFM69_MODE_STANDBY);
    rfm69_write_reg(dev, REG_TEMP1, RF_TEMP1_MEAS_START);
    while ((rfm69_read_reg(dev, REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
    // RFM69_COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction
    // 'complement' corrects the slope, rising temp = rising val
    return ~rfm69_read_reg(dev, REG_TEMP2) + RFM69_COURSE_TEMP_COEF + cal_factor;
}

//------------------------------------------------------------------------------
void rfm69_rc_calibration(rfm69 *const dev)
{
    rfm69_write_reg(dev, REG_OSC1, RF_OSC1_RCCAL_START);
    while ((rfm69_read_reg(dev, REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

//------------------------------------------------------------------------------
uint8_t rfm69_read_reg(rfm69 *const dev, uint8_t addr)
{
    uint8_t data;
    rfm69_read_buffer(dev, addr, &data, 1);
    return data;
}

//------------------------------------------------------------------------------
void rfm69_write_reg(rfm69 *const dev, uint8_t addr, uint8_t data)
{
    rfm69_write_buffer(dev, addr, &data, 1);
}

//------------------------------------------------------------------------------


/**
 * ============================================================================
 * @brief Private functions definitions
 * ============================================================================
 */
//------------------------------------------------------------------------------
void rfm69_send_frame(rfm69 *const dev, uint16_t to_address, const void* buffer,
					  uint8_t buffer_size, bool request_ack, bool rfm69_send_ack)
{
    rfm69_set_mode(dev, RFM69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
    while ((rfm69_read_reg(dev, REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
    //rfm69_write_reg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
    if (buffer_size > RFM69_MAX_DATA_LEN)
    {
        buffer_size = RFM69_MAX_DATA_LEN;
    }

    // Control byte
    uint8_t ctrl_byte = 0x00;
    if (rfm69_send_ack)
    {
        ctrl_byte = RFM69_CTL_SENDACK;
    }
    else if (request_ack)
    {
        ctrl_byte = RFM69_CTL_REQACK;
    }

    // Add ack_rssi request flag
    if (dev->pwr_atc.enabled)
    {
        ctrl_byte |= RFM69_CTL_REQACK_RSSI;
    }

    if (to_address > 0xFF)
    {
        ctrl_byte |= (to_address & 0x300) >> 6; //assign last 2 bits of address if > 255
    }

    if (dev->settings.node_id > 0xFF)
    {
        ctrl_byte |= (dev->settings.node_id & 0x300) >> 8;   //assign last 2 bits of address if > 255
    }

    // write frame to to FIFO
    uint8_t frame[RFM69_FIFO_SIZE];
    frame[0] = buffer_size + 3;  // LU_TODO
    frame[1] = (uint8_t)to_address;
    frame[2] = (uint8_t)dev->settings.node_id;
    frame[3] = ctrl_byte;
    memcpy(&frame[4], buffer, buffer_size);
    rfm69_write_buffer(dev, REG_FIFO, frame, sizeof(frame));
    // no need to wait for transmit mode to be ready since its handled by the radio
    rfm69_set_mode(dev, RFM69_MODE_TX);

    while ((rfm69_read_reg(dev, REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00); // wait for packet sent

    rfm69_set_mode(dev, RFM69_MODE_STANDBY);
}

//------------------------------------------------------------------------------
void rfm69_interrupt_handler(rfm69 *const dev)
{
    if (dev->mode == RFM69_MODE_RX && (rfm69_read_reg(dev, REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
    {
        rfm69_set_mode(dev, RFM69_MODE_STANDBY);
        uint8_t buffer[RFM69_FIFO_SIZE];
        rfm69_read_buffer(dev, REG_FIFO, buffer, RFM69_HEADER_LEN);
        dev->payload_data.payload_len = buffer[0];
        dev->payload_data.payload_len = dev->payload_data.payload_len > RFM69_FIFO_SIZE ? RFM69_FIFO_SIZE : dev->payload_data.payload_len; // precaution
        dev->payload_data.target_id = buffer[1];
        dev->payload_data.sender_id = buffer[2];
        uint8_t ctrl_byte = buffer[3];

        // 10 bit address (most significant 2 bits stored in bits(2,3) of CTL byte
        dev->payload_data.target_id |= ((uint16_t)ctrl_byte & 0x0C) << 6;
        // 10 bit address (most significant 2 bits stored in bits(0,1) of CTL byte
        dev->payload_data.sender_id |= ((uint16_t)ctrl_byte & 0x03) << 8;
        //printf("dev->payload_data.payload_len: %x\n\r", dev->payload_data.payload_len);
        //printf("dev->payload_data.target_id: %x\n\r", dev->payload_data.target_id );
        //printf("dev->payload_data.sender_id: %x\n\r", dev->payload_data.sender_id);

        // match this node's address, or broadcast address or anything in promiscuous mode
        if (!(dev->settings.promiscuous_mode ||
                dev->payload_data.target_id == dev->settings.node_id ||
                dev->payload_data.target_id == RFM69_BROADCAST_ADDR)
            || dev->payload_data.payload_len < 3) // address situation could receive packets that are malformed and don't fit this libraries extra fields
        {
            dev->payload_data.payload_len = 0;
            rfm69_receive_begin(dev);
            return;
        }

        dev->payload_data.data_len = dev->payload_data.payload_len - 3; // LU_TODO RFM69_HEADER_LEN;
        dev->payload_data.ack_received = ctrl_byte & RFM69_CTL_SENDACK; // extract ACK-received flag 1
        dev->payload_data.ack_requested = ctrl_byte & RFM69_CTL_REQACK; // extract ACK-requested flag 0
        dev->pwr_atc.ack_rssi_requested = ctrl_byte & RFM69_CTL_REQACK_RSSI; // extract ack_rssi_requested status 0 ATC_ENABLED = false ON GATEWY

        // If message is ACK and Power_ATC enabled, extract ACK_RSSI
        if (dev->payload_data.ack_received && dev->pwr_atc.enabled)
        {
            rfm69_read_buffer(dev, REG_FIFO, (uint8_t*)&(dev->pwr_atc.ack_rssi), sizeof(dev->pwr_atc.ack_rssi)); // Read ACK_RSSI

            // Apply radio power level autocontrol algorithm
            if (dev->pwr_atc.ack_rssi < (dev->pwr_atc.target_rssi-RFM69_PWR_ATC_RSSI_HIST))
            {
                rfm69_update_pwr_atc_level(dev, true);
            }
            else if (dev->pwr_atc.ack_rssi > (dev->pwr_atc.target_rssi+RFM69_PWR_ATC_RSSI_HIST))
            {
                rfm69_update_pwr_atc_level(dev, false);
            }
        }
        else
        {
            rfm69_read_buffer(dev, REG_FIFO, dev->payload_data.data, dev->payload_data.data_len); // Read data
            dev->payload_data.data[dev->payload_data.data_len] = 0; // Add null at end of string
        }
        // Switch to RX mode
        rfm69_set_mode(dev, RFM69_MODE_RX);
    }

    // Read current RSSI
    dev->payload_data.rssi = rfm69_read_rssi(dev, false);
}

//------------------------------------------------------------------------------
// internal function
ISR_PREFIX void rfm69_isr0(void* ctx)
{
    rfm69 *const dev = (rfm69*)ctx;
    dev->have_data = true;
    // if (dev->on_receive)
    // {
    //     dev->on_receive(dev);
    // }
}

//------------------------------------------------------------------------------
void rfm69_clear_fifo(rfm69 *const dev)
{
    rfm69_write_reg(dev, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
}

//------------------------------------------------------------------------------
void rfm69_update_pwr_atc_level(rfm69 *const dev, bool increase)
{
    const int8_t pwr_min_level = dev->settings.is_rfm69hw ? RFM69HW_MIN_POWER_LEVEL_DBM : RFM69W_MIN_POWER_LEVEL_DBM;
    const int8_t pwr_max_level = dev->settings.is_rfm69hw ? RFM69HW_MAX_POWER_LEVEL_DBM : RFM69W_MAX_POWER_LEVEL_DBM;
    if (dev->pwr_atc.enabled)
    {
        if (increase)
        {
            dev->pwr_atc.transmit_pwr_level_dbm++;
            //printf("pwr_atc+++ %d\n\r", dev->pwr_atc.transmit_pwr_level_dbm);
            if (dev->pwr_atc.transmit_pwr_level_dbm > pwr_max_level)
            {
                dev->pwr_atc.transmit_pwr_level_dbm = pwr_max_level;
            }
        }
        else // increase == false
        {
            dev->pwr_atc.transmit_pwr_level_dbm--;
            //printf("pwr_atc--- %d\n\r", dev->pwr_atc.transmit_pwr_level_dbm);
            if (dev->pwr_atc.transmit_pwr_level_dbm < pwr_min_level)
            {
                dev->pwr_atc.transmit_pwr_level_dbm = pwr_min_level;
            }
        }
    }
}

//------------------------------------------------------------------------------
