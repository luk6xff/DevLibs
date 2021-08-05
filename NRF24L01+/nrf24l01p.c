#include "nrf24l01p.h"
#include <stdio.h>


/*
 * The following FIFOs are present in nRF24L01+:
 *   TX three level, 32 byte FIFO
 *   RX three level, 32 byte FIFO
 */
#define _NRF24L01P_TX_FIFO_COUNT   3
#define _NRF24L01P_RX_FIFO_COUNT   3

#define _NRF24L01P_TX_FIFO_SIZE   32
#define _NRF24L01P_RX_FIFO_SIZE   32

#define _NRF24L01P_SPI_MAX_DATA_RATE     10000000

// COMMANDS
#define _NRF24L01P_SPI_CMD_RD_REG            0x00
#define _NRF24L01P_SPI_CMD_WR_REG            0x20
#define _NRF24L01P_SPI_CMD_RD_RX_PAYLOAD     0x61
#define _NRF24L01P_SPI_CMD_WR_TX_PAYLOAD     0xa0
#define _NRF24L01P_SPI_CMD_FLUSH_TX          0xe1
#define _NRF24L01P_SPI_CMD_FLUSH_RX          0xe2
#define _NRF24L01P_SPI_CMD_REUSE_TX_PL       0xe3
#define _NRF24L01P_SPI_CMD_R_RX_PL_WID       0x60
#define _NRF24L01P_SPI_CMD_W_ACK_PAYLOAD     0xa8
#define _NRF24L01P_SPI_CMD_W_TX_PYLD_NO_ACK  0xb0
#define _NRF24L01P_SPI_CMD_NOP               0xff

// REGISTERS
#define _NRF24L01P_REG_CONFIG                0x00
#define _NRF24L01P_REG_EN_AA                 0x01
#define _NRF24L01P_REG_EN_RXADDR             0x02
#define _NRF24L01P_REG_SETUP_AW              0x03
#define _NRF24L01P_REG_SETUP_RETR            0x04
#define _NRF24L01P_REG_RF_CH                 0x05
#define _NRF24L01P_REG_RF_SETUP              0x06
#define _NRF24L01P_REG_STATUS                0x07
#define _NRF24L01P_REG_OBSERVE_TX            0x08
#define _NRF24L01P_REG_RPD                   0x09
#define _NRF24L01P_REG_RX_ADDR_P0            0x0a
#define _NRF24L01P_REG_RX_ADDR_P1            0x0b
#define _NRF24L01P_REG_RX_ADDR_P2            0x0c
#define _NRF24L01P_REG_RX_ADDR_P3            0x0d
#define _NRF24L01P_REG_RX_ADDR_P4            0x0e
#define _NRF24L01P_REG_RX_ADDR_P5            0x0f
#define _NRF24L01P_REG_TX_ADDR               0x10
#define _NRF24L01P_REG_RX_PW_P0              0x11
#define _NRF24L01P_REG_RX_PW_P1              0x12
#define _NRF24L01P_REG_RX_PW_P2              0x13
#define _NRF24L01P_REG_RX_PW_P3              0x14
#define _NRF24L01P_REG_RX_PW_P4              0x15
#define _NRF24L01P_REG_RX_PW_P5              0x16
#define _NRF24L01P_REG_FIFO_STATUS           0x17
#define _NRF24L01P_REG_DYNPD                 0x1c
#define _NRF24L01P_REG_FEATURE               0x1d

#define _NRF24L01P_REG_addr_MASK          0x1f

// CONFIG register:
#define _NRF24L01P_CONFIG_PRIM_RX        (1<<0)
#define _NRF24L01P_CONFIG_PWR_UP         (1<<1)
#define _NRF24L01P_CONFIG_CRC0           (1<<2)
#define _NRF24L01P_CONFIG_EN_CRC         (1<<3)
#define _NRF24L01P_CONFIG_MASK_MAX_RT    (1<<4)
#define _NRF24L01P_CONFIG_MASK_TX_DS     (1<<5)
#define _NRF24L01P_CONFIG_MASK_RX_DR     (1<<6)

#define _NRF24L01P_CONFIG_CRC_MASK       (_NRF24L01P_CONFIG_EN_CRC|_NRF24L01P_CONFIG_CRC0)
#define _NRF24L01P_CONFIG_CRC_NONE       (0)
#define _NRF24L01P_CONFIG_CRC_8BIT       (_NRF24L01P_CONFIG_EN_CRC)
#define _NRF24L01P_CONFIG_CRC_16BIT      (_NRF24L01P_CONFIG_EN_CRC|_NRF24L01P_CONFIG_CRC0)

// EN_AA register:
#define _NRF24L01P_EN_AA_NONE            0

// EN_RXADDR register:
#define _NRF24L01P_EN_RXADDR_NONE        0

// SETUP_AW register:
#define _NRF24L01P_SETUP_AW_AW_MASK      (0x3<<0)
#define _NRF24L01P_SETUP_AW_AW_3BYTE     (0x1<<0)
#define _NRF24L01P_SETUP_AW_AW_4BYTE     (0x2<<0)
#define _NRF24L01P_SETUP_AW_AW_5BYTE     (0x3<<0)

// SETUP_RETR register:
#define _NRF24L01P_SETUP_RETR_NONE       0

// RF_SETUP register:
#define _NRF24L01P_RF_SETUP_RF_PWR_MASK          (0x3<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_0DBM          (0x3<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM    (0x2<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM   (0x1<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM   (0x0<<1)

#define _NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT       (1 << 3)
#define _NRF24L01P_RF_SETUP_RF_DR_LOW_BIT        (1 << 5)
#define _NRF24L01P_RF_SETUP_RF_DR_MASK           (_NRF24L01P_RF_SETUP_RF_DR_LOW_BIT|_NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)
#define _NRF24L01P_RF_SETUP_RF_DR_250KBPS        (_NRF24L01P_RF_SETUP_RF_DR_LOW_BIT)
#define _NRF24L01P_RF_SETUP_RF_DR_1MBPS          (0)
#define _NRF24L01P_RF_SETUP_RF_DR_2MBPS          (_NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)

// STATUS register:
#define _NRF24L01P_STATUS_TX_FULL        (1<<0)
#define _NRF24L01P_STATUS_RX_P_NO        (0x7<<1)
#define _NRF24L01P_STATUS_MAX_RT         (1<<4)
#define _NRF24L01P_STATUS_TX_DS          (1<<5)
#define _NRF24L01P_STATUS_RX_DR          (1<<6)

// RX_PW_P0..RX_PW_P5 registers:
#define _NRF24L01P_RX_PW_Px_MASK         0x3F

#define _NRF24L01P_TIMING_Tundef2pd_us        100   // 100ms
#define _NRF24L01P_TIMING_Tstby2a_us            1   //   1ms
#define _NRF24L01P_TIMING_Thce_us               1   //   1ms
#define _NRF24L01P_TIMING_Tpd2stby_us           5   //   5ms worst case
#define _NRF24L01P_TIMING_Tpece2csn_us          1   //   1ms


/**
 * Functions
 */
//------------------------------------------------------------------------------
void nrf24l01p_init(nrf24l01p * const dev)
{
    dev->mode = _NRF24L01P_MODE_UNKNOWN;

    nrf24l01p_disable(dev);

    nrf24l01p_set_csn_pin(dev, true);

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_CONFIG, 0); // Power Down
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_MAX_RT|_NRF24L01P_STATUS_TX_DS|_NRF24L01P_STATUS_RX_DR);   // Clear any pending interrupts

    // Setup default configuration
    nrf24l01p_disable_all_rx_pipes(dev);
    nrf24l01p_set_rf_freq(dev, DEFAULT_NRF24L01P_RF_FREQUENCY);
    nrf24l01p_set_rf_tx_power(dev, DEFAULT_NRF24L01P_TX_PWR);
    nrf24l01p_set_data_rate(dev, DEFAULT_NRF24L01P_DATARATE);
    nrf24l01p_set_crc_length(dev, DEFAULT_NRF24L01P_CRC);
    nrf24l01p_set_tx_addr(dev, DEFAULT_NRF24L01P_ADDRESS, DEFAULT_NRF24L01P_ADDRESS_WIDTH);
    nrf24l01p_set_rx_addr(dev, DEFAULT_NRF24L01P_ADDRESS, DEFAULT_NRF24L01P_ADDRESS_WIDTH, NRF24L01P_PIPE_P0);
    nrf24l01p_disable_auto_ack(dev);
    nrf24l01p_disable_auto_retransmit(dev);
    nrf24l01p_set_transfer_size(dev, DEFAULT_NRF24L01P_TRANSFER_SIZE, NRF24L01P_PIPE_P0);

    dev->mode = _NRF24L01P_MODE_POWER_DOWN;

}

//------------------------------------------------------------------------------
bool nrf24l01p_is_connected(nrf24l01p * const dev)
{
	uint8_t setup = nrf24l01p_read_reg(dev, _NRF24L01P_REG_SETUP_AW);
	if(setup >= 1 && setup <= 3)
	{
		return true;
	}
	return false;
}

//------------------------------------------------------------------------------
void nrf24l01p_power_up(nrf24l01p * const dev)
{

    uint8_t config = nrf24l01p_read_reg(dev, _NRF24L01P_REG_CONFIG);

    config |= _NRF24L01P_CONFIG_PWR_UP;

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_CONFIG, config);

    // Wait until the nRF24L01+ powers up
    nrf24l01p_delay_ms(_NRF24L01P_TIMING_Tpd2stby_us);

    dev->mode = _NRF24L01P_MODE_STANDBY;
}

//------------------------------------------------------------------------------
void nrf24l01p_power_down(nrf24l01p * const dev)
{

    uint8_t config = nrf24l01p_read_reg(dev, _NRF24L01P_REG_CONFIG);

    config &= ~_NRF24L01P_CONFIG_PWR_UP;

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_CONFIG, config);

    // Wait until the nRF24L01+ powers down
    nrf24l01p_delay_ms(_NRF24L01P_TIMING_Tpd2stby_us);
    dev->mode = _NRF24L01P_MODE_POWER_DOWN;
}

//------------------------------------------------------------------------------
void nrf24l01p_set_rx_mode(nrf24l01p * const dev)
{
    if (_NRF24L01P_MODE_POWER_DOWN == dev->mode)
    {
        nrf24l01p_power_up(dev);
    }

    uint8_t config = nrf24l01p_read_reg(dev, _NRF24L01P_REG_CONFIG);

    config |= _NRF24L01P_CONFIG_PRIM_RX;

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_CONFIG, config);

    dev->mode = _NRF24L01P_MODE_RX;
}

//------------------------------------------------------------------------------
void nrf24l01p_set_tx_mode(nrf24l01p * const dev)
{
    if (_NRF24L01P_MODE_POWER_DOWN == dev->mode)
    {
        nrf24l01p_power_up(dev);
    }

    uint8_t config = nrf24l01p_read_reg(dev, _NRF24L01P_REG_CONFIG);

    config &= ~_NRF24L01P_CONFIG_PRIM_RX;

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_CONFIG, config);

    dev->mode = _NRF24L01P_MODE_TX;
}

//------------------------------------------------------------------------------
void nrf24l01p_enable(nrf24l01p * const dev)
{
    nrf24l01p_set_ce_pin(dev, true);
    nrf24l01p_delay_ms(_NRF24L01P_TIMING_Tpece2csn_us);
}

//------------------------------------------------------------------------------
void nrf24l01p_disable(nrf24l01p * const dev)
{
    nrf24l01p_set_ce_pin(dev, false);
}

//------------------------------------------------------------------------------
bool nrf24l01p_set_rf_freq(nrf24l01p * const dev, int frequency)
{
    if ((frequency < NRF24L01P_MIN_RF_FREQUENCY) || (frequency > NRF24L01P_MAX_RF_FREQUENCY))
    {
        // printf("nRF24L01P: Invalid RF Frequency setting %d\r\n", frequency);
        return false;
    }

    const int channel = (frequency - NRF24L01P_MIN_RF_FREQUENCY) & 0x7F;

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_RF_CH, channel);
    return true;
}

//------------------------------------------------------------------------------
int nrf24l01p_get_rf_freq(nrf24l01p * const dev)
{
    int channel = nrf24l01p_read_reg(dev, _NRF24L01P_REG_RF_CH) & 0x7F;
    return (channel + NRF24L01P_MIN_RF_FREQUENCY);

}

//------------------------------------------------------------------------------
void nrf24l01p_set_rf_tx_power(nrf24l01p * const dev, int power) {

    int rf_setup = nrf24l01p_read_reg(dev, _NRF24L01P_REG_RF_SETUP) & ~_NRF24L01P_RF_SETUP_RF_PWR_MASK;

    switch (power)
    {

        case NRF24L01P_TX_PWR_ZERO_DB:
            rf_setup |= _NRF24L01P_RF_SETUP_RF_PWR_0DBM;
            break;

        case NRF24L01P_TX_PWR_MINUS_6_DB:
            rf_setup |= _NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM;
            break;

        case NRF24L01P_TX_PWR_MINUS_12_DB:
            rf_setup |= _NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM;
            break;

        case NRF24L01P_TX_PWR_MINUS_18_DB:
            rf_setup |= _NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM;
            break;

        default:
            //printf("nRF24L01P: Invalid RF Output Power setting %d\r\n", power);
            return;
    }
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_RF_SETUP, rf_setup);
}

//------------------------------------------------------------------------------
int nrf24l01p_get_rf_tx_power(nrf24l01p * const dev)
{
    int pwr = nrf24l01p_read_reg(dev, _NRF24L01P_REG_RF_SETUP) & _NRF24L01P_RF_SETUP_RF_PWR_MASK;

    switch (pwr)
    {
        case _NRF24L01P_RF_SETUP_RF_PWR_0DBM:
            return NRF24L01P_TX_PWR_ZERO_DB;

        case _NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM:
            return NRF24L01P_TX_PWR_MINUS_6_DB;

        case _NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM:
            return NRF24L01P_TX_PWR_MINUS_12_DB;

        case _NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM:
            return NRF24L01P_TX_PWR_MINUS_18_DB;

        default:
            //printf("nRF24L01P: Unknown RF Output Power value %d\r\n", pwr);
            return 0;
    }
}

//------------------------------------------------------------------------------
void nrf24l01p_set_data_rate(nrf24l01p * const dev, int rate)
{
    int rf_setup = nrf24l01p_read_reg(dev, _NRF24L01P_REG_RF_SETUP) & ~_NRF24L01P_RF_SETUP_RF_DR_MASK;

    switch (rate)
    {
        case NRF24L01P_DATARATE_250_KBPS:
            rf_setup |= _NRF24L01P_RF_SETUP_RF_DR_250KBPS;
            break;

        case NRF24L01P_DATARATE_1_MBPS:
            rf_setup |= _NRF24L01P_RF_SETUP_RF_DR_1MBPS;
            break;

        case NRF24L01P_DATARATE_2_MBPS:
            rf_setup |= _NRF24L01P_RF_SETUP_RF_DR_2MBPS;
            break;

        default:
            //printf("nRF24L01P: Invalid Air Data Rate setting %d\r\n", rate);
            return;

    }

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_RF_SETUP, rf_setup);
}

//------------------------------------------------------------------------------
int nrf24l01p_get_data_rate(nrf24l01p * const dev)
{
    int rf_data_rate = nrf24l01p_read_reg(dev, _NRF24L01P_REG_RF_SETUP) & _NRF24L01P_RF_SETUP_RF_DR_MASK;

    switch (rf_data_rate)
    {
        case _NRF24L01P_RF_SETUP_RF_DR_250KBPS:
            return NRF24L01P_DATARATE_250_KBPS;

        case _NRF24L01P_RF_SETUP_RF_DR_1MBPS:
            return NRF24L01P_DATARATE_1_MBPS;

        case _NRF24L01P_RF_SETUP_RF_DR_2MBPS:
            return NRF24L01P_DATARATE_2_MBPS;

        default:
            //printf("nRF24L01P: Unknown Air Data Rate value %d\r\n", rf_data_rate);
            return 0;
    }
}

//------------------------------------------------------------------------------
void nrf24l01p_set_crc_length(nrf24l01p * const dev, int length)
{
    uint8_t config = nrf24l01p_read_reg(dev, _NRF24L01P_REG_CONFIG) & ~_NRF24L01P_CONFIG_CRC_MASK;

    switch (length)
    {
        case NRF24L01P_CRC_NONE:
            config |= _NRF24L01P_CONFIG_CRC_NONE;
            break;

        case NRF24L01P_CRC_8_BIT:
            config |= _NRF24L01P_CONFIG_CRC_8BIT;
            break;

        case NRF24L01P_CRC_16_BIT:
            config |= _NRF24L01P_CONFIG_CRC_16BIT;
            break;

        default:
            //printf("nRF24L01P: Invalid CRC Width setting %d\r\n", length);
            return;
    }
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_CONFIG, config);
}

//------------------------------------------------------------------------------
int nrf24l01p_get_crc_length(nrf24l01p * const dev)
{
    int crc_width = nrf24l01p_read_reg(dev, _NRF24L01P_REG_CONFIG) & _NRF24L01P_CONFIG_CRC_MASK;

    switch (crc_width)
    {
        case _NRF24L01P_CONFIG_CRC_NONE:
            return NRF24L01P_CRC_NONE;

        case _NRF24L01P_CONFIG_CRC_8BIT:
            return NRF24L01P_CRC_8_BIT;

        case _NRF24L01P_CONFIG_CRC_16BIT:
            return NRF24L01P_CRC_16_BIT;

        default:
            //printf("nRF24L01P: Unknown CRC Width value %d\r\n", crc_width);
            return 0;
    }
}

//------------------------------------------------------------------------------
void nrf24l01p_set_transfer_size(nrf24l01p * const dev, uint8_t size, uint8_t pipe_num)
{
    if ((pipe_num < NRF24L01P_PIPE_P0) || (pipe_num > NRF24L01P_PIPE_P5))
    {
        //printf("nRF24L01P: Invalid Transfer Size pipe_num number %d\r\n", pipe_num);
        return;
    }

    if ((size < 0) || (size > _NRF24L01P_RX_FIFO_SIZE))
    {
        //printf("nRF24L01P: Invalid Transfer Size setting %d\r\n", size);
        return;
    }

    int rx_pwr_px_register = _NRF24L01P_REG_RX_PW_P0 + (pipe_num - NRF24L01P_PIPE_P0);

    nrf24l01p_write_reg(dev, rx_pwr_px_register, (size & _NRF24L01P_RX_PW_Px_MASK));
}

//------------------------------------------------------------------------------
int nrf24l01p_get_transfer_size(nrf24l01p * const dev, uint8_t pipe_num)
{

    if ((pipe_num < NRF24L01P_PIPE_P0) || (pipe_num > NRF24L01P_PIPE_P5))
    {
        //printf("nRF24L01P: Invalid Transfer Size pipe_num number %d\r\n", pipe_num);
        return 0;
    }

    int rx_pwr_px_register = _NRF24L01P_REG_RX_PW_P0 + (pipe_num - NRF24L01P_PIPE_P0);

    uint8_t size = nrf24l01p_read_reg(dev, rx_pwr_px_register);

    return (size & _NRF24L01P_RX_PW_Px_MASK);

}

//------------------------------------------------------------------------------
void nrf24l01p_disable_all_rx_pipes(nrf24l01p * const dev)
{
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_EN_RXADDR, _NRF24L01P_EN_RXADDR_NONE);
}

//------------------------------------------------------------------------------
void nrf24l01p_disable_auto_ack(nrf24l01p * const dev)
{
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_EN_AA, _NRF24L01P_EN_AA_NONE);
}

//------------------------------------------------------------------------------
void nrf24l01p_enable_auto_ack(nrf24l01p * const dev, uint8_t pipe_num)
{

    if ((pipe_num < NRF24L01P_PIPE_P0) || (pipe_num > NRF24L01P_PIPE_P5))
    {
        //printf("nRF24L01P: Invalid Enable AutoAcknowledge pipe_num number %d\r\n", pipe_num);
        return;
    }

    int en_aa = nrf24l01p_read_reg(dev, _NRF24L01P_REG_EN_AA);

    en_aa |= (1 << (pipe_num - NRF24L01P_PIPE_P0));

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_EN_AA, en_aa);
}

//------------------------------------------------------------------------------
void nrf24l01p_disable_auto_retransmit(nrf24l01p * const dev)
{
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_SETUP_RETR, _NRF24L01P_SETUP_RETR_NONE);
}

//------------------------------------------------------------------------------
void nrf24l01p_set_rx_addr(nrf24l01p * const dev, uint64_t addr, int length, uint8_t pipe_num)
{

    if ((pipe_num < NRF24L01P_PIPE_P0) || (pipe_num > NRF24L01P_PIPE_P5))
    {
        //printf("nRF24L01P: Invalid nrf24l01p_set_rx_addr pipe_num number %d\r\n", pipe_num);
        return;
    }

    if ((pipe_num == NRF24L01P_PIPE_P0) || (pipe_num == NRF24L01P_PIPE_P1))
    {
        int setup_aw = nrf24l01p_read_reg(dev, _NRF24L01P_REG_SETUP_AW) & ~_NRF24L01P_SETUP_AW_AW_MASK;
        switch (length)
        {
            case 3:
                setup_aw |= _NRF24L01P_SETUP_AW_AW_3BYTE;
                break;

            case 4:
                setup_aw |= _NRF24L01P_SETUP_AW_AW_4BYTE;
                break;

            case 5:
                setup_aw |= _NRF24L01P_SETUP_AW_AW_5BYTE;
                break;

            default:
                //printf("nRF24L01P: Invalid nrf24l01p_set_rx_addr length setting %d\r\n", length);
                return;
        }

        nrf24l01p_write_reg(dev, _NRF24L01P_REG_SETUP_AW, setup_aw);

    }
    else
    {
        length = 1;
    }

    int rx_addr_px_register = _NRF24L01P_REG_RX_ADDR_P0 + (pipe_num - NRF24L01P_PIPE_P0);

    int cn = (_NRF24L01P_SPI_CMD_WR_REG | (rx_addr_px_register & _NRF24L01P_REG_addr_MASK));

    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, cn);

    while (length-- > 0)
    {
        // LSByte first
        nrf24l01p_spi_write(dev, (int)(addr & 0xFF));
        addr >>= 8;
    }

    nrf24l01p_set_csn_pin(dev, true);

    int en_rx_addr = nrf24l01p_read_reg(dev, _NRF24L01P_REG_EN_RXADDR);

    en_rx_addr |= (1 << (pipe_num - NRF24L01P_PIPE_P0));

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_EN_RXADDR, en_rx_addr);
}

//------------------------------------------------------------------------------
void nrf24l01p_set_tx_addr(nrf24l01p * const dev, uint64_t addr, int length)
{
    int setup_aw = nrf24l01p_read_reg(dev, _NRF24L01P_REG_SETUP_AW) & ~_NRF24L01P_SETUP_AW_AW_MASK;

    switch (length)
    {
        case 3:
            setup_aw |= _NRF24L01P_SETUP_AW_AW_3BYTE;
            break;

        case 4:
            setup_aw |= _NRF24L01P_SETUP_AW_AW_4BYTE;
            break;

        case 5:
            setup_aw |= _NRF24L01P_SETUP_AW_AW_5BYTE;
            break;

        default:
            //printf("nRF24L01P: Invalid nrf24l01p_set_tx_addr length setting %d\r\n", length);
            return;
    }

    nrf24l01p_write_reg(dev, _NRF24L01P_REG_SETUP_AW, setup_aw);

    int cn = (_NRF24L01P_SPI_CMD_WR_REG | (_NRF24L01P_REG_TX_ADDR & _NRF24L01P_REG_addr_MASK));

    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, cn);

    while (length-- > 0)
    {
        // LSByte first
        nrf24l01p_spi_write(dev, (int)(addr & 0xFF));
        addr >>= 8;
    }

    nrf24l01p_set_csn_pin(dev, true);

}

//------------------------------------------------------------------------------
uint64_t nrf24l01p_get_rx_addr(nrf24l01p * const dev, uint8_t pipe_num)
{
    if ((pipe_num < NRF24L01P_PIPE_P0) || (pipe_num > NRF24L01P_PIPE_P5))
    {
        //printf("nRF24L01P: Invalid nrf24l01p_set_rx_addr pipe_num number %d\r\n", pipe_num);
        return 0;
    }

    int length;

    if ((pipe_num == NRF24L01P_PIPE_P0) || (pipe_num == NRF24L01P_PIPE_P1))
    {
        int setup_aw = nrf24l01p_read_reg(dev, _NRF24L01P_REG_SETUP_AW) & _NRF24L01P_SETUP_AW_AW_MASK;

        switch (setup_aw) {

            case _NRF24L01P_SETUP_AW_AW_3BYTE:
                length = 3;
                break;

            case _NRF24L01P_SETUP_AW_AW_4BYTE:
                length = 4;
                break;

            case _NRF24L01P_SETUP_AW_AW_5BYTE:
                length = 5;
                break;

            default:
                //printf("nRF24L01P: Unknown nrf24l01p_get_rx_addr length value %d\r\n", setup_aw);
                return 0;
        }
    }
    else
    {
        length = 1;
    }

    int rx_addr_px_register = _NRF24L01P_REG_RX_ADDR_P0 + (pipe_num - NRF24L01P_PIPE_P0);

    int cn = (_NRF24L01P_SPI_CMD_RD_REG | (rx_addr_px_register & _NRF24L01P_REG_addr_MASK));

    uint64_t addr = 0;

    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, cn);

    for (int i=0; i<length; i++)
    {
        // LSByte first
        addr |= (((uint64_t)(nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_NOP) & 0xFF)) << (i*8));
    }

    nrf24l01p_set_csn_pin(dev, true);

    if (!((pipe_num == NRF24L01P_PIPE_P0) || (pipe_num == NRF24L01P_PIPE_P1))) {

        addr |= (nrf24l01p_get_rx_addr(dev, NRF24L01P_PIPE_P1) & ~((uint64_t) 0xFF));

    }

    return addr;

}

//------------------------------------------------------------------------------
uint64_t nrf24l01p_get_tx_addr(nrf24l01p * const dev)
{
    int setup_aw = nrf24l01p_read_reg(dev, _NRF24L01P_REG_SETUP_AW) & _NRF24L01P_SETUP_AW_AW_MASK;

    int length;

    switch (setup_aw)
    {
        case _NRF24L01P_SETUP_AW_AW_3BYTE:
            length = 3;
            break;

        case _NRF24L01P_SETUP_AW_AW_4BYTE:
            length = 4;
            break;

        case _NRF24L01P_SETUP_AW_AW_5BYTE:
            length = 5;
            break;

        default:
            //printf("nRF24L01P: Unknown nrf24l01p_get_tx_addr length value %d\r\n", setup_aw);
            return 0;
    }

    int cn = (_NRF24L01P_SPI_CMD_RD_REG | (_NRF24L01P_REG_TX_ADDR & _NRF24L01P_REG_addr_MASK));

    uint64_t addr = 0;

    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, cn);

    for (int i=0; i<length; i++)
    {
        // LSByte first
        addr |= (((uint64_t)(nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_NOP) & 0xFF)) << (i*8));
    }

    nrf24l01p_set_csn_pin(dev, true);

    return addr;
}

//------------------------------------------------------------------------------
bool nrf24l01p_available(nrf24l01p * const dev, uint8_t pipe_num)
{

    if ((pipe_num < NRF24L01P_PIPE_P0) || (pipe_num > NRF24L01P_PIPE_P5))
    {
        printf("nRF24L01P: Invalid nrf24l01p_available pipe_num number %d\r\n", pipe_num);
        return false;
    }

    int status = nrf24l01p_get_status_reg(dev);

    return ((status & _NRF24L01P_STATUS_RX_DR) && (((status & _NRF24L01P_STATUS_RX_P_NO) >> 1) == (pipe_num & 0x7)));
}

//------------------------------------------------------------------------------
int nrf24l01p_write(nrf24l01p * const dev, uint8_t pipe_num, uint8_t *data, int len)
{
    // @note: The pipe_num number is ignored in a Transmit / nrf24l01p_write

    // Save the CE state
    bool ce_state = nrf24l01p_get_ce_pin(dev);
    nrf24l01p_disable(dev);

    if (len <= 0) return 0;

    if (len > _NRF24L01P_TX_FIFO_SIZE)
    {
        len = _NRF24L01P_TX_FIFO_SIZE;
    }

    // Clear the Status bit
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_TX_DS);

    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_WR_TX_PAYLOAD);

    for (int i = 0; i < len; i++)
    {
        nrf24l01p_spi_write(dev, *data++);
    }

    nrf24l01p_set_csn_pin(dev, true);

    int original_mode = dev->mode;
    nrf24l01p_set_tx_mode(dev);

    nrf24l01p_enable(dev);
    nrf24l01p_delay_ms(_NRF24L01P_TIMING_Thce_us);
    nrf24l01p_disable(dev);

    while (!(nrf24l01p_get_status_reg(dev) & _NRF24L01P_STATUS_TX_DS))
    {
        // Wait for the transfer to complete
    }

    // Clear the Status bit
    nrf24l01p_write_reg(dev, _NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_TX_DS);

    if (original_mode == _NRF24L01P_MODE_RX)
    {
        nrf24l01p_set_rx_mode(dev);
    }

    // Restore CE pin state
    nrf24l01p_set_ce_pin(dev, ce_state);
    nrf24l01p_delay_ms(_NRF24L01P_TIMING_Tpece2csn_us);

    return len;
}

//------------------------------------------------------------------------------
int nrf24l01p_read(nrf24l01p * const dev, uint8_t pipe_num, uint8_t *data, int len)
{

    if (len <= 0)
    {
    	return 0;
    }

    if (len > _NRF24L01P_RX_FIFO_SIZE)
    {
    	len = _NRF24L01P_RX_FIFO_SIZE;
    }

    if (nrf24l01p_available(dev, pipe_num))
    {
        nrf24l01p_set_csn_pin(dev, false);

        int status = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_R_RX_PL_WID);

        int rx_payload_width = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_NOP);

        nrf24l01p_set_csn_pin(dev, true);

        if ((rx_payload_width < 0) || (rx_payload_width > _NRF24L01P_RX_FIFO_SIZE))
        {

            // Received payload error: need to flush the FIFO
            nrf24l01p_set_csn_pin(dev, false);

            int status = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_FLUSH_RX);

            int rx_payload_width = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_NOP);

            nrf24l01p_set_csn_pin(dev, true);

            // At this point, we should retry the reception,
            //  but for now we'll just fall through...
        }
        else
        {
            if (rx_payload_width < len)
            {
                len = rx_payload_width;
            }

            nrf24l01p_set_csn_pin(dev, false);

            int status = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_RD_RX_PAYLOAD);

            for (int i = 0; i < len; i++) {

                *data++ = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_NOP);

            }

            nrf24l01p_set_csn_pin(dev, true);

            // Clear the Status bit
            nrf24l01p_write_reg(dev, _NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_RX_DR);

            return len;
        }
    }
    else
    {
        // What should we do if there is no 'nrf24l01p_available' data?
        //  We could wait for data to arrive, but for now, we'll
        //  just return with no data.
        return 0;

    }

    //
    // We get here because an error condition occured;
    //  We could wait for data to arrive, but for now, we'll
    //  just return with no data.
    //
    return -1;

}

//------------------------------------------------------------------------------
void nrf24l01p_write_reg(nrf24l01p * const dev, uint8_t reg_addr, uint8_t reg_data)
{
    // Save the CE state
    bool ce_state = nrf24l01p_get_ce_pin(dev);
    nrf24l01p_disable(dev);

    int cn = (_NRF24L01P_SPI_CMD_WR_REG | (reg_addr & _NRF24L01P_REG_addr_MASK));

    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, cn);

    nrf24l01p_spi_write(dev, reg_data & 0xFF);

    nrf24l01p_set_csn_pin(dev, true);

    // Restore CE pin state
    nrf24l01p_set_ce_pin(dev, ce_state);
    nrf24l01p_delay_ms(_NRF24L01P_TIMING_Tpece2csn_us);
}

//------------------------------------------------------------------------------
int nrf24l01p_read_reg(nrf24l01p * const dev, uint8_t reg_addr)
{
    int cn = (_NRF24L01P_SPI_CMD_RD_REG | (reg_addr & _NRF24L01P_REG_addr_MASK));

    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, cn);

    int dn = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_NOP);

    nrf24l01p_set_csn_pin(dev, true);

    return dn;
}

//------------------------------------------------------------------------------
int nrf24l01p_get_status_reg(nrf24l01p * const dev)
{
    nrf24l01p_set_csn_pin(dev, false);

    int status = nrf24l01p_spi_write(dev, _NRF24L01P_SPI_CMD_NOP);

    nrf24l01p_set_csn_pin(dev, true);

    return status;
}

//------------------------------------------------------------------------------
