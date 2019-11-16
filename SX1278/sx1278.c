/**
 *  @brief:  Implementation of a SX1278 radio functions
 *  @author: luk6xff based on SEMTCH code: https://github.com/Lora-net/LoRaMac-node
 *  @email:  luszko@op.pl
 *  @date:   2019-11-15
 */
#include "registers.h"
#include "sx1278.h"

#include <math.h>
#include <string.h>

/**
 * ============================================================================
 * @brief Private global constants
 * ============================================================================
 */
//-----------------------------------------------------------------------------
/**
 * Precomputed FSK bandwidth registers values
 */
static const BandwidthMap_t SX1278FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Bandwidth
};

//-----------------------------------------------------------------------------
/**
 * @brief Precomputed LORA bandwidth registers values
 */
static const BandwidthMap_t SX1278LoRaBandwidths[] =
{
    {   7800, 0 }, //  7.8 kHz requires TCXO
    {  10400, 1 }, // 10.4 kHz requires TCXO
    {  15600, 2 }, // 15.6 kHz requires TCXO
    {  20800, 3 }, // 20.8 kHz requires TCXO
    {  31250, 4 }, // 31.25 kHz requires TCXO
    {  41700, 5 }, // 41.7 kHz requires TCXO
    {  62500, 6 }, // 62.5 kHz requires TCXO
    { 125000, 7 }, // 125 kHz the LoRa protocol default
    { 250000, 8 }, // 250 kHz
    { 500000, 9 }, // 500 kHz
    { 600000, 10 },// Invalid Bandwidth, reserved
 };

//-----------------------------------------------------------------------------
/**
 * @brief Radio hardware registers initialization definition
 */
static const RadioRegisters_t SX1278RadioRegsInit[] =
{                                                 \
    { MODEM_FSK , REG_LNA                , 0x23 },\
    { MODEM_FSK , REG_RXCONFIG           , 0x1E },\
    { MODEM_FSK , REG_RSSICONFIG         , 0xD2 },\
    { MODEM_FSK , REG_AFCFEI             , 0x01 },\
    { MODEM_FSK , REG_PREAMBLEDETECT     , 0xAA },\
    { MODEM_FSK , REG_OSC                , 0x07 },\
    { MODEM_FSK , REG_SYNCCONFIG         , 0x12 },\
    { MODEM_FSK , REG_SYNCVALUE1         , 0xC1 },\
    { MODEM_FSK , REG_SYNCVALUE2         , 0x94 },\
    { MODEM_FSK , REG_SYNCVALUE3         , 0xC1 },\
    { MODEM_FSK , REG_PACKETCONFIG1      , 0xD8 },\
    { MODEM_FSK , REG_FIFOTHRESH         , 0x8F },\
    { MODEM_FSK , REG_IMAGECAL           , 0x02 },\
    { MODEM_FSK , REG_DIOMAPPING1        , 0x00 },\
    { MODEM_FSK , REG_DIOMAPPING2        , 0x30 },\
    { MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0x40 },\
};


/**
 * ============================================================================
 * @brief Private functions prototypes
 * ============================================================================
 */

/**
 * @brief Performs the Rx chain calibration for LF and HF bands
 * @note Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration(void);

/**
 * Returns the known FSK bandwidth registers value
 *
 * @param [IN] bandwidth Bandwidth value in Hz
 * @retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue(uint32_t bandwidth);

/**
 * Returns the known LORA bandwidth registers value
 *
 * @param [IN] bandwidth Bandwidth value in Hz
 * @retval regValue Bandwidth register value.
 */
static uint8_t GetLoRaBandwidthRegValue(uint32_t bandwidth);


/**
 * @brief DIO 0 IRQ callback
 */
static void OnDio0Irq();

/**
 * @brief DIO 1 IRQ callback
 */
static void OnDio1Irq();

/**
 * @brief DIO 2 IRQ callback
 */
static void OnDio2Irq();

/**
 * @brief DIO 3 IRQ callback
 */
static void OnDio3Irq();

/**
 * @brief DIO 4 IRQ callback
 */
static void OnDio4Irq();

/**
 * @brief DIO 5 IRQ callback
 */
static void OnDio5Irq();

/**
 * @brief Tx & Rx timeout timer callback
 */
static void OnTimeoutIrq();



/**
 * ============================================================================
 * @brief Private global variables
 * ============================================================================
 */

/**
 * Radio callbacks variable
 */
static RadioEvents_t* RadioEvents;

/**
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/**
 * Hardware DIO IRQ functions
 */
DioIrqHandler dioIrq[] = { OnDio0Irq, OnDio1Irq,
                            OnDio2Irq, OnDio3Irq,
                            OnDio4Irq, NULL };

/**
 * Radio settings
 */
RadioSettings_t settings;



/**
 * ============================================================================
 * @brief Public functions definitions
 * ============================================================================
 */
//-----------------------------------------------------------------------------
bool SX1278Init(RadioEvents_t *events)
{
    RadioEvents = events;

    if (SX1278Read(REG_VERSION) == 0x00)
    {
        return false;
    }
    
    SX1278Reset();
    RxChainCalibration();
    SX1278SetOpMode(RF_OPMODE_SLEEP);

    SX1278IoIrqInit(dioIrq);
    SX1278RadioRegistersInit();
    SX1278SetModem(MODEM_FSK);
    settings.State = RF_IDLE;
    return true;
}

//-----------------------------------------------------------------------------
void SX1278RadioRegistersInit()
{
    uint8_t i = 0;
    for(i = 0; i < sizeof(SX1278RadioRegsInit) / sizeof(RadioRegisters_t); i++)
    {
        SX1278SetModem(SX1278RadioRegsInit[i].Modem);
        SX1278Write(SX1278RadioRegsInit[i].Addr, SX1278RadioRegsInit[i].Value);
    }    
}

//-----------------------------------------------------------------------------
RadioState_t SX1278GetStatus(void)
{
    return settings.State;
}

//-----------------------------------------------------------------------------
void SX1278SetChannel(uint32_t freq)
{
    settings.Channel = freq;
    freq = (uint32_t)((double)freq / (double)FREQ_STEP);
    SX1278Write(REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    SX1278Write(REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    SX1278Write(REG_FRFLSB, (uint8_t)(freq & 0xFF));
}

//-----------------------------------------------------------------------------
bool SX1278IsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime)
{
    // TODO handle carrierSenseTime
    int16_t rssi = 0;

    SX1278SetModem(modem);

    SX1278SetChannel(freq);

    SX1278SetOpMode(RF_OPMODE_RECEIVER);

    SX1278DelayMs(1);

    rssi = SX1278GetRssi(modem);

    SX1278SetSleep();
    if (rssi > rssiThresh)
    {
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
uint32_t SX1278Random(void)
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1278SetModem(MODEM_LORA);

    // Disable LoRa modem interrupts
    SX1278Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED);

    // Set radio in continuous reception
    SX1278SetOpMode(RF_OPMODE_RECEIVER);

    for(i = 0; i < 32; i++)
    {
        SX1278DelayMs(1);
        // Unfiltered RSSI value SX1278Reading. Only takes the LSB value
        rnd |= ((uint32_t)SX1278Read(REG_LR_RSSIWIDEBAND) & 0x01) << i;
    }

    SX1278SetSleep();

    return rnd;
}

//-----------------------------------------------------------------------------
void SX1278SetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous)
{
    SX1278SetModem(modem);

    switch (modem)
    {
        case MODEM_FSK:
        {
            settings.Fsk.Bandwidth = bandwidth;
            settings.Fsk.Datarate = datarate;
            settings.Fsk.BandwidthAfc = bandwidthAfc;
            settings.Fsk.FixLen = fixLen;
            settings.Fsk.PayloadLen = payloadLen;
            settings.Fsk.CrcOn = crcOn;
            settings.Fsk.IqInverted = iqInverted;
            settings.Fsk.RxContinuous = rxContinuous;
            settings.Fsk.PreambleLen = preambleLen;
            settings.Fsk.RxSingleTimeout = symbTimeout * ((1.0 / (double)datarate) * 8.0) * 1000;

            datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
            SX1278Write(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
            SX1278Write(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

            SX1278Write(REG_RXBW, GetFskBandwidthRegValue(bandwidth));
            SX1278Write(REG_AFCBW, GetFskBandwidthRegValue(bandwidthAfc));

            SX1278Write(REG_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
            SX1278Write(REG_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

            if (fixLen == 1)
            {
                SX1278Write(REG_PAYLOADLENGTH, payloadLen);
            }
            else
            {
                SX1278Write(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
            }
            
            SX1278Write(REG_PACKETCONFIG1,
                         (SX1278Read(REG_PACKETCONFIG1) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                           ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                           (crcOn << 4));
            SX1278Write(REG_PACKETCONFIG2, (SX1278Read(REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
        }
        break;
        case MODEM_LORA:
        {
            if (bandwidth > 11) // specified in Hz, needs mapping
            {
            	bandwidth = GetLoRaBandwidthRegValue(bandwidth);
            }
            if (bandwidth > LORA_BANDWIDTH_500kHz)
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while(1);
            }
            settings.LoRa.Bandwidth = bandwidth;
            settings.LoRa.Datarate = datarate;
            settings.LoRa.Coderate = coderate;
            settings.LoRa.PreambleLen = preambleLen;
            settings.LoRa.FixLen = fixLen;
            settings.LoRa.PayloadLen = payloadLen;
            settings.LoRa.CrcOn = crcOn;
            settings.LoRa.FreqHopOn = freqHopOn;
            settings.LoRa.HopPeriod = hopPeriod;
            settings.LoRa.IqInverted = iqInverted;
            settings.LoRa.RxContinuous = rxContinuous;

            if (datarate > LORA_SF12)
            {
                datarate = LORA_SF12;
            }
            else if (datarate < LORA_SF6)
            {
                datarate = LORA_SF6;
            }

            if (((bandwidth == LORA_BANDWIDTH_125kHz) && ((datarate == LORA_SF11) || (datarate == LORA_SF12))) ||
               ((bandwidth == LORA_BANDWIDTH_250kHz) && (datarate == LORA_SF12)))
            {
                settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                settings.LoRa.LowDatarateOptimize = 0x00;
            }

            SX1278Write(REG_LR_MODEMCONFIG1,
                         (SX1278Read(REG_LR_MODEMCONFIG1) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                           (bandwidth << 4) | (coderate << 1) |
                           fixLen);

            SX1278Write(REG_LR_MODEMCONFIG2,
                         (SX1278Read(REG_LR_MODEMCONFIG2) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
                           (datarate << 4) | (crcOn << 2) |
                           ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

            SX1278Write(REG_LR_MODEMCONFIG3,
                         (SX1278Read(REG_LR_MODEMCONFIG3) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                           (settings.LoRa.LowDatarateOptimize << 3));

            SX1278Write(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));

            SX1278Write(REG_LR_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
            SX1278Write(REG_LR_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

            if (fixLen == 1)
            {
                SX1278Write(REG_LR_PAYLOADLENGTH, payloadLen);
            }

            if (settings.LoRa.FreqHopOn == true)
            {
                SX1278Write(REG_LR_PLLHOP, (SX1278Read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                SX1278Write(REG_LR_HOPPERIOD, settings.LoRa.HopPeriod);
            }

            if ((bandwidth == LORA_BANDWIDTH_500kHz) && (settings.Channel > RF_MID_BAND_THRESH))
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1278Write(REG_LR_HIGHBWOPTIMIZE1, 0x02);
                SX1278Write(REG_LR_HIGHBWOPTIMIZE2, 0x64);
            }
            else if (bandwidth == LORA_BANDWIDTH_500kHz)
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1278Write(REG_LR_HIGHBWOPTIMIZE1, 0x02);
                SX1278Write(REG_LR_HIGHBWOPTIMIZE2, 0x7F);
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1278Write(REG_LR_HIGHBWOPTIMIZE1, 0x03);
            }

            if (datarate == LORA_SF6)
            {
                SX1278Write(REG_LR_DETECTOPTIMIZE,
                             (SX1278Read(REG_LR_DETECTOPTIMIZE) &
                               RFLR_DETECTIONOPTIMIZE_MASK) |
                               RFLR_DETECTIONOPTIMIZE_SF6);
                SX1278Write(REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6);
            }
            else
            {
                SX1278Write(REG_LR_DETECTOPTIMIZE,
                            (SX1278Read(REG_LR_DETECTOPTIMIZE) &
                            RFLR_DETECTIONOPTIMIZE_MASK) |
                            RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                SX1278Write(REG_LR_DETECTIONTHRESHOLD,
                            RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
        }
        break;
    }
}

//-----------------------------------------------------------------------------
void SX1278SetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    SX1278SetModem(modem);
    SX1278SetRfTxPower(power);

    switch (modem)
    {
        case MODEM_FSK:
        {
            settings.Fsk.Power = power;
            settings.Fsk.Fdev = fdev;
            settings.Fsk.Bandwidth = bandwidth;
            settings.Fsk.Datarate = datarate;
            settings.Fsk.PreambleLen = preambleLen;
            settings.Fsk.FixLen = fixLen;
            settings.Fsk.CrcOn = crcOn;
            settings.Fsk.IqInverted = iqInverted;
            settings.Fsk.TxTimeout = timeout;

            fdev = (uint16_t)((double)fdev / (double)FREQ_STEP);
            SX1278Write(REG_FDEVMSB, (uint8_t)(fdev >> 8));
            SX1278Write(REG_FDEVLSB, (uint8_t)(fdev & 0xFF));

            datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
            SX1278Write(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
            SX1278Write(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

            SX1278Write(REG_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
            SX1278Write(REG_PREAMBLELSB, preambleLen & 0xFF);

            SX1278Write(REG_PACKETCONFIG1,
                        (SX1278Read(REG_PACKETCONFIG1) &
                        RF_PACKETCONFIG1_CRC_MASK &
                        RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                        ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                        (crcOn << 4));
            SX1278Write(REG_PACKETCONFIG2, (SX1278Read(REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
        }
        break;
        case MODEM_LORA:
        {
            settings.LoRa.Power = power;
            if (bandwidth > LORA_BANDWIDTH_500kHz)
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while(1);
            }

            settings.LoRa.Bandwidth = bandwidth;
            settings.LoRa.Datarate = datarate;
            settings.LoRa.Coderate = coderate;
            settings.LoRa.PreambleLen = preambleLen;
            settings.LoRa.FixLen = fixLen;
            settings.LoRa.FreqHopOn = freqHopOn;
            settings.LoRa.HopPeriod = hopPeriod;
            settings.LoRa.CrcOn = crcOn;
            settings.LoRa.IqInverted = iqInverted;
            settings.LoRa.TxTimeout = timeout;

            if (datarate > LORA_SF12)
            {
                datarate = LORA_SF12;
            }
            else if (datarate < LORA_SF6)
            {
                datarate = LORA_SF6;
            }
            if (((bandwidth == LORA_BANDWIDTH_125kHz) && ((datarate == LORA_SF11) || (datarate == LORA_SF12))) ||
                ((bandwidth == LORA_BANDWIDTH_250kHz) && (datarate == LORA_SF12)))
            {
                settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if (settings.LoRa.FreqHopOn == true)
            {
                SX1278Write(REG_LR_PLLHOP, (SX1278Read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                SX1278Write(REG_LR_HOPPERIOD, settings.LoRa.HopPeriod);
            }

            SX1278Write(REG_LR_MODEMCONFIG1,
                         (SX1278Read(REG_LR_MODEMCONFIG1) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                           (bandwidth << 4) | (coderate << 1) |
                           fixLen);

            SX1278Write(REG_LR_MODEMCONFIG2,
                         (SX1278Read(REG_LR_MODEMCONFIG2) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) |
                           (datarate << 4) | (crcOn << 2));

            SX1278Write(REG_LR_MODEMCONFIG3,
                         (SX1278Read(REG_LR_MODEMCONFIG3) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                           (settings.LoRa.LowDatarateOptimize << 3));

            SX1278Write(REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
            SX1278Write(REG_LR_PREAMBLELSB, preambleLen & 0xFF);

            if (datarate == LORA_SF6)
            {
                SX1278Write(REG_LR_DETECTOPTIMIZE,
                             (SX1278Read(REG_LR_DETECTOPTIMIZE) &
                               RFLR_DETECTIONOPTIMIZE_MASK) |
                               RFLR_DETECTIONOPTIMIZE_SF6);
                SX1278Write(REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6);
            }
            else
            {
                SX1278Write(REG_LR_DETECTOPTIMIZE,
                             (SX1278Read(REG_LR_DETECTOPTIMIZE) &
                             RFLR_DETECTIONOPTIMIZE_MASK) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                SX1278Write(REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
        }
        break;
    }
}

//-----------------------------------------------------------------------------
uint32_t SX1278TimeOnAir(RadioModems_t modem, uint8_t pktLen)
{
    uint32_t airTime = 0;

    switch (modem)
    {
    case MODEM_FSK:
        {
            airTime = rint((8 * (settings.Fsk.PreambleLen +
                                ((SX1278Read(REG_SYNCCONFIG) & ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1) +
                                ((settings.Fsk.FixLen == 0x01) ? 0.0 : 1.0) +
                                (((SX1278Read(REG_PACKETCONFIG1) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK) != 0x00) ? 1.0 : 0) +
                                pktLen +
                                ((settings.Fsk.CrcOn == 0x01) ? 2.0 : 0)) /
                                settings.Fsk.Datarate) * 1000);
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            // NOTE: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch (settings.LoRa.Bandwidth)
            {
                case LORA_BANDWIDTH_7kHz: // 7.8 kHz
                    bw = 78e2;
                    break;
                case LORA_BANDWIDTH_10kHz: // 10.4 kHz
                    bw = 104e2;
                    break;
                case LORA_BANDWIDTH_15kHz: // 15.6 kHz
                    bw = 156e2;
                    break;
                case LORA_BANDWIDTH_20kHz: // 20.8 kHz
                    bw = 208e2;
                    break;
                case LORA_BANDWIDTH_31kHz: // 31.25 kHz
                    bw = 312e2;
                    break;
                case LORA_BANDWIDTH_41kHz: // 41.7 kHz
                    bw = 414e2;
                    break;
                case LORA_BANDWIDTH_62kHz: // 62.5 kHz
                    bw = 625e2;
                    break;
                case LORA_BANDWIDTH_125kHz: // 125 kHz
                    bw = 125e3;
                    break;
                case LORA_BANDWIDTH_250kHz: // 250 kHz
                    bw = 250e3;
                    break;
                case LORA_BANDWIDTH_500kHz: // 500 kHz
                    bw = 500e3;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / (1 << settings.LoRa.Datarate);
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = (settings.LoRa.PreambleLen + 4.25) * ts;
            // Symbol length of payload and time
            double tmp = ceil((8 * pktLen - 4 * settings.LoRa.Datarate +
                                 28 + 16 * settings.LoRa.CrcOn -
                                 (settings.LoRa.FixLen ? 20 : 0)) /
                                 (double)(4 * (settings.LoRa.Datarate -
                                 ((settings.LoRa.LowDatarateOptimize > 0) ? 2 : 0)))) *
                                 (settings.LoRa.Coderate + 4);
            double nPayload = 8 + ((tmp > 0) ? tmp : 0);
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = floor(tOnAir * 1000 + 0.999);
        }
        break;
    }
    return airTime;
}

//-----------------------------------------------------------------------------
void SX1278Send(uint8_t *buffer, uint8_t size)
{
    uint32_t txTimeout = 0;

    switch (settings.Modem)
    {
        case MODEM_FSK:
        {
            settings.FskPacketHandler.NbBytes = 0;
            settings.FskPacketHandler.Size = size;

            if (settings.Fsk.FixLen == false)
            {
                SX1278WriteFifo((uint8_t*)&size, 1);
            }
            else
            {
                SX1278Write(REG_PAYLOADLENGTH, size);
            }

            if ((size > 0) && (size <= 64))
            {
                settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy(RxTxBuffer, buffer, size);
                settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            SX1278WriteFifo(buffer, settings.FskPacketHandler.ChunkSize);
            settings.FskPacketHandler.NbBytes += settings.FskPacketHandler.ChunkSize;
            txTimeout = settings.Fsk.TxTimeout;
        }
        break;
        case MODEM_LORA:
        {
            if (settings.LoRa.IqInverted == true)
            {
                SX1278Write(REG_LR_INVERTIQ, ((SX1278Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
                SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            }
            else
            {
                SX1278Write(REG_LR_INVERTIQ, ((SX1278Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
                SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1278Write(REG_LR_PAYLOADLENGTH, size);

            // Full buffer used for Tx
            SX1278Write(REG_LR_FIFOTXBASEADDR, 0);
            SX1278Write(REG_LR_FIFOADDRPTR, 0);

            // FIFO operations can not take place in SX1278SetSleep mode
            if ((SX1278Read(REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP)
            {
                SX1278SetStandby();
                SX1278DelayMs(1);
            }
            // SX1278Write payload buffer
            SX1278WriteFifo(buffer, size);
            txTimeout = settings.LoRa.TxTimeout;
        }
        break;
    }

    SX1278SetTx(txTimeout);
}

//-----------------------------------------------------------------------------
void SX1278SetSleep(void)
{
    SX1278SetTimeout(TXTimeoutTimer, NULL, 0);
    SX1278SetTimeout(RXTimeoutTimer, NULL, 0);

    SX1278SetOpMode(RF_OPMODE_SLEEP);
    //TODO Disable TCXO radio is in SLEEP mode if available
    settings.State = RF_IDLE;
}

//-----------------------------------------------------------------------------
void SX1278SetStandby(void)
{
    SX1278SetTimeout(TXTimeoutTimer, NULL, 0);
    SX1278SetTimeout(RXTimeoutTimer, NULL, 0);
    SX1278SetTimeout(RXTimeoutSyncWordTimer, NULL, 0);

    SX1278SetOpMode(RF_OPMODE_STANDBY);
    settings.State = RF_IDLE;
}

//-----------------------------------------------------------------------------
void SX1278SetRx(uint32_t timeout)
{
    bool rxContinuous = false;

    switch (settings.Modem)
    {
    case MODEM_FSK:
        {
            rxContinuous = settings.Fsk.RxContinuous;

            // DIO0=PayloadSX1278Ready
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeSX1278Ready
            SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK &
                                                                        RF_DIOMAPPING1_DIO1_MASK &
                                                                        RF_DIOMAPPING1_DIO2_MASK) |
                                                                        RF_DIOMAPPING1_DIO0_00 |
                                                                        RF_DIOMAPPING1_DIO1_00 |
                                                                        RF_DIOMAPPING1_DIO2_11);

            SX1278Write(REG_DIOMAPPING2, (SX1278Read(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK &
                                                                        RF_DIOMAPPING2_MAP_MASK) |
                                                                        RF_DIOMAPPING2_DIO4_11 |
                                                                        RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

            settings.FskPacketHandler.FifoThresh = SX1278Read(REG_FIFOTHRESH) & 0x3F;

            SX1278Write(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

            settings.FskPacketHandler.PreambleDetected = false;
            settings.FskPacketHandler.SyncWordDetected = false;
            settings.FskPacketHandler.NbBytes = 0;
            settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if (settings.LoRa.IqInverted == true)
            {
                SX1278Write(REG_LR_INVERTIQ, ((SX1278Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
                SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            }
            else
            {
                SX1278Write(REG_LR_INVERTIQ, ((SX1278Read(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
                SX1278Write(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if (settings.LoRa.Bandwidth < LORA_BANDWIDTH_500kHz)
            {
                SX1278Write(REG_LR_DETECTOPTIMIZE, SX1278Read(REG_LR_DETECTOPTIMIZE) & 0x7F);
                SX1278Write(REG_LR_IFFREQ2, 0x00);
                switch (settings.LoRa.Bandwidth)
                {
                    case LORA_BANDWIDTH_7kHz: // 7.8 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x48 );
                        SX1278SetChannel(settings.Channel + 7810);
                        break;
                    case LORA_BANDWIDTH_10kHz: // 10.4 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x44 );
                        SX1278SetChannel(settings.Channel + 10420);
                        break;
                    case LORA_BANDWIDTH_15kHz: // 15.6 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x44 );
                        SX1278SetChannel(settings.Channel + 15620);
                        break;
                    case LORA_BANDWIDTH_20kHz: // 20.8 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x44 );
                        SX1278SetChannel(settings.Channel + 20830);
                        break;
                    case LORA_BANDWIDTH_31kHz: // 31.25 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x44 );
                        SX1278SetChannel(settings.Channel + 31250);
                        break;
                    case LORA_BANDWIDTH_41kHz: // 41.4 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x44 );
                        SX1278SetChannel(settings.Channel + 41670);
                        break;
                    case LORA_BANDWIDTH_62kHz: // 62.5 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x40 );
                        break;
                    case LORA_BANDWIDTH_125kHz: // 125 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x40 );
                        break;
                    case LORA_BANDWIDTH_250kHz: // 250 kHz
                        SX1278Write(REG_LR_IFFREQ1, 0x40 );
                        break;
                }
            }
            else
            {
                SX1278Write(REG_LR_DETECTOPTIMIZE, SX1278Read(REG_LR_DETECTOPTIMIZE) | 0x80);
            }

            rxContinuous = settings.LoRa.RxContinuous;

            if (settings.LoRa.FreqHopOn == true)
            {
                SX1278Write(REG_LR_IRQFLAGSMASK,//RFLR_IRQFLAGS_RXTIMEOUT |
                                                //RFLR_IRQFLAGS_RXDONE |
                                                //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                RFLR_IRQFLAGS_VALIDHEADER |
                                                RFLR_IRQFLAGS_TXDONE |
                                                RFLR_IRQFLAGS_CADDONE |
                                                //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
            }
            else
            {
                SX1278Write(REG_LR_IRQFLAGSMASK,//RFLR_IRQFLAGS_RXTIMEOUT |
                                                //RFLR_IRQFLAGS_RXDONE |
                                                //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                RFLR_IRQFLAGS_VALIDHEADER |
                                                RFLR_IRQFLAGS_TXDONE |
                                                RFLR_IRQFLAGS_CADDONE |
                                                RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone
                SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);
            }
            SX1278Write(REG_LR_FIFORXBASEADDR, 0);
            SX1278Write(REG_LR_FIFOADDRPTR, 0);
        }
        break;
    }

    memset(RxTxBuffer, 0, (size_t)RX_BUFFER_SIZE);

    settings.State = RF_RX_RUNNING;
    if (timeout != 0)
    {
        SX1278SetTimeout(RXTimeoutTimer, &OnTimeoutIrq, timeout);
    }

    if (settings.Modem == MODEM_FSK)
    {
        SX1278SetOpMode(RF_OPMODE_RECEIVER);

        if (rxContinuous == false)
        {
            SX1278SetTimeout(RXTimeoutSyncWordTimer, &OnTimeoutIrq, settings.Fsk.RxSingleTimeout);
        }
    }
    else // MODEM_LORA
    {
        if (rxContinuous == true)
        {
            SX1278SetOpMode(RFLR_OPMODE_RECEIVER);
        }
        else
        {
            SX1278SetOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
        }
    }
}

//-----------------------------------------------------------------------------
void SX1278SetTx(uint32_t timeout)
{

    switch (settings.Modem)
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeSX1278Ready
            SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK &
                                                                        RF_DIOMAPPING1_DIO1_MASK &
                                                                        RF_DIOMAPPING1_DIO2_MASK) |
                                                                        RF_DIOMAPPING1_DIO1_01);

            SX1278Write(REG_DIOMAPPING2, (SX1278Read(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK));
            settings.FskPacketHandler.FifoThresh = SX1278Read(REG_FIFOTHRESH) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if (settings.LoRa.FreqHopOn == true)
            {
                SX1278Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00);
            }
            else
            {
                SX1278Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=TxDone
                SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
            }
        }
        break;
    }

    settings.State = RF_TX_RUNNING;
    SX1278SetTimeout(TXTimeoutTimer, &OnTimeoutIrq, timeout);
    SX1278SetOpMode(RF_OPMODE_TRANSMITTER);
}

//-----------------------------------------------------------------------------
void SX1278StartCad(void)
{
    switch (settings.Modem)
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
            SX1278Write(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                       );

            if (dioIrq[3]) 
            {
                // DIO3=CADDone
                SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO3_MASK ) | RFLR_DIOMAPPING1_DIO3_00 );
            } 
            else 
            {
                // DIO0=CADDone
                SX1278Write(REG_DIOMAPPING1, (SX1278Read(REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_10 );
            }

            settings.State = RF_CAD;
            SX1278SetOpMode(RFLR_OPMODE_CAD);
        }
        break;
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
void SX1278SetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time)
{
    uint32_t timeout = (uint32_t)(time);

    SX1278SetChannel(freq);

    SX1278SetTxConfig(MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout);

    SX1278Write(REG_PACKETCONFIG2, (SX1278Read(REG_PACKETCONFIG2) & RF_PACKETCONFIG2_DATAMODE_MASK));
    // Disable radio interrupts
    SX1278Write(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11);
    SX1278Write(REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10);

    settings.State = RF_TX_RUNNING;
    SX1278SetTimeout(TXTimeoutTimer, &OnTimeoutIrq, timeout);
    SX1278SetOpMode(RF_OPMODE_TRANSMITTER);
}

//-----------------------------------------------------------------------------
int16_t SX1278GetRssi(RadioModems_t modem)
{
    int16_t rssi = 0;

    switch (modem)
    {
        case MODEM_FSK:
            rssi = -(SX1278Read(REG_RSSIVALUE) >> 1);
            break;
        case MODEM_LORA:
            if (settings.Channel > RF_MID_BAND_THRESH)
            {
                rssi = RSSI_OFFSET_HF + SX1278Read(REG_LR_RSSIVALUE);
            }
            else
            {
                rssi = RSSI_OFFSET_LF + SX1278Read(REG_LR_RSSIVALUE);
            }
            break;
        default:
            rssi = -1;
            break;
    }
    return rssi;
}

//-----------------------------------------------------------------------------
int32_t SX1278GetFrequencyError(RadioModems_t modem )
{
    int32_t val = 0;
    
    if (modem != MODEM_LORA)
        return 0;
    
    val = (SX1278Read(REG_LR_FEIMSB) & 0b1111) << 16; // high word, 4 valid bits only
    val |= ((SX1278Read(REG_LR_FEIMID) << 8) | SX1278Read(REG_LR_FEILSB)); // high byte, low byte
    if (val & 0x80000) //convert sign bit
        val |= 0xfff00000;
    
    int32_t bandwidth = 0;
    for (int i = 0; i < (int)(sizeof(SX1278LoRaBandwidths) / sizeof(BandwidthMap_t)) -1; i++ ) {
        if (SX1278LoRaBandwidths[i].RegValue == settings.LoRa.Bandwidth) {
            bandwidth = SX1278LoRaBandwidths[i].bandwidth;
            break;
        }
    }
    if (!bandwidth)
    	return 0;
    
    float bandWidthkHz = (float)bandwidth/1000;
    
    int32_t hz = (((float)val * (float)(1<<24)) / ((float)XTAL_FREQ)) * (bandWidthkHz / 500.0);
        
    return hz;
}

//-----------------------------------------------------------------------------
void SX1278SetOpMode(uint8_t opMode)
{
    // if(opMode == RF_OPMODE_SLEEP ) // TODO NOT USED on RA-01
    // {
    //     SX1278SetAntSwLowPower( true );
    // }
    // else
    // {
    //     // Enable TCXO if operating mode different from SLEEP.
    //     SX1278SetBoardTcxo( true );
    //     SX1278SetAntSwLowPower( false );
    //     SX1278SetAntSw( opMode );
    // }
    SX1278Write(REG_OPMODE, (SX1278Read(REG_OPMODE) & RF_OPMODE_MASK) | opMode);
}

//-----------------------------------------------------------------------------
void SX1278SetModem(RadioModems_t modem)
{
    if ((SX1278Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) != 0)
    {
        settings.Modem = MODEM_LORA;
    }
    else
    {
        settings.Modem = MODEM_FSK;
    }

    if (settings.Modem == modem)
    {
        return;
    }

    settings.Modem = modem;
    switch (settings.Modem)
    {
        default:
        case MODEM_FSK:
            SX1278SetSleep();
            SX1278Write(REG_OPMODE, (SX1278Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);

            SX1278Write(REG_DIOMAPPING1, 0x00);
            SX1278Write(REG_DIOMAPPING2, 0x30); // DIO5=ModeSX1278Ready
            break;
        case MODEM_LORA:
            SX1278SetSleep();
            SX1278Write(REG_OPMODE, (SX1278Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);

            SX1278Write(REG_DIOMAPPING1, 0x00);
            SX1278Write(REG_DIOMAPPING2, 0x00);
            break;
    }
}

//-----------------------------------------------------------------------------
void SX1278SetMaxPayloadLength(RadioModems_t modem, uint8_t max)
{
    SX1278SetModem(modem);

    switch (modem)
    {
    case MODEM_FSK:
        if (settings.Fsk.FixLen == false)
        {
            SX1278Write(REG_PAYLOADLENGTH, max);
        }
        break;
    case MODEM_LORA:
        SX1278Write(REG_LR_PAYLOADMAXLENGTH, max);
        break;
    }
}

//-----------------------------------------------------------------------------
void SX1278SetPublicNetwork(bool enable)
{
    SX1278SetModem(MODEM_LORA);
    settings.LoRa.PublicNetwork = enable;
    if (enable == true)
    {
        // Change LoRa modem SyncWord
        SX1278Write(REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    }
    else
    {
        // Change LoRa modem SyncWord
        SX1278Write(REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);
    }
}

//-----------------------------------------------------------------------------
void OnTimeoutIrq()
{
    switch (settings.State)
    {
    case RF_RX_RUNNING:
        if (settings.Modem == MODEM_FSK)
        {
            settings.FskPacketHandler.PreambleDetected = false;
            settings.FskPacketHandler.SyncWordDetected = false;
            settings.FskPacketHandler.NbBytes = 0;
            settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1278Write(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH);
            SX1278Write(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

            if (settings.Fsk.RxContinuous == true)
            {
                // Continuous mode restart Rx chain
                SX1278Write(REG_RXCONFIG, SX1278Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                SX1278SetTimeout(RXTimeoutSyncWordTimer, &OnTimeoutIrq, settings.Fsk.RxSingleTimeout);
            }
            else
            {
                settings.State = RF_IDLE;
                SX1278SetTimeout(RXTimeoutSyncWordTimer, NULL, 0);
            }
        }
        if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
        {
            RadioEvents->RxTimeout();
        }
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // But it has been observed that when it happens it is a result of a corrupted SPI transfer
        // it depends on the platform design.
        // 
        // The workaround is to put the radio in a known state. Thus, we re-initialize it.

        // BEGIN WORKAROUND

        // Reset the radio
        SX1278Reset();

        // Calibrate Rx chain
        RxChainCalibration();

        // Initialize radio default values
        SX1278SetOpMode(RF_OPMODE_SLEEP);
        SX1278RadioRegistersInit();

        SX1278SetModem(MODEM_FSK);

        // Restore previous network type setting.
        SX1278SetPublicNetwork(settings.LoRa.PublicNetwork);
        // END WORKAROUND

        settings.State = RF_IDLE;
        if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
        {
            RadioEvents->TxTimeout();
        }
        break;
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
void SX1278SetRfTxPower(int8_t power)
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1278Read(REG_PACONFIG);
    paDac = SX1278Read(REG_PADAC);

    paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | SX1278GetPaSelect(settings.Channel);
    paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK) | 0x70;

    if((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST)
    {
        if(power > 17)
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
        if((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON)
        {
            if(power < 5)
            {
                power = 5;
            }
            if(power > 20)
            {
                power = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        }
        else
        {
            if(power < 2)
            {
                power = 2;
            }
            if(power > 17)
            {
                power = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    }
    else
    {
        if(power < -1)
        {
            power = -1;
        }
        if(power > 14)
        {
            power = 14;
        }
        paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power + 1) & 0x0F);
    }
    SX1278Write(REG_PACONFIG, paConfig);
    SX1278Write(REG_PADAC, paDac);
}

//-----------------------------------------------------------------------------
uint8_t SX1278GetPaSelect(uint32_t channel)
{
    if(channel > RF_MID_BAND_THRESH)
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

//-----------------------------------------------------------------------------
bool SX1278CheckRfFrequency(uint32_t frequency)
{
    // Implement check. Currently all frequencies are supported
    return true;
}

//-----------------------------------------------------------------------------
void SX1278Write(uint8_t addr, uint8_t data)
{
    SX1278WriteBuffer(addr, &data, 1);
}

//-----------------------------------------------------------------------------
uint8_t SX1278Read(uint8_t addr)
{
    uint8_t data;
    SX1278ReadBuffer(addr, &data, 1);
    return data;
}

//-----------------------------------------------------------------------------
void SX1278WriteFifo(uint8_t *buffer, uint8_t size)
{
    SX1278WriteBuffer(0, buffer, size);
}

//-----------------------------------------------------------------------------
void SX1278ReadFifo(uint8_t *buffer, uint8_t size)
{
    SX1278ReadBuffer(0, buffer, size);
}


//-----------------------------------------------------------------------------



/**
 * ============================================================================
 * @brief Private functions definitions
 * ============================================================================
 */
//-----------------------------------------------------------------------------
void RxChainCalibration(void)
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1278Read(REG_PACONFIG);
    initialFreq = (double)(((uint32_t)SX1278Read(REG_FRFMSB) << 16) |
                              ((uint32_t)SX1278Read(REG_FRFMID) << 8) |
                              ((uint32_t)SX1278Read(REG_FRFLSB))) * (double)FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1278Write(REG_PACONFIG, 0x00);

    // Launch Rx chain calibration for LF band
    SX1278Write (REG_IMAGECAL, (SX1278Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while((SX1278Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }

    // Sets a Frequency in HF band
    SX1278SetChannel(868000000);

    // Launch Rx chain calibration for HF band
    SX1278Write(REG_IMAGECAL, (SX1278Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while((SX1278Read(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }

    // Restore context
    SX1278Write(REG_PACONFIG, regPaConfigInitVal);
    SX1278SetChannel(initialFreq);
}

//-----------------------------------------------------------------------------
uint8_t GetFskBandwidthRegValue(uint32_t bandwidth)
{
    uint8_t i;

    for (i = 0; i < (sizeof(SX1278FskBandwidths) / sizeof(BandwidthMap_t)) - 1; i++)
    {
        if ((bandwidth >= SX1278FskBandwidths[i].bandwidth) && (bandwidth < SX1278FskBandwidths[i + 1].bandwidth))
        {
            return SX1278FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while(1);
}

//-----------------------------------------------------------------------------
uint8_t GetLoRaBandwidthRegValue(uint32_t bandwidth)
{
    uint8_t i;
    
    for (i = 0; i < (sizeof(SX1278LoRaBandwidths) / sizeof(BandwidthMap_t)) - 1; i++)
    {
        if ((bandwidth >= SX1278LoRaBandwidths[i].bandwidth) && (bandwidth < SX1278LoRaBandwidths[i + 1].bandwidth))
        {
            return SX1278LoRaBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while(1);
}

//-----------------------------------------------------------------------------
void OnDio0Irq()
{
    volatile uint8_t irqFlags = 0;

    switch (settings.State)
    {
        case RF_RX_RUNNING:
            //TimerStop(&RxTimeoutTimer);
            // RxDone interrupt
            switch (settings.Modem)
            {
            case MODEM_FSK:
                if (settings.Fsk.CrcOn == true)
                {
                    irqFlags = SX1278Read(REG_IRQFLAGS2);
                    if ((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK)
                    {
                        // Clear Irqs
                        SX1278Write(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                   RF_IRQFLAGS1_PREAMBLEDETECT |
                                                   RF_IRQFLAGS1_SYNCADDRESSMATCH);
                        SX1278Write(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

                        SX1278SetTimeout(RXTimeoutTimer, NULL, 0);

                        if (settings.Fsk.RxContinuous == false)
                        {
                            SX1278SetTimeout(RXTimeoutSyncWordTimer, NULL, 0);
                            settings.State = RF_IDLE;
                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            SX1278Write(REG_RXCONFIG, SX1278Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                            SX1278SetTimeout(RXTimeoutSyncWordTimer, &OnTimeoutIrq, settings.Fsk.RxSingleTimeout);
                        }

                        if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                        {
                            RadioEvents->RxError();
                        }
                        settings.FskPacketHandler.PreambleDetected = false;
                        settings.FskPacketHandler.SyncWordDetected = false;
                        settings.FskPacketHandler.NbBytes = 0;
                        settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }

                // SX1278Read received packet size
                if ((settings.FskPacketHandler.Size == 0) && (settings.FskPacketHandler.NbBytes == 0))
                {
                    if (settings.Fsk.FixLen == false)
                    {
                        SX1278ReadFifo((uint8_t*)&settings.FskPacketHandler.Size, 1);
                    }
                    else
                    {
                        settings.FskPacketHandler.Size = SX1278Read(REG_PAYLOADLENGTH);
                    }
                    SX1278ReadFifo(RxTxBuffer + settings.FskPacketHandler.NbBytes, settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes);
                    settings.FskPacketHandler.NbBytes += (settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes);
                }
                else
                {
                    SX1278ReadFifo(RxTxBuffer + settings.FskPacketHandler.NbBytes, settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes);
                    settings.FskPacketHandler.NbBytes += (settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes);
                }

                SX1278SetTimeout(RXTimeoutTimer, NULL, 0);

                if (settings.Fsk.RxContinuous == false)
                {
                    settings.State = RF_IDLE;
                    SX1278SetTimeout(RXTimeoutSyncWordTimer, NULL, 0);
                }
                else
                {
                    // Continuous mode restart Rx chain
                    SX1278Write(REG_RXCONFIG, SX1278Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                    SX1278SetTimeout(RXTimeoutSyncWordTimer, &OnTimeoutIrq, settings.Fsk.RxSingleTimeout);
                }

                if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
                {
                    RadioEvents->RxDone(RxTxBuffer, settings.FskPacketHandler.Size, settings.FskPacketHandler.RssiValue, 0);
                }
                settings.FskPacketHandler.PreambleDetected = false;
                settings.FskPacketHandler.SyncWordDetected = false;
                settings.FskPacketHandler.NbBytes = 0;
                settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    // Clear Irq
                    SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

                    irqFlags = SX1278Read(REG_LR_IRQFLAGS);
                    if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
                    {
                        // Clear Irq
                        SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

                        if (settings.LoRa.RxContinuous == false)
                        {
                            settings.State = RF_IDLE;
                        }
                        SX1278SetTimeout(RXTimeoutTimer, NULL, 0);

                        if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                        {
                            RadioEvents->RxError();
                        }
                        break;
                    }
                    // Returns SNR value [dB] rounded to the nearest integer value
                    settings.LoRaPacketHandler.SnrValue = (((int8_t)SX1278Read(REG_LR_PKTSNRVALUE)) + 2) >> 2;
                    int16_t rssi = SX1278Read(REG_LR_PKTRSSIVALUE);
                    if (settings.LoRaPacketHandler.SnrValue < 0)
                    {
                        if (settings.Channel > RF_MID_BAND_THRESH)
                        {
                            settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                   settings.LoRaPacketHandler.SnrValue;
                        }
                        else
                        {
                            settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                   settings.LoRaPacketHandler.SnrValue;
                        }
                    }
                    else
                    {
                        if(settings.Channel > RF_MID_BAND_THRESH)
                        {
                            settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    settings.LoRaPacketHandler.Size = SX1278Read( REG_LR_RXNBBYTES);
                    SX1278Write( REG_LR_FIFOADDRPTR, SX1278Read( REG_LR_FIFORXCURRENTADDR));
                    SX1278ReadFifo( RxTxBuffer, settings.LoRaPacketHandler.Size);

                    if(settings.LoRa.RxContinuous == false)
                    {
                        settings.State = RF_IDLE;
                    }

                    SX1278SetTimeout(RXTimeoutTimer, NULL, 0);

                    if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
                    {
                        RadioEvents->RxDone(RxTxBuffer, settings.LoRaPacketHandler.Size, settings.LoRaPacketHandler.RssiValue, settings.LoRaPacketHandler.SnrValue);
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            SX1278SetTimeout(TXTimeoutTimer, NULL, 0);
            // TxDone interrupt
            switch (settings.Modem)
            {
            case MODEM_LORA:
                // Clear Irq
                SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
                // Intentional fall through
            case MODEM_FSK:
            default:
                settings.State = RF_IDLE;
                if ((RadioEvents != NULL) && (RadioEvents->TxDone != NULL))
                {
                    RadioEvents->TxDone();
                }
                break;
            }
            break;
        default:
            break;
    }
}

//-----------------------------------------------------------------------------
void OnDio1Irq()
{
    switch (settings.State)
    {
        case RF_RX_RUNNING:
            switch (settings.Modem)
            {
                case MODEM_FSK:
                    // Stop Timer
                    SX1278SetTimeout(RXTimeoutSyncWordTimer, NULL, 0);
                    
                    // FifoLevel interrupt
                    // SX1278Read received packet size
                    if ((settings.FskPacketHandler.Size == 0) && (settings.FskPacketHandler.NbBytes == 0))
                    {
                        if (settings.Fsk.FixLen == false)
                        {
                            SX1278ReadFifo((uint8_t*)&settings.FskPacketHandler.Size, 1);
                        }
                        else
                        {
                            settings.FskPacketHandler.Size = SX1278Read(REG_PAYLOADLENGTH);
                        }
                    }
                    // ERRATA 3.1 - PayloadReady Set for 31.25ns if FIFO is Empty
                    //
                    //              When FifoLevel interrupt is used to offload the
                    //              FIFO, the microcontroller should  monitor  both
                    //              PayloadReady  and FifoLevel interrupts, and
                    //              read only (FifoThreshold-1) bytes off the FIFO
                    //              when FifoLevel fires
                    if ((settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes) > settings.FskPacketHandler.FifoThresh)
                    {
                        SX1278ReadFifo((RxTxBuffer + settings.FskPacketHandler.NbBytes), settings.FskPacketHandler.FifoThresh);
                        settings.FskPacketHandler.NbBytes += settings.FskPacketHandler.FifoThresh;
                    }
                    else
                    {
                        SX1278ReadFifo((RxTxBuffer + settings.FskPacketHandler.NbBytes), settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes);
                        settings.FskPacketHandler.NbBytes += (settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes);
                    }
                    break;
                case MODEM_LORA:
                    // Sync time out
                    SX1278SetTimeout(RXTimeoutTimer, NULL, 0);
                    // Clear Irq
                    SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);

                    settings.State = RF_IDLE;
                    if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
                    {
                        RadioEvents->RxTimeout();
                    }
                    break;
                default:
                    break;
                }
                break;
            case RF_TX_RUNNING:
                switch (settings.Modem)
                {
                case MODEM_FSK:
                    // FifoEmpty interrupt
                    if ((settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes) > settings.FskPacketHandler.ChunkSize)
                    {
                        SX1278WriteFifo((RxTxBuffer + settings.FskPacketHandler.NbBytes), settings.FskPacketHandler.ChunkSize);
                        settings.FskPacketHandler.NbBytes += settings.FskPacketHandler.ChunkSize;
                    }
                    else
                    {
                        // SX1278Write the last chunk of data
                        SX1278WriteFifo(RxTxBuffer + settings.FskPacketHandler.NbBytes, settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes);
                        settings.FskPacketHandler.NbBytes += settings.FskPacketHandler.Size - settings.FskPacketHandler.NbBytes;
                    }
                    break;
                case MODEM_LORA:
                    break;
                default:
                    break;
                }
            break;
        default:
            break;
    }
}

//-----------------------------------------------------------------------------
void OnDio2Irq()
{
    switch (settings.State)
    {
        case RF_RX_RUNNING:
            switch (settings.Modem)
            {
                case MODEM_FSK:
                    // Checks if DIO4 is connected. If it is not PreambleDtected is set to true.
                    if (dioIrq[4] == NULL)
                    {
                        settings.FskPacketHandler.PreambleDetected = true;
                    }

                    if ((settings.FskPacketHandler.PreambleDetected == true) && (settings.FskPacketHandler.SyncWordDetected == false))
                    {
                        SX1278SetTimeout(RXTimeoutSyncWordTimer, NULL, 0);

                        settings.FskPacketHandler.SyncWordDetected = true;

                        settings.FskPacketHandler.RssiValue = -(SX1278Read(REG_RSSIVALUE) >> 1);

                        settings.FskPacketHandler.AfcValue = (int32_t)(double)(((uint16_t)SX1278Read(REG_AFCMSB) << 8) |
                                                                            (uint16_t)SX1278Read(REG_AFCLSB)) *
                                                                            (double)FREQ_STEP;
                        settings.FskPacketHandler.RxGain = (SX1278Read(REG_LNA) >> 5) & 0x07;
                    }
                    break;
                case MODEM_LORA:
                    if (settings.LoRa.FreqHopOn == true)
                    {
                        // Clear Irq
                        SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                        if ((RadioEvents != NULL) && (RadioEvents->FhssChangeChannel != NULL))
                        {
                            RadioEvents->FhssChangeChannel((SX1278Read(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case RF_TX_RUNNING:
            switch (settings.Modem)
            {
                case MODEM_FSK:
                    break;
                case MODEM_LORA:
                    if (settings.LoRa.FreqHopOn == true)
                    {
                        // Clear Irq
                        SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                        if ((RadioEvents != NULL) && (RadioEvents->FhssChangeChannel != NULL))
                        {
                            RadioEvents->FhssChangeChannel((SX1278Read(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

//-----------------------------------------------------------------------------
void OnDio3Irq()
{
    switch (settings.Modem)
    {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            if ((SX1278Read(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) == RFLR_IRQFLAGS_CADDETECTED)
            {
                // Clear Irq
                SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
                if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
                {
                    RadioEvents->CadDone(true);
                }
            }
            else
            {
                // Clear Irq
                SX1278Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
                if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
                {
                    RadioEvents->CadDone(false);
                }
            }
            break;
        default:
            break;
    }
}

//-----------------------------------------------------------------------------
void OnDio4Irq()
{
    switch (settings.Modem)
    {
    case MODEM_FSK:
        {
            if (settings.FskPacketHandler.PreambleDetected == false)
            {
                settings.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
void OnDio5Irq()
{
    switch (settings.Modem)
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

//-----------------------------------------------------------------------------