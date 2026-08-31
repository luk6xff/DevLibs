/**
 *  @brief:  Implementation of a SX1278 platform dependent [MBED] radio functions
 *  @author: luk6xff
 *  @email:  luszko@op.pl
 *  @date:   2019-11-15
 */

#include "sx1278-mbed.h"


/**
 * SPI Interface
 */
SPI* spi; // mosimiso, sclk
DigitalOut* nss;

/**
 * SX1278 Reset pin
 */
DigitalInOut* reset;

/**
 * SX1278 DIO pins
 */
InterruptIn* dio0;
InterruptIn* dio1;
InterruptIn* dio2; 
InterruptIn* dio3;
InterruptIn* dio4;
DigitalIn* dio5;

/**
 * Tx and Rx timers
 */
Timeout txTimeoutTimer;
Timeout rxTimeoutTimer;
Timeout rxTimeoutSyncWord;


//-----------------------------------------------------------------------------
void SX1278MbedInit(RadioEvents_t *events,
                    PinName _mosi, PinName _miso, PinName _sclk, PinName _nss, 
                    PinName _reset,
                    PinName _dio0, PinName _dio1, PinName _dio2, PinName _dio3, PinName _dio4, PinName _dio5)

{
    spi = new SPI(_mosi, _miso, _sclk);
    nss = new DigitalOut(_nss);
    reset = new DigitalInOut(_reset);
    dio0 = new InterruptIn(_dio0);
    dio1 = new InterruptIn(_dio1);
    dio2 = new InterruptIn(_dio2);
    dio3 = new InterruptIn(_dio3);
    dio4 = new InterruptIn(_dio4);
    dio5 = new DigitalIn(_dio5);

    SX1278Init(events);
}

//-----------------------------------------------------------------------------
void SX1278MbedDeInit()
{
    // IO
    SX1278IoDeInit();
    // Timers
    txTimeoutTimer.detach();
    rxTimeoutTimer.detach();
    rxTimeoutSyncWord.detach();
}

//-----------------------------------------------------------------------------
void SX1278IoInit(void)
{
    // Init SPI
    *nss = 1;    
    spi->format(8,0);   
    uint32_t frequencyToSet = 8000000;
    spi->frequency(frequencyToSet);
    SX1278DelayMs(100);
}

//-----------------------------------------------------------------------------
void SX1278IoDeInit(void)
{
    delete(spi);
    delete(nss);
    delete(reset); 
    delete(dio0);
    delete(dio1);
    delete(dio2);
    delete(dio3);
    delete(dio4);
    delete(dio5);
}


//-----------------------------------------------------------------------------
void SX1278IoIrqInit(DioIrqHandler *irqHandlers)
{
    dio0->rise(mbed::callback(irqHandlers[0]));
    dio1->rise(mbed::callback(irqHandlers[1]));
    dio2->rise(mbed::callback(irqHandlers[2]));
    dio3->rise(mbed::callback(irqHandlers[3]));
    dio4->rise(mbed::callback(irqHandlers[4]));
}

//-----------------------------------------------------------------------------
void SX1278Reset(void)
{
    reset->output();
    *reset = 0;
    SX1278DelayMs(1);
    reset->input();
    SX1278DelayMs(6);
}

//-----------------------------------------------------------------------------
void SX1278WriteBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    *nss = 0;
    spi->write(addr | 0x80);
    for(i = 0; i < size; i++)
    {
        spi->write(buffer[i]);
    }
    *nss = 1;
}

//-----------------------------------------------------------------------------
void SX1278ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    *nss = 0;
    spi->write(addr & 0x7F);
    for(i = 0; i < size; i++)
    {
        buffer[i] = spi->write(0);
    }
    *nss = 1;
}



//-----------------------------------------------------------------------------
void SX1278SetTimeout(TimeoutTimer_t timer, timeoutFuncPtr func, int timeout_ms)
{
    switch(timer)
    {
	    case RXTimeoutTimer:
        {
            if (func)
            {
                rxTimeoutTimer.attach_us(mbed::callback(func), timeout_ms*1000);
            }
            else
            {
                rxTimeoutTimer.detach();
            }
            break;
        }
        case TXTimeoutTimer:
        {
            if (func)
            {
                txTimeoutTimer.attach_us(mbed::callback(func), timeout_ms*1000);
            }
            else
            {
                txTimeoutTimer.detach();
            }
            break;
        }
        case RXTimeoutSyncWordTimer:
        {
            if (func)
            {
                rxTimeoutSyncWord.attach_us(mbed::callback(func), timeout_ms*1000);
            }
            else
            {
                rxTimeoutSyncWord.detach();
            }
            break;
        }
    }
}

//-----------------------------------------------------------------------------
void SX1278DelayMs(int ms)
{
    wait_us(ms*1000);
}

//-----------------------------------------------------------------------------