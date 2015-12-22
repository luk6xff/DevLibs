/**
 * @file    BufferedSerial.cpp
 * @brief   Software Buffer - Extends mbed Serial functionallity adding irq driven TX and RX
 * @author  sam grove
 * @version 1.0
 * @see
 *
 * Copyright (c) 2013
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "BufferedSerial.h"
#include <stdarg.h>

BufferedSerial::BufferedSerial(PinName tx, PinName rx, uint32_t buf_size, uint32_t tx_multiple, const char* name)
    : RawSerial(tx, rx) , rxbuf(buf_size), txbuf((uint32_t)(tx_multiple*buf_size))
{
    RawSerial::attach(this, &BufferedSerial::rxIrq, Serial::RxIrq);
    this->buf_size = buf_size;
    this->tx_multiple = tx_multiple;   
    return;
}

BufferedSerial::~BufferedSerial(void)
{
    RawSerial::attach(NULL, RawSerial::RxIrq);
    RawSerial::attach(NULL, RawSerial::TxIrq);

    return;
}

int BufferedSerial::readable(void)
{
    return rxbuf.available();  // note: look if things are in the buffer
}

int BufferedSerial::writeable(void)
{
    return 1;   // buffer allows overwriting by design, always true
}

int BufferedSerial::getc(void)
{
    return rxbuf;
}

int BufferedSerial::putc(int c)
{
    txbuf = (char)c;
    BufferedSerial::prime();

    return c;
}

int BufferedSerial::puts(const char *s)
{
    if (s != NULL) {
        const char* ptr = s;
    
        while(*(ptr) != 0) {
            txbuf = *(ptr++);
        }
        txbuf = '\n';  // done per puts definition
        BufferedSerial::prime();
    
        return (ptr - s) + 1;
    }
    return 0;
}

int BufferedSerial::printf(const char* format, ...)
{
    char buffer[this->buf_size];
    memset(buffer,0,this->buf_size);
    int r = 0;

    va_list arg;
    va_start(arg, format);
    r = vsprintf(buffer, format, arg);
    // this may not hit the heap but should alert the user anyways
    if(r > this->buf_size) {
        error("%s %d buffer overwrite (max_buf_size: %d exceeded: %d)!\r\n", __FILE__, __LINE__,this->buf_size,r);
        va_end(arg);
        return 0;
    }
    va_end(arg);
    r = BufferedSerial::write(buffer, r);

    return r;
}

ssize_t BufferedSerial::write(const void *s, size_t length)
{
    if (s != NULL && length > 0) {
        const char* ptr = (const char*)s;
        const char* end = ptr + length;
    
        while (ptr != end) {
            txbuf = *(ptr++);
        }
        BufferedSerial::prime();
    
        return ptr - (const char*)s;
    }
    return 0;
}


void BufferedSerial::rxIrq(void)
{
    // read from the peripheral and make sure something is available
    if(serial_readable(&_serial)) {
        rxbuf = serial_getc(&_serial); // if so load them into a buffer
    }

    return;
}

void BufferedSerial::txIrq(void)
{
    // see if there is room in the hardware fifo and if something is in the software fifo
    while(serial_writable(&_serial)) {
        if(txbuf.available()) {
            serial_putc(&_serial, (int)txbuf.get());
        } else {
            // disable the TX interrupt when there is nothing left to send
            RawSerial::attach(NULL, RawSerial::TxIrq);
            break;
        }
    }

    return;
}

void BufferedSerial::prime(void)
{
    // if already busy then the irq will pick this up
    if(serial_writable(&_serial)) {
        RawSerial::attach(NULL, RawSerial::TxIrq);    // make sure not to cause contention in the irq
        BufferedSerial::txIrq();                // only write to hardware in one place
        RawSerial::attach(this, &BufferedSerial::txIrq, RawSerial::TxIrq);
    }

    return;
}


void BufferedSerial::clearRxBuf(void)
{
    this->rxbuf.clear();
}


void BufferedSerial::clearTxBuf(void)
{
    this->txbuf.clear();
}
