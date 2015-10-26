#ifndef CRC_H_
#define CRC_H_

#include "mbed.h"

uint16_t computeCRC16(uint8_t const *data_p, uint8_t len);

#endif /* CRC_H_ */