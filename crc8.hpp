#ifndef CRC8_H
#define CRC8_H

#include <avr/pgmspace.h>

extern const uint8_t crc8LookupTable[256] PROGMEM;

uint8_t calculateCRC8(uint8_t crc8, uint8_t *data, size_t size);

#endif /* CRC8_H */