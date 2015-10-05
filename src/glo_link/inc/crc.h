#ifndef CRC_H_INCLUDED
#define CRC_H_INCLUDED

#include <stdint.h>

// Return cyclic-redundancy-check (CRC) value of data buffer with specified size.
// The 'init' parameter is the starting value for the CRC. Uses 0x1021 poly.
uint16_t calculate_crc(uint8_t * buffer, uint32_t size, uint16_t init);

#endif 
