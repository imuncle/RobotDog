#ifndef SCS009_H
#define SCS009_H

#include "stm32f4xx.h"

void snycWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, int *nDat);
void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);

#endif
