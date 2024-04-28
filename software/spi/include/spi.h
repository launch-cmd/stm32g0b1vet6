#ifndef _SPI_H__
#define _SPI_H__

#include "stdint.h"

void spiInit();
void spiTranferByte(uint8_t b);
void spiWaitForIdle();
#endif