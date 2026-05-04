
#ifndef __REMOTE_DRIVER_H__
#define __REMOTE_DRIVER_H__

#include "global.h"
#include "main.h"

void Remote_Init();
void UART_Receive_DMA(uint8_t *buffer, uint16_t size);
void decode_trame();

#endif

