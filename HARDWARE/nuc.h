#ifndef __NUC_H
#define __NUC_H

#include "main.h"

void NUC_Init(void);

void NUC_Send_Data(void);

extern void USAR_UART1_IDLECallback(UART_HandleTypeDef *huart);

#endif
