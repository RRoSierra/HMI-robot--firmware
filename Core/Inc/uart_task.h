#ifndef UART_TASK_H
#define UART_TASK_H

#include "cmsis_os.h"

void UARTTaskFunction(void const * argument);
void UART_StartRecepcion(void);

#endif
