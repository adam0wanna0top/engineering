#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H
#include "main.h"

#define USART_RX_BUF_LENGTH     512
#define REFEREE_FIFO_BUF_LENGTH 1024


extern void Referee_Task(void const * argument);
#endif
