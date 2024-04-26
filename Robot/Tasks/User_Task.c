#include "User_Task.h"

#include "FreeRTOS.h"

void User_Task(void const * argument)
{
    while(1)
        vTaskDelay(100);
}