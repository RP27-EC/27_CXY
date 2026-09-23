#include "driver.h"
void DRIVER_Init(void)
{
    USART5_Init();
    CAN1_Filter_Init();
}
