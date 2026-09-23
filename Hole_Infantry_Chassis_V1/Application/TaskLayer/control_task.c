#include "control_task.h"
#include "chassis.h"
#include "FreeRTOS.h"
#include "task.h"
void StartCtrlTask(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    (void)argument;
    for (;;) {
        Chassis_Step();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
    }
}
