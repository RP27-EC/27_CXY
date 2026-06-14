#include "led_task.h"


void StartLedTask(void const *argument)
{

  for (;;)
  {
	led_work(&led);
    osDelay(1);
  }
}
