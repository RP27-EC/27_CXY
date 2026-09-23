#include "device.h"
#include "rc_sensor.h"
#include "chassis.h"
void DEVICE_Init(void)
{
    rc_sensor.init(&rc_sensor);
    rm_motor_list_init();
    Chassis_Init();
}
