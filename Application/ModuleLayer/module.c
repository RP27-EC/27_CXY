#include "module.h"

void Module_Init(void)
{
    fric.init(&fric);
	gimbal.init(&gimbal);
    shoot.init(&shoot);
    car.init(&car);
}

void Module_Work(void)
{
    car.update(&car);
    gimbal.work(&gimbal);
    fric.work(&fric);
    shoot.work(&shoot);
}
