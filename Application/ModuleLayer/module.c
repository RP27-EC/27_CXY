#include "module.h"

void Module_Init(void)
{
    fric.init(&fric);
	Gimbal.init(&Gimbal);
    shoot.init(&shoot);
    car.init(&car);
}

void Module_Work(void)
{
    car.update(&car);
    Gimbal.work(&Gimbal);
    fric.work(&fric);
    shoot.work(&shoot);
    Board_Tx_Update(&Board_Tx_Info);
}
