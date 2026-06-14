/**
 * @file  device.c
 */

/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "shoot.h"
#include "gimbal.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
	imu_sensor.init(&imu_sensor);
	rc_sensor.init(&rc_sensor);
	rm_motor_list_init();
	dm_motor_list_init();
	dail_motor.init(&dail_motor);
}

void DEVICE_Heart_Beat(void)     
{
	imu_sensor.heart_beat(&imu_sensor.work_state);
	/*实则上板并没有遥控器*/
	rc_sensor.heart_beat(&rc_sensor);
	rm_motor_list_heart_beat();
	dm_motor_list_heart_beat();
}
