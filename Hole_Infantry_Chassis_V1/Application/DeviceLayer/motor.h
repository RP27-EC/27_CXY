#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "chassis_minimal_config.h"
#include "can_protocol.h"
#include "rm_motor.h"
#include "motor_def.h"
#include "drv_can.h"


typedef enum{
	WHEEL_LF= 0,
	WHEEL_LB,
	WHEEL_RF,
  WHEEL_RB, 
	WHEEL_CNT,
}Wheel_List_e;


#define   ID_WHEEL_LF    (0x200 + CHASSIS_LF_ESC_ID)
#define   ID_WHEEL_LB    (0x200 + CHASSIS_LB_ESC_ID)
#define   ID_WHEEL_RF    (0x200 + CHASSIS_RF_ESC_ID)
#define   ID_WHEEL_RB    (0x200 + CHASSIS_RB_ESC_ID)

extern Motor_RM_t wheel_motor[WHEEL_CNT];
extern Motor_RM_Group_t wheel_group;

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init(void);
void rm_motor_list_heart_beat(void);
void kt_motor_list_init(void);
void ht_motor_list_init(void);
void dm_motor_list_init(void);
uint8_t rm_motor_list_workstate(void);

#endif

