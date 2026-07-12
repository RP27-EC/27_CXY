#ifndef __MOTOR_H

#define __MOTOR_H

#include "rp_config.h"

#include "can_protocol.h"

#include "rm_motor.h"

#include "KT_motor.h"

#include "HT_motor.h"

#include "DM_motor.h"

#include "motor_def.h"

#include "drv_can.h"

/*pid结构体------------------------------------------------------------*/
typedef struct dail_pid_info_struct
{

    pid_ctrl_t *speed; // 速度环

    pid_ctrl_t *position_outer; // 位置环外环

    pid_ctrl_t *position_inner; // 位置环内环

} dail_pid_info_t;

/*电机ID宏定义------------------------------------------------*/

#define ID_FRIC_L 0x201
#define ID_FRIC_R 0x202
#define ID_GIMB_P 0x11
#define ID_GIMB_Y 0x12
#define ID_LIFT 0x204
#define ID_DIAL 0x141


extern Motor_RM_Group_t RM_Group;
extern Motor_RM_t rm_motor[];
extern Motor_DM_Group_t DM_Group;
extern Motor_DM_t dm_motor[];

extern KT_motor_t dail_motor;
extern dail_pid_info_t dail_pid;
/* Exported functions --------------------------------------------------------*/

void rm_motor_list_init(void);

void rm_motor_list_heart_beat(void);

void dm_motor_list_init(void);

void dm_motor_list_heart_beat(void);

#endif
