/**
 ******************************************************************************
 * @file    control_task.c
 * @brief   上主控控制任务
 ******************************************************************************
 */
#include "control_task.h"
#include "vision_protocol.h"
#include "usart.h"
static void All_CAN_Send_Here(void);
void StartControlTask(void const *argument)
{
    for (;;)
    {
		#if IMU_USE_EKF
			imu_sensor.update(&imu_sensor);
		#endif
        Module_Work();
        Vision_Board_Update();
        Vision_DataTx();
        All_CAN_Send_Here();
        osDelay(1);
    }
}

void All_CAN_Send_Here(void)
{
    /*板通正常并且遥控器开控*/
   if (Board_Rx_Info.state_pkt.car_state != SLEEP_MODE && Board_HeartBeat.status == DEV_ONLINE)
   {
       RM_Group.group_set_torque(&RM_Group);
       DM_Group.group_set_torque(&DM_Group);
	   shoot.send(&shoot);
//	
   }
   else
   {
       RM_Group.group_sleep(&RM_Group);
	   RM_Group.group_set_torque(&RM_Group);
       DM_Group.group_sleep(&DM_Group);
	   DM_Group.group_set_torque(&DM_Group);
       shoot.dail_info.dail_motor->W_iqControl(shoot.dail_info.dail_motor, 0);
       shoot.dail_info.dail_motor->tx_W_cmd(shoot.dail_info.dail_motor, TORQUE_CLOSE_LOOP_ID);
   }
}
