/* Includes ------------------------------------------------------------------*/
#include "motor.h"

extern CAN_HandleTypeDef hcan1;

/*yaw pitch DM4310x2
/ fric 3508x2
/ dial LK4005
/ lift 2006x1
*/

/*DM START*/
Motor_DM_Born_Info_t Pitch_Born_Info = {
    .txId = 0x01,
    .hcan = &hcan1,
};

Motor_DM_Rx_Info_t Pitch_Rx_Info;
Motor_DM_Tx_Info_t Pitch_Tx_Info;
Motor_DM_State_t Pitch_State;
Motor_DM_Ctrl_Info_t Pitch_Ctrl;


Motor_DM_Born_Info_t Yaw_Born_Info = {
    .txId = 0x02,
    .hcan = &hcan1,
};

Motor_DM_Rx_Info_t Yaw_Rx_Info;
Motor_DM_Tx_Info_t Yaw_Tx_Info;
Motor_DM_State_t Yaw_State;
Motor_DM_Ctrl_Info_t Yaw_Ctrl;


Motor_DM_t dm_motor[] = 
{
	[YAW]=
	{
		.born_info = &Yaw_Born_Info,
		.rx_info = &Yaw_Rx_Info,
		.tx_info = &Yaw_Tx_Info,
		.state = &Yaw_State,
		.ctrl = &Yaw_Ctrl,
		.single_init = &DM_Single_Motor_Init,
		.type = dm_4310,
	},
	
	[PITCH]= 
	{
		.born_info = &Pitch_Born_Info,
		.rx_info = &Pitch_Rx_Info,
		.tx_info = &Pitch_Tx_Info,
		.state = &Pitch_State,
		.ctrl = &Pitch_Ctrl,
		.single_init = &DM_Single_Motor_Init,
		.type = dm_4310,
	},
};


Motor_DM_Group_t DM_Group = {
    .motor[PITCH] = &dm_motor[PITCH],
    .motor[YAW] = &dm_motor[YAW],
    .motor[2] = NULL,
    .motor[3] = NULL,
    .motor_num = 3,
    .group_init = Group_Motor_Init,
};
/*DM END*/

/*RM START*/
Motor_RM_Born_Info_t R_Fric_Born = {
    .rxId = 1,
    .hcan = &hcan1,
    .type = _3508_Single,
    .stdId = 0x200,
};

Motor_RM_Tx_Info_t R_Fric_Tx;
Motor_RM_State_t R_Fric_State;
Motor_RM_Rx_Info_t R_Fric_Rx;

pid_ctrl_t R_Fric_Speed_Ctrl ;

Motor_RM_Ctrl_Info_t R_Fric_Ctrl = {
    .speed_ctrl = &R_Fric_Speed_Ctrl,
};

Motor_RM_Born_Info_t L_Fric_Born = {
    .rxId = 0,
    .hcan = &hcan1,
    .type = _3508_Single,
    .stdId = 0x200,
};

Motor_RM_Tx_Info_t L_Fric_Tx;
Motor_RM_State_t L_Fric_State;
Motor_RM_Rx_Info_t L_Fric_Rx;

pid_ctrl_t L_Fric_Speed_Ctrl ;

Motor_RM_Ctrl_Info_t L_Fric_Ctrl = {
    .speed_ctrl = &L_Fric_Speed_Ctrl,
};


Motor_RM_Born_Info_t Dial_Born = {
    .rxId = 2,
    .hcan = &hcan1,
    .type = _2006_Single,
    .stdId = 0x200,
};

Motor_RM_Tx_Info_t Dial_Tx;
Motor_RM_State_t Dial_State;
Motor_RM_Rx_Info_t Dial_Rx;

pid_ctrl_t Dial_Speed_Ctrl;

Motor_RM_Ctrl_Info_t Dial_Ctrl = {
    .speed_ctrl = &Dial_Speed_Ctrl,
};


Motor_RM_Born_Info_t Lift_Born = {
    .rxId = 2,
    .hcan = &hcan1,
    .type = _2006_Single,
    .stdId = 0x1FF,
};

Motor_RM_Tx_Info_t Lift_Tx;
Motor_RM_State_t Lift_State;
Motor_RM_Rx_Info_t Lift_Rx;

pid_ctrl_t Lift_Speed_Ctrl;

Motor_RM_Ctrl_Info_t Lift_Ctrl = {
    .speed_ctrl = &Lift_Speed_Ctrl,
};

Motor_RM_t rm_motor[] = {
    [R_Fric] = {
        .born_info = &R_Fric_Born,
        .rx_info = &R_Fric_Rx,
        .tx_info = &R_Fric_Tx,
        .state = &R_Fric_State,
        .single_init = RM_Motor_Init,
        .ctrl = &R_Fric_Ctrl,
    },
    [L_Fric] = {
        .born_info = &L_Fric_Born,
        .rx_info = &L_Fric_Rx,
        .tx_info = &L_Fric_Tx,
        .state = &L_Fric_State,
        .single_init = RM_Motor_Init,
        .ctrl = &L_Fric_Ctrl,
    },

    [Dial] = {
        .born_info = &Dial_Born,
        .rx_info = &Dial_Rx,
        .tx_info = &Dial_Tx,
        .state = &Dial_State,
        .single_init = RM_Motor_Init,
        .ctrl = &Dial_Ctrl,
    },
    [LIFT] = {
        .born_info = &Lift_Born,
        .rx_info = &Lift_Rx,
        .tx_info = &Lift_Tx,
        .state = &Lift_State,
        .single_init = RM_Motor_Init,
        .ctrl = &Lift_Ctrl,
    }
};

Motor_RM_Group_t RM_Group = {
    .motor[R_Fric] = &rm_motor[R_Fric],
    .motor[L_Fric] = &rm_motor[L_Fric],
    .motor[Dial] = &rm_motor[Dial],
    .motor[LIFT] = &rm_motor[LIFT],
    .stdId = 0x200,
    .hcan = &hcan1,
    .group_init = RM_Group_Motor_Init,
};



/*LK4005 START*/

static pid_ctrl_t dail_speed =
    {
        .kp = 0.15f,
        .ki = 0.2f,
        .kd = 0.f,
        .integral_max = 500.f,
        .out_max = 1000.f,
};
static pid_ctrl_t dail_position_out =
    {
        .kp = 0.10,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 1000000.f,
};
static pid_ctrl_t dail_position_inner =
    {
        .kp = 0.06f,
        .ki = 0.f,
        .kd = 0.f,
        .integral_max = 0.f,
        .out_max = 1000.f,
};

dail_pid_info_t dail_pid = {
    .speed = &dail_speed,
    .position_inner = &dail_position_inner,
    .position_outer = &dail_position_out,
};

KT_motor_t dail_motor = {

    .KT_motor_info = {
        .id = {
            .tx_id = ID_DIAL,
            .rx_id = ID_DIAL,
            .drive_type = M_CAN1,
            .motor_type = KT4005,
        },
        .tx_info = {
            .angle_single_Control = 0,
            .angle_single_Control_maxSpeed = 0,
            .angle_single_Control_spinDirection = 0,
            .angle_add_Control = 0,
            .angle_add_Control_maxSpeed = 0,
            .angle_sum_Control = 0,
            .angle_sum_Control_maxSpeed = 0,
            .iqControl = 0,
            .speedControl = 0,
        },
    },
    .init = KT_motor_class_init,
};

/* Exported functions --------------------------------------------------------*/
void rm_motor_list_init()
{
    RM_Group.group_init(&RM_Group);
}

void rm_motor_list_heart_beat()
{
    RM_Group.group_heartbeat(&RM_Group);
}

void dm_motor_list_init()
{
    DM_Group.group_init(&DM_Group);
}

void dm_motor_list_heart_beat()
{
    DM_Group.group_heartbeat(&DM_Group);
}
