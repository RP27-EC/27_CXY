/**
 ******************************************************************************
 * File Name          : gimbal.c
 * Description        : 云台控制实现 — 上主控
 ******************************************************************************
 * @attention
 * Copyright (c) 2026 HMY
 ******************************************************************************
 *     狗洞部分代码说明：
 *     1.复位，云台复位目标值交由下板计算提供，初始化时为机械，后续为陀螺仪复位，仅做检测
 *     2.Gimbal_info_update() 中更新底盘狗洞标志位，非升降运动状态下可控
 *     3.communicate.c 中输出标志位is_down,是否处于最高，判断yaw是否可控
 ******************************************************************************/


#include "gimbal.h"
#include "communicate.h"
#include "rp_math.h"
#include "car.h"

#define M_PI 3.14159265358979323846f
//#define DEBUG
/* Private function prototypes -----------------------------------------------*/
static void Gimbal_info_update(gimbal_t *gimbal);//云台信息传入
static void Gimbal_state_change(gimbal_t *gimbal);//云台状态更新(视觉or板间)
static void Gimbal_angle_protect(gimbal_t *gimbal);//云台目标角度保护并传目标值到pid
static void gravity_f_cal(gimbal_t *gimbal);//重力补偿
static void Gimbal_Pid_cal(gimbal_t *gimbal);//pid更新
static void Gimbal_protect(gimbal_t *gimbal);//云台最高优先级保护

// static void Gimbal_lift_Init(gimbal_t *gimbal);//升降复位
static void Gimbal_Dog_reset_angle_check(gimbal_t *gimbal);//云台到位检查
static void Gimbal_DOG_Mode_change(gimbal_t *gimbal);//狗洞模式切换

static void Gimbal_Pid_Init(gimbal_t *gimbal);//pid更新
/*pubilic functions-------------------------------------------------------------*/

void Gimbal_Init(gimbal_t *gimbal); // 初始化实现
void Gimbal_Work(gimbal_t *gimbal); // 主处理函数实现

/* Public variables ---------------------------------------------------------*/
gimbal_t Gimbal =
    {
        .pitch_motor = &dm_motor[PITCH],
        .yaw_motor = &dm_motor[YAW],
        .lift_motor = &rm_motor[LIFT],

        .init = Gimbal_Init,
        .work = Gimbal_Work,
        .all_pid_calc = all_pid_calc,

        .ctrl_type = BOARD_CTRL,
        .gimbal_reset_state = DEV_RESET_NO,//云台复位

    .init_info = {
        .init_time = 0,
        .init_time_max = 6000.f,
        .pitchInitAngleTolerance = 1.0f,
        .yawInitAngleTolerance = 1.0f,
        .yawInitSpeedTolerance = 1.0f,
        .pitchInitSpeedTolerance = 30.f,
    },

    //不使用角度
    .Lift = {
		.is_use_angle = 1,
		
        .Lift_direction = 1, // 电机旋转正方向
        .lift_state = LIFT_DTU,

        //需调试
        .Find_current = 1800.0f,
        .Lift_distance = 3890.0f,
        .Limit_find_speed = 1200.0f,
		.block_time = 0,
		.block_time_max = 60,
        .Limit_find_time = 0,
		.Limit_find_time_max = 2000,
		
		.Lift_timeout = false,
		.is_find_limit = false,

        //自动计算
        .Lift_angle_Max = 0.0f,
        .Lift_angle_Min = 0.0f,
		
		.angle_tolerance = 10.f,
    },
    };
	
	float find_target_test = 1600.f;
	float	find_target_test_2 = 8000.f;

/* Private member functions ---------------------------------------------------*/
void Gimbal_Work(gimbal_t *gimbal)
{
	/*外部信息更新*/
    Gimbal_info_update(gimbal);

	/*模式切换改变目标值来源*/
    #ifndef DEBUG
    Gimbal_state_change(gimbal);
    Gimbal_angle_protect(gimbal);
    #endif
	
	/*狗洞模式切换*/
    Gimbal_DOG_Mode_change(gimbal);

    Gimbal_Pid_cal(gimbal);

    gimbal->lift_motor->tx_info->torque = gimbal->base_info.output_gimbal_L;
    gimbal->pitch_motor->tx_info->torque = gimbal->base_info.output_gimbal_p;
    gimbal->yaw_motor->tx_info->torque = gimbal->base_info.output_gimbal_y;
    
    Gimbal_protect(gimbal);
}

int out = 0,current = 0;


/*外部数据更新*/
static void Gimbal_info_update(gimbal_t *gimbal)
{
	static dev_work_state_t last_motor_mode = DEV_OFFLINE;
	out = gimbal->lift_motor->tx_info->torque;
	current = gimbal->lift_motor->rx_info->torque_current_raw;
     //陀螺仪数据
    gimbal->base_info.yaw_imu_angle = imu_sensor.info->base_info.yaw;
    gimbal->base_info.yaw_imu_speed = imu_sensor.info->base_info.rate_yaw;
    gimbal->base_info.pitch_imu_angle = imu_sensor.info->base_info.pitch;
    gimbal->base_info.pitch_imu_speed = imu_sensor.info->base_info.rate_pitch;

    //电机数据
    gimbal->base_info.yaw_motor_angle = gimbal->yaw_motor->rx_info->motor_angle;
    gimbal->base_info.yaw_mec_360_angle = gimbal->yaw_motor->rx_info->motor_angle / (2 * PI) * 360.f;
    gimbal->base_info.yaw_mec_speed = gimbal->yaw_motor->rx_info->speed;
    gimbal->base_info.pitch_motor_angle = gimbal->pitch_motor->rx_info->motor_angle - PITCH_MOTOR_ANGLE_MIDDLE;
	gimbal->base_info.pitch_motor_angle = motor_half_cycle(gimbal->base_info.pitch_motor_angle , 2*PI);
    gimbal->base_info.pitch_mec_360_angle = gimbal->base_info.pitch_motor_angle / (2 * PI) * 360.f;
	gimbal->base_info.pitch_mec_speed = gimbal->pitch_motor->rx_info->speed;
	
    gimbal->base_info.yaw_ctrl_imu_target = Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar;
    gimbal->base_info.yaw_ctrl_mec_target = Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar;
    gimbal->base_info.pitch_ctrl_imu_target = Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar;
    gimbal->base_info.pitch_ctrl_mec_target = Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar;
	
	gimbal->base_info.Lift_Motor_speed = gimbal->lift_motor->rx_info->speed;
	gimbal->base_info.Lift_Motor_angle = gimbal->lift_motor->rx_info->motor_angle_sum;
	
	if(gimbal->lift_motor->state->status == DEV_ONLINE)
	{
		gimbal->Lift.current_angle = gimbal->base_info.Lift_Motor_angle = gimbal->lift_motor->rx_info->motor_angle_sum - \
									 gimbal->Lift.Lift_angle_Min;
	}
	else if(gimbal->lift_motor->state->status == DEV_ONLINE && last_motor_mode == DEV_OFFLINE)
	{
		gimbal->Lift.Lift_angle_Min = gimbal->base_info.Lift_Motor_angle - gimbal->Lift.current_angle;
		gimbal->Lift.Lift_angle_Max = gimbal->Lift.Lift_angle_Min + gimbal->Lift.Lift_distance;
	}


	gimbal->base_info.vision_pitch_angle = vision.VtoE->pitch;
	gimbal->base_info.vision_yaw_angle = vision.VtoE->yaw;
	
    static Car_Ctrl_Mode_e last_mode = SLEEP_MODE;

    /*整车控制模式切换*/
    if(car.car_ctrl_mode == SLEEP_MODE)
    {
        gimbal->init_info.init_flag = 0;
		gimbal->init_info.init_time = 0;
		gimbal->Lift.is_find_limit = false;
		gimbal->Lift.Lift_timeout = false;
		gimbal->Lift.Limit_find_time = 0;
		gimbal->Lift.Find_current = find_target_test;
		gimbal->Lift.lift_state = LIFT_UTD;
        gimbal->gimbal_reset_state = DEV_RESET_NO;
		
		gimbal->Lift.Lift_angle_Min = 0.0f;
        gimbal->Lift.Lift_angle_Max = 0.0f;
						
    }
	if(gimbal->lift_motor->state->status == DEV_ONLINE && last_motor_mode == DEV_OFFLINE)
	{
		gimbal->Lift.Lift_angle_Min = 0.0f;
        gimbal->Lift.Lift_angle_Max = 0.0f;
	}
	
	
	if(car.car_ctrl_mode == SLEEP_MODE)
        gimbal->gimbal_mode = G_SLEEP;

#ifdef DEBUG
    #else
    else if(gimbal->init_info.init_flag == 0)
        gimbal->gimbal_mode = G_INIT;
    else if(gimbal->init_info.init_flag == 1)
        {
            if(Board_Rx_Info.state_pkt.gimbal_mode == 0)
                gimbal->gimbal_mode = G_MEC;
            else if(Board_Rx_Info.state_pkt.gimbal_mode == 1)
                gimbal->gimbal_mode = G_GYRO;
        }
    
    #endif

    #ifdef DEBUG
    gimbal->ctrl_type = SELF_DEBUG;
    #else
    /*云台控制数据来源 自定义用于调试pid 板间 or 视觉*/
    if (Board_Rx_Info.state_pkt.vision_mode != 0 &&vision.VtoE->is_find_target == 1 &&  gimbal->gimbal_mode == G_GYRO)
        gimbal->ctrl_type = VISION_CTRL;
    else
        gimbal->ctrl_type = BOARD_CTRL;
    #endif


        //升降指令
        if (Board_Rx_Info.shoot_pkt.is_hole == 1 && gimbal->Lift.lift_state == LIFT_UP)
        {
            gimbal->Lift.lift_state = LIFT_UTD;
			if(gimbal->Lift.is_use_angle == 0)
			{
				gimbal->Lift.Find_current = find_target_test_2;
				gimbal->Lift.Lift_timeout = false;
				gimbal->Lift.Limit_find_time = 0;
			}
        }
        
        else if (Board_Rx_Info.shoot_pkt.is_hole == 0 && gimbal->Lift.lift_state == LIFT_DOWN)
		{
            gimbal->Lift.lift_state = LIFT_DTU;
			if(gimbal->Lift.is_use_angle == 0)
			{
				gimbal->Lift.Find_current = find_target_test_2;
				gimbal->Lift.Lift_timeout = false;
				gimbal->Lift.Limit_find_time = 0;
			}
		}


        gimbal->Lift.Last_Lift_state = gimbal->Lift.lift_state;
        last_mode = car.car_ctrl_mode;
		last_motor_mode = gimbal->lift_motor->state->status;
		

}

static void Gimbal_state_change(gimbal_t *gimbal)
{
    switch (gimbal->ctrl_type)
    {
    case  BOARD_CTRL:
        if(gimbal->gimbal_mode == G_MEC || gimbal->gimbal_mode == G_INIT)
        {
            gimbal->pid_info.pitch_target_raw = gimbal->base_info.pitch_ctrl_mec_target/M_PI*180.f;
            gimbal->pid_info.yaw_target_raw = gimbal->base_info.yaw_ctrl_mec_target/M_PI*180.f;
        }
        else if(gimbal->gimbal_mode == G_GYRO)
        {
            gimbal->pid_info.pitch_target_raw = gimbal->base_info.pitch_ctrl_imu_target;
            gimbal->pid_info.yaw_target_raw = gimbal->base_info.yaw_ctrl_imu_target;
        }
        break;
    
    case VISION_CTRL:  
		if(vision.status->rx_state == DEV_ONLINE)
		{
			gimbal->pid_info.pitch_target_raw = gimbal->base_info.vision_pitch_angle;
			gimbal->pid_info.yaw_target_raw = gimbal->base_info.vision_yaw_angle;
		}
			
		else
		{
			gimbal->pid_info.pitch_target_raw = gimbal->base_info.pitch_ctrl_imu_target;
            gimbal->pid_info.yaw_target_raw = gimbal->base_info.yaw_ctrl_imu_target;
		}
        break;

    //没有设定目标值（自己改）
    case SELF_DEBUG:
        
        break;
    default:
        break;
    }
}


/*堵转检测*/
static void block_check(gimbal_t *gimbal)
{
    if (my_abs(gimbal->lift_motor->rx_info->torque_current_raw) >= gimbal->Lift.Find_current)
			{
				/*堵转判断*/
				if(gimbal->Lift.block_time >= gimbal->Lift.block_time_max)
				{
					gimbal->Lift.block_time = 0;
					
                    if(gimbal->Lift.is_use_angle == 1)
                    {
                        if(gimbal->Lift.Lift_mode == LIFT_SPEED)
                        {
                            gimbal->Lift.Lift_angle_Min = (float)gimbal->lift_motor->rx_info->motor_angle_sum + 50;
                            gimbal->Lift.Lift_angle_Max = gimbal->Lift.Lift_angle_Min + gimbal->Lift.Lift_distance;
							gimbal->Lift.Find_current = find_target_test_2;
							gimbal->Lift.block_time_max = 500;
                        }
                        if(gimbal->Lift.Lift_mode == LIFT_ANGLE)
                        {
                            gimbal->Lift.block_flag = true;
                        }
                    }
				gimbal->Lift.Lift_mode = LIFT_STOP;

				if(gimbal->Lift.lift_state == LIFT_UTD)
					gimbal->Lift.lift_state = LIFT_DOWN;
				
				else if(gimbal->Lift.lift_state == LIFT_DTU)
					gimbal->Lift.lift_state = LIFT_UP;
			
				gimbal->Lift.is_find_limit = true;
				gimbal->Lift.Limit_find_time = 0;
				}
				
				else
				gimbal->Lift.block_time ++;
			}
}

/**
 * @brief  升降结构状态切换保护
 * @param  gimbal：云台句柄
 * @note   在进行云台状态切换时需优先检查并考虑升降机构状态并进行状态切换
 */
static void Gimbal_DOG_Mode_change(gimbal_t *gimbal)
{
    //对位下降状态机
    Gimbal_Dog_reset_angle_check(gimbal);

    //复位完进入升降
    if (gimbal->init_info.init_flag == 1 && gimbal->gimbal_reset_state == DEV_RESET_OK)
    {
        switch (gimbal->Lift.lift_state)
        {
        case  LIFT_UTD:
			if(gimbal->Lift.is_use_angle == 0)
			{
				gimbal->Lift.Lift_mode = LIFT_SPEED;
				gimbal->pid_info.lift_target_speed_raw = -1*gimbal->Lift.Limit_find_speed * gimbal->Lift.Lift_direction;
            }
			else
			{
				gimbal->Lift.Lift_mode = LIFT_ANGLE;
				gimbal->pid_info.lift_target_raw = gimbal->Lift.Lift_angle_Min;
				
				if(fabs(gimbal->base_info.Lift_Motor_angle - gimbal->Lift.Lift_angle_Min) \
                    <= gimbal->Lift.angle_tolerance)
                gimbal->Lift.lift_state = LIFT_DOWN;
			}
            break;

        case  LIFT_DTU:
			if(gimbal->Lift.is_use_angle == 0)
			{
				gimbal->Lift.Lift_mode = LIFT_SPEED;
				gimbal->pid_info.lift_target_speed_raw = 1*gimbal->Lift.Limit_find_speed * gimbal->Lift.Lift_direction;
			}
			else
			{
				gimbal->Lift.Lift_mode = LIFT_ANGLE;
				gimbal->pid_info.lift_target_raw = gimbal->Lift.Lift_angle_Max;
				
				if(fabs(gimbal->base_info.Lift_Motor_angle - gimbal->Lift.Lift_angle_Max) \
                    <= gimbal->Lift.angle_tolerance)
                gimbal->Lift.lift_state = LIFT_UP;
			}
           break;
        default:
            gimbal->Lift.Lift_mode = LIFT_STOP;
            break;
        }
		
		/*限位寻找时间，测参数用*/
			if(gimbal->Lift.Limit_find_time > gimbal->Lift.Limit_find_time_max \
				&&gimbal->init_info.init_flag == 0)
			{
				gimbal->Lift.Lift_mode = LIFT_STOP;
            	if(gimbal->Lift.lift_state == LIFT_UTD)
                	gimbal->Lift.lift_state = LIFT_DOWN;
            	else if(gimbal->Lift.lift_state == LIFT_DTU)
                	gimbal->Lift.lift_state = LIFT_UP;
				gimbal->Lift.Limit_find_time = 0;
				gimbal->Lift.Lift_timeout = true;
			}
			else if(gimbal->init_info.init_flag == 0)
				gimbal->Lift.Limit_find_time ++;
	}
    if(gimbal->gimbal_mode == G_SLEEP)
			gimbal->Lift.Lift_mode = LIFT_STOP;

			block_check(gimbal);
			
    gimbal->Lift.Last_Lift_state = gimbal->Lift.lift_state;
}


/**
 * @brief  云台最高优先级保护
 * @param  gimbal：云台句柄
 */
static void Gimbal_protect(gimbal_t *gimbal)
{
    if(gimbal->gimbal_mode == G_SLEEP)//遥控器失联
    {
        //直接睡大觉
        gimbal->lift_motor->tx_info->torque = 0;
        gimbal->pitch_motor->tx_info->torque = 0;
        gimbal->yaw_motor->tx_info->torque = 0;
    }
}

float yaw ;
    float pitch ;
static void Gimbal_angle_protect(gimbal_t *gimbal)
{
     yaw = gimbal->pid_info.yaw_target_raw;
     pitch = gimbal->pid_info.pitch_target_raw;

    //多圈处理
        while (my_abs(yaw) >= 180.f)
            yaw -= sgn(yaw) * 360.f;

        while (my_abs(pitch) >= 180.f)
            pitch -= sgn(pitch) * 360.f;

    //最大角度限制
    if(gimbal->gimbal_mode == G_MEC || gimbal->gimbal_mode == G_INIT)
    {
        if(pitch > GIMBAL_MAX_MEC_ANGEL)
            pitch = GIMBAL_MAX_MEC_ANGEL;
        else if(pitch < GIMBAL_MIN_MEC_ANGEL)
            pitch = GIMBAL_MIN_MEC_ANGEL;
    }
    else if(gimbal->gimbal_mode == G_GYRO)
    {
        if(pitch > GIMBAL_MAX_GYRO_ANGEL)
            pitch = GIMBAL_MAX_GYRO_ANGEL;
        else if(pitch < GIMBAL_MIN_GYRO_ANGEL)
            pitch = GIMBAL_MIN_GYRO_ANGEL;
    }

   switch (gimbal->gimbal_mode)
   {
	case G_INIT:
    case G_MEC:
          gimbal->pid_info.pitch_target = pitch;
          gimbal->pid_info.yaw_target = yaw;
          break;
     case G_GYRO:
          gimbal->pid_info.pitch_target = pitch;
          gimbal->pid_info.yaw_target = yaw;
          break;
     default:
          break;
   }
}

static void Gimbal_Init_proccess(gimbal_t *gimbal)
{
	if(gimbal->Lift.lift_state == LIFT_DOWN)
			{
				gimbal->init_info.init_flag = 1;
				gimbal->init_info.init_time = 0;
				gimbal->Lift.Find_current = find_target_test_2;
			}
     else if(my_abs(gimbal->base_info.pitch_mec_360_angle - 0.f) < gimbal->init_info.pitchInitAngleTolerance 
			&& my_abs(gimbal->base_info.yaw_mec_360_angle - YAW_MOTOR_ANGLE_MIDDLE/PI*180) < gimbal->init_info.yawInitAngleTolerance
			&& my_abs(gimbal->base_info.pitch_mec_speed) < gimbal->init_info.pitchInitSpeedTolerance
			&& my_abs(gimbal->base_info.yaw_mec_speed) < gimbal->init_info.yawInitSpeedTolerance)
			 
		{
			gimbal->gimbal_reset_state = DEV_RESET_OK;
			gimbal->Lift.Lift_mode = LIFT_SPEED;
			gimbal->pid_info.lift_target_speed_raw = -1*gimbal->Lift.Limit_find_speed * gimbal->Lift.Lift_direction;
	    }
		
		//超时仅用于测试
        if(gimbal->init_info.init_time > gimbal->init_info.init_time_max)
        {
			gimbal->gimbal_reset_state = DEV_RESET_NO;
            gimbal->init_info.init_time = 0;
			gimbal->init_info.init_flag = 1;
			gimbal->Lift.lift_state = LIFT_DOWN;
			gimbal->Lift.Lift_mode = LIFT_STOP;
        }
        else if (gimbal->init_info.init_flag == 0)
        {
            gimbal->init_info.init_time++;
        }
}

static void Gimbal_Dog_PID_cal(gimbal_t *gimbal)
{
     switch(gimbal->Lift.Lift_mode)
    {
        case LIFT_SPEED:
            gimbal->base_info.output_gimbal_L = gimbal->all_pid_calc(NULL,&gimbal->pid_info.lift_speed_pid,
                                                                        gimbal->pid_info.lift_target_speed_raw,0,
                                                                        gimbal->base_info.Lift_Motor_speed,1,2);
            break;
        case LIFT_ANGLE:
            gimbal->base_info.output_gimbal_L = gimbal->all_pid_calc(&gimbal->pid_info.lift_angle_outer,
                                                                     &gimbal->pid_info.lift_angle_inner, gimbal->pid_info.lift_target_raw, 
                                                                     gimbal->base_info.Lift_Motor_angle, gimbal->base_info.Lift_Motor_speed, -1, 0);
            break;
        case LIFT_STOP:
            gimbal->base_info.output_gimbal_L = 0;
    }        
	#ifdef VISION_DEBUG
	 gimbal->base_info.output_gimbal_L = 0;
	#endif
}

static void Gimbal_Pid_cal(gimbal_t *gimbal)
{
    /*云台状态机*/
    static gimbal_mode_e last_mode = G_SLEEP;

	gravity_f_cal(gimbal);
	
    switch (gimbal->gimbal_mode)
    {
    case G_INIT:
        Gimbal_Init_proccess(gimbal);
		
    case G_MEC:
        gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc(&gimbal->pid_info.pitch_mec_outer,
                                                                 &gimbal->pid_info.pitch_mec_inner, gimbal->pid_info.pitch_target,
                                                                 gimbal->base_info.pitch_mec_360_angle, gimbal->base_info.pitch_mec_speed, -1, 3)\
											+gimbal->base_info.gravity_f;//重力补偿

        gimbal->base_info.output_gimbal_y = gimbal->all_pid_calc(&gimbal->pid_info.yaw_mec_outer,
                                                                 &gimbal->pid_info.yaw_mec_inner, gimbal->pid_info.yaw_target,
                                                                 gimbal->base_info.yaw_mec_360_angle, gimbal->base_info.yaw_mec_speed, -1, 3);
        break;
    case G_GYRO:
        // 获取目标量
        gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc(&gimbal->pid_info.pitch_gyro_outer,
                                                                 &gimbal->pid_info.pitch_gyro_inner, gimbal->pid_info.pitch_target,
                                                                 gimbal->base_info.pitch_imu_angle, gimbal->base_info.pitch_imu_speed, -1, 3)\
																 +gimbal->base_info.gravity_f;//重力补偿;
        gimbal->base_info.output_gimbal_y = gimbal->all_pid_calc(&gimbal->pid_info.yaw_gyro_outer,
                                                                 &gimbal->pid_info.yaw_gyro_inner, gimbal->pid_info.yaw_target,
                                                                 gimbal->base_info.yaw_imu_angle, gimbal->base_info.yaw_imu_speed, -1, 3);
        break;
    case G_SLEEP:
        // 真正保险是最后给卸力can信息
        gimbal->base_info.output_gimbal_p = 0.f;
        gimbal->base_info.output_gimbal_y = 0.f;
        break;
    default:
        gimbal->base_info.output_gimbal_y = 0.f;
		gimbal->base_info.output_gimbal_p = 0.f;  
        break;
    }

    Gimbal_Dog_PID_cal(gimbal);
}

void Gimbal_Init(gimbal_t *gimbal)
{
    Gimbal_Pid_Init(gimbal);


    gimbal->pid_info.pitch_target_raw = 0.f;
    gimbal->pid_info.yaw_target_raw = YAW_MOTOR_ANGLE_MIDDLE/PI*180;

    gimbal->init_info.init_flag = 0;
}

//重力补偿计算
static void gravity_f_cal(gimbal_t *gimbal)
{
	double middle_angle = 2.678706762f - PITCH_MOTOR_ANGLE_MIDDLE;
	float k = 4.2072;
	float b = -2.496;

	gimbal->base_info.gravity_f = cos(gimbal->base_info.pitch_motor_angle - middle_angle)*k+b;
}




//这里假设下板会使底盘自动归位，只需检查是否到位
static void Gimbal_Dog_reset_angle_check(gimbal_t *gimbal)
{
    /*if语句构成的状态机*/
//    if (gimbal->gimbal_reset_state == DEV_RESET_NO && gimbal->gimbal_mode != G_INIT)
//        // 复位状态切陀螺仪等待追随
//        gimbal->gimbal_mode = G_GYRO;

    // 检测追随结果
    if (gimbal->gimbal_reset_state == DEV_RESET_NO &&
        fabs(gimbal->base_info.yaw_motor_angle - YAW_MOTOR_ANGLE_MIDDLE) <= gimbal->init_info.yawInitAngleTolerance \
        && fabs(gimbal->base_info.pitch_motor_angle) <= gimbal->init_info.pitchInitAngleTolerance)
        gimbal->gimbal_reset_state = DEV_RESET_OK;

}

static void Gimbal_Pid_Init(gimbal_t *gimbal)
{
    // yaw 陀螺仪外环 PID
    gimbal->pid_info.yaw_gyro_outer.kp = 20.0f;
    gimbal->pid_info.yaw_gyro_outer.ki = 0.05f;
    gimbal->pid_info.yaw_gyro_outer.kd = 0.0f;
    gimbal->pid_info.yaw_gyro_outer.integral_max =200.0f;//200.0f;
    gimbal->pid_info.yaw_gyro_outer.out_max = 500.0f;

    // yaw 陀螺仪内环 PID
    gimbal->pid_info.yaw_gyro_inner.kp = 0.04f;
    gimbal->pid_info.yaw_gyro_inner.ki = 0.0f;
    gimbal->pid_info.yaw_gyro_inner.kd = 0.0f;
    gimbal->pid_info.yaw_gyro_inner.integral_max = 0.0f;
    gimbal->pid_info.yaw_gyro_inner.out_max = 10.0f;

    // pitch 陀螺仪外环 PID
    gimbal->pid_info.pitch_gyro_outer.kp = 56.0f;
    gimbal->pid_info.pitch_gyro_outer.ki = 0.0f;
    gimbal->pid_info.pitch_gyro_outer.kd = 0.0f;
    gimbal->pid_info.pitch_gyro_outer.integral_max = 0.0f;//500.0f;
    gimbal->pid_info.pitch_gyro_outer.out_max = 100.0f;//100.0f;

    // pitch 陀螺仪内环 PID
    gimbal->pid_info.pitch_gyro_inner.kp = 0.03f;
    gimbal->pid_info.pitch_gyro_inner.ki = 0.0f;
    gimbal->pid_info.pitch_gyro_inner.kd = 0.0f;
    gimbal->pid_info.pitch_gyro_inner.integral_max = 0.0f;
    gimbal->pid_info.pitch_gyro_inner.out_max = 10.0f;

    // yaw 机械外环 PID
    gimbal->pid_info.yaw_mec_outer.kp = 0.6f;
    gimbal->pid_info.yaw_mec_outer.ki = 0.0f;
    gimbal->pid_info.yaw_mec_outer.kd = 0.0f;
    gimbal->pid_info.yaw_mec_outer.integral_max = 0.0f;
    gimbal->pid_info.yaw_mec_outer.out_max = 30.0f;

    // yaw 机械内环 PID
    gimbal->pid_info.yaw_mec_inner.kp = 1.3f;
    gimbal->pid_info.yaw_mec_inner.ki = 0.0f;
    gimbal->pid_info.yaw_mec_inner.kd = 0.0f;
    gimbal->pid_info.yaw_mec_inner.integral_max = 0.0f;
    gimbal->pid_info.yaw_mec_inner.out_max = 10.0f;

    // pitch 机械外环 PID
    gimbal->pid_info.pitch_mec_outer.kp = 1.2f;
    gimbal->pid_info.pitch_mec_outer.ki = 0.0f;
    gimbal->pid_info.pitch_mec_outer.kd = 0.0f;
    gimbal->pid_info.pitch_mec_outer.integral_max = 500.0f;
    gimbal->pid_info.pitch_mec_outer.out_max = 10.0f;

    // pitch 机械内环 PID
    gimbal->pid_info.pitch_mec_inner.kp = 1.3f;
    gimbal->pid_info.pitch_mec_inner.ki = 0.0f;
    gimbal->pid_info.pitch_mec_inner.kd = 0.0f;
    gimbal->pid_info.pitch_mec_inner.integral_max = 0.0f;
    gimbal->pid_info.pitch_mec_inner.out_max = 10.0f;

    //抬升机构PID
    gimbal->pid_info.lift_angle_outer.kp = 8.0f;
    gimbal->pid_info.lift_angle_outer.ki = 0.0f;
    gimbal->pid_info.lift_angle_outer.kd = 0.0f;
    gimbal->pid_info.lift_angle_outer.integral_max = 0.0f;
    gimbal->pid_info.lift_angle_outer.out_max = 1000.0f;

    gimbal->pid_info.lift_angle_inner.kp = 40.0f;
    gimbal->pid_info.lift_angle_inner.ki = 0.0f;
    gimbal->pid_info.lift_angle_inner.kd = 0.0f;
    gimbal->pid_info.lift_angle_inner.integral_max = 0.0f;
    gimbal->pid_info.lift_angle_inner.out_max = 1000.0f;

    gimbal->pid_info.lift_speed_pid.kp = 8.0f;
    gimbal->pid_info.lift_speed_pid.ki = 0.0f;
    gimbal->pid_info.lift_speed_pid.kd = 0.0f;
    gimbal->pid_info.lift_speed_pid.integral_max = 0.0f;
    gimbal->pid_info.lift_speed_pid.out_max = 300.0f;
}
