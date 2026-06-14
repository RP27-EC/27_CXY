/**
 ******************************************************************************
 * File Name          : gimbal.c
 * Description        : 云台控制实现 — 上主控
 ******************************************************************************
 * @attention
 * Copyright (c) 2026 HMY
 ******************************************************************************
 */

#include "gimbal.h"
#include "communicate.h"
#include "rp_math.h"
#include "car.h"

#define M_PI 3.14159265358979323846f

/* Private function prototypes -----------------------------------------------*/
//初始化
static void Gimbal_Status_Update(gimbal_t *gimbal);//云台状态更新
static void Gimbal_Pid_cal(gimbal_t *gimbal);//PID更新
static void Gimbal_Info_Update(gimbal_t *gimbal);// 云台信息更新

//检查
static void Gimbal_Pid_Init(gimbal_t *gimbal);//云台PID信息初始化
static void Gimbal_target_check(gimbal_t *gimbal);//云台输出目标值检查
static void Gimbal_imu_info_update(gimbal_t *gimbal);//陀螺仪模式下角度信息更新
static void Gimbal_mec_info_update(gimbal_t *gimbal);//机械模式下角度信息更新，包括狗洞目标值特殊处理
static void Gimbal_yaw_angle_check(gimbal_t *gimbal);//角度检查
static void Gimbal_pitch_angle_limit(gimbal_t *gimbal);//角度检查

//狗洞
static void Gimbal_Find_Mec_Limit(gimbal_t *gimbal); // 云台找机械限位
static void Gimbal_Dog_reset_angle_check(gimbal_t *gimbal);//进入狗洞前的角度检查
static void Gimbal_DOG_Mode_change(gimbal_t *gimbal); // 狗洞模式切换
static void Gimbal_Dog_Angle_Limit(gimbal_t *gimbal);//狗洞移动角度限制
static void Gimbal_protect(gimbal_t *gimbal);   //云台最高级别保护
/*pubilic functions-------------------------------------------------------------*/

void Gimbal_Init(gimbal_t *gimbal); // 初始化实现
void Gimbal_Work(gimbal_t *gimbal); // 主处理函数实现

/* Public variables ---------------------------------------------------------*/

gimbal_t gimbal = {
    .pitch_motor = &dm_motor[PITCH],
    .yaw_motor = &dm_motor[YAW],
    .lift_motor = &dm_motor[LIFT],
    .init = Gimbal_Init,
    .work = Gimbal_Work,
    .all_pid_calc = &all_pid_calc,
    .gimbal_reset_state = DEV_RESET_NO,

    .init_info = {
        .init_time = 0,
        .init_time_max = 3000.f,
        .pitchInitAngleTolerance = 0.3f,
        .yawInitAngleTolerance = 0.3f,
        .yawInitSpeedTolerance = 0.2f,
        .pitchInitSpeedTolerance = 30.f,
    },

    .Lift = {
        .Lift_direction = 1, // 电机旋转正方向

        //需校准
        .Find_current = 0.0f,
        .Lift_distance = 0.0f,
        .Limit_find_speed = 0.0f,
        .Limit_find_time = 0.0f,

        //自动计算
        .Lift_angle_Max = 0.0f,
        .Lift_angle_Min = 0.0f,
    },
};

/* Private member functions ---------------------------------------------------*/
/**
 * @brief  初始化实现
 * @param  gimbal: 云台句柄
 */
void Gimbal_Init(gimbal_t *gimbal)
{
    gimbal->init = Gimbal_Init;
    gimbal->work = Gimbal_Work;

    Gimbal_Pid_Init(gimbal); // PID参数初始化
    
    //初始为机械模式，保证无论云台状态均对位
    gimbal->base_info.gimbal_mode = C_Mec;

    //初始状态默认需要向上找限位，即使在最高处
    gimbal->Lift.lift_state = LIFT_DOWN;

    //需要复位的时候陀螺仪模式底盘找头
    gimbal->gimbal_reset_state = DEV_RESET_OK;

    // 需要伸展找限位
    gimbal->init_info.init_flag = 0;
}


/**
 * @brief  主处理函数实现
 * @param  gimbal: 云台句柄
 */
void Gimbal_Work(gimbal_t *gimbal)
{
    Gimbal_Info_Update(gimbal); // 更新云台信息

    Gimbal_Status_Update(gimbal); // 根据车状态更新云台状态
    //找完限位才可继续往下走，此时yaw，pitch不动,pitch可能要做限位
    //这里需询问升降时状态

    //开控初始化
    if (gimbal->init_info.init_flag == 0 && gimbal->car_state != SLEEP_MODE)
    {
        Gimbal_Find_Mec_Limit(gimbal);

    }

    /*检查*/
    Gimbal_yaw_angle_check(gimbal);
    Gimbal_pitch_angle_limit(gimbal);

    Gimbal_Pid_cal(gimbal); // 状态机控制pid计算

    gimbal->lift_motor->tx_info->torque = gimbal->base_info.output_gimbal_L;
    gimbal->pitch_motor->tx_info->torque = gimbal->base_info.output_gimbal_p;
    gimbal->yaw_motor->tx_info->torque = gimbal->base_info.output_gimbal_y;

    Gimbal_protect(gimbal);

    
}


/*private functions*/

static void Gimbal_Status_Update(gimbal_t *gimbal)
{
    static car_mode_e Last_state;

    switch (gimbal->base_info.gimbal_mode)
    {
    // 整车状态切换前均检查云台升降状态,若状态不兑立刻切换
    case C_Mec:
        if (gimbal->Lift.lift_state == LIFT_DOWN)
            gimbal->Lift.lift_state = LIFT_DTU;
        gimbal->gimbal_mode = G_MEC;
        break;
    case C_Gyro:
        /*保证先升起云台再进入陀螺仪状态*/
        if (gimbal->Lift.lift_state != LIFT_UP)
        {
            if (gimbal->Lift.lift_state == LIFT_DOWN)
                gimbal->Lift.lift_state = LIFT_DTU;
            gimbal->gimbal_mode = G_MEC;
        }
        gimbal->gimbal_mode = G_GYRO;
        break;
    case C_Dog:
        if (gimbal->Lift.lift_state != LIFT_DOWN)
        {
            gimbal->Lift.lift_state = LIFT_UTD;
        }
        gimbal->gimbal_mode = G_MEC;
        break;
    case C_Aim:
        /*非云台升起状态下升起云台*/
        if (gimbal->Lift.lift_state != LIFT_UP)
        {
            if (gimbal->Lift.lift_state == LIFT_DOWN)
                gimbal->Lift.lift_state = LIFT_DTU;
            gimbal->gimbal_mode = G_MEC;
        }
        gimbal->gimbal_mode = G_GYRO;
        break;
    default:
        break;
    }

    //优先度最高进行检查和保护
    Gimbal_DOG_Mode_change(gimbal);
    Last_state = gimbal->base_info.gimbal_mode;
}

/**
 * @brief  云台最高优先级保护
 * @param  gimbal：云台句柄
 */
static void Gimbal_protect(gimbal_t *gimbal)
{
    if(gimbal->car_state == SLEEP_MODE)//遥控器失联
    {
        //直接睡大觉
        gimbal->lift_motor->tx_info->torque = 0;
        gimbal->pitch_motor->tx_info->torque = 0;
        gimbal->yaw_motor->tx_info->torque = 0;
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
    if (gimbal->gimbal_reset_state == DEV_RESET_OK)
    {
        switch (gimbal->Lift.lift_state)
        {
        case  LIFT_UTD:
            gimbal->Lift.Lift_mode = LIFT_ANGLE;
            gimbal->base_info.Lift_angle_target = gimbal->Lift.Lift_angle_Min;

            //到位判断
            if(fabs(gimbal->base_info.Lift_Motor_angle - gimbal->Lift.Lift_angle_Min) \
                    <= gimbal->Lift.angle_tolerance)
                gimbal->Lift.lift_state = LIFT_DOWN;
            break;

        case  LIFT_DTU:
            gimbal->Lift.Lift_mode = LIFT_ANGLE;
            gimbal->base_info.Lift_angle_target= gimbal->Lift.Lift_angle_Max;
            //到位判断
            if (fabs(gimbal->base_info.Lift_Motor_angle - gimbal->Lift.Lift_angle_Max) \
                     <= gimbal->Lift.angle_tolerance)
                gimbal->Lift.lift_state = LIFT_UP;

        default:
            break;
        }
    }
    gimbal->Lift.Last_Lift_state = gimbal->Lift.lift_state;
}

//这里假设下板会使底盘自动归位，只需检查是否到位
static void Gimbal_Dog_reset_angle_check(gimbal_t *gimbal)
{
    /*if语句构成的状态机*/
    if (gimbal->Lift.Last_Lift_state == LIFT_UP && gimbal->Lift.lift_state == LIFT_UTD)
        // 一旦需要从升起状态降下，先复位
        gimbal->gimbal_reset_state = DEV_RESET_NO;

    if (gimbal->gimbal_reset_state == DEV_RESET_NO)
        // 复位状态切陀螺仪等待追随
        gimbal->gimbal_mode = G_GYRO;

    // 检测追随结果
    if (gimbal->gimbal_reset_state == DEV_RESET_NO &&
        fabs(gimbal->base_info.yaw_motor_angle - YAW_MOTOR_ANGLE_MIDDLE) <= gimbal->init_info.yawInitAngleTolerance)
        gimbal->gimbal_reset_state = DEV_RESET_OK;

}

//ptich角度限制
static void Gimbal_pitch_angle_limit(gimbal_t *gimbal)
{
    float angle = gimbal->base_info.pitch_angle_target;
    if (gimbal->base_info.gimbal_mode == G_MEC)
    {
        if (angle >= GIMBAL_MAX_MEC_ANGEL)
            angle = GIMBAL_MAX_MEC_ANGEL;
        if (angle <= GIMBAL_MIN_MEC_ANGEL)
            angle = GIMBAL_MIN_MEC_ANGEL;
    }

    else if (gimbal->base_info.gimbal_mode == G_GYRO)
    {
        if (angle >= GIMBAL_MAX_GYRO_ANGEL)
            angle = GIMBAL_MAX_GYRO_ANGEL;
        if (angle <= GIMBAL_MIN_GYRO_ANGEL)
            angle = GIMBAL_MIN_GYRO_ANGEL;
    }
}

//云台yaw范围检查
static void Gimbal_yaw_angle_check(gimbal_t *gimbal)
{
    //机械角角度处理范围 -PI~PI     陀螺仪范围 -180~180
    //就近+过圈
    float angle = gimbal->base_info.yaw_angle_target;
    if (gimbal->base_info.gimbal_mode == G_MEC)
    {
        while (fabs(angle) >= M_PI)
            angle -= sgn(angle) *2 * M_PI;
    }
    else if (gimbal->base_info.gimbal_mode == G_GYRO)
    {
        while (my_abs(angle) >= 180.f)
            angle -= sgn(angle) * 360.f;
    }
    gimbal->base_info.yaw_angle_target = angle;

    Gimbal_target_check(gimbal);//半圈处理
}


static void Gimbal_imu_info_update(gimbal_t *gimbal)
{
    //常规以下板目标值为准
    gimbal->base_info.pitch_angle_target = gimbal->base_info.pitch_ctrl_mec_target;
    gimbal->base_info.yaw_angle_target = gimbal->base_info.yaw_ctrl_mec_target;

    gimbal->base_info.yaw_angle_measure = gimbal->base_info.yaw_imu_angle;
    gimbal->base_info.yaw_speed_measure = gimbal->base_info.yaw_imu_speed;

    gimbal->base_info.pitch_angle_measure = gimbal->base_info.pitch_imu_angle;
    gimbal->base_info.pitch_speed_measure = gimbal->base_info.pitch_imu_speed;

    if (gimbal->base_info.gimbal_mode == C_Aim)
    {
        //视觉模式下以视觉为基准
        gimbal->base_info.pitch_angle_target = gimbal->base_info.vision_pitch_angle;
        gimbal->base_info.yaw_angle_target = gimbal->base_info.vision_yaw_angle;
    }
    // 状态切换imu目标清零，看下板有没有写
    // else if (gimbal->base_info.last_gimbal_mode == G_MEC)
    // {
    //     gimbal->base_info.pitch_angle_target = 0;
    // }
    
}

static void Gimbal_mec_info_update(gimbal_t *gimbal)
{
    gimbal->base_info.pitch_angle_target = gimbal->base_info.pitch_ctrl_mec_target;
    gimbal->base_info.yaw_angle_target = gimbal->base_info.yaw_ctrl_mec_target;

    gimbal->base_info.yaw_angle_measure = gimbal->base_info.yaw_mec_360_angle;
    gimbal->base_info.yaw_speed_measure = gimbal->base_info.yaw_mec_speed;

    gimbal->base_info.pitch_angle_measure = gimbal->base_info.pitch_mec_360_angle;
    gimbal->base_info.pitch_speed_measure = gimbal->base_info.pitch_mec_speed;
}



static void Gimbal_Find_Mec_Limit(gimbal_t *gimbal){
    
    /*需补充逻辑*/
    /*图传小平台配合云台折叠逻辑*/
    if(gimbal->init_info.init_flag == 0)
    {
        gimbal->Lift.Lift_mode = LIFT_SPEED;
        gimbal->base_info.Lift_speed_target = gimbal->Lift.Limit_find_speed;
    }
    //大于电流阈值判定，找到限位
    if (gimbal->lift_motor->rx_info->torque >= gimbal->Lift.Find_current)
    {
        //记录最高位并反求最低值
        gimbal->Lift.Lift_angle_Max = gimbal->lift_motor->rx_info->motor_angle;
        gimbal->Lift.Lift_angle_Min = gimbal->Lift.Lift_angle_Max + gimbal->Lift.Lift_distance;

        gimbal->Lift.Lift_mode = gimbal->Lift.Lift_mode = LIFT_ANGLE;
        gimbal->base_info.Lift_angle_target = gimbal->Lift.Lift_angle_Max;
        gimbal->Lift.lift_state = LIFT_UP;
        gimbal->init_info.init_flag = 1;
    }
        

}

// 云台信息更新
static void Gimbal_Info_Update(gimbal_t *gimbal)
{
    //陀螺仪数据
    gimbal->base_info.yaw_imu_angle = imu_sensor.info->base_info.yaw;
    gimbal->base_info.yaw_imu_speed = imu_sensor.info->base_info.rate_yaw;
    gimbal->base_info.pitch_imu_angle = imu_sensor.info->base_info.pitch;
    gimbal->base_info.pitch_imu_speed = imu_sensor.info->base_info.rate_pitch;

    //电机数据
    gimbal->base_info.yaw_motor_angle = gimbal->yaw_motor->rx_info->motor_angle;
    gimbal->base_info.yaw_mec_360_angle = gimbal->yaw_motor->rx_info->motor_angle / (2 * PI) * 360.f;
    gimbal->base_info.yaw_mec_speed = gimbal->yaw_motor->rx_info->speed;
    gimbal->base_info.pitch_motor_angle = gimbal->pitch_motor->rx_info->motor_angle;
    gimbal->base_info.pitch_mec_360_angle = gimbal->pitch_motor->rx_info->motor_angle / (2 * PI) * 360.f;

    gimbal->base_info.yaw_ctrl_imu_target = Board_Rx_Info.gimbal_target_pkt.yaw_imu_tar;
    gimbal->base_info.yaw_ctrl_mec_target = Board_Rx_Info.gimbal_target_pkt.yaw_mec_tar;
    gimbal->base_info.pitch_ctrl_imu_target = Board_Rx_Info.gimbal_target_pkt.pitch_imu_tar;
    gimbal->base_info.pitch_ctrl_mec_target = Board_Rx_Info.gimbal_target_pkt.pitch_mec_tar;

    gimbal->base_info.Lift_Motor_angle = gimbal->lift_motor->rx_info->motor_angle_sum;
    gimbal->base_info.Lift_Motor_speed = gimbal->lift_motor->rx_info->speed;

    //整车状态数据
    gimbal->car_state = car.car_ctrl_mode;

    /*狗洞优先度最高，默认是普通下板传来的模式*/
    if (Board_Rx_Info.state_pkt.is_hole == 1)
        gimbal->base_info.gimbal_mode = C_Dog;
    // 开启视觉模式再加上找到目标给控云台
    else if(Board_Rx_Info.state_pkt.vision_mode != 0 && vision.VtoE->flag_union.bit.is_find_target == 1)
        gimbal->base_info.gimbal_mode = C_Aim;
    else
        gimbal->base_info.gimbal_mode = Board_Rx_Info.state_pkt.gimbal_mode;

    
}

static void Gimbal_Dog_Angle_Limit(gimbal_t *gimbal)
{
    //处于升降中或者狗洞进行yaw角度限制
    if (gimbal->Lift.lift_state != LIFT_UP && gimbal->gimbal_reset_state == DEV_RESET_OK)
    {
        gimbal->base_info.yaw_angle_target = YAW_MOTOR_ANGLE_MIDDLE;
        gimbal->base_info.pitch_angle_target = PITCH_MOTOR_ANGLE_MIDDLE + 3;//防止干涉加上3°
        //到位后才不抬头
        if (gimbal->Lift.lift_state == LIFT_DOWN)
            gimbal->base_info.pitch_angle_target = PITCH_MOTOR_ANGLE_MIDDLE;
    }
}

static void Gimbal_Pid_cal(gimbal_t *gimbal)
{
    /*云台状态机*/
    static gimbal_mode_e last_mode = GIMB_SLEEP;

    float target = 0;

    switch (gimbal->base_info.gimbal_mode)
    {
    case G_MEC:
        /*获取测量值和目标值*/
        Gimbal_mec_info_update(gimbal);

        //狗洞角度特殊处理
        Gimbal_Dog_Angle_Limit(gimbal);

        target = gimbal->base_info.pitch_angle_target / M_PI * 180.f;
        gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc(gimbal->pid_info.pitch_mec_outer,
                                                                 gimbal->pid_info.pitch_mec_inner, target,
                                                                 gimbal->base_info.pitch_angle_measure, gimbal->base_info.pitch_mec_speed, -1, 3);

        target = gimbal->base_info.yaw_angle_target / M_PI * 180.f;
        gimbal->base_info.output_gimbal_y = gimbal->all_pid_calc(gimbal->pid_info.yaw_mec_outer,
                                                                 gimbal->pid_info.yaw_mec_inner, target,
                                                                 gimbal->base_info.yaw_speed_measure, gimbal->base_info.yaw_mec_speed, -1, 3);
        break;
    case G_GYRO:
        // 获取目标量
        Gimbal_imu_info_update(gimbal);
        gimbal->base_info.output_gimbal_p = gimbal->all_pid_calc(gimbal->pid_info.pitch_gyro_outer,
                                                                 gimbal->pid_info.pitch_gyro_inner, gimbal->base_info.pitch_angle_target,
                                                                 gimbal->base_info.pitch_imu_angle, gimbal->base_info.pitch_imu_speed, 1, 3);
        gimbal->base_info.output_gimbal_y = gimbal->all_pid_calc(gimbal->pid_info.yaw_gyro_outer,
                                                                 gimbal->pid_info.yaw_gyro_inner, gimbal->base_info.yaw_angle_target,
                                                                 gimbal->base_info.yaw_imu_angle, gimbal->base_info.yaw_imu_speed, 1, 3);
        break;
    case GIMB_SLEEP:
        // 真正保险是最后给卸力can信息
        gimbal->base_info.output_gimbal_p = 0.f;
        gimbal->base_info.output_gimbal_y = 0.f;
        break;
    default:
        gimbal->base_info.output_gimbal_y = 0.f;
        break;
    }

    switch(gimbal->Lift.Lift_mode)
    {
        case LIFT_SPEED:
            gimbal->base_info.output_gimbal_L = gimbal->all_pid_calc(NULL,gimbal->pid_info.lift_speed_pid,
                                                                        gimbal->base_info.Lift_speed_target,0,
                                                                        gimbal->base_info.Lift_Motor_speed,1,2);
            break;
        case LIFT_ANGLE:
            gimbal->base_info.output_gimbal_L = gimbal->all_pid_calc(gimbal->pid_info.lift_angle_outer,
                                                                     gimbal->pid_info.lift_angle_inner, gimbal->base_info.Lift_angle_target,
                                                                     gimbal->base_info.Lift_Motor_angle, gimbal->base_info.Lift_Motor_speed, 1, 2);
            break;
        case LIFT_STOP:
            gimbal->base_info.output_gimbal_L = 0;
    }
}
static void Gimbal_Pid_Init(gimbal_t *gimbal)
{
    // yaw 陀螺仪外环 PID
    gimbal->pid_info.yaw_gyro_outer->kp = 30.0f;
    gimbal->pid_info.yaw_gyro_outer->ki = 0.08f;
    gimbal->pid_info.yaw_gyro_outer->kd = 0.0f;
    gimbal->pid_info.yaw_gyro_outer->integral_max = 100.0f;
    gimbal->pid_info.yaw_gyro_outer->out_max = 700.0f;

    // yaw 陀螺仪内环 PID
    gimbal->pid_info.yaw_gyro_inner->kp = 0.02f;
    gimbal->pid_info.yaw_gyro_inner->ki = 0.0f;
    gimbal->pid_info.yaw_gyro_inner->kd = 0.0f;
    gimbal->pid_info.yaw_gyro_inner->integral_max = 0.0f;
    gimbal->pid_info.yaw_gyro_inner->out_max = 12.0f;

    // pitch 陀螺仪外环 PID
    gimbal->pid_info.pitch_gyro_outer->kp = 35.0f;
    gimbal->pid_info.pitch_gyro_outer->ki = 0.13f;
    gimbal->pid_info.pitch_gyro_outer->kd = 0.0f;
    gimbal->pid_info.pitch_gyro_outer->integral_max = 500.0f;
    gimbal->pid_info.pitch_gyro_outer->out_max = 1000.0f;

    // pitch 陀螺仪内环 PID
    gimbal->pid_info.pitch_gyro_inner->kp = 0.025f;
    gimbal->pid_info.pitch_gyro_inner->ki = 0.0f;
    gimbal->pid_info.pitch_gyro_inner->kd = 0.0f;
    gimbal->pid_info.pitch_gyro_inner->integral_max = 0.0f;
    gimbal->pid_info.pitch_gyro_inner->out_max = 12.0f;

    // yaw 机械外环 PID
    gimbal->pid_info.yaw_mec_outer->kp = 80.0f;
    gimbal->pid_info.yaw_mec_outer->ki = 0.0f;
    gimbal->pid_info.yaw_mec_outer->kd = 0.0f;
    gimbal->pid_info.yaw_mec_outer->integral_max = 400.0f;
    gimbal->pid_info.yaw_mec_outer->out_max = 1000.0f;

    // yaw 机械内环 PID
    gimbal->pid_info.yaw_mec_inner->kp = 0.01f;
    gimbal->pid_info.yaw_mec_inner->ki = 0.0f;
    gimbal->pid_info.yaw_mec_inner->kd = 0.0f;
    gimbal->pid_info.yaw_mec_inner->integral_max = 0.0f;
    gimbal->pid_info.yaw_mec_inner->out_max = 12.0f;

    // pitch 机械外环 PID
    gimbal->pid_info.pitch_mec_outer->kp = 30.0f;
    gimbal->pid_info.pitch_mec_outer->ki = 0.1f;
    gimbal->pid_info.pitch_mec_outer->kd = 0.0f;
    gimbal->pid_info.pitch_mec_outer->integral_max = 500.0f;
    gimbal->pid_info.pitch_mec_outer->out_max = 1000.0f;

    // pitch 机械内环 PID
    gimbal->pid_info.pitch_mec_inner->kp = 0.01f;
    gimbal->pid_info.pitch_mec_inner->ki = 0.0f;
    gimbal->pid_info.pitch_mec_inner->kd = 0.0f;
    gimbal->pid_info.pitch_mec_inner->integral_max = 0.0f;
    gimbal->pid_info.pitch_mec_inner->out_max = 12.0f;

    gimbal->pid_info.lift_angle_inner->kp = 0.0f;
    gimbal->pid_info.lift_angle_inner->ki = 0.0f;
    gimbal->pid_info.lift_angle_inner->kd = 0.0f;
    gimbal->pid_info.lift_angle_inner->integral_max = 0.0f;
    gimbal->pid_info.lift_angle_inner->out_max = 12.0f;

    gimbal->pid_info.lift_angle_outer->kp = 0.01f;
    gimbal->pid_info.lift_angle_outer->ki = 0.0f;
    gimbal->pid_info.lift_angle_outer->kd = 0.0f;
    gimbal->pid_info.lift_angle_outer->integral_max = 0.0f;
    gimbal->pid_info.lift_angle_outer->out_max = 12.0f;

    gimbal->pid_info.lift_speed_pid->kp = 0.01f;
    gimbal->pid_info.lift_speed_pid->ki = 0.0f;
    gimbal->pid_info.lift_speed_pid->kd = 0.0f;
    gimbal->pid_info.lift_speed_pid->integral_max = 0.0f;
    gimbal->pid_info.lift_speed_pid->out_max = 12.0f;
}

//范围检查后再传入PID
static void Gimbal_target_check(gimbal_t *gimbal)
{
    gimbal->base_info.yaw_angle_target = motor_half_cycle(gimbal->base_info.yaw_angle_target, 360.0f);

    //pitch大概率用不上
    gimbal->base_info.pitch_angle_target = motor_half_cycle(gimbal->base_info.pitch_angle_target, 360.0f);

}
