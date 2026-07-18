#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "rp_config.h"
#include "rp_device_config.h"
#include "myrobot_def.h"
#include "motor.h"
#include "bmi.h"
#include "rp_math.h"


//需改 且需保证电机正方向与陀螺仪正方向相同     
#define YAW_MOTOR_ANGLE_MIDDLE      -0.67270875	//同时也是进入狗洞模式的唯一角度 2.48340917f	
#define PITCH_MOTOR_ANGLE_MIDDLE 	2.59309077f  //上正下负
#define GIMBAL_MAX_MEC_ANGEL		30.f
#define GIMBAL_MIN_MEC_ANGEL 		-8.f
#define GIMBAL_MAX_GYRO_ANGEL		(gimbal->base_info.pitch_imu_angle + 30.f - gimbal->base_info.pitch_mec_360_angle)
#define GIMBAL_MIN_GYRO_ANGEL 		(gimbal->base_info.pitch_imu_angle - gimbal->base_info.pitch_mec_360_angle - 10.f)
				

/*整车移动模式*/
typedef enum
{
	G_SLEEP, // 卸力
	G_INIT,     // 归中初始化模式，等待云台复位完成
	G_GYRO,     // 底盘跟头
	G_MEC,      // 头跟底盘
} gimbal_mode_e;

typedef enum
{
	LIFT_CTRL,
    BOARD_CTRL,
    VISION_CTRL,
    SELF_DEBUG,
}
gimbal_ctrl_type_e;

/*云台初始化信息*/
typedef struct
{
	uint8_t init_flag; // 初始化标志位，0未初始化，1初始化完成

	uint16_t init_time; // 初始化时间

	uint16_t init_time_max; // 初始化yaw、pitch归零点超时时间

	float pitchInitAngleTolerance; // pitch初始化机械角度容忍度

	float yawInitAngleTolerance; // yaw初始化机械角度容忍度

	float pitchInitSpeedTolerance; // pitch初始化机械角速度容忍度

	float yawInitSpeedTolerance; // yaw初始化机械角速度容忍度

} gimbal_init_info_t;

typedef struct
{
    /*未处理目标值*/
    float yaw_target_raw;
    float pitch_target_raw;
    float lift_target_raw;
    float lift_target_speed_raw;

	float yaw_target;
	float pitch_target;
	float lift_target;

    pid_ctrl_t yaw_gyro_outer; // yaw陀螺仪角度环外环
	pid_ctrl_t yaw_gyro_inner; // yaw陀螺仪角度环内环
	pid_ctrl_t yaw_mec_outer;	// yaw机械角度环外环
	pid_ctrl_t yaw_mec_inner;	// yaw机械角度环内环

	pid_ctrl_t pitch_gyro_outer; // pitch陀螺仪角度环外环
	pid_ctrl_t pitch_gyro_inner; // pitch陀螺仪角度环内环
	pid_ctrl_t pitch_mec_outer;  // pitch机械角度环外环
	pid_ctrl_t pitch_mec_inner;  // pitch机械角度环内环

	pid_ctrl_t lift_speed_pid;	//升降机构速度环
	pid_ctrl_t lift_angle_outer;	//位置内环
	pid_ctrl_t lift_angle_inner;	//位置外环

 } gimbal_pid_info_t;


/*云台基础信息包*/
typedef struct __attribute__((packed))
{
	

	float pitch_imu_angle;	//角度-180~180
	float pitch_imu_speed;
	float pitch_motor_angle; //-3.14~3.14
	float pitch_mec_360_angle;
	float pitch_mec_speed;
	float output_gimbal_p;

	float yaw_imu_angle;
	float yaw_imu_speed;
	float yaw_motor_angle;
	float yaw_mec_360_angle;
	float yaw_mec_speed;
	float output_gimbal_y;


	float yaw_ctrl_imu_target;//遥控器的控制目标值 -PI~PI
	float yaw_ctrl_mec_target;//-180~180
	float pitch_ctrl_mec_target;
	float pitch_ctrl_imu_target;

	float Lift_Motor_angle;
	float Lift_Motor_speed;

	float output_gimbal_L;

	float vision_yaw_angle;//视觉目标值 左正右负 -180~180
	float vision_pitch_angle;

	float gravity_f;
	
	// car_mode_e gimbal_mode; // 0机械,1陀螺,2狗洞
	// car_mode_e last_gimbal_mode;
} gimbal_base_info_t;

typedef enum
{
	LIFT_DOWN = 0,
	LIFT_UP,
	LIFT_UTD,//up to down
	LIFT_DTU,
} Lift_State_e;

typedef enum
{
	LIFT_STOP = 0,
	LIFT_SPEED,
	LIFT_ANGLE,
} Lift_Mode_e;

typedef struct
{
	Lift_State_e lift_state;	//所处位置
	Lift_State_e Last_Lift_state;	//上一处位置

	float Lift_angle_Min; // 初始限位寻找角度最小值
	float Lift_angle_Max; // 初始限位寻找角度最大值

	Lift_Mode_e Lift_mode;	//PID模式
	
	bool is_use_angle;
	bool is_find_limit;	//升降机构是否完成寻找限位

	int Lift_direction;	//转动正方向校准

	// 初始限位寻找
	uint16_t Limit_find_time; // 初始限位寻找时间
	uint16_t Limit_find_time_max; // 初始限位寻找最大时间
	float Limit_find_speed; // 初始限位寻找速度
	float Find_current; // 找到限位的电流阈值
	float current_angle;//当前角度
	float Lift_distance;	//中间行程
	float Lift_timeout;	//升降超时

	bool block_flag;
	float block_time;
	float block_time_max;

	float angle_tolerance; //升降角度复位允许误差
}Lift_t;

typedef struct gimbal_class_t
{
	int car_state;				   //失联 遥控器 键鼠
	
	Motor_DM_t *pitch_motor;       // pitch电机指针
	Motor_DM_t *yaw_motor;         // yaw电机指针
	Motor_RM_t *lift_motor;        // lift电机指针

	Lift_t Lift;    //升降相关

	gimbal_base_info_t base_info;  // 云台基础信息
	gimbal_pid_info_t  pid_info;   // 云台PID参数
	gimbal_init_info_t init_info;

	void (*init)(struct gimbal_class_t *gimbal);   // 初始化函数
	void (*work)(struct gimbal_class_t *gimbal);   // 主处理函数

	float (*all_pid_calc)(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float inner_kp, uint8_t err_cal_mode);

	Dev_Reset_State_e gimbal_reset_state;  // 云台升降复位状态
	gimbal_mode_e gimbal_mode;		   // 云台控制模式
    gimbal_ctrl_type_e ctrl_type;       //云台控制数据来源
} gimbal_t;

extern gimbal_t Gimbal;
#endif
