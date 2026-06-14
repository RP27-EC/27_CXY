/**
  ******************************************************************************
  * File Name          : shoot.h
  * Description        : 摩擦轮控制模块 — 上主控专用
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 RM_Project.
  ******************************************************************************
  *
  ==============================================================================
                        ##### How To Use #####
  ==============================================================================
  (#) 上主控仅负责摩擦轮速度控制，不涉及拨盘
      目标转速由下主控通过 Board_Rx_Info.fric_target_speed 下发

  (#) 初始化
      fric.init(&fric);

  (#) 主循环调用
      fric.work(&fric);

  (#) 状态机说明
      FRIC_STATE_STOPPING: 停止状态，PID控制到转速0
      FRIC_STATE_SLEEP:    睡眠状态，卸力（电流=0）
      FRIC_STATE_RUN:       运行状态，PID控制到目标转速
      FRIC_STATE_REVERSE:  反转防堵状态
  *
  ==============================================================================
                        ##### 硬件配置 #####
  ==============================================================================
  (#) 左摩擦轮: rm_motor[L_Fric]
  (#) 右摩擦轮: rm_motor[R_Fric]
  (#) 目标速度: Board_Rx_Info.fric_target_speed (来自下主控)
  (#) 遥控器在线: Board_Rx_Info.flag.bit.is_rc_online
  *
  ******************************************************************************
*/

#ifndef __shoot_H_
#define __shoot_H_

/* Includes ------------------------------------------------------------------*/
#include "rp_device_config.h"
#include "rc_sensor.h"
#include "motor.h"
#include "communicate.h"
#include "PID.h"
#include "dial.h"

/* Public types ------------------------------------------------------------*/
/**
 * @brief  摩擦轮控制状态枚举
 */
typedef enum {
    FRIC_STATE_STOPPING = 0,  // 停止状态：PID控制到转速0
    FRIC_STATE_SLEEP = 1,     // 睡眠状态：卸力（电流=0）
    FRIC_STATE_RUN = 2,       // 运行状态：PID控制到目标转速
    FRIC_STATE_REVERSE = 3,   // 反转防堵状态：反转目标转速
} fric_state_e;

/**
 * @brief  停止检测配置子结构体
 */
typedef struct {
    float speed_threshold;   // 停转判定速度阈值（低于此值认为接近停止）
    uint16_t confirm_ms;    // 停转确认持续时间(ms)
} fric_stop_cfg_t;

/**
 * @brief  堵转检测配置子结构体
 */
typedef struct {
    float current_threshold;  // 堵转判定电流阈值（大于此值认为堵转）
    float speed_threshold;   // 堵转判定速度阈值（小于此值认为堵转）
    uint16_t confirm_ms;     // 堵转确认持续时间(ms)
} fric_block_cfg_t;

/**
 * @brief  反转参数配置子结构体
 */
typedef struct {
    int16_t speed;           // 反转目标转速
    uint16_t duration_ms;    // 反转持续时间(ms)
} fric_reverse_cfg_t;

/**
 * @brief  摩擦轮方向配置子结构体
 */
typedef struct {
    uint8_t L_direction;     // 左摩擦轮方向：1=正转，0=反转
    uint8_t R_direction;     // 右摩擦轮方向：1=正转，0=反转
} fric_dir_cfg_t;

/**
 * @brief  摩擦轮根配置结构体（所有可配置参数集中管理）
 */
typedef struct {
    pid_ctrl_t L_pid;           // 左摩擦轮PID参数
    pid_ctrl_t R_pid;           // 右摩擦轮PID参数
    fric_stop_cfg_t stop;       // 停止检测配置
    fric_block_cfg_t block;     // 堵转检测配置
    fric_reverse_cfg_t reverse; // 反转参数配置
    fric_dir_cfg_t dir;         // 方向配置
} fric_cfg_t;

/**
 * @brief  摩擦轮信息结构体（外部输入解耦用）
 */
typedef struct {
    int16_t L_speed;        // 左摩擦轮转速（正方向）
    int16_t R_speed;        // 右摩擦轮转速（负方向）
      int16_t L_speed_abs;        // 左摩擦轮转速绝对值
    int16_t R_speed_abs;        // 右摩擦轮转速绝对值
    int16_t L_current;      // 左摩擦轮电流（正方向）
    int16_t R_current;      // 右摩擦轮电流（负方向）
    float target_speed;     // 目标转速（来自下主控）
    int16_t target;         // PID计算目标值（左正右负）
    uint8_t rc_online;      // 遥控器在线状态
} fric_info_t;

/**
 * @brief  摩擦轮输出结构体
 */
typedef struct {
    int16_t L_output;     // 左摩擦轮输出电流
    int16_t R_output;     // 右摩擦轮输出电流
} fric_output_t;

/**
 * @brief  摩擦轮主结构体
 */
typedef struct fric_struct_t {
    /* 电机指针 */
    Motor_RM_t *L_motor;      // 左摩擦轮电机指针
    Motor_RM_t *R_motor;      // 右摩擦轮电机指针

    /* 外部输入（解耦用） */
    fric_info_t info;         // 摩擦轮信息
    fric_cfg_t cfg;           // 配置参数
    fric_output_t output;     // 输出

    /* 状态机 */
    fric_state_e state;                   // 当前状态
    uint32_t state_enter_tick;           // 状态进入时刻(ms)
    uint32_t block_confirm_tick;          // 堵转确认计数

    /* 成员函数 */
    void (*init)(struct fric_struct_t *fric);   // 初始化函数
    void (*work)(struct fric_struct_t *fric);   // 主处理函数
} fric_t;

/* External variables -------------------------------------------------------*/
extern fric_t fric;  // 摩擦轮全局实例

/* Public function declarations ---------------------------------------------*/
void Fric_Init(fric_t *fric);   // 初始化函数（绑定函数指针）

#endif /* __shoot_H_ */
