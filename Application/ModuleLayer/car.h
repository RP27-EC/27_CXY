/**
  ******************************************************************************
  * @file    car.h
  * @brief   整车控制模块头文件
  ******************************************************************************
  */
#ifndef __car_H_
#define __car_H_

#include "communicate.h"
#include "gimbal.h"
#include "dial.h"

/* 仅保留结构体声明，实际控制逻辑在下主控 */
typedef enum
{
    mec_CAR,
    gyro_CAR,
    offline_CAR
} Car_Move_Mode_e;

typedef struct Launch_Command_struct_t
{

    bool Enable_Shoot_Flag; // 是否允许发射标志位，决定是否sleep
    bool Shoot_Ctrl_Flag;   // 开火扳机标志位，单发和连发都需要这个标志位来控制开火电平
    uint8_t Shoot_Mode;     // 0单发，1连发

} Shoot_Flag_t;

typedef struct
{
    bool vision_mode_flag; // 视觉模式标志位，true=视觉模式，false=非视觉模式
    bool normal_vision_flag;    // 普通视觉标志位，打开视觉默认打开
    bool small_energy_engine_flag;  // 小符
    bool big_energy_engine_flag; // 大符
    bool outpost_flag;  //前哨
    bool hero_flag;     //英雄

    uint8_t is_ready_shoot; // 可以马上发射标志位,由电机状态，自瞄开启和拨盘状态共同决定
} Vision_flag_t;

typedef enum
{
    SLEEP_MODE = 0,
    RC_CTRL_MODE,
    KEY_CTRL_MODE,
} Car_Ctrl_Mode_e;

typedef struct __attribute__((packed)) car_struct {
    Car_Move_Mode_e car_move_mode;
    Dev_Reset_State_e car_reset_state;
    Car_Ctrl_Mode_e car_ctrl_mode;

    Shoot_Flag_t shoot_flag;
    Vision_flag_t vision_flag;

    Vision_t *vision;
    gimbal_t *gimbal;
    shoot_t *shoot;

    uint8_t unlock_car_flag;
    uint16_t init_cnt;
    uint16_t init_cnt_max;

    void (*init)(struct car_struct *car);
    void (*update)(struct car_struct *car);
} car_t;


extern car_t car;

#endif
