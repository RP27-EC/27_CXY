/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   底盘控制模块头文件（上主控不使用，已精简）
  ******************************************************************************
  */
#ifndef __chassis_H_
#define __chassis_H_

#include "communicate.h"

typedef enum {
    CHASSIS_SPEED_PID = 0,
    CHASSIS_POSITION_PID = 1,
} chassis_pid_mode_e;

typedef struct __attribute__((packed)) chassis_base_info_t {
    int16_t target_front_speed;
    int16_t target_right_speed;
    int16_t target_cycle_speed;
} chassis_base_info_t;

typedef struct chassis_class_t {
    chassis_pid_mode_e pid_mode;
    chassis_base_info_t base_info;
    void (*work)(struct chassis_class_t *chassis);
} chassis_t;

extern chassis_t chassis;

#endif
