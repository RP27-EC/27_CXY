/**
  ******************************************************************************
  * @file    myrobot_def.h
  * @brief   整车常量定义
  *          注意：底盘宏在下主控使用，上主控仅保留必要常量
  ******************************************************************************
  */
#ifndef __MYROBOT_DEF_H
#define __MYROBOT_DEF_H

#include "stm32f4xx_hal.h"
#include "rc_sensor.h"

/* 上主控使用 */
#define GYRO_MODE (rc_sensor.info->s2 == 3)
#define GYRO_CYCLE_MODE (rc_sensor.info->s2 == 1)
#define MEC_MODE (rc_sensor.info->s2 == 2)

/* 整车参数（供下主控使用） */
#define CAR_INIT_TIME 1000
#define CYCLE_SPEED 5000.f
#define CHASSIS_MAX_SPEED 8000.f
#define RC_MAX_CNT 660

#endif
