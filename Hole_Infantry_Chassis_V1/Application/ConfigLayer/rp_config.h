
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "string.h"
// 驱动层配置
#include "rp_driver_config.h"
// 设备层配置
#include "rp_device_config.h"
// 用户层配置
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/

//电容开关
#define   CAP_SWITCH             0
//功率限制开关
#define   POWER_LIMIT_SWITCH     0
//底盘开关
#define   CHASSIS_SWITCH         1
//云台开关，断头模式
#define   GIMBAL_SWITCH          0
//发射开关
#define   LAUNCH_SWITCH          0
//打滑处理开关
#define   SLIP_SWITCH            0
//小陀螺模式,0是常速，1是变速，2是超级变速
#define   TURN_MODE              0













/*选择IMU解算算法为Mahony*/
#define IMU_USE_MAHONY  0
/*选择IMU解算算法为EKF*/
#define IMU_USE_EKF 	1


/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#endif

