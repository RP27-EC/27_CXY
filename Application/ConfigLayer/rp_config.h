

#ifndef __RP_CONFIG_H

#define __RP_CONFIG_H



/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

#include "stdbool.h"

#include "string.h"

// 驱动层配置

#include "rp_driver_config.h"

// 设备层配置

#include "rp_device_config.h"

// 用户层配置

#include "rp_user_config.h"



/* Exported macro ------------------------------------------------------------*/
/*选择IMU解算算法为Mahony*/

#define IMU_USE_MAHONY 0

/*选择IMU解算算法为EKF*/

#define IMU_USE_EKF 1

/* 摩擦轮控制模式: 0=仅L/R, 1=U/L/R */

#define SHOOT_CTRL_WITH_UP_FRIC 0

/*单独调试拨盘不控制摩擦轮*/
//#define DIAL_DEBUG  

/*单独调试拨盘电机pid*/
//#define DIAL_PID

/*开启无热量限制*/
//#define TEST_NO_LIMIT_SHOOT


/*视觉调试*/
 //#define VISION_DEBUG
//0.0013 38
/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/



#endif

