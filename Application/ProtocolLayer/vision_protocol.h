/**

 * @file vision_protocol.h

 * @author Isaac

 * @brief 视觉通信协议

 * @version 0.1

 * @date 2023-11-21

 *

 * @copyright Copyright (c) 2023

 *

 */

#ifndef __VISION_PROTOCOL_H

#define __VISION_PROTOCOL_H

#include "gimbal.h"

#include "rp_config.h"

#include "led.h"

#define VISION_OFFLINE_CNT_MAX (80) // 离线最大计数(ms)

/**

* @brief 电控发给视觉的数据帧结构体

*/

typedef struct __attribute__((packed))
{
	uint8_t SOF;        // 帧头固定 0xA5

    uint8_t mode;          // 1-自瞄 2-小符 3-大符 4-转动前哨 5-停转前哨 6-基地

	uint8_t CRC8;			 // 循环冗余校验，用于校验帧头部分的数据完整性

	uint8_t is_ready;		// 是否允许打弹

    uint8_t my_color;      // 我方颜色：0-red 1-blue

    bool is_start;         // 比赛开始标志：0-否 1-是

	float yaw; 			// 当前yaw角

	float pitch;		 // 当前pitch角

	float roll; 		// 当前云台roll角

	uint16_t CRC16; 	// 循环冗余校验，用于校验整个数据帧的完整性

} ElectricalToVisionFrame;

/**

 * @brief 视觉发给电控的数据帧结构体

 */

typedef struct __attribute__((packed))

{
	uint8_t SOF; // 帧头，数据帧的起始标志

	uint8_t	mode; // 1-自瞄 2-小符 3-大符 4-转动前哨 5-停转前哨 6-基地

	uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性

	float yaw; // 目标yaw角

	float pitch; // 目标pitch角

	uint8_t is_find_target ; // 用于决定是否给视觉控pitch、yaw

	uint8_t is_enable_shootting ; // 用于是否可以打弹

	uint8_t is_keep_shooting ; // 用于拨盘速度环还是角度环

	uint8_t is_find_buff ; // 用于是否找到buff

	uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性

} VisionToElectricalFrame;

/**

 * @brief 视觉通信 状态结构体

 */

typedef struct __attribute__((packed))

{

	dev_work_state_t tx_state; // 发送状态

	dev_work_state_t rx_state; // 接受状态

	uint32_t send_time; // 发送间隔

	uint32_t rx_tick; // 接受到信息时的时间

	uint8_t offline_cnt; // 接受离线计数

	uint8_t offline_cnt_max; // 接受离线最大计数

} Vision_Status_t;

/**

 * @brief 时间戳信息

 *

 */

typedef struct __attribute__((packed))

{

	uint32_t vision_shoot_timing[3];

	uint32_t shooting_begin_tick; // 开始打弹时用上一帧接受视觉的tick

} Vision_Timestamp_Info_t;

/**

 * @brief 视觉通信 总结构体

 *

 */

typedef struct __attribute__((packed))

{

	/* data */

	VisionToElectricalFrame *VtoE;

	ElectricalToVisionFrame *EtoV;

	Vision_Timestamp_Info_t *timestamp_info;

	Vision_Status_t *status;

} Vision_t;

extern Vision_t vision;

void Vison_Interrupt_Update(void);

void Vision_led_work(void);

void Vision_DataTx(void);

void Vision_DataRx(uint8_t *rxBuf);

void Vision_Board_Update(void);

void Rearrange_Vision_Timing_Buff(uint32_t *vision_timing_buff, uint8_t size);

void Vision_HearBeat(void);

#endif
