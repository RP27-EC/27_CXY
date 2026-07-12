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

	uint8_t SOF; // 帧头，数据帧的起始标志

	__packed union
	{ // 状态标志位联合体（32位）

		uint32_t all_flags; // 整体32位标志值

		__packed struct
		{

			uint8_t own_color : 1; // 己方颜色

			uint8_t game_start : 1; // 比赛开始

			uint8_t is_ready : 1; // 是否允许打弹（热量够 && 复位完毕）

			uint8_t outpost_mode : 1; // 前哨模式

			uint8_t big_energy_engine_mode : 1; // 大符模式

			uint8_t small_energy_engine_mode : 1; // 小符模式

			uint8_t hero_mode : 1; // 英雄模式

			uint32_t reserved : 25; // 位6-31：可扩展

		} bit; // 按位访问的子结构

	} flag_union;

	uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性

	float yaw; // 当前yaw角

	float pitch; // 当前pitch角

	float roll; // 当前云台roll角

	float yaw_speed; // yaw轴速度

	float pitch_speed; // pitch轴速度

	uint32_t user_debug; // 用户调试信息：

	uint16_t CRC16; // 循环冗余校验，用于校验整个数据帧的完整性

} ElectricalToVisionFrame;

// typedef struct __attribute__((packed)) ElectricalToVisionFrame
// {
//     uint8_t header;        // 帧头固定 0xA5
//     uint8_t mode;          // 模式字段
//     uint8_t CRC8;          // CRC8，覆盖 header + mode
//     uint8_t is_ready;
//     uint8_t my_color;      // 我方颜色：0-red 1-blue
//     bool is_start;         // 比赛开始标志：0-否 1-是
//     float yaw;             // 云台yaw
//     float pitch;           // 云台pitch
//     float roll;            // 云台roll
//     uint16_t CRC16;        // 整帧CRC16校验
// } ElectricalToVisionFrame;
/**

 * @brief 视觉发给电控的数据帧结构体

 */

typedef struct __attribute__((packed))

{
	uint8_t SOF; // 帧头，数据帧的起始标志

	__packed union
	{ // 状态标志位联合体（32位）

		uint32_t all_flags; // 整体32位标志值

		__packed struct
		{

			uint8_t is_find_target : 1; // 用于决定是否给视觉控pitch、yaw

			uint8_t is_keep_shooting : 1; // 用于拨盘速度环还是角度环

			uint8_t is_enable_shootting : 1; // 用于是否可以打弹

			uint8_t detect_num : 4; // 锁到几号（0哨兵，6前哨，15为没锁到)

			uint32_t reserved : 25; // 保留位

		} bit; // 按位访问的子结构

	} flag_union;

	uint8_t CRC8; // 循环冗余校验，用于校验帧头部分的数据完整性

	float yaw; // 目标yaw角

	float pitch; // 目标pitch角

	uint32_t user_debug; // 用户调试信息，自定义debug

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
