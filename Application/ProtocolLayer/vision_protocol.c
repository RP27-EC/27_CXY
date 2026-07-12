/**

 * @file vision_protocol.c

 * @author Isaac

 * @brief 视觉通信协议，负责接受发送和心跳包

 * @version 0.1

 * @date 2023-11-21

 * @copyright Copyright (c) 2023

 *

 */

#include "car.h"

#include "vision_protocol.h"

#include "communicate.h"

#include "stdbool.h"

#include "string.h"

#include "usbd_cdc_if.h"

#include "usart.h"

#include "crc.h"

void Vision_TxTime_Calculating(void);

void Append_Vision_Timing_Buff(uint32_t *vision_timing_buff, uint8_t size, uint16_t delay);

ElectricalToVisionFrame vision_tx_info =

	{

		.SOF = 0xA5, // 帧首字节

};

VisionToElectricalFrame vision_rx_info;

Vision_Timestamp_Info_t vision_timestamp_info;

Vision_Status_t vision_status =

	{

		.offline_cnt_max = VISION_OFFLINE_CNT_MAX,

		.rx_state = DEV_OFFLINE,

		.tx_state = DEV_OFFLINE,

};

Vision_t vision =

	{
		.EtoV = &vision_tx_info,

		.VtoE = &vision_rx_info,

		.timestamp_info = &vision_timestamp_info,

		.status = &vision_status,

};

uint8_t vision_txBuf[80];

/**

 * @brief 发送时间间隔计算

 *

 * @param tx_info

 */

void Vision_TxTime_Calculating(void)

{

	static uint32_t last_tick = 0;

	uint32_t tick = HAL_GetTick(); // 记录现在的tick值

	vision.status->send_time = tick - last_tick; // 计算发送间隔

	last_tick = tick;
}

/**

 * @brief 视觉通信收

 * @param rxBuf

 * @return 收到数据：DEV_ONLINE 没有到数据：DEV_OFFLINE

 * @note

 */

void Vision_DataRx(uint8_t *rxBuf)

{

	/* 帧首字节是否为0xA5 */

	if (rxBuf[0] == 0xA5)

	{

		/* 帧头CRC8校验*/

		if (Verify_CRC8_Check_Sum(rxBuf, 6) == true)

		{

			/* 帧尾CRC16校验 */

			if (Verify_CRC16_Check_Sum(rxBuf, sizeof(VisionToElectricalFrame)) == true)

			{

				memcpy(&vision_rx_info, rxBuf, sizeof(VisionToElectricalFrame));

				vision.status->offline_cnt = 0;

				vision.status->rx_tick = HAL_GetTick(); // 记录接受到信息时的时间，好像没用到
			}
		}
	}
}

/**

 * @name    Vision_DataTx

 * @brief   视觉通信发(串口1)*/

void Vision_DataTx(void)

{

	memcpy(vision_txBuf, &vision_tx_info, sizeof(ElectricalToVisionFrame)); // 设置发送信息

	Append_CRC8_Check_Sum(vision_txBuf, 6); // 添加CRC8校验码

	Append_CRC16_Check_Sum(vision_txBuf, sizeof(ElectricalToVisionFrame)); // 添加CRC16校验码

	if (CDC_Transmit_FS(vision_txBuf, sizeof(ElectricalToVisionFrame)) == USBD_OK) // 串口发送

	{

		vision.status->tx_state = DEV_ONLINE;

		Vision_TxTime_Calculating(); // 发送时间间隔计算
	}

	else

	{

		vision.status->tx_state = DEV_OFFLINE;
	}
}

void USART1_rxDataHandler(uint8_t *rxBuf) // 后续换指针

{

	Vision_DataRx(rxBuf);
}

/*视觉上板更新*/

void Vision_Board_Update(void)

{
	vision.EtoV->flag_union.bit.big_energy_engine_mode =(car.vision_flag.big_energy_engine_flag) ? 1 : 0;
	vision.EtoV->flag_union.bit.small_energy_engine_mode = (car.vision_flag.small_energy_engine_flag) ? 1 : 0;
	vision.EtoV->flag_union.bit.is_ready = car.vision_flag.is_ready_shoot ? 1 : 0;
	vision.EtoV->flag_union.bit.outpost_mode = (car.vision_flag.outpost_flag) ? 1 : 0;
	vision.EtoV->flag_union.bit.own_color = Board_Rx_Info.state_pkt.my_color;

	vision.EtoV->yaw = Gimbal.base_info.yaw_imu_angle;

	vision.EtoV->pitch = Gimbal.base_info.pitch_imu_angle;

	vision.EtoV->pitch_speed = Gimbal.base_info.pitch_imu_speed;

 	vision.EtoV->yaw_speed = Gimbal.base_info.yaw_imu_speed;

	vision.EtoV->roll = imu_sensor.info->base_info.roll;
} 

/**

 * @name    Vision_HearBeat

 * @brief   视觉通信心跳

 * @note    由监控任务调用

 */

void Vision_HearBeat(void)

{

	vision.status->offline_cnt++;

	if (vision.status->offline_cnt >

		vision.status->offline_cnt_max)

	{

		vision.status->offline_cnt =

			vision.status->offline_cnt_max;

		vision.status->rx_state = DEV_OFFLINE;
	}

	else if (vision.status->rx_state == DEV_OFFLINE)

	{

		vision.status->rx_state = DEV_ONLINE;
	}
}

/**

 * @name    Append_Vision_Timing_Buff

 * @brief   将视觉发过来的发射延时转化为SystemTick，并放到数组里

 * @note    Vision_DataRx里调用

 */

void Append_Vision_Timing_Buff(uint32_t *vision_timing_buff, uint8_t size, uint16_t delay)

{

	if (size <= 0)

	{

		return; // 如果数组大小为0或负数，直接返回
	}

	if (delay == 0)

	{

		delay = 1;
	}

	// 将剩余的元素前移一格

	for (uint8_t i = 1; i < size; i++)

	{

		vision_timing_buff[i - 1] = vision_timing_buff[i];
	}

	// 在数组的最后一个位置添加形参的值

	vision_timing_buff[size - 1] = delay + HAL_GetTick();
}

/**

 * @name     Vision_led_work

 * @brief    不同视觉状态led不同显示

 * @note     红灯失联，绿灯闪烁表示收到信息但是没找到目标，绿灯常亮表示找到目标

 */

void Vision_led_work(void)

{

	if (vision.status->rx_state == DEV_OFFLINE)

	{

		led.colour = LED_colour_red;

		led.state = LED_ON;
	}

	else if (vision.VtoE->flag_union.bit.is_find_target == 1)

	{

		led.colour = LED_colour_green;

		led.state = LED_ON;
	}

	else if (vision.status->rx_state == DEV_ONLINE)

	{

		led.colour = LED_colour_green;

		led.state = LED_BLINK;
	}

	else

	{

		led.state = LED_OFF;
	}
}
