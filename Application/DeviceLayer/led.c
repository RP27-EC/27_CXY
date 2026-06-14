#include "led.h"
#include "vision_protocol.h"


/*定义LED*************************************************************/

led_t led = {

	.state = LED_OFF,

	.colour = LED_colour_red,

	.blink_fre = 5,

};

/**
 * @brief 视觉通信状态显示灯
 *        在线: 绿灯闪烁，周期500ms
 *        离线: 红灯常亮
 * @note  由 monitor_task 周期性调用
 */
void Vision_Sta_led_work(void)
{
	if (vision.status->rx_state == DEV_OFFLINE)
	{
		led.colour = LED_colour_red;
		led.state = LED_ON;
	}
	else
	{
		led.colour = LED_colour_green;
		led.state = LED_BLINK;
		led.blink_fre = 2.0f;
	}
}

/**

 * @brief c板led总控，一个时刻只有一种颜色的灯亮，只对应一种状态，每次修改最好都修改颜色和状态

 */

void led_work(led_t *led)

{

	if (led->blink_fre == 0)

	{

		led->blink_fre = 1;
	}

	uint32_t toggle_tick_cnt = 1 / led->blink_fre * 1000 / 2;

	static uint32_t last_toggle_tick;

	if (led->state == LED_OFF)

	{

		LED_RED_OFF;

		LED_BLUE_OFF;

		LED_GREEN_OFF;
	}

	switch (led->colour)

	{

	case LED_colour_red:

	{

		switch (led->state)

		{

		case LED_ON:

		{

			LED_RED_ON;

			LED_BLUE_OFF;

			LED_GREEN_OFF;

			break;
		}

		case LED_BLINK:

		{

			if (HAL_GetTick() - last_toggle_tick > toggle_tick_cnt)

			{

				LED_RED_Toggle;

				last_toggle_tick = HAL_GetTick();
			}

			LED_BLUE_OFF;

			LED_GREEN_OFF;

			break;
		}

		default:

			break;
		}

		break;
	}

	case LED_colour_blue:

	{

		switch (led->state)

		{

		case LED_ON:

		{

			LED_RED_OFF;

			LED_BLUE_ON;

			LED_GREEN_OFF;

			break;
		}

		case LED_BLINK:

		{

			if (HAL_GetTick() - last_toggle_tick > toggle_tick_cnt)

			{

				LED_BLUE_Toggle;

				last_toggle_tick = HAL_GetTick();
			}

			LED_RED_OFF;

			LED_GREEN_OFF;

			break;
		}

		default:

			break;
		}

		break;
	}

	case LED_colour_green:

	{

		switch (led->state)

		{

		case LED_ON:

		{

			LED_RED_OFF;

			LED_BLUE_OFF;

			LED_GREEN_ON;

			break;
		}

		case LED_BLINK:

		{

			if (HAL_GetTick() - last_toggle_tick > toggle_tick_cnt)

			{

				LED_GREEN_Toggle;

				last_toggle_tick = HAL_GetTick();
			}

			LED_RED_OFF;

			LED_BLUE_OFF;

			break;
		}

		default:

			break;
		}

		break;
	}

	default:

		break;
	}
}
