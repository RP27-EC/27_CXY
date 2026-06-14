#include "community_task.h"

void StartCommunityTask(void const *argument)
{


	for (;;)
	{

		rc_sensor.check(&rc_sensor);		//拨轮拨杆跳变判断、数据异常检查
		rc_interrupt_update(&rc_sensor);   //鼠标值均值滤波
		keyboard_update(rc_sensor.info); // 键鼠状态检测
		osDelay(1);
	}
}
