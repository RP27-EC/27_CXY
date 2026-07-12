#include "can_protocol.h"
#include "chassis.h"
#include "motor.h"
#include "communicate.h"
/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	switch (rxId)
	{
		case ID_GIMB_P:
			DM_Group.motor[PITCH]->rx(DM_Group.motor[PITCH],rxBuf);
		break;
		case ID_DIAL:
			dail_motor.get_info(&dail_motor,rxBuf);
			break;
		case ID_FRIC_L:
			rm_motor[L_Fric].rx(&rm_motor[L_Fric], rxBuf);
			break;

		case ID_FRIC_R:
			rm_motor[R_Fric].rx(&rm_motor[R_Fric], rxBuf);
			break;

		case ID_LIFT:
			rm_motor[LIFT].rx(&rm_motor[LIFT],rxBuf);
			break;

	default:
 		break;
	}
}

/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{

	switch (canId)
	{
	case ID_Board_Rx1:
		Board_Rx_01(rxBuf);
		break;
	case ID_Board_Rx2:
		Board_Rx_02(rxBuf);
		break;
	case ID_Board_Rx3:
		Board_Rx_03(rxBuf);
		break;
	case ID_Board_Rx4:
		Board_Rx_04(rxBuf);
		break;
	case ID_GIMB_Y:
		DM_Group.motor[YAW]->rx(DM_Group.motor[YAW],rxBuf);
		break;
	default:
		break;
	}
}
