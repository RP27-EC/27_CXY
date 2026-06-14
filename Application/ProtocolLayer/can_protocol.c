#include "can_protocol.h"
#include "chassis.h"
#include "motor.h"
#include "communicate.h"
/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
//	switch (rxId)
//	{

//	case ID_FRIC_L:
//		rm_motor[L_Fric].rx(&rm_motor[L_Fric], rxBuf);
//		break;

//	case ID_FRIC_R:
//		rm_motor[R_Fric].rx(&rm_motor[R_Fric], rxBuf);
//		break;

//	case 0x11:
//		
//		break;

//		// case ID_FRIC_UP:
//		// 	rm_motor[UP_Fric].rx(&rm_motor[UP_Fric], rxBuf);
//		// 	break;

//	default:
// 		break;
//	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{

	switch (canId)
	{
	case 0xD1:
		Board_Rx_01(rxBuf);
		break;
	case 0xD2:
		Board_Rx_02(rxBuf);
		break;
	case 0xD3:
		Board_Rx_03(rxBuf);
		break;
	case 0xD4:
		Board_Rx_04(rxBuf);
		break;
	default:
		break;
	}
}
