#include "can_protocol.h"
#include "chassis.h"
volatile uint32_t chassis_motor_rx_tick[4];
volatile uint8_t chassis_motor_seen;
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
        if (rxId == 0x201U + wheel_motor[i].born_info->rxId) {
            wheel_motor[i].rx(&wheel_motor[i], rxBuf);
            chassis_motor_rx_tick[i] = HAL_GetTick();
            chassis_motor_seen |= (uint8_t)(1U << i);
            return;
        }
    }
}
void CAN2_rxDataHandler(uint32_t id, uint8_t *data) { (void)id; (void)data; }
void CAN3_rxDataHandler(uint32_t id, uint8_t *data) { (void)id; (void)data; }
