//Ò£¿ØÆ÷×´Ì¬

#include "rc_sensor.h"
#include "chassis_minimal_config.h"
extern void rc_sensor_init(rc_sensor_t *rc);
extern void rc_sensor_update(rc_sensor_t *rc, uint8_t *data);
extern volatile uint32_t rc_last_rx_tick;
extern volatile uint8_t rc_frame_valid;
static void rc_heartbeat(rc_sensor_t *rc)
{
    uint32_t age = HAL_GetTick() - rc_last_rx_tick;
    rc->info->offline_cnt = (age >= CHASSIS_RC_TIMEOUT_MS) ? CHASSIS_RC_TIMEOUT_MS : (int16_t)age;
    rc->work_state = (rc_frame_valid && age < CHASSIS_RC_TIMEOUT_MS) ? DEV_ONLINE : DEV_OFFLINE;
}
static void rc_check(rc_sensor_t *rc) { rc_heartbeat(rc); }
rc_sensor_info_t rc_sensor_info;
rc_sensor_t rc_sensor = {
    .info = &rc_sensor_info, .init = rc_sensor_init, .update = rc_sensor_update,
    .check = rc_check, .heart_beat = rc_heartbeat,
    .work_state = DEV_OFFLINE, .id = DEV_ID_RC
};
bool RC_IsChannelReset(void)
{
    return rc_sensor_info.ch2 >= -CHASSIS_RC_DEADBAND && rc_sensor_info.ch2 <= CHASSIS_RC_DEADBAND &&
           rc_sensor_info.ch3 >= -CHASSIS_RC_DEADBAND && rc_sensor_info.ch3 <= CHASSIS_RC_DEADBAND;
}
void RC_ResetData(rc_sensor_t *rc)
{
    memset(rc->info, 0, sizeof(*rc->info));
    rc->info->s1.value = RC_SW_DOWN;
    rc->info->s2.value = RC_SW_DOWN;
}
