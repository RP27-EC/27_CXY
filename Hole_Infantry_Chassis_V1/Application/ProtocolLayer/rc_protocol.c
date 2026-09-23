#include "rc_protocol.h"
#include "rc_sensor.h"
#include "chassis_minimal_config.h"
volatile uint32_t rc_last_rx_tick, rc_stop_sequence;
volatile uint8_t rc_frame_valid;

void rc_sensor_init(rc_sensor_t *rc)
{
    RC_ResetData(rc);
    rc->info->offline_max_cnt = CHASSIS_RC_TIMEOUT_MS;
    rc->info->offline_cnt = CHASSIS_RC_TIMEOUT_MS;
    rc->work_state = DEV_OFFLINE;
    rc->errno = NONE_ERR;
    rc_frame_valid = 0;
    rc_last_rx_tick = 0;
    rc_stop_sequence = 0;
}

void rc_sensor_update(rc_sensor_t *rc, uint8_t *b)
{
    /* Original DBUS bit layout; retain the full -660..660 range. */
    int16_t ch[4];
    uint8_t s1 = (b[5] >> 6) & 3U;
    uint8_t s2 = (b[5] >> 4) & 3U;
    ch[0] = (int16_t)((b[0] | b[1] << 8) & 0x7ff) - 1024;
    ch[1] = (int16_t)((b[1] >> 3 | b[2] << 5) & 0x7ff) - 1024;
    ch[2] = (int16_t)((b[2] >> 6 | b[3] << 2 | b[4] << 10) & 0x7ff) - 1024;
    ch[3] = (int16_t)((b[4] >> 1 | b[5] << 7) & 0x7ff) - 1024;
    for (uint8_t i = 0; i < 4; ++i) {
        if (ch[i] < -660 || ch[i] > 660) { s1 = 0; break; }
    }
    if (s1 == 0 || s2 == 0) {
        rc_frame_valid = 0;
        ++rc_stop_sequence;
        rc->work_state = DEV_OFFLINE;
        rc->errno = DEV_DATA_ERR;
        return;
    }
    if (!rc_frame_valid || (uint32_t)(HAL_GetTick() - rc_last_rx_tick) >= CHASSIS_RC_TIMEOUT_MS ||
        s1 != RC_SW_UP || s2 != RC_SW_UP) ++rc_stop_sequence;
    rc->info->ch0 = ch[0]; rc->info->ch1 = ch[1];
    rc->info->ch2 = ch[2]; rc->info->ch3 = ch[3];
    rc->info->s1.value = s1; rc->info->s2.value = s2;
    rc->info->offline_cnt = 0;
    rc->errno = NONE_ERR;
    rc_last_rx_tick = HAL_GetTick();
    rc_frame_valid = 1;
    rc->work_state = DEV_ONLINE;
}
void USART5_rxDataHandler(uint8_t *rxBuf)
{
    rc_sensor_update(&rc_sensor, rxBuf);
}
