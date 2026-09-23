#include "chassis.h"
#include "chassis_minimal_config.h"
#include "rc_sensor.h"
#include <math.h>

extern volatile uint32_t rc_last_rx_tick, rc_stop_sequence;
extern volatile uint8_t rc_frame_valid;
Chassis_Debug_t chassis_debug;
static uint32_t stop_sequence;
static const float wheel_sign[4] = {
    CHASSIS_LF_SIGN, CHASSIS_LB_SIGN, CHASSIS_RF_SIGN, CHASSIS_RB_SIGN
};
/* Original inverse kinematics, order LF / LB / RF / RB, wz fixed to zero. */

//麦轮解算
static const float front_mix[4] = {-1, -1, 1, 1};
static const float left_mix[4]  = { 1, -1, 1,-1};
static void chassis_init_compat(Chassis_t *self) { (void)self; Chassis_Init(); }
static void chassis_work_compat(Chassis_t *self) { (void)self; Chassis_Step(); }
Chassis_t chassis = {.wheel = &wheel_group, .init = chassis_init_compat, .work = chassis_work_compat};

static void reset_pid(pid_ctrl_t *p)
{
    p->target = p->measure = p->err = p->last_err = 0;
    p->pout = p->iout = p->dout = p->out = p->last_dout = p->integral = 0;
}


static void zero_output(void)
{
    memset(&chassis.target, 0, sizeof(chassis.target));
    memset(&chassis.out, 0, sizeof(chassis.out));
    for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
        reset_pid(wheel_motor[i].ctrl->speed_ctrl);
        memset(wheel_motor[i].tx_info, 0, sizeof(*wheel_motor[i].tx_info));
        chassis_debug.target_rpm[i] = 0;
        chassis_debug.current_raw[i] = 0;
    }
}


//底盘初始化
void Chassis_Init(void)
{
    memset(&chassis_debug, 0, sizeof(chassis_debug));
    chassis_motor_seen = 0;
    stop_sequence = rc_stop_sequence;
    for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
        pid_ctrl_t *p = wheel_motor[i].ctrl->speed_ctrl;
        p->kp = CHASSIS_PID_KP; p->ki = CHASSIS_PID_KI; p->kd = CHASSIS_PID_KD;
        p->integral_max = CHASSIS_PID_INTEGRAL_MAX;
        p->out_max = CHASSIS_TORQUE_LIMIT_NM;
        wheel_motor[i].state->offline_cnt = wheel_motor[i].state->offline_cnt_max;
        wheel_motor[i].state->status = DEV_OFFLINE;
    }
    zero_output(); /* No CAN transmission until DRIVER_Init has started FDCAN. */
}
static float channel_normalize(int16_t v)
{
    if (v > 660) v = 660;
    if (v < -660) v = -660;
    if (v > CHASSIS_RC_DEADBAND) return (v - CHASSIS_RC_DEADBAND) / (660.0f - CHASSIS_RC_DEADBAND);
    if (v < -CHASSIS_RC_DEADBAND) return (v + CHASSIS_RC_DEADBAND) / (660.0f - CHASSIS_RC_DEADBAND);
    return 0;
}
/* Called with interrupts masked: reject stale/invalid feedback and rotor overspeed.
 * fault: 1=RC, 2=motor timeout, 3=overspeed, 4=CAN TX; 0=healthy/stopped. */

//掉线保护
static uint8_t input_fault(uint32_t now)
{
    if (!rc_frame_valid || (uint32_t)(now - rc_last_rx_tick) >= CHASSIS_RC_TIMEOUT_MS) return 1;
    for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
        if (!(chassis_motor_seen & (1U << i)) ||
            (uint32_t)(now - chassis_motor_rx_tick[i]) >= CHASSIS_MOTOR_TIMEOUT_MS) return 2;
        if (wheel_motor[i].rx_info->encoder_speed > CHASSIS_HARD_MAX_RPM ||
            wheel_motor[i].rx_info->encoder_speed < -CHASSIS_HARD_MAX_RPM) return 3;
    }
    return 0;
}


//发送电流
static HAL_StatusTypeDef send_current(void)
{
    uint8_t data[8] = {0};
    uint32_t pending = hfdcan1.Instance->TXBRP;
    /* Never leave old nonzero commands queued behind a stop command. */
    if (pending != 0U) {
        HAL_FDCAN_AbortTxRequest(&hfdcan1, pending);
        if (hfdcan1.Instance->TXBRP != 0U) return HAL_BUSY;
    }
    for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
        uint8_t slot = wheel_motor[i].born_info->rxId * 2U;
        uint16_t raw = (uint16_t)chassis_debug.current_raw[i];
        data[slot] = (uint8_t)(raw >> 8); data[slot + 1] = (uint8_t)raw;
    }
    return CAN_SendData(&hfdcan1, 0x200, data);
}


//底盘核心主循环
void Chassis_Step(void)
{
    uint32_t irq = __get_PRIMASK();
    uint32_t sequence;
    uint8_t s1, s2, centered;
    float front, left, peak = 1.0f;
    float measured[4], mixed[4];
    __disable_irq();
    rc_sensor.heart_beat(&rc_sensor);
    sequence = rc_stop_sequence;
    chassis_debug.fault = input_fault(HAL_GetTick());
    s1 = rc_sensor.info->s1.value; s2 = rc_sensor.info->s2.value;
    centered = RC_IsChannelReset();
    front = CHASSIS_FRONT_RC_SIGN * channel_normalize(rc_sensor.info->ch3);
    left = CHASSIS_LEFT_RC_SIGN * channel_normalize(rc_sensor.info->ch2);
    for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
        measured[i] = wheel_motor[i].rx_info->speed;
        chassis_debug.feedback_rpm[i] = wheel_motor[i].rx_info->encoder_speed;
        wheel_motor[i].state->status = ((chassis_motor_seen & (1U << i)) &&
            (uint32_t)(HAL_GetTick() - chassis_motor_rx_tick[i]) < CHASSIS_MOTOR_TIMEOUT_MS) ? DEV_ONLINE : DEV_OFFLINE;
    }
    __set_PRIMASK(irq);
    if (chassis_debug.fault) {
        chassis_debug.armed = chassis_debug.ready = 0;
    } else if (s1 == RC_SW_DOWN && centered) {
        chassis_debug.armed = 0; chassis_debug.ready = 1;
    } else if (s1 != RC_SW_UP || s2 != RC_SW_UP) {
        /* Allow the mechanical DOWN -> MID -> UP switch travel while centered. */
        if (chassis_debug.armed || !centered) chassis_debug.ready = 0;
        chassis_debug.armed = 0;
    } else {
        if (sequence != stop_sequence) chassis_debug.armed = chassis_debug.ready = 0;
        if (chassis_debug.ready && centered) chassis_debug.armed = 1;
    }
    stop_sequence = sequence;
    chassis.mode = chassis_debug.armed ? C_BOSS : C_SLEEP;
    if (!chassis_debug.armed) {
        zero_output();
    } else {
        for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
            mixed[i] = wheel_sign[i] * (front_mix[i] * front + left_mix[i] * left);
            if (fabsf(mixed[i]) > peak) peak = fabsf(mixed[i]);
        }
        chassis.target.front_speed = front * CHASSIS_DEFAULT_MAX_RPM / peak;
        chassis.target.left_speed = left * CHASSIS_DEFAULT_MAX_RPM / peak;
        chassis.target.cycle_speed = 0;
        for (uint8_t i = 0; i < WHEEL_CNT; ++i) {
            Motor_RM_t *m = &wheel_motor[i];
            pid_ctrl_t *p = m->ctrl->speed_ctrl;
            chassis_debug.target_rpm[i] = mixed[i] * CHASSIS_DEFAULT_MAX_RPM / peak;
            p->target = chassis_debug.target_rpm[i] * (6.28318530718f / (60.0f * _3508_REDUCT_RATIO));
            chassis.target.motor_speed[i] = p->target; /* existing driver unit: rad/s */
            p->measure = measured[i]; p->err = p->target - p->measure;
            single_pid_ctrl(p);
            m->tx_info->torque = p->out;
            m->tx_info->torque_current = p->out / _3508_TORQUE_CONSTANT;
            m->tx_info->torque_current_raw = (int16_t)(m->tx_info->torque_current * (16384.0f / _3508_MAX_CURRENT));
            chassis_debug.current_raw[i] = m->tx_info->torque_current_raw;
        }
    }
    /* Recheck immediately at the only transmit point; no stale PID output escapes. */
    irq = __get_PRIMASK(); __disable_irq();
    if (input_fault(HAL_GetTick()) || sequence != rc_stop_sequence) {
        chassis_debug.ready = 0;
    }
    if (input_fault(HAL_GetTick()) || sequence != rc_stop_sequence ||
        rc_sensor.info->s1.value != RC_SW_UP || rc_sensor.info->s2.value != RC_SW_UP) {
        chassis_debug.armed = 0;
        zero_output();
    }
    if (send_current() != HAL_OK) {
        ++chassis_debug.tx_errors;
        chassis_debug.fault = 4;
        chassis_debug.armed = chassis_debug.ready = 0;
        zero_output();
        (void)send_current(); /* next task period retries zero if bus remains unavailable */
    }
    __set_PRIMASK(irq);
}
