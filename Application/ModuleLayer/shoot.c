/**
 ******************************************************************************
 * File Name          : shoot.c
 * Description        : 摩擦轮控制实现 — 上主控专用
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RM_Project.
 ******************************************************************************
 */

#include "shoot.h"
#include "rp_math.h"

/* Private function prototypes -----------------------------------------------*/
void Fric_Init(fric_t *fric);                      // 初始化实现
void Fric_Work(fric_t *fric);                      // 主处理函数实现
static void Fric_Extern_Update(fric_t *fric);      // 更新外部输入信息
static void Fric_Check_HighPriority(fric_t *fric); // 检查全局最高优先级并跳转
static void Fric_Check_Block(fric_t *fric);        // 检查堵转并更新计数
static void Fric_Speed_Pid(fric_t *fric);          // 速度PID计算
static void Fric_Output_Local(fric_t *fric);       // 输出到电机

static void Fric_State_Stopping(fric_t *fric); // 停止状态处理
static void Fric_State_Sleep(fric_t *fric);    // 睡眠状态处理
static void Fric_State_Run(fric_t *fric);      // 运行状态处理
static void Fric_State_Reverse(fric_t *fric);  // 反转状态处理

/* Private variables ---------------------------------------------------------*/
fric_t fric = {
    .init = Fric_Init,
    .work = Fric_Work,
}; // 摩擦轮模块全局实例

/* Public functions --------------------------------------------------------*/

/* Private member functions ---------------------------------------------------*/
/**
 * @brief  初始化实现
 * @param  fric: 摩擦轮句柄
 */
void Fric_Init(fric_t *fric)
{
    fric->init = Fric_Init;
    fric->work = Fric_Work;
    // 电机指针初始化
    fric->L_motor = &rm_motor[L_Fric];
    fric->R_motor = &rm_motor[R_Fric];

    // 左摩擦轮PID参数配置
    fric->cfg.L_pid.kp = 6.0f;
    fric->cfg.L_pid.ki = 0.0f;
    fric->cfg.L_pid.kd = 0.0f;
    fric->cfg.L_pid.integral_max = 7000.0f;
    fric->cfg.L_pid.out_max = 10000.0f;

    // 右摩擦轮PID参数配置
    fric->cfg.R_pid.kp = 6.0f;
    fric->cfg.R_pid.ki = 0.0f;
    fric->cfg.R_pid.kd = 0.0f;
    fric->cfg.R_pid.integral_max = 7000.0f;
    fric->cfg.R_pid.out_max = 10000.0f;

    // 停止检测配置
    fric->cfg.stop.speed_threshold = 100.0f; // 停转判定速度阈值
    fric->cfg.stop.confirm_ms = 1000;        // 停转确认持续时间

    // 堵转检测配置
    fric->cfg.block.current_threshold = 5000.0f; // 堵转判定电流阈值
    fric->cfg.block.speed_threshold = 100.0f;    // 堵转判定速度阈值
    fric->cfg.block.confirm_ms = 400;            // 堵转确认持续时间

    // 反转参数配置
    fric->cfg.reverse.speed = -1000;     // 反转目标转速
    fric->cfg.reverse.duration_ms = 500; // 反转持续时间

    // 方向配置
    fric->cfg.dir.L_direction = 1; // 左摩擦轮正转
    fric->cfg.dir.R_direction = -1; // 右摩擦轮反转（镜像放置）

    // 初始状态
    fric->state = FRIC_STATE_STOPPING;
    fric->state_enter_tick = 0;
    fric->block_confirm_tick = 0;
}

/**
 * @brief  停止状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Stopping(fric_t *fric)
{
    uint32_t now = HAL_GetTick();

    // 两个摩擦轮转速绝对值均小于阈值
    uint8_t L_near_stop = (my_abs(fric->info.L_speed) < fric->cfg.stop.speed_threshold);
    uint8_t R_near_stop = (my_abs(fric->info.R_speed) < fric->cfg.stop.speed_threshold);

    if (L_near_stop && R_near_stop)
    {
        // 速度已降至阈值，检查持续时间
        if ((now - fric->state_enter_tick) >= fric->cfg.stop.confirm_ms)
        {
            fric->state = FRIC_STATE_SLEEP;
            fric->state_enter_tick = now;
        }
    }
    else
    {
        // 速度还没降到阈值，重置进入时刻
        fric->state_enter_tick = now;
    }

    // PID控制到转速0
    fric->info.target = 0;
    Fric_Speed_Pid(fric);
}

/**
 * @brief  睡眠状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Sleep(fric_t *fric)
{
    // 卸力（输出为0）
    fric->L_motor->tx_info->torque = 0;
    fric->R_motor->tx_info->torque = 0;

    // 目标速度不为0 且 rc_online=true，跳转至运行状态
    if ((my_abs(fric->info.target_speed) >= 1.0f) && (fric->info.rc_online != 0))
    {
        fric->state = FRIC_STATE_RUN;
        fric->state_enter_tick = HAL_GetTick();
        fric->block_confirm_tick = 0;
    }
}

/**
 * @brief  运行状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Run(fric_t *fric)
{
    // 检查堵转
    Fric_Check_Block(fric);

    // 堵转确认超时，跳转至反转状态
    if (fric->block_confirm_tick >= fric->cfg.block.confirm_ms)
    {
        fric->state = FRIC_STATE_REVERSE;
        fric->state_enter_tick = HAL_GetTick();
    }

    // 设置目标速度
    fric->info.target = (int16_t)fric->info.target_speed;
    Fric_Speed_Pid(fric);
}

/**
 * @brief  反转状态处理
 * @param  fric: 摩擦轮句柄
 */
static void Fric_State_Reverse(fric_t *fric)
{
    uint32_t now = HAL_GetTick();

    // 持续超过reverse_duration_ms，跳转回运行状态
    if ((now - fric->state_enter_tick) >= fric->cfg.reverse.duration_ms)
    {
        fric->state = FRIC_STATE_RUN;
        fric->state_enter_tick = now;
        fric->block_confirm_tick = 0;
    }

    // 反转目标转速
    fric->info.target = fric->cfg.reverse.speed;
    Fric_Speed_Pid(fric);
}

/**
 * @brief  更新外部输入信息
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Extern_Update(fric_t *fric)
{
    // 更新目标转速（来自拨盘计算量）
    fric->info.target_speed = shoot.adapt_info.final_fric_target_speed;

    // 更新遥控器在线状态
    fric->info.rc_online = Board_Rx_Info.state_pkt.car_state;//0卸力（关控）  1遥控器  2胶枪

    // 更新左摩擦轮速度
    fric->info.L_speed = fric->L_motor->rx_info->encoder_speed;

    // 更新右摩擦轮速度
    fric->info.R_speed = fric->R_motor->rx_info->encoder_speed;

    // 更新左摩擦轮电流
    fric->info.L_current = fric->L_motor->rx_info->torque_current_raw;

    // 更新右摩擦轮电流
    fric->info.R_current = fric->R_motor->rx_info->torque_current_raw;

    fric->info.L_speed_abs = my_abs(fric->info.L_speed);
    fric->info.R_speed_abs = my_abs(fric->info.R_speed);
}

/**
 * @brief  检查全局最高优先级规则并跳转
 * @param  fric: 摩擦轮句柄
 * @note   任何状态下，rc_offline或目标速度为0则跳转至停止状态
 */
static void Fric_Check_HighPriority(fric_t *fric)
{
    // 遥控器不在线 或 目标速度为0
    uint8_t trigger = (fric->info.rc_online == 0) || (my_abs(fric->info.target_speed) < 1.0f);

    if (trigger && (fric->state != FRIC_STATE_STOPPING))
    {
        fric->state = FRIC_STATE_STOPPING;
        fric->state_enter_tick = HAL_GetTick();
    }
}

/**
 * @brief  检查堵转并更新计数
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Check_Block(fric_t *fric)
{
    // 检查左摩擦轮：电流大于阈值 且 速度小于阈值
    uint8_t L_block = (my_abs(fric->info.L_current) > fric->cfg.block.current_threshold) &&
                      (my_abs(fric->info.L_speed) < fric->cfg.block.speed_threshold);

    // 检查右摩擦轮：电流大于阈值 且 速度小于阈值
    uint8_t R_block = (my_abs(fric->info.R_current) > fric->cfg.block.current_threshold) &&
                      (my_abs(fric->info.R_speed) < fric->cfg.block.speed_threshold);

    if (L_block || R_block)
    {
        fric->block_confirm_tick++;
    }
    else
    {
        fric->block_confirm_tick = 0;
    }
}

/**
 * @brief  摩擦轮速度PID计算
 * @param  fric: 摩擦轮句柄
 * @note   同时计算左右摩擦轮速度环PID，根据方向标志位决定右电机目标符号
 */
static void Fric_Speed_Pid(fric_t *fric)
{
    int16_t target = fric->info.target;

    // 左摩擦轮PID计算
    {
        pid_ctrl_t *L_pid = &fric->cfg.L_pid;
        int16_t L_target = fric->cfg.dir.L_direction ? target : -target;
        L_pid->target = (float)L_target;
        L_pid->measure = fric->L_motor->rx_info->encoder_speed;

        pid_err_cal(L_pid);
        single_pid_ctrl(L_pid);
    }

    // 右摩擦轮PID计算
    {
        pid_ctrl_t *R_pid = &fric->cfg.R_pid;

        // 根据方向标志位决定目标值
        int16_t R_target = 0;
		if(fric->cfg.dir.R_direction == 1)
			R_target = target;
		else
			R_target = -target;
        R_pid->target = (float)R_target;
        R_pid->measure = fric->R_motor->rx_info->encoder_speed;

        pid_err_cal(R_pid);
        single_pid_ctrl(R_pid);
    }
}

/**
 * @brief  输出到电机（保存到输出结构体便于外部监控）
 * @param  fric: 摩擦轮句柄
 */
static void Fric_Output_Local(fric_t *fric)
{
    // 保存到输出结构体便于外部监控
    fric->output.L_output = (int16_t)fric->cfg.L_pid.out;
    fric->output.R_output = (int16_t)fric->cfg.R_pid.out;

    // 输出到电机
    fric->L_motor->tx_info->torque =fric->output.L_output;
    fric->R_motor->tx_info->torque =fric->output.R_output;

    
}

/**
 * @brief  主处理函数实现
 * @param  fric: 摩擦轮句柄
 */
void Fric_Work(fric_t *fric)
{
    // 更新外部输入信息（解耦）
    Fric_Extern_Update(fric);

    // 全局最高优先级检查：任何状态下，rc_offline或目标速度为0则跳转停止
    Fric_Check_HighPriority(fric);

    // 根据状态执行对应动作
    switch (fric->state)
    {
    case FRIC_STATE_STOPPING:
        Fric_State_Stopping(fric);
        break;

    case FRIC_STATE_SLEEP:
        Fric_State_Sleep(fric);
        break;

    case FRIC_STATE_RUN:
        Fric_State_Run(fric);
        break;

    case FRIC_STATE_REVERSE:
        Fric_State_Reverse(fric);
        break;

    default:
        fric->state = FRIC_STATE_STOPPING;
        fric->state_enter_tick = HAL_GetTick();
        break;
    }

    // 输出到电机输出接口变量，不含can发送
    Fric_Output_Local(fric);
}
