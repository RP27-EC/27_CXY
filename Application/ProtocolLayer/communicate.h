#ifndef __communicate_H_
#define __communicate_H_
#include "rp_device_config.h"
#include "rc_sensor.h"
#include "drv_can.h"
#include "can_protocol.h"
#include "vision_protocol.h"
#include "judge_protocol.h"
#include "gimbal.h"
#include "rp_math.h"

#define ID_Board_Tx1 0xD1
#define ID_Board_Tx2 0xD2

#define ID_Board_Rx1 0xC1
#define ID_Board_Rx2 0xC2
#define ID_Board_Rx3 0xC3
#define ID_Board_Rx4 0xC4

/*整车模式01*/
typedef struct
{
    uint8_t car_state : 2;   // 0卸力,1遥控,2键鼠  (2位)
    uint8_t gimbal_mode : 1;    // 0机械,1陀螺 (1位 )
    uint8_t vision_mode : 3; // 0无视觉模式，1是普通自瞄，2是小符，3是大符，4是前哨，5是英雄  (3位 → &0x07)
    uint8_t game_start : 1;  // 0/1 比赛开始      (1位 → &0x01) 主要控制拨盘热量限制
    uint8_t my_color : 1;     // 颜色 0/1          (1位)
    uint8_t is_hole : 1;       // 0/1 遥控器开启狗洞模式        (1位)
} Board_State_Pkt_t;


/*裁判系统热量02*/
typedef struct
{
    float shoot_speed;
    float shoot_freq;
    uint16_t shoot_heat_max;
    uint16_t shoot_heat;
} Board_Judge_Shoot_Pkt_t;


/*云台目标03*/
typedef struct
{
    float yaw_mec_tar;
    float yaw_imu_tar;
    float pitch_mec_tar;
    float pitch_imu_tar;
} Board_Gimbal_Target_Pkt_t;

/*发射指令04*/
typedef struct
{
    uint8_t shoot_count; //限制最大发弹量
    uint8_t shoot_count; // 当前发弹数量
    uint8_t launch_state : 1; // 0是关发射机构，1是开发射机构
    uint8_t shoot_mode : 1;
    uint8_t shoot_level : 1;
} Board_Shoot_Pkt_t;

/*血量信息*/
typedef struct
{
    uint8_t blood_0; // 英雄
    uint8_t blood_1; // 工程
    uint8_t blood_2; // 哨兵
    uint8_t blood_3; // 步兵
    uint8_t blood_4; // 无人机
    uint8_t blood_5; // 雷达
    uint8_t blood_6; // 基地
    uint8_t blood_7; // 前哨

} Board_Blood_Pkt_t;

/*云台信息*/
typedef struct
{
    float yaw_mec;
    float yaw_imu;
    float pitch_mec;
    float pitch_imu;
} Board_Gimbal_Meg_t;


/*电机状态*/
typedef struct
{
    uint8_t yaw_motor_state : 1;    // 第0位
    uint8_t pitch_motor_state : 1;  // 第1位
    uint8_t lift_motor_state : 1; // 第2位
    uint8_t r_fric_state : 1;       // 第3位
    uint8_t l_fric_state : 1;       // 第4位
    uint8_t dial_motor_state : 1;   // 第5位
    uint8_t vision_state : 1;       // 第6位
    uint8_t Lift_state : 1;         //第7位
} Board_State_Meg_t;


/*视觉信息*/
typedef struct
{
    float vision_yaw_tar;
    float vision_pitch_tar;
    uint8_t is_find_target : 1; // 和Board_State_Meg_t的标志位发一个包（拼好包）
} Board_Vision_Meg_t;


/*接收*/
typedef struct
{
    Board_State_Pkt_t state_pkt;    //七位
    Board_Judge_Shoot_Pkt_t judge_shoot_pkt;  //八字节额外多出限制发射数量，考虑直接给出可用热量  
    Board_Gimbal_Target_Pkt_t gimbal_target_pkt;  //多出升降标志位  
    Board_Shoot_Pkt_t shoot_pkt;        //3位
    Board_Blood_Pkt_t blood_pkt;    //八字节

} Board_Rx_Info_t;

/*发送*/
typedef struct
{
    Board_Gimbal_Meg_t gimbal_meg;  
    Board_Vision_Meg_t vision_meg;  
    Board_State_Meg_t state_meg;    

} Board_Tx_Info_t;

typedef struct
{

    dev_work_state_t status; // 工作状态

    uint32_t send_time; // 发包时间

    uint32_t rx_tick; // 收到数据时间戳

    uint8_t offline_cnt_pack_1; // 各包离线计数

    uint8_t offline_cnt_pack_2;

    uint8_t offline_cnt_pack_3;

    uint8_t offline_cnt_pack_4;

    uint8_t offline_cnt_max; // 离线计数上限

} Board_HeartBeat_t;

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info);
extern Board_Tx_Info_t Board_Tx_Info;
extern Board_Rx_Info_t Board_Rx_Info;
extern Board_HeartBeat_t Board_HeartBeat;

void Board_Rx_01(uint8_t *rxbuf);
void Board_Rx_02(uint8_t *rxbuf);
void Board_Rx_03(uint8_t *rxbuf);   
void Board_Rx_04(uint8_t *rxbuf);

void Send_To_Down_Board(void);
void C_Board_Communicate_HeartBeat(void);

#endif
