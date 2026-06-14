/**
  ******************************************************************************
  * @file    car.c
  * @brief   整车控制模块（上主控不使用，已精简）
  *          注意：底盘控制逻辑在下主控
  ******************************************************************************
  */
#include "car.h"
#include "communicate.h"
#include "gimbal.h"
#include "dial.h"

/* Private function prototypes -----------------------------------------------*/
/*main task*/
void car_init(car_t *car);
void car_update(car_t *car);


/*static*/
static void Car_Shoot_Mode_Update(car_t *car);

/* Public variables ---------------------------------------------------------*/
car_t car  = 
{
    .init = &car_init,
    .update = &car_update,
  };

/* Private functions ---------------------------------------------------------*/
  
void car_init(car_t *car)
{
  /*初始化部分结构体内容*/
    car->car_move_mode = offline_CAR;
    car->car_reset_state = DEV_RESET_NO;
    car->car_ctrl_mode = RC_CTRL_MODE;

    car->vision = &vision;
    car->gimbal = &gimbal;
    car->shoot = &shoot;

    car->unlock_car_flag = 0;
    car->init_cnt = 0;
    car->init_cnt_max = 1000; // 初始化计数上限
}

/*接受板间信息*/
//主要更新拨盘和云台，视觉信息
void car_update(car_t *car)
{

  /*接收控制信息*/
  switch (Board_Rx_Info.state_pkt.car_state)
  {
  case 0: // 卸力
    car->car_ctrl_mode = SLEEP_MODE;
    break;
  case 1: // 遥控
    car->car_ctrl_mode = RC_CTRL_MODE;
    break;
  case 2: // 键鼠
    car->car_ctrl_mode = KEY_CTRL_MODE;
  default:
    car->car_ctrl_mode = SLEEP_MODE; // 默认安全模式
    break;
  }


  /*接收车体模式*/
  switch (Board_Rx_Info.state_pkt.car_state)
  {
    case 0:
      car->car_move_mode = mec_CAR;
      break;
    case 1:
      car->car_move_mode = gyro_CAR;
      break;
    default:
      car->car_move_mode = mec_CAR; // 默认离线模式
  }


  /*接收视觉模式*/
  switch(Board_Rx_Info.state_pkt.vision_mode)
  {
    case 0:
        car->vision_flag.vision_mode_flag = false;
        break;
    case 1:
        car->vision_flag.vision_mode_flag = true;
        car->vision_flag.normal_vision_flag = true;
        break;
    case 2:
        car->vision_flag.vision_mode_flag = true;
        car->vision_flag.small_energy_engine_flag = true;
        break;
    case 3:
        car->vision_flag.vision_mode_flag = true;
        car->vision_flag.big_energy_engine_flag = true;
        break;
    case 4:
        car->vision_flag.vision_mode_flag = true;
        car->vision_flag.outpost_flag = true;
        break;
    case 5:
        car->vision_flag.vision_mode_flag = true;
        car->vision_flag.hero_flag = true;
        break;
    default:
        car->vision_flag.vision_mode_flag = false; // 默认非视觉模式
  }

  //根据标志位更新逻辑
  Car_Shoot_Mode_Update(car);
}


/*将信息传递到下板*/
static void Car_Shoot_Mode_Update(car_t *car)
{
  Vision_t* Vision = car->vision;
  Board_Rx_Info_t board_rx;

  //开启视觉时发射标志位交予控制，在dial中被调用
  if(car->vision_flag.normal_vision_flag == true)
  {
    car->shoot_flag.Shoot_Ctrl_Flag = Vision->VtoE->flag_union.bit.is_enable_shootting;
    
    //遥控控单连，键鼠视觉控
    if (car->car_ctrl_mode == KEY_CTRL_MODE)
      car->shoot_flag.Shoot_Mode = Vision->VtoE->flag_union.bit.is_keep_shooting;
    else
      car->shoot_flag.Shoot_Mode = Board_Rx_Info.shoot_pkt.shoot_mode;
  }
  //非视觉模式下给下板控
  else if(car->vision_flag.vision_mode_flag == false)
  {
    car->shoot_flag.Shoot_Ctrl_Flag = Board_Rx_Info.shoot_pkt.shoot_level;
    car->shoot_flag.Shoot_Mode = Board_Rx_Info.shoot_pkt.shoot_mode;
  }

  if(rm_motor[L_Fric].state->status == DEV_OFFLINE && rm_motor[R_Fric].state->status == DEV_OFFLINE\
      && shoot.state == S_WAITING && car->vision_flag.vision_mode_flag == true)
  {
    car->vision_flag.is_ready_shoot = 1;
  }
  else
  {
    car->vision_flag.is_ready_shoot = 0;
  }
}
