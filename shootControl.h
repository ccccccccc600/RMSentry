#ifndef __SHOOT_CONTROL_H
#define __SHOOT_CONTROL_H

#include "3508_driver.h"
#include "2006_driver.h"

/*摩擦轮发射速度宏*/
#define LEFT_SHOT_SPEED   6000
#define RIGHT_SHOT_SPEED  6000
/*拨弹速度宏*/
#define UP_TRIGGER_SPEED    -2500
#define DOMN_TRIGGER_SPEED  -2500
/*保险宏*/
#define VALVE_OPEN  1
#define VALVE_CLOSE 0
/* 发射限制 */
#define HEAT_MARGIN            30   // 停发阈值 = 规则上限 - 30
#define HEAT_RESUME_THRESHOLD  40   // 必须降到40以下才恢复

enum Shoot_Mode{
	
	CYCLE_FIRE_MODE,//循环发射
	SINGLE_SHOT_MODE,//单发
};
typedef struct{
	
	int valve;//射击阀门
	int mode;//发射控制模式

}Shoot_ModeMessageTypedef;

/*拨盘电机句柄*/
extern M2006_HandleTypeDef trigger_motor;

/*摩擦轮电机句柄 */
extern M3508_HandleTypeDef frictionWheel_motorHandle[2];

/*射击模式相关参数*/
extern Shoot_ModeMessageTypedef shoot_modemessage;

/**
  * @func			void Shoot_Init(void)
  * @brief          射击驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void);


void Shoot_FrictionWheelControl(int16_t setValue);
/**
  * @func			void Shoot_Control(int16_t* downtriggeroutput,int16_t* uptriggeroutput)
  * @brief          拨盘模式切换
  * @param[in]      downtriggeroutput：下拨弹pid输出值
  * @param[in]      uptriggeroutput：上拨弹pid输出值
  * @retval         none
  */
void Shoot_TriggerControl(int16_t setSpeed_triggeroutput);
static uint16_t clamp_u16_sub(uint16_t a, uint16_t b);
void ammo_dial_anti_block_with_heat_limit(uint16_t heat,uint16_t rule_heat_limit,int16_t friction_speed,int16_t trigger_speed);
void Shoot_TriggerAndFrictionWheelControl(int16_t friction_value, int16_t setSpeed_triggeroutput);
void ammo_dial_anti_block_core(int16_t friction_speed ,int16_t trigger_speed);
#endif