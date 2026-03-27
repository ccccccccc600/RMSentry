/**
****************************(C) COPYRIGHT 2025 征途****************************
  * @file       shoot_task.c/h
  * @brief      
  *             这里是射击任务程序，包含摩擦轮驱动，拨盘驱动等功能程序 （要改2摩擦轮版本，1个拨弹盘）
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-29-2024    chenjiangnan       
  *
  @verbatim
  ==============================================================================
	biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~biu~
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 征途****************************
  */
#include "shootControl.h"
#include "remoteControl.h"
#include "bsp_dji.h"
#include "ReceiveBaseboard_task.h"


/*拨盘电机句柄*/
M2006_HandleTypeDef trigger_motor;
/*摩擦轮电机句柄 */
M3508_HandleTypeDef frictionWheel_motorHandle[2];//改
/*射击模式相关参数*/
Shoot_ModeMessageTypedef shoot_modemessage;

/**
  * @func			      void Shoot_Init(void)
  * @brief          射击部分初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void)
{
	/*摩擦轮电机初始化*/
	/*拨弹电机初始化 id设定，输入电机结构体，确认pid模式*/
	M3508_Init(0x02, &frictionWheel_motorHandle[0], PID_POSITION, 0);
	M3508_Init(0x03, &frictionWheel_motorHandle[1], PID_POSITION, 0);
  M2006_Init(0x04, &trigger_motor,PID_POSITION);
	/*拨弹信息初始化*/
	shoot_modemessage.valve=VALVE_CLOSE;
	shoot_modemessage.mode=-1;
	trigger_motor.state = MOTOR_STATE_NORMAL;
	trigger_motor.reverse_timer = 0;
	trigger_motor.block_timer = 0;
}

static uint16_t clamp_u16_sub(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : 0u;
}


/**
 * @brief 拨弹盘防卡 + 热量限制发射（停发=规则上限-30，恢复=热量<40）
 * @param heat 当前热量
 * @param rule_heat_limit 规则热量上限
 * @param friction_speed 摩擦轮目标转速(rpm)
 * @param trigger_speed  拨弹盘目标转速(rpm)（允许发射时）
 * @note 需在10ms定时中断/主循环中调用
 */
void ammo_dial_anti_block_with_heat_limit(uint16_t heat, uint16_t rule_heat_limit, int16_t friction_speed, int16_t trigger_speed)
{
    static uint8_t heat_lock = 0; // 0=允许发射 1=热量锁定禁发
    uint16_t stop_threshold = clamp_u16_sub(rule_heat_limit, HEAT_MARGIN);
    /* 1) 更新热量锁定状态（带回差） */
    if (!heat_lock)
    {
        if (heat >= stop_threshold)
            heat_lock = 1;
    }
    else
    {
        if (heat < HEAT_RESUME_THRESHOLD)
            heat_lock = 0;
    }
    /* 2) 根据热量状态决定拨弹盘目标转速 */
    if (heat_lock)
    {
        /* 热量到上限：拨弹盘停转（摩擦轮是否停看你需求，这里保持 friction_speed） */
        ammo_dial_anti_block_core(friction_speed, 0);
    }
    else
    {
        /* 允许发射：走你原来的防卡逻辑 */
        ammo_dial_anti_block_core(friction_speed, trigger_speed);
    }
}

/**
 * @brief 拨弹盘防卡弹核心逻辑（反转延时后直接恢复）
 * @param friction_speed 期望摩擦轮转速（rpm）
 * @param trigger_speed 拨弹盘期望转速（rpm）
 * @note 需在10ms定时中断/主循环中调用
 */
void ammo_dial_anti_block_core(int16_t friction_speed ,int16_t trigger_speed){
    // 1. 读取电机实际转速
    M2006_GetBasicData(&trigger_motor);

    // 2. 正常状态下更新目标转速
    if (trigger_motor.state == MOTOR_STATE_NORMAL){
        trigger_motor.speed_control_data.set = trigger_speed;
    }

    // 3. 状态机核心逻辑
    switch (trigger_motor.state){
        case MOTOR_STATE_NORMAL:
            // 正常运行：设置转速 + 重置堵转计数
            Shoot_TriggerAndFrictionWheelControl(friction_speed, trigger_speed);
            trigger_motor.block_count = 0; // 正常状态重置计数

            // 堵转判定：目标转速>0 且 实际转速远小于目标
            if (trigger_motor.speed_control_data.set > 0 && 
                (trigger_motor.basic_data.speed_rpm < (trigger_speed - TARGET_SPEED_TOLERANCE))){
                // 启动堵转计时
                if (trigger_motor.block_timer == 0){
                    trigger_motor.block_timer = HAL_GetTick();
                }
                // 超时判定为堵转
                if ((HAL_GetTick() - trigger_motor.block_timer) >= BLOCK_DETECTION_TIMEOUT){
                    trigger_motor.state = MOTOR_STATE_BLOCKED; // 触发堵转状态
                    trigger_motor.block_timer = 0;
                }
            } else {
                trigger_motor.block_timer = 0; // 转速正常重置计时
            }
            break;
			
        case MOTOR_STATE_BLOCKED:
            // 累计堵转次数，超3次停机保护
            trigger_motor.block_count++;
            if (trigger_motor.block_count >= BLOCK_TIMES){ 
                Shoot_TriggerAndFrictionWheelControl(friction_speed, 0); // 拨弹盘停机
                trigger_motor.state = MOTOR_STATE_BLOCK_STOP; // 进入停机状态
                trigger_motor.block_timer = 0;
                trigger_motor.reverse_timer = 0;
							  trigger_motor.stop_timer = HAL_GetTick();
                trigger_motor.block_count = 0; // 重置计数，避免下次直接停机
                break;
            }
        
            // 未超次数：启动反转（持续REVERSE_RUN_DURATION时长）
            trigger_motor.state = MOTOR_STATE_REVERSE; // 进入恢复状态
            trigger_motor.reverse_timer = HAL_GetTick(); // 记录反转起始时间
            Shoot_TriggerAndFrictionWheelControl(friction_speed, REVERSE_SPEED);
            break;
			
        case MOTOR_STATE_REVERSE:
            // 反转持续指定时长后，直接恢复正常转速
            if ((HAL_GetTick() - trigger_motor.reverse_timer) >= REVERSE_RUN_DURATION){
                trigger_motor.state = MOTOR_STATE_NORMAL; // 回到默认状态
                trigger_motor.reverse_timer = 0;
                Shoot_TriggerAndFrictionWheelControl(friction_speed, trigger_speed); // 恢复原转速
            }
            break;
						
        case MOTOR_STATE_BLOCK_STOP:
						// 堵转3次后停机3秒状态
            // 判定是否停机满3秒
            if ((HAL_GetTick() - trigger_motor.stop_timer) >= BLOCK_STOP_DURATION){
                trigger_motor.state = MOTOR_STATE_NORMAL; // 恢复正常状态
                trigger_motor.stop_timer = 0; // 重置停机计时器
                // 停机结束后恢复原转速
                Shoot_TriggerAndFrictionWheelControl(friction_speed, trigger_speed);
            } else {
                // 停机期间保持拨弹盘停机
                Shoot_TriggerAndFrictionWheelControl(friction_speed, 0);
            }
            break;
				
        default:
            // 异常状态恢复
            trigger_motor.state = MOTOR_STATE_NORMAL;
            trigger_motor.block_timer = 0;
            trigger_motor.reverse_timer = 0;
            trigger_motor.block_count = 0;
						trigger_motor.stop_timer = 0; 
            Shoot_TriggerAndFrictionWheelControl(friction_speed, 0); // 停机保护
            break;
    }
}

/**
  * @func			      void Shoot_FrictionWheelControl(int16_t setValue)
  * @brief          摩擦轮电机控制
  * @param[in]      setValue：摩擦轮目标速度
  * @retval         none
  */
void Shoot_FrictionWheelControl(int16_t setValue)
{
	//获取ref
	M3508_GetBasicData(&frictionWheel_motorHandle[0], 0);
	M3508_GetBasicData(&frictionWheel_motorHandle[1], 0);
	
  //获取set
	int16_t setSpeed[2] = {0};
	int16_t setCurrent[2] = {0};
	setSpeed[0] = setValue;
	setSpeed[1] = -setValue;
	
  //计算速度环pid算出电流值
	setCurrent[0] = M3508_SpeedLoopPIDController(&frictionWheel_motorHandle[0].speed_control_data, frictionWheel_motorHandle[0].basic_data.speed_rpm, setSpeed[0]);
	setCurrent[1] = M3508_SpeedLoopPIDController(&frictionWheel_motorHandle[1].speed_control_data, frictionWheel_motorHandle[1].basic_data.speed_rpm, setSpeed[1]);
	
  //发送电流到电调
//	CAN_cmd_shootFric(setCurrent[0], setCurrent[1]);
}

/**
  * @func			      void Shoot_TriggerControl(int16_t setSpeed_triggeroutput)
  * @brief          拨盘速度控制
  * @param[in]      setSpeed_triggeroutput：拨盘目标速度
  * @retval         none
  */
void Shoot_TriggerControl(int16_t setSpeed_triggeroutput)
{	
	int16_t setSpeed= 0;
	int16_t setCurrent = 0;
	/* 获取ref */
	M2006_GetBasicData(&trigger_motor);//获取电机数据之后pid用结构体里面的speed_rpm作为实际转速ref
  /* 获取目标速度 */
	setSpeed =  setSpeed_triggeroutput;
	/* pid计算电流 */
  setCurrent= M2006_SpeedLoopPIDController(&trigger_motor.speed_control_data,trigger_motor.basic_data.speed_rpm, setSpeed);
  /* 发送目标电流 */
	CAN_cmd_trigger(setCurrent);
}

/**
  * @func			      void Shoot_TriggerAndFrictionWheelControl(int16_t friction_value, int16_t setSpeed_triggeroutput)
  * @brief          拨盘摩擦轮速度控制
  * @param[in]      friction_value：摩擦轮目标速度
  * @param[in]      setSpeed_triggeroutput：拨盘目标速度
  * @retval         none
  */
void Shoot_TriggerAndFrictionWheelControl(int16_t friction_value, int16_t setSpeed_triggeroutput)
{
  int16_t setSpeed[2] = {0};
	int16_t setCurrent[2] = {0};
	int16_t setTriggerSpeed = 0;
	int16_t setTriggerCurrent = 0;
	
	//获取ref
	M2006_GetBasicData(&trigger_motor);	
	M3508_GetBasicData(&frictionWheel_motorHandle[0], 0);
	M3508_GetBasicData(&frictionWheel_motorHandle[1], 0);
	
  //获取set
  setTriggerSpeed =  setSpeed_triggeroutput;
	setSpeed[0] = -friction_value;
	setSpeed[1] = friction_value;
	
  //计算速度环pid算出电流值
  setTriggerCurrent = M2006_SpeedLoopPIDController(&trigger_motor.speed_control_data,trigger_motor.basic_data.speed_rpm, setTriggerSpeed);
	setCurrent[0] = M3508_SpeedLoopPIDController(&frictionWheel_motorHandle[0].speed_control_data, frictionWheel_motorHandle[0].basic_data.speed_rpm, setSpeed[0]);
	setCurrent[1] = M3508_SpeedLoopPIDController(&frictionWheel_motorHandle[1].speed_control_data, frictionWheel_motorHandle[1].basic_data.speed_rpm, setSpeed[1]);
	
  //发送电流到电调
	CAN_cmd_shootFric(setCurrent[0], setCurrent[1], setTriggerCurrent);  
}