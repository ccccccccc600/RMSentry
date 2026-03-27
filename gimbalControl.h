#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#define GIMBAL_PITCH_POSITION_MAX 4600
#define GIMBAL_PITCH_POSITION_MEDIAN 4000
#define GIMBAL_PITCH_POSITION_MIN 3540

#include "6020_driver.h"
#include "Sliding.h"

#define YAW_MEDIAN 4092//yaw 轴机械中值（编码器值）
//#define YAW_MEDIAN 156.917725f// yaw 轴机械中值

#define PITCH_MAXANGLE 6525//4736//最大俯仰角（改成达妙的±50°）
#define PITCH_MINANGLE 5695//2620//最小俯仰角
#define PITCH_MEDIAN   6080//3405//pitch 轴机械中值（编码器）

/*云台电机相关句柄*/
extern M6020_HandleTypeDef   yaw_gimbal_motor;
extern M6020_HandleTypeDef pitch_gimbal_motor;
extern cSMC YawSmcController;
/*重力补偿相关参数*/
extern int compensation_factor;
extern float pitch_cos;
extern float pitch_sin;
/*云台yaw轴相关参数*/
extern float yaw_beganangle;
extern int16_t yaw_beganecd;  //以编码器为参考
/*云台ptich轴相关参数*/
extern int16_t pitch_beganecd;


/**
  * @func			void Gimbal_Init(void)
  * @brief          云台驱动初始化
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init(void);

/**
  * @func			int16_t Pitch_Gimbal_Task(int16_t targetpostion)
  * @brief          pitch轴串级pid控制任务
  * @param[in]      targetpostion：pitch轴目标绝对编码器值
  * @retval         pitch轴串级pid输出
  */
int16_t Pitch_Gimbal_ControlOfECD(int16_t targetpostion);

/**
  * @func			int16_t Yaw_Gimbal_Task(int16_t targetpostion)
  * @brief          yawh轴串级pid控制任务
  * @brief          以yawh轴绝对编码器为参考
  * @param[in]      targetpostion：yaw轴目标绝对编码器值
  * @retval         yaw轴串级pid输出
  */
int16_t Yaw_Gimbal_ControlOfECD(int16_t targetpostion);

/**
  * @func			int16_t Yaw_Gimbal_Task2(float realangle ,float targetangle)
  * @brief          yawh轴串级pid控制任务2
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      realangle：yaw轴实时角度，范围[-180,180]
  * @param[in]      targetpostion：yaw轴目标角度,范围[-180,180]
  * @retval         yaw轴串级pid输出
  */
int16_t Yaw_Gimbal_ControlOfIMU(float realangle ,float targetangle);

int16_t Yaw_Gimbal_ControlOfVision(float target, float rev);

/**
  * @func			float Update_ReferenceBeganAngle(User_USART* jy901_data)
  * @brief          更新参考角度
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      jy901_data：jy901句柄
  * @retval         yaw轴更新后的参考角度
  */
float Update_ReferenceBeganAngle(float imuYaw);

/**
  * @func			float Get_ReferenceMedianAngle(User_USART* jy901_data,M6020_HandleTypeDef* device)
  * @brief          计算yaw轴在机械中值位置的角度
  * @brief          以陀螺仪角度(世界坐标)为参考
  * @param[in]      jy901_data：jy901句柄
  * @param[in]      device：6020电机句柄
  * @retval         yaw轴在机械中值位置的角度
  */
float Get_ReferenceMedianAngle(float imuYaw, M6020_HandleTypeDef* device);

/**
  * @func			int16_t Update_ReferenceBeganeECD(M6020_HandleTypeDef* device)
  * @brief          更新参考角度
  * @brief          以绝对编码值为参考
  * @param[in]      device：6020电机句柄
  * @retval         yaw轴更新后的参考角度(绝对编码器值)
  */
int16_t Update_ReferenceBeganECD(M6020_HandleTypeDef* device);

/**
  * @func			int16_t GravityCompensation(const M6020_HandleTypeDef* device)
  * @brief          pitch轴重力补偿
  * @param[in]      device：6020电机句柄
  * @retval         补偿值
  */
int16_t GravityCompensation(const M6020_HandleTypeDef* device);
int16_t Yaw_speedControl(int16_t target_rpm);
extern pid_type_def DM4310_pid;
#endif
