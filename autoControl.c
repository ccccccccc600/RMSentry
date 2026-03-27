#include "autoControl.h"
#include "gimbalControl.h"
#include "remoteControl.h"
#include "jy901.h"
#include "task.h"
#include "ReceiveBaseboard_task.h"
Nav_DataHandle_t auto_nav;
AutoAim_DataHandle_t auto_aim;
ArmorHitDetection_DataHandle_t armor_dataHandle;
int auto_key1;
int auto_key2;
int auto_tim_yaw;
int auto_tim_pitch;
float auto_yaw_targetpostion;
float auto_pitch_targetpostion;
extern JY901_DataHandleTypedef gimbalJY901_imuHandle;

void AutoControl_Init(void)
{
	auto_aim.datafloat.d_pitch=0;//偏移量初始化为零
	auto_aim.datafloat.d_yaw=0;//偏移量初始化为零
	auto_aim.datafloat.distance = 0;
	auto_aim.dataint16_t.d_pitch=0;//偏移量初始化为零
	auto_aim.dataint16_t.d_yaw=0;//偏移量初始化为零
	auto_aim.dataint16_t.distance = 0;
	auto_aim.aim_signal=AUTO_AIM_LOSE;//接收信号初始化为丢失目标
	auto_aim.shoot_signal=AUTO_AIM_SHOOT_TURN_OFF;//发射信号初始化为关闭
	/*自瞄相关参数*/
	auto_key1=0;
	auto_key2=0;
	auto_tim_yaw=0;
	auto_tim_pitch = 0;
	auto_yaw_targetpostion=0;
	auto_pitch_targetpostion=PITCH_MEDIAN;
	
	armor_dataHandle.armor_id = 0;
	armor_dataHandle.last_armor_id = 0;
	armor_dataHandle.signal = 0;

}


/*导航*/
//int16_t Velcity_NormalA2Rpm(float velcity)
//{
//	return velcity*CHASSIS_WHEEL_MOTRO_REDUCTION_RATIO*30/(CHASSIS_WHEEL_RADIUS*PI);
//}


/*自瞄*/


AutoAim_Signal GetAutoAIM(void)
{
	if(auto_aim.aim_signal == AUTO_AIM_GET)//识别到装甲板
	{
		return AUTO_AIM_GET;
	}
	else//没有识别到装甲板
	{
		return AUTO_AIM_LOSE;
	}
}

void AutoAim_Task(uint8_t changleSignal)
{
	
	if(GetAutoAIM() == AUTO_AIM_GET)//检测到装甲板时
	{
		armor_dataHandle.delaySingal = 1;
		
       /*承接其他模式下的目标值*/
       if(changleSignal || auto_key1 == 0)
       {
           Get_ReferenceMedianAngle(gimbalJY901_imuHandle.angle.angle[2], &yaw_gimbal_motor);//
           Update_ReferenceBeganECD(&yaw_gimbal_motor);//刷新数据
           Update_ReferenceBeganECD(&pitch_gimbal_motor);
           auto_key1=1;
           if(auto_key2 == 1){auto_key2=0;}
           auto_yaw_targetpostion=yaw_beganangle;
           auto_pitch_targetpostion=pitch_beganecd;
       }
       /*目标值更新*/
   //  auto_yaw_targetpostion+=auto_aim.d_yaw*AUTO_AIM_YAW_RATE;//角度
       auto_pitch_targetpostion+=auto_aim.datafloat.d_pitch*AUTO_AIM_PITCH_RATE;//编码器值
       
       int16_t yaw_output = Yaw_Gimbal_ControlOfVision(0, auto_aim.datafloat.d_yaw);//yaw轴过一个位置环
       int16_t pitch_output = 0;
			 
       /* pitch限幅后给pitch_output赋值 */
       if(auto_pitch_targetpostion >= PITCH_MAXANGLE)
				 {
           auto_pitch_targetpostion = PITCH_MAXANGLE;
           pitch_output = Pitch_Gimbal_ControlOfECD(auto_pitch_targetpostion);
         }
			else if(auto_pitch_targetpostion <= PITCH_MINANGLE)
				 {
           auto_pitch_targetpostion = PITCH_MINANGLE;
           pitch_output = Pitch_Gimbal_ControlOfECD(auto_pitch_targetpostion);
         }
		  else
				 {
           pitch_output = Pitch_Gimbal_ControlOfECD(auto_pitch_targetpostion);
         }

       /*shoot*/
       CAN_cmd_gimbal_yaw(yaw_output);
			//加一个pitch控制J4310_ControlData_Send函数输入pitch_output
	}
	else
	{
		M6020_GetBasicData(&yaw_gimbal_motor);//获取数据
		if(changleSignal || auto_key2==0)
		{
			Get_ReferenceMedianAngle(gimbalJY901_imuHandle.angle.angle[2], &yaw_gimbal_motor);
			Update_ReferenceBeganECD(&yaw_gimbal_motor);
			Update_ReferenceBeganECD(&pitch_gimbal_motor);
			auto_key2=1;
			if(auto_key1 == 1){auto_key1 = 0;}
			auto_yaw_targetpostion = yaw_beganangle;
			auto_pitch_targetpostion=pitch_beganecd;
		}

		if(auto_tim_yaw >=3)
		{
			if(!armor_dataHandle.delaySingal)
			{
				if(auto_aim.datafloat.last_d_yaw < 0){
					auto_yaw_targetpostion+=1;
				}else{
					auto_yaw_targetpostion-=1;
					}
			}
			
			auto_tim_yaw=0;
		}

		if(auto_tim_pitch >= 2){
			static int way=0;
			if(way==0)
			{
				auto_pitch_targetpostion+=10;
				if(auto_pitch_targetpostion >= PITCH_MEDIAN)
				{
					way=1;															
				}
			}
			else
			{
				auto_pitch_targetpostion-=10;
				if(auto_pitch_targetpostion <= PITCH_MINANGLE)
				{
					way=0;
				}
			}
			
			auto_tim_pitch = 0;
		}

//		if(Robot_All_Info.hurt_data.HP_deduction_reason == 0x05)  
//		{
//			armor_dataHandle.signal = 1;
//		}

		if(armor_dataHandle.signal){
			armor_dataHandle.signal = 0;
			int16_t revECD = yaw_gimbal_motor.basic_data.ecd;
			float revAngle = gimbalJY901_imuHandle.angle.angle[2];
			float setAngle = 0;
			
			armor_dataHandle.delaySingal = 1;
//			armor_dataHandle.armor_id = Robot_All_Info.hurt_data.armor_id;
			switch(armor_dataHandle.armor_id)
			{
				case 0:{
					setAngle = revAngle + (float)(FRONT_ARMOR_ECD - revECD)*360/8191;
				}break;

				case 3:{
					setAngle = revAngle +(float)(RIGHT_ARMOR_ECD - revECD)*360/8191;
				}break;

				case 2:{
					setAngle = revAngle +(float)(BEHIND_ARMOR_ECD - revECD)*360/8191;
				}break;

				case 1:{
					setAngle = revAngle + (float)(LEFT_ARMOR_ECD - revECD)*360/8191;
				}break;
			}
			auto_yaw_targetpostion = setAngle;

//			Robot_All_Info.hurt_data.HP_deduction_reason = 0; //手动清空受伤标志
		}

		int16_t yaw_output=Yaw_Gimbal_ControlOfIMU(gimbalJY901_imuHandle.angle.angle[2], auto_yaw_targetpostion);
		int16_t pitch_output=Pitch_Gimbal_ControlOfECD(auto_pitch_targetpostion);
		int16_t g_compensation=GravityCompensation(&pitch_gimbal_motor);
		CAN_cmd_gimbal_yaw(yaw_output);
		//加一个pitch控制CAN_cmd_gimbal_pitch
	}
}

void NUC_speed_send_to_baseBoard(int16_t PointX, int16_t PointY, int16_t PointZ, int16_t Control_mode)
{
	CAN_TxHeaderTypeDef  shoot_tx_message;
	uint8_t shoot_can_send_data[8] = {0};
	
	uint32_t send_mail_box;
	shoot_tx_message.StdId = 0x110;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	
	shoot_can_send_data[0] = PointX;
	shoot_can_send_data[1] = PointX >> 8;
	shoot_can_send_data[2] = PointY;
	shoot_can_send_data[3] = PointY >> 8;
	shoot_can_send_data[4] = PointZ;
	shoot_can_send_data[5] = PointZ >> 8;
	shoot_can_send_data[6] = Control_mode;
	shoot_can_send_data[7] = Control_mode >> 8;
	
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)==0){};
  HAL_CAN_AddTxMessage(&hcan2, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}
