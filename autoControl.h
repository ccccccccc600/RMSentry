#ifndef __AUTOCONTROL_H
#define __AUTOCONTROL_H

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"

#define FRONT_ARMOR_ECD 4000
#define BEHIND_ARMOR_ECD 7991
#define LEFT_ARMOR_ECD 6056
#define RIGHT_ARMOR_ECD 1950

#define NAV_CAN_SEND_ID 0x250 //发送给小电脑的CAN ID
#define NAV_CAN_REV_ID 0x520  //接收小电脑的CAN ID
#define AIM_CAN_SEND_ID 0x101 //发送给小电脑的CAN ID
#define AIM_CAN_REV_ID 0x105  //接收小电脑的CAN ID
#define CAN_PITCH_MOTOR_ID 0x000 //达妙电机数据接收的CAN ID

typedef struct{
	struct{
		double linear_x;
		double linear_y;
		double rotate_z;
	}datafloat_t;
	
	struct{
		int16_t linear_x;
		int16_t linear_y;
		int16_t rotate_z;
	}dataint16_t;

}Nav_DataHandle_t;

#define AUTO_AIM_YAW_RATE 0.06//0.0015
#define AUTO_AIM_PITCH_RATE 0.02
typedef enum{
  AUTO_AIM_LOSE=0,
  AUTO_AIM_GET=1,
  AUTO_AIM_SHOOT_TURN_OFF=2,
  AUTO_AIM_SHOOT_TURN_ON=3,
	
}AutoAim_Signal;

typedef struct{
	//云台偏移量数据
	struct{
		float d_yaw;
		float d_pitch;
		float distance;
		float last_d_yaw;
		float last_d_pitch;
	}datafloat;
	
	struct{
		int16_t d_yaw;
		int16_t d_pitch;
		int16_t distance;
		int16_t last_d_yaw;
		int16_t last_d_pitch;
	}dataint16_t;
	
	//自瞄识别信号
	uint8_t aim_signal;
	//发射信号
	uint8_t shoot_signal;
}AutoAim_DataHandle_t;

typedef struct{
	uint8_t signal;
	uint8_t delaySingal;
	uint8_t armor_id;
	uint8_t last_armor_id;
}ArmorHitDetection_DataHandle_t;
extern ArmorHitDetection_DataHandle_t armor_dataHandle;

//typedef struct{
//    uint16_t id;
//    uint8_t rx_data[8];
//}NavReceiveQueueData_t;
//extern QueueHandle_t NavReceive_queueHandle;

extern Nav_DataHandle_t auto_nav;
extern AutoAim_DataHandle_t auto_aim;
extern int auto_key1;
extern int auto_key2;
extern int auto_tim_yaw;
extern int auto_tim_pitch;
extern float auto_yaw_targetpostion;
extern float auto_pitch_targetpostion;

void AutoControl_Init(void);

/*导航*/
//int16_t Velcity_NormalA2Rpm(float velcity);
void NUC_speed_send_to_baseBoard(int16_t PointX, int16_t PointY, int16_t PointZ, int16_t Control_mode);

/*自瞄*/


AutoAim_Signal GetAutoAIM(void);




void AutoAim_Task(uint8_t changleSignal);
#endif
