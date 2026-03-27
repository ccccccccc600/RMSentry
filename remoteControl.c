#include "remoteControl.h"
#include "gimbalControl.h"
#include "stdlib.h"
/*define remote handle*/

RC_Ctl_t RC_Ctl;   					     //声明遥控器数据结构体
RC_USER_t rc_user;

void REMOTE_DMA_Init(UART_HandleTypeDef* huart, uint8_t* rx_buf, RC_Ctl_t *rc)
{
    memset(rc, 0, sizeof(*rc));
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, rx_buf, 18);
	  rc_user.pitch = PITCH_MEDIAN;
}


/**
  * @brief          处理接收的遥控器数据包
  * @param[in]      none
  * @retval         none
  */

void RC_Process(RC_Ctl_t* rc_ctl, uint8_t* buf)
{
		/*拆包*/
	  rc_ctl->rc.ch0 = (buf[0]| (buf[1] << 8)) & 0x07ff;          
		rc_ctl->rc.ch1 = ((buf[1] >> 3) | (buf[2] << 5)) & 0x07ff;       
		rc_ctl->rc.ch2 = ((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07ff;          
		rc_ctl->rc.ch3 = ((buf[4] >> 1) | (buf[5] << 7)) & 0x07ff;   
	
		rc_ctl->rc.s1  = ((buf[5] >> 4)& 0x000C) >> 2;                           
		rc_ctl->rc.s2  = ((buf[5] >> 4)& 0x0003);    
		rc_ctl->rc.roller=((((int16_t)buf[16])|(int16_t)buf[17]<<8)&0x07FF);
		
	  rc_ctl->rc.ch0-=1024;
	  rc_ctl->rc.ch1-=1024;
    rc_ctl->rc.ch2-=1024;
	  rc_ctl->rc.ch3-=1024;
	  rc_ctl->rc.roller-=1024;
	  
		if(abs(rc_ctl->rc.ch0)<5)rc_ctl->rc.ch0=0;
		if(abs(rc_ctl->rc.ch1)<5)rc_ctl->rc.ch1=0;
		if(abs(rc_ctl->rc.ch2)<20)
		{
		  rc_ctl->rc.ch2=0;
		}
		else
		{
		  rc_ctl->rc.ch2-=20;
		}
		if(abs(rc_ctl->rc.ch3)<5)rc_ctl->rc.ch3=0;
	  
		rc_ctl->mouse.x = buf[6] | (buf[7] << 8);                              //!< Mouse X axis        
		rc_ctl->mouse.y = buf[8] | (buf[9] << 8);                              //!< Mouse Y axis      
		rc_ctl->mouse.z = buf[10]| (buf[11]<< 8);                              //!< Mouse Z axis         
		rc_ctl->mouse.press_l = buf[12];                                       //!< Mouse Left Is Press      
		rc_ctl->mouse.press_r = buf[13];                                       //!< Mouse Right Is Press 
		rc_ctl->key.v = buf[14];             //| (buf[15] << 8);   		       //!< KeyBoard value
}

void RC_User_Process(const RC_Ctl_t* rc_ctl, RC_USER_t* rc_user)
{
	// if (rc_user->pitch == 0)
	// {
	// 	rc_user->pitch = PITCH_MEDIAN;
	// }
	
	/*map channel*/
	switch(rc_ctl->rc.s1)
	{
		case RC_SW_UP:{

		}break;

		case RC_SW_MID:{
			rc_user->move_x = (rc_ctl->rc.ch1 - 1024) * RC_SENT_ABSOLUTE_MOVE_X_RATE;
			rc_user->move_y = (rc_ctl->rc.ch0 - 1024) * RC_SENT_ABSOLUTE_MOVE_Y_RATE;
			rc_user->yaw   += (rc_ctl->rc.roller - 1024)*RC_SENT_RELATIVE_YAW_RATE/2; 	//通道roller控制云台yaw旋转
			rc_user->pitch += (rc_ctl->rc.ch3 - 1024 ) * RC_SENT_RELATIVE_PITCH_RATE;		//通道1控制云台pitch旋转
		}break;

		case RC_SW_DOWN:{
			rc_user->move_x = (rc_ctl->rc.ch1 - 1024) * RC_SENT_ABSOLUTE_MOVE_X_RATE;
			rc_user->move_y = (rc_ctl->rc.ch0 - 1024) * RC_SENT_ABSOLUTE_MOVE_Y_RATE;
			rc_user->yaw   -= ( RC_Ctl.rc.ch2-1024 )*RC_SENT_RELATIVE_YAW_RATE;//通道0控制云台yaw旋转
			rc_user->pitch += ( RC_Ctl.rc.ch3-1024 )*RC_SENT_RELATIVE_PITCH_RATE;//通道1控制云台pitch旋转
		}break;

		default:{

		}
	}
	
	/*limit value*/
	if(rc_user->pitch>PITCH_MAXANGLE){rc_user->pitch=PITCH_MAXANGLE;}
	else if(rc_user->pitch<PITCH_MINANGLE){rc_user->pitch=PITCH_MINANGLE;}
}



