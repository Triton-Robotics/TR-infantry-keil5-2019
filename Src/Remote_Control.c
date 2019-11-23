/**
  ******************************************************************************
  * @file    Remote_Control.c
  * @author  DJI 
  * @version V1.0.0
  * @date    2015/11/15
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "Remote_Control.h"

RC_Type remote_control;
uint16_t test_keyval = 0;
uint32_t  Latest_Remote_Control_Pack_Time = 0;
uint32_t  LED_Flash_Timer_remote_control = 0;
/*******************************************************************************************
  * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief  DR16���ջ�Э��������
  * @Param		RC_Type* rc���洢ң�������ݵĽṹ�塡��uint8_t* buff�����ڽ���Ļ���
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
//	rc->ch1 = (*buff | *(buff+1) << 8) & 0x07FF;	offset  = 1024
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	rc->switch_joystick = (buff[16] | buff[17]<<8) & 0x7FF;
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];	// is pressed?
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key board code
	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
			
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;
		
			
	}
	
}

extern uint16_t TIM_COUNT[];
int16_t HighTime;


int is_w_pressed(void)
{
	if ((remote_control.keyBoard.key_code & 0x01) == 0x01) {
		return 1;
	}
	
		return 0;
}
int is_a_pressed(void)
{
	if ((remote_control.keyBoard.key_code & 0x04) == 0x04) {
		return 1;
	}
	
		return 0;
}
int is_s_pressed(void)
{

		if ((remote_control.keyBoard.key_code & 0x02) == 0x02) {
		return 1;
	}
	
		return 0;
}
int is_d_pressed(void)
{
		if ((remote_control.keyBoard.key_code & 0x08) == 0x08) {
		return 1;
	}
	
		return 0;
}
int is_q_pressed(void)
{
		test_keyval = remote_control.keyBoard.key_code&0x02;
		if ((remote_control.keyBoard.key_code & 0x0040) == 0x0040) {
		return 1;
	}
	
		return 0;
}
int is_e_pressed(void)
{
		if ((remote_control.keyBoard.key_code & 0x0080) == 0x0080) {
		return 1;
	}
	
		return 0;
}

int is_mouse_moved(void)
{
	if (remote_control.mouse.x != 0 || remote_control.mouse.y != 0)
		return 1;
	else
		return 0;
}

float calculate_mouse_x_speed(void)
{
	float move_speed = remote_control.mouse.x;
	if(move_speed > 177)
	{
		move_speed = 177;
	}
	move_speed = move_speed/177;
	return  move_speed;
}

/*******************************************************************************************
  * @Func		void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
  * @Brief  PWM���ջ��������
  * @Param		TIM_HandleTypeDef *htim ���ڲ���PWM����Ķ�ʱ����
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	HighTime = (TIM_COUNT[1] - TIM_COUNT[0])>0?(TIM_COUNT[1] - TIM_COUNT[0]):((TIM_COUNT[1] - TIM_COUNT[0])+10000);
	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
			
  remote_control.ch4 = (HighTime - 4000)*660/4000;
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);		
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;
					
	}
	
}
