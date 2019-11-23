/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman_filter.h"
#include "./bsp/bsp_can.h"
#include "./bsp/bsp_imu.h"
#include "./bsp/pid.h"
#include "Remote_Control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define INFANTRY_1

/* uart vars */
char * result;
float heading = 999;
float heading_pos = 999;
uint8_t aRxBuffer;		
uint8_t Uart1_RxBuff[256];	
uint8_t Uart1_Rx_Cnt = 0;
uint8_t	cAlmStr[] = "overflow!!!\r\n";

PID_TypeDef motor_pid[4];
PID_TypeDef chassis_rotate_pos;

PID_TypeDef yaw_pid_lock_speed;
PID_TypeDef yaw_pid_lock_pos;

PID_TypeDef pitch_pid_speed;
PID_TypeDef yaw_pid_speed;
PID_TypeDef pitch_pid_pos;
PID_TypeDef yaw_pid_pos;
PID_TypeDef yaw_pid_aim_pos;

PID_TypeDef trigger_pid_speed;
// yaw 345
// pitch 
int32_t set_spd = 0;

int32_t end_Yaw = 0;
// Calculate Time
int32_t counter = 0;
uint32_t imuCalTime = 0;
uint32_t imuFirst = 0;
uint32_t imuLast = 0;

uint32_t pid_pos_first = 0;
uint32_t pid_pos_last = 0;
uint32_t pid_pos_time = 0;

uint32_t pid_speed_last = 0;
uint32_t pid_speed_time = 0;

uint32_t pid_time = 0;

#ifdef INFANTRY_1
int32_t pitch_zero_position = -51;
int32_t yaw_zero_position = 27;
float sbase = 2;
#endif
#ifdef INFANTRY_2
int32_t pitch_zero_position = -55;
int32_t yaw_zero_position = -60;
float sbase = 1.5;
#endif
// Infantry 3
#ifdef INFANTRY_3
int32_t pitch_zero_position = -175;
int32_t yaw_zero_position = 2;
float sbase = 2;
#endif


int32_t yaw_test_speed = 0;
//Trigger
int32_t test_speed = 1000;

// Friction Wheel
int32_t launch_pwm = 1150;

float pitch_angle = 0;
float yaw_angle = 0;

int32_t wheel_speeds[4];
int32_t x_speed = 0;
int32_t y_speed = 0;
float rotate_speed = 0;
float r_speed = 0;

// integration of speed for mouse
float x_integrand = 0;
float y_integrand = 0;

int32_t test_Callback = 0;

static int key_sta = 0;
int speed_step_sign = +1;

int lock_control = 0;

int w_pressed = 0;

uint16_t TIM_COUNT[2];
#define SpeedStep 500

float pitch_filter_value = 0;
float yaw_filter_value = 0;
float output_value = 0;

int center_angle;

int firstTime = 1;
int firstAim = 1;

uint8_t chassis_mode = 0;

float speed_scale = 0;

kalman1_state pitch_filter;
kalman1_state yaw_filter;

extern imu_t imu;

uint32_t x_stopped_time;
uint8_t mouse_x_moved;

int send_control = 0;
int niuYao = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Key_Scan(){
		
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET){
						
			if(key_sta == 0){
					
				key_sta = 1;
				
				set_spd += SpeedStep*speed_step_sign;
				
				if(set_spd>8000)
				{
					speed_step_sign = -1;
				}
				
				if(set_spd<=0){
						
					set_spd = 0;
					speed_step_sign = 1;
					
				}
					
			}
			
		}else{
			
			key_sta = 0;
		
		}
	
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  my_can_filter_init_recv_all(&hcan1);     // Initialize CAN filter
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   // Initialize CAN Signal Receiver
  HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100);   //Initialize UART(Serial) Signal Receiver (IDLE?)
  HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer, 1); //Initialize UART(Serial) Signal Receiver

  HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	/*< Initialize PID for the 4 wheels >*/
  for(int i=0; i<4; i++)
  {	
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0,6.0,0,0.5);
  }
	
	
	pid_init(&chassis_rotate_pos);
	pid_init(&yaw_pid_lock_pos);
	pid_init(&yaw_pid_lock_speed);
	pid_init(&pitch_pid_pos);
	pid_init(&pitch_pid_speed);
	pid_init(&yaw_pid_pos);
	pid_init(&yaw_pid_speed);
	pid_init(&trigger_pid_speed);
	pid_init(&yaw_pid_aim_pos);


	// Dodge Mode PID for yaw axis positional control
	yaw_pid_lock_pos.f_param_init(&yaw_pid_lock_pos, PID_Position, 30000, 30000, 0, 0, 8000, 0, 15, 0.0, 30);
	yaw_pid_lock_pos.target = yaw_zero_position;
	yaw_pid_lock_pos.f_cal_pid(&yaw_pid_lock_pos, moto_yaw.angle_360);	
	
	// Dodge Mode PID for yaw axis speed control
	yaw_pid_lock_speed.f_param_init(&yaw_pid_lock_speed, PID_Speed, 30000, 30000, 0, 0, 8000, 0, 120, 0, 0);
	yaw_pid_lock_speed.target = 0;
	yaw_pid_lock_speed.f_cal_pid(&yaw_pid_lock_speed, moto_yaw.ecd_raw_rate);	
	
	// Normal Mode PID for yaw axis positional control
	yaw_pid_pos.f_param_init(&yaw_pid_pos, PID_Position, 1500,1500,-2,0,2000,0,30,0,60);
	yaw_pid_pos.target = yaw_zero_position;
	yaw_pid_pos.f_cal_pid(&yaw_pid_pos, moto_yaw.angle_360);
	
	// Normal Mode PID for yaw axis speed control
	yaw_pid_speed.f_param_init(&yaw_pid_speed, PID_Speed, 30000,30000,0,0,8000,0,80,0,0);
	yaw_pid_speed.target = yaw_pid_pos.output;
	yaw_pid_speed.f_cal_pid(&yaw_pid_speed, moto_yaw.speed_rpm);
	
	// Legacy Code problem
    yaw_pid_speed.target = 0;
	yaw_pid_pos.target = heading_pos;		
	
	// Aim Mode PID for yaw axis positional control (using normal mode speed control PID)
	yaw_pid_aim_pos.f_param_init(&yaw_pid_aim_pos, PID_Position, 30000, 30000, 0, 0, 8000, 0, 3, 0.0, 15);
	yaw_pid_aim_pos.target = yaw_zero_position;
	yaw_pid_aim_pos.f_cal_pid(&yaw_pid_aim_pos, moto_yaw.angle_360);	
	
	
	chassis_rotate_pos.f_param_init(&chassis_rotate_pos, PID_Position, 5000, 5000, 0, 0, 5000, 0, 50, 0, 6);
	chassis_rotate_pos.target = yaw_zero_position;
	chassis_rotate_pos.f_cal_pid(&chassis_rotate_pos, moto_yaw.angle_360);
	
	
	
	pitch_pid_pos.f_param_init(&pitch_pid_pos, PID_Position, 1000,5000,-2,0,2000,0,30,0,80);
	pitch_pid_pos.target = pitch_angle;
	pitch_pid_pos.f_cal_pid(&pitch_pid_pos, -moto_pit.angle_360);
	
	pitch_pid_speed.f_param_init(&pitch_pid_speed, PID_Speed, 30000,30000,0,0,8000,0,50,0,0);
	pitch_pid_speed.target = -pitch_pid_pos.output;
	pitch_pid_speed.f_cal_pid(&pitch_pid_speed, moto_pit.ecd_raw_rate);


	// infantry
	trigger_pid_speed.f_param_init(&trigger_pid_speed, PID_Speed, 8000, 5000, 10, 0, 8000, 0, 1.5, 0.1, 0);
	// hero
	//trigger_pid_speed.f_param_init(&trigger_pid_speed, PID_Speed, 8000, 5000, 10, 0, 8000, 0, 5, 0, 0);
	

	kalman1_init(&yaw_filter, moto_yaw.speed_rpm, 1e2);
	kalman1_init(&pitch_filter, pitch_pid_speed.output, 1e3);
	
	
	
	mpu_device_init();
	init_quaternion();
	imu.yaw = 999;
	imu.rol = 999;
	
	pitch_angle = pitch_zero_position;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(send_control) {
			send_control = 0;
		} else {
			send_control = 1;
		}

    if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500){   //���500ms��û���յ�ң�������ݣ�֤��ң���������Ѿ�����

		// process controller input
    }else{
      set_spd = remote_control.ch4*8000/660;
			y_speed = -remote_control.ch4*8000/660;
			x_speed = -remote_control.ch3*8000/660;
			rotate_speed = -remote_control.ch1*8000.0/660.0;

			yaw_angle = yaw_zero_position - remote_control.ch1*20.0/660.0;

			pitch_angle = pitch_zero_position + remote_control.ch2*12.0/660.0;
    }

		if(is_mouse_moved()) {
			rotate_speed = -calculate_mouse_x_speed()*8000;
		}

		if(is_w_pressed())
		{
			y_speed = -0.2*8000;
		}

		if(is_s_pressed()){

			y_speed = 0.2*8000;
		}

		if(is_a_pressed()) {
			x_speed = 0.2*8000;
		}

		if(is_d_pressed()) {
			x_speed = -0.2*8000;
		}

		if(is_q_pressed()) {
			rotate_speed = 0.2*8000;
		}

		if(is_e_pressed()) {
			rotate_speed = -0.2*8000;
		}

		switch(remote_control.switch_left){
			case 1: //left up
			{
				htim4.Instance->CCR1 = 1800;
				htim4.Instance->CCR2 = 1800;
				break;
			}
			case 2: //left Down
			{
				htim4.Instance->CCR1 = 1800;
				htim4.Instance->CCR2 = 1800;
				break;
			}
			case 3: // left middle
				htim4.Instance->CCR1 = 1000;
				htim4.Instance->CCR2 = 1000;
		}

		// turret aiming
		if(niuYao){
			if (firstTime){
			  yaw_pid_lock_pos.target = heading_pos;
				//chassis_rotate_pos.target = yaw_zero_position - 30;
			}

			if(heading != 999 && heading_pos != 999) {



				if ( moto_yaw.angle_360 - yaw_zero_position < - 20 ){
					chassis_rotate_pos.target = yaw_zero_position + 20;

				}
				if ( moto_yaw.angle_360 - yaw_zero_position > 20 ){
					chassis_rotate_pos.target = yaw_zero_position - 20;
				}

				// Set the yaw_moto target to chassis_rotate target
				//yaw_pid_lock_pos.target = chassis_rotate_pos.target;

				yaw_pid_lock_pos.f_cal_pid(&yaw_pid_lock_pos, heading_pos);
				yaw_pid_lock_speed.target = -yaw_pid_lock_pos.output;


				yaw_pid_lock_speed.f_cal_pid( &yaw_pid_lock_speed, moto_yaw.ecd_raw_rate);
				yaw_pid_speed.output = yaw_pid_lock_speed.output;



		    chassis_rotate_pos.f_cal_pid(&chassis_rotate_pos, moto_yaw.angle_360);


				//Test: Close chassis rotate
				//chassis_rotate_pos.output=0;

				rotate_speed = -chassis_rotate_pos.output;
				wheel_speeds[0] = x_speed + y_speed + rotate_speed;
				wheel_speeds[1] = -x_speed + y_speed - rotate_speed;
				wheel_speeds[2] = -x_speed + y_speed + rotate_speed;
				wheel_speeds[3] = x_speed + y_speed - rotate_speed;

				wheel_speeds[0] = wheel_speeds[0] * -1;
				wheel_speeds[2] = wheel_speeds[2] * -1;
			}

			firstTime = 0;

		} else {
		  yaw_pid_pos.target = yaw_zero_position;
		  yaw_pid_pos.f_cal_pid(&yaw_pid_pos, moto_yaw.angle_360);

	  	yaw_pid_speed.target = yaw_pid_pos.output;
		  yaw_pid_speed.f_cal_pid(&yaw_pid_speed, moto_yaw.ecd_raw_rate);


		  wheel_speeds[0] = x_speed + y_speed + rotate_speed;
		  wheel_speeds[1] = -x_speed + y_speed - rotate_speed;
		  wheel_speeds[2] = -x_speed + y_speed + rotate_speed;
		  wheel_speeds[3] = x_speed + y_speed - rotate_speed;

		  wheel_speeds[0] = wheel_speeds[0] * -1;
		  wheel_speeds[2] = wheel_speeds[2] * -1;

			chassis_rotate_pos.output=0;
			chassis_rotate_pos.target = yaw_zero_position - 20;
			firstTime = 1;
		}

		// lose control
		if (moto_yaw.angle_360 > 110 ){
			end_Yaw = 1;
		}

//////////////////////////////////Calculate PID////////////////////////////////////////////////////////
    for(int i=0; i<4; i++)
    {
      motor_pid[i].target = wheel_speeds[i];
      motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i].speed_rpm);    //�����趨ֵ����PID���㡣
    }



		pitch_pid_pos.target = pitch_angle;
		pitch_pid_pos.f_cal_pid(&pitch_pid_pos, moto_pit.angle_360);

		pitch_pid_speed.target = pitch_pid_pos.output;
		pitch_pid_speed.f_cal_pid(&pitch_pid_speed, moto_pit.ecd_raw_rate);


		switch(remote_control.switch_right) {
			// right up, turn on trigger
			case 1:
			{
				trigger_pid_speed.target = test_speed;
				trigger_pid_speed.f_cal_pid(&trigger_pid_speed, moto_trigger.ecd_raw_rate);
				niuYao = 1;
				break;
			}
			// right down, turn  on trigger
			case 2:
			{
				trigger_pid_speed.target = test_speed;
				trigger_pid_speed.f_cal_pid(&trigger_pid_speed, moto_trigger.ecd_raw_rate);
			  niuYao = 1;
				break;
			}
			// right middle, turn off trigger
			case 3:
			{
				trigger_pid_speed.target = 0;
				trigger_pid_speed.f_cal_pid(&trigger_pid_speed, moto_trigger.ecd_raw_rate);
				niuYao = 0;
				break;
			}
		}
//////////////////////////////////////////////////////////////////////////////////////////////////////

		if (end_Yaw != 0){
		  yaw_pid_speed.output = 0;
			for(int i=0; i<4; i++){
        motor_pid[i].output = 0;
      }
			pitch_pid_speed.output = 0;
			chassis_rotate_pos.output=0;
			trigger_pid_speed.output = 0;
		}



		// set chassis motors' currents via CAN
    set_moto_current(&hcan1, motor_pid[0].output,   //��PID�ļ�����ͨ��CAN���͵����
                        motor_pid[1].output,
                        motor_pid[2].output,
                        motor_pid[3].output);


		// send calculated current to yaw, pitch, and trigger motors

		if(send_control) {
				send_gimbal_cur(&hcan1, yaw_pid_speed.output, pitch_pid_speed.output, trigger_pid_speed.output);
			}

			HAL_Delay(5);      //PID����Ƶ��100HZ



  }
  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	
	if(Uart1_Rx_Cnt >= 255)  
	{
		Uart1_Rx_Cnt = 0;
		memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff));
		HAL_UART_Transmit(&huart6, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);	
	}
	else
	{
		Uart1_RxBuff[Uart1_Rx_Cnt++] = aRxBuffer;   
	

		if((Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x0A)&&(Uart1_RxBuff[Uart1_Rx_Cnt-2] == 0x0D) && (Uart1_RxBuff[Uart1_Rx_Cnt-3] != 0x21)) 
		{
			//HAL_UART_Transmit(&huart6, (uint8_t *)&Uart1_RxBuff, Uart1_Rx_Cnt,0xFFFF);
			char * heading_string;
			heading_string = (char*) &Uart1_RxBuff;
			heading = (float)atof(heading_string);
			Uart1_Rx_Cnt = 0;
			memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff)); 
		}
		else if((Uart1_RxBuff[Uart1_Rx_Cnt-1] == 0x0A)&&(Uart1_RxBuff[Uart1_Rx_Cnt-2] == 0x0D) && (Uart1_RxBuff[Uart1_Rx_Cnt-3] == 0x21)) 
		{
			char * heading_string;
			heading_string = (char*) &Uart1_RxBuff;
			heading_pos = (float)atof(heading_string);
			Uart1_Rx_Cnt = 0;
			memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff));
      if( counter == 0 ){			
			  imuFirst = HAL_GetTick();
		  }
      if (counter == 1 ){
				imuLast = HAL_GetTick();
				imuCalTime = imuLast - imuFirst;
			}
			if (counter <= 3 ) {
				counter++;
			}
		}
		  
	 
	}
	
	HAL_UART_Receive_IT(&huart6, (uint8_t *)&aRxBuffer, 1); 
  
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
		
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
