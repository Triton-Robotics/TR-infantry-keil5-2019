/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "can.h"
#include "bsp_can.h"


moto_measure_t moto_chassis[4] = {0};//4 chassis moto
moto_measure_t moto_yaw;
moto_measure_t moto_pit;
moto_measure_t moto_trigger;
long angle = 0;


void get_yaw_data(CAN_HandleTypeDef* hcan);
void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;
	static CanTxMsgTypeDef		Tx1Message;
	static CanRxMsgTypeDef 		Rx1Message;


	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.BankNumber = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		//err_deadloop(); //show error!
	}


	if(_hcan == &hcan1){
		_hcan->pTxMsg = &Tx1Message;
		_hcan->pRxMsg = &Rx1Message;
	}


}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{

	if(HAL_GetTick() - FlashTimer>500){
			
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		FlashTimer = HAL_GetTick();
		
	}

	//ignore can1 or can2.
	switch(_hcan->pRxMsg->StdId){
		case CAN_CHASSIS1_ID:
		case CAN_CHASSIS2_ID:
		case CAN_CHASSIS3_ID:
		case CAN_CHASSIS4_ID:
			{
				static u8 i;
				i = _hcan->pRxMsg->StdId - CAN_CHASSIS1_ID;
				
				get_moto_measure(&moto_chassis[i], _hcan);
			}
			break;
		case CAN_YAW_ID:
			get_moto_measure(&moto_yaw, _hcan);
			break;
		case CAN_PIT_ID:
			get_moto_measure (&moto_pit, _hcan);
			break;
		case CAN_TRIGGER_ID:
			get_moto_measure (&moto_trigger, _hcan);
			break;
	}
		
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);


}
void get_yaw_data(CAN_HandleTypeDef* hcan)
{
	angle = (long) (hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]);
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->last_total = ptr->total_angle;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->speed_rpm  = (hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
	ptr->real_current = (hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5])*5.f/16384.f;
	
	
	if(ptr->angle - ptr->last_angle > 4096) {
		ptr->round_cnt --;
		ptr->ecd_raw_rate = ptr->angle - ptr->last_angle - 8192;
	}
	else if (ptr->angle - ptr->last_angle < -4096) {
		ptr->round_cnt ++;
		ptr->ecd_raw_rate = ptr->angle - ptr->last_angle + 8192;
	} else {
		ptr->ecd_raw_rate = ptr->angle - ptr->last_angle;
	}
	
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
	
	ptr->angle_360 = ptr->total_angle / ENCODER_ANGLE_RATIO;
	
	ptr->last_time = ptr->this_time;
	ptr->this_time = HAL_GetTick();
	
	
	//ptr->ecd_speed = (ptr->total_angle - ptr->last_total) / (ptr->this_time - ptr->last_time);
	ptr->ecd_speed = (ptr->total_angle - ptr->last_total);
	
}

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

	hcan->pTxMsg->StdId = 0x200;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	hcan->pTxMsg->Data[0] = (iq1 >> 8);
	hcan->pTxMsg->Data[1] = iq1;
	hcan->pTxMsg->Data[2] = (iq2 >> 8);
	hcan->pTxMsg->Data[3] = iq2;
	hcan->pTxMsg->Data[4] = iq3 >> 8;
	hcan->pTxMsg->Data[5] = iq3;
	hcan->pTxMsg->Data[6] = iq4 >> 8;
	hcan->pTxMsg->Data[7] = iq4;
	
	HAL_CAN_Transmit(hcan, 100);
}	

void send_gimbal_cur(CAN_HandleTypeDef * hcan, int16_t yaw_iq, int16_t pit_iq, int16_t trigger_iq)
{
  hcan->pTxMsg->StdId   = 0x1ff;
  hcan->pTxMsg->IDE     = CAN_ID_STD;
  hcan->pTxMsg->RTR     = CAN_RTR_DATA;
  hcan->pTxMsg->DLC     = 8;
  hcan->pTxMsg->Data[0] = yaw_iq >> 8;
  hcan->pTxMsg->Data[1] = yaw_iq;
  hcan->pTxMsg->Data[2] = pit_iq >> 8;
  hcan->pTxMsg->Data[3] = pit_iq;
  hcan->pTxMsg->Data[4] = trigger_iq >> 8;
  hcan->pTxMsg->Data[5] = trigger_iq;
  hcan->pTxMsg->Data[6] = 0;
  hcan->pTxMsg->Data[7] = 0;
  HAL_CAN_Transmit(hcan, 100);
}
