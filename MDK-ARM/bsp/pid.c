/**
  ******************************************************************************
  * @file    pid.c
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   ��ÿһ��pid�ṹ�嶼Ҫ�Ƚ��к��������ӣ��ٽ��г�ʼ��
  ******************************************************************************
  * @attention Ӧ�����ö��ײ��(d)��̨������ȶ�
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "stm32f4xx.h"

#define ABS(x)		((x>0)? x: -x) 

PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;
extern int isMove;

/*������ʼ��--------------------------------------------------------------*/
static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	float  target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;             //û�õ�
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

void pid_easy_init(
	PID_TypeDef * pid,
	PID_ID id,
	uint16_t period,
	float kp,
	float ki,
	float kd)
	{
		pid->id = id;
	
		pid->ControlPeriod = period;
		
		pid->kp = kp;
		pid->ki = ki;
		pid->kd = kd;
		
		pid->DeadBand = 0;
		
		pid->target = 0;
	}
	
void pid_set_maxout(PID_TypeDef * pid, uint16_t maxout)
{
	pid->MaxOutput = maxout;
	pid->IntegralLimit = maxout;
	pid->Max_Err = maxout;
}


/*��;���Ĳ����趨--------------------------------------------------------------*/
void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void pid_set_deadBand(PID_TypeDef * pid, float deadband)
{
	pid->DeadBand = deadband;
}

/*pid����-----------------------------------------------------------------------*/

	
static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	//�Ƿ��������
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//�����Ƿ񳬳�����
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid����� ?-dout or + dout?
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //�˲���
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}


	return pid->output;
}

/*pid�ṹ���ʼ����ÿһ��pid������Ҫ����һ��-----------------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}

/* frequency control */
int pid_need_compute(PID_TypeDef* pid, int sampletime)
{
	
	return (HAL_GetTick() - pid->lasttime >= sampletime) ? 1 : 0;
}
