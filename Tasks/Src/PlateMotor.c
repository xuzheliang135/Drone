/**
  ******************************************************************************
  * File Name          : PlateMotor.c
  * Description        : 拨盘电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 定时循环
	* 编码器模式对编码器脉冲计数
	* PWM波控制速度
  ******************************************************************************
  */

#include "includes.h"

PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;

extern FrictionWheelState_e FrictionWheelState;
extern Shoot_State_e ShootState;
static int s_count_bullet = 0;
int stuck = 0;	//卡弹标志位，未卡弹为false，卡弹为true

LaunchMode_e launchMode = SINGLE_MULTI;
RotateDir_e PlateMotorDir = FORWARD;

int RotateAdd =0;
int32_t last_fdb = 0x0;
int32_t this_fdb = 0x0;

void PlateMotorTask()
{
	static int s_count_1s = 0;
	static int s_stuck_cnt = 0;
	
		s_count_1s++;
		if(s_count_1s == 500)
		{
			s_count_1s = 0;
			s_count_bullet = 0;
		}
		if(inputmode == KEY_MOUSE_INPUT)//键鼠模式下直接在数据处理程序中实现//   
		{
			//ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;//打一发弹编码器输出脉冲数
			//遥控器一帧14ms，此任务循环7次，最终是打了7发
			//ShootOneBullet();
		}

	//遥控器输入模式下，只要处于发射态，就一直转动
		if(ShootState == SHOOTING && inputmode == REMOTE_INPUT) 
		{
			RotateAdd += 4;
			//fw_printfln("ref = %f",ShootMotorPositionPID.ref);
			if(RotateAdd>OneShoot)
			{
				ShootOneBullet();
				RotateAdd = 0;
			}
		}
		else if(ShootState == NOSHOOTING && inputmode == REMOTE_INPUT)
		{
			RotateAdd = 0;
		}

		//卡弹检测
		//当参考值和反馈值长时间保持较大差别时，判定卡弹
		if(ShootMotorPositionPID.ref-ShootMotorPositionPID.fdb>OneShoot*3 || ShootMotorPositionPID.ref-ShootMotorPositionPID.fdb<OneShoot*(-3))
		{
			++s_stuck_cnt;
			if(s_stuck_cnt>250)
			{
				s_stuck_cnt = 0;
				stuck = 1;
			}
		}
		else
		{
			s_stuck_cnt = 0;
		}
		
		if(stuck == 1)
		{
			stuck = 0;
			ShootRefModify();
		}
		
		if(FrictionWheelState == FRICTION_WHEEL_ON)//拨盘转动前提条件：摩擦轮转动
		{
			this_fdb = GetQuadEncoderDiff(); 
			
			//fw_printfln("last_fdb = %d",last_fdb);
			//fw_printfln("this_fdb = %d",this_fdb);
			
			if(this_fdb<last_fdb-10000 && getPlateMotorDir()==FORWARD)	//cnt寄存器溢出判断 正转
			{
				ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb+(65536+this_fdb-last_fdb);
			}
			else if(this_fdb>last_fdb+10000 && getPlateMotorDir()==REVERSE)	//cnt寄存器溢出判断 反转
			{
				ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb-(65536-this_fdb+last_fdb);
			}
			else if((this_fdb-last_fdb)<500 && (this_fdb-last_fdb)>-500)
				ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb + this_fdb-last_fdb;

			last_fdb = this_fdb;
			//fw_printfln("fdb = %f",ShootMotorPositionPID.fdb);
			ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
	
			if(ShootMotorPositionPID.output<0) //反转
			{
				setPlateMotorDir(REVERSE);
				ShootMotorPositionPID.output = -ShootMotorPositionPID.output;
			}
			else
				setPlateMotorDir(FORWARD);
			
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ShootMotorPositionPID.output);
		}
		
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);//摩擦轮不转，立刻关闭拨盘
		}
		
//		vTaskDelayUntil( &xLastWakeTimeQZK, ( 2 / portTICK_RATE_MS ) );//这里进入阻塞态等待2ms   //RTOS
}

void ShootOneBullet()
{
	s_count_bullet ++;
	if(s_count_bullet <= 4)
	{
	ShootMotorPositionPID.ref = ShootMotorPositionPID.ref + OneShoot;
	}
}

void ShootRefModify()
{
	while(ShootMotorPositionPID.ref>ShootMotorPositionPID.fdb && ShootMotorPositionPID.ref>2*OneShoot)
		ShootMotorPositionPID.ref = ShootMotorPositionPID.ref - 2*OneShoot;
	
}

int32_t GetQuadEncoderDiff(void)
{
  int32_t cnt = 0;    
	cnt = __HAL_TIM_GET_COUNTER(&htim5) - 0x0;
	return cnt;
}

void setLaunchMode(LaunchMode_e lm)
{
	launchMode = lm;
}

LaunchMode_e getLaunchMode()
{
	return launchMode;
}

void toggleLaunchMode()
{
	if(getLaunchMode() == SINGLE_MULTI)
		setLaunchMode(CONSTENT_4);
	else
		setLaunchMode(SINGLE_MULTI);
}



//拨盘电机驱动函数
//拨盘电机速度/位置控制,卡弹检测及处理
void plateMotorInit(void){
	setPlateMotorDir(FORWARD);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim5, 0x0);
//	fw_printf("PMInit Success\t\n");
	
}

void setPlateMotorDir(RotateDir_e dir)
{
	if(dir==FORWARD)
	{
		PlateMotorDir = FORWARD;
		HAL_GPIO_WritePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin,GPIO_PIN_SET);
	}
	if(dir==REVERSE)
	{
		PlateMotorDir = REVERSE;
		HAL_GPIO_WritePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin,GPIO_PIN_RESET);
	}
}

RotateDir_e getPlateMotorDir(){
	return PlateMotorDir;
}
