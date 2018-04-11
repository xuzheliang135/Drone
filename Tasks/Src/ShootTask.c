/**
  ******************************************************************************
  * File Name          : ShootTask.c
  * Description        : 射击任务（摩擦轮控制）
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
FrictionWheelState_e FrictionWheelState;
Shoot_State_e ShootState;
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;		//摩擦轮斜坡
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//键盘速度斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
uint16_t remoteShootDelay = 500;
static uint32_t RotateCNT = 0;	//长按连发计数
static uint16_t CNT_1s = 75;		//用于避免四连发模式下两秒内连射8发过于密集的情况
static uint16_t CNT_250ms = 18;	//???????????


void InitUserTimer(void)
{
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_2);
}

//手柄模式射击控制函数	
void SetFrictionWheelSpeed(uint16_t x)
{
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);//配置数目不够
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}
	
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	switch(FrictionWheelState)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   
			{
				ShootState = NOSHOOTING;
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_START_TURNNING;	 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				ShootState = NOSHOOTING;
				SetFrictionWheelSpeed(1000);
				FrictionWheelState = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				//SetFrictionWheelSpeed(1300); 
				//SetFrictionWheelSpeed(1000);
				//g_friction_wheel_state = FRICTION_WHEEL_ON; 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					FrictionWheelState = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				FrictionWheelState = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				ShootState = NOSHOOTING;
			}
			else if(sw->switch_value_raw == 2)
			{
				ShootState = SHOOTING;
			}
			else
			{
				ShootState = NOSHOOTING;
			}					 
		} break;				
	}
}

//键鼠模式射击控制函数
void MouseShootControl(Mouse *mouse)
{
	++CNT_1s;
	++CNT_250ms;
	static int16_t closeDelayCount = 0;   
	switch(FrictionWheelState)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(mouse->last_press_r == 0 && mouse->press_r == 1)   
			{
				ShootState = NOSHOOTING;
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_START_TURNNING;	 
				//LASER_ON(); 
				closeDelayCount = 0;
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   
			{
				//LASER_OFF();//zy0802
				FrictionWheelState = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				ShootState = NOSHOOTING;
			}
			else
			{
		    /*摩擦轮转速修改FRICTION_WHEEL_MAX_DUTY*/
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					FrictionWheelState = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   //
			{
				//LASER_OFF();//zy0802
				FrictionWheelState = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				ShootState = NOSHOOTING;
			}			
			else if(mouse->last_press_l == 0 && mouse->press_l== 1)  //检测鼠标左键单击动作
			{
				ShootState = SHOOTING;
				/*if(getLaunchMode() == SINGLE_MULTI && FrictionWheelState == FRICTION_WHEEL_ON)		//?????,??????
				{
					if(CNT_250ms>17)
					{
						CNT_250ms = 0;
						//shootOneGolf();
						
					}
				}
				else if(getLaunchMode() == CONSTENT_4 && FrictionWheelState == FRICTION_WHEEL_ON)	//??????,??????
				{
					
					if(CNT_1s>75)
					{
						CNT_1s = 0;
						//shootOneGolf();
						//shootOneGolf();
						//shootOneGolf();
						//shootOneGolf();
					}
				}*/
			}
			else if(mouse->last_press_l == 0 && mouse->press_l== 0)	//松开鼠标左键的状态
			{
				ShootState = NOSHOOTING;	
				RotateCNT = 0;			
			}			
			else if(mouse->last_press_l == 1 && mouse->press_l== 1 /*&& getLaunchMode() == SINGLE_MULTI*/)//【单发模式下】长按，执行：
			{
				RotateCNT+=50;
				/*if(RotateCNT>=OneShoot)
				{
					shootOneGolf();
					RotateCNT = 0;
				}*/
					
		}
				
		} break;				
	}	
	mouse->last_press_r = mouse->press_r;
	mouse->last_press_l = mouse->press_l;
}