/**
  ******************************************************************************
  * File Name          : PlateMotor.h
  * Description        : 拨盘电机任务
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
#ifndef FRAMEWORK_TASKS_PLATECONTROL_H
#define FRAMEWORK_TASKS_PLATECONTROL_H

#include "pid_regulator.h"

#define OneShoot (1011) //722 7扇拨盘  1011 5扇拨盘

void PlateMotorTask(void);
void ShootOneBullet(void);
void ShootRefModify(void);
int32_t GetQuadEncoderDiff(void);

typedef enum
{
	SINGLE_MULTI,
	CONSTENT_4
}LaunchMode_e;

void setLaunchMode(LaunchMode_e launchMode);
LaunchMode_e getLaunchMode(void);
void toggleLaunchMode(void);

#define SHOOT_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	7.5f,\
	0.05f,\
	0.6f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define SHOOT_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	50.0f,\
	0.5f,\
	0.0f,\
	0,\
	0,\
	0,\
	1000,\
	200,\
	100,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#endif


//拨盘电机驱动函数
//拨盘电机速度/位置控制,卡弹检测及处理
#ifndef DRIVERS_PLATEMOTOR_H
#define DRIVERS_PLATEMOTOR_H


typedef enum{REVERSE, FORWARD,}RotateDir_e;
void plateMotorInit(void);
void setPlateMotorDir(RotateDir_e dir);
RotateDir_e getPlateMotorDir(void);


#endif
