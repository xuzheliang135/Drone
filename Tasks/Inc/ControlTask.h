/**
  ******************************************************************************
  * File Name          : ControlTask.h
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "includes.h"

#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.7f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	6.0f,\
	0.0f,\
	1.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

typedef enum
{
	PREPARE_STATE,     	
	NORMAL_STATE,		    
	STOP_STATE        
}WorkState_e;

extern WorkState_e WorkState;
extern fw_PID_Regulator_t pitchPositionPID;
extern fw_PID_Regulator_t pitchSpeedPID;

extern fw_PID_Regulator_t yawPositionPID;
extern fw_PID_Regulator_t yawSpeedPID;

extern fw_PID_Regulator_t FLPositionPID;
extern fw_PID_Regulator_t FLSpeedPID;

extern float yawRealAngle;
extern float pitchRealAngle;

void CMControlInit(void);

#endif /*__ CONTROLTASK_H */
