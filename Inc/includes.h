/**
  ******************************************************************************
  * File Name          : includes.h
  * Description        : 统一包含文件
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __INCLUDES_H
#define __INCLUDES_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "RemoteTask.h"
#include "pid_regulator.h"
#include "ControlTask.h"
#include "IMUTask.h"
#include "CANTask.h"
//#include "pid_regulator.h"
#include "ShootTask.h"
#include "drivers_ramp.h"
#include "ManifoldTask.h"
#include "JudgeTask.h"
#include "UpperTask.h"

#endif /* __INCLUDES_H */