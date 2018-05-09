/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)


//手动标定0点
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(15.0, 10.0, 500, 1000.0, 10000.0, 23000.0, 23000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(40.0, 1.0, 15.0, 10000.0, 10000.0, 10000.0, 4000.0);

fw_PID_Regulator_t FLPositionPID = fw_PID_INIT(8.0, 0.0, 0.0, 400.0, 400.0, 400.0, 400.0);
fw_PID_Regulator_t FLSpeedPID = fw_PID_INIT(40.0, 1.0, 15.0, 400.0, 400.0, 400.0, 400.0);

fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(40.0, 10.0, 500, 1000.0, 10000.0, 23000.0, 23000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(20.0, 1.0, 10 , 20000.0, 20000.0, 20000.0, 20000.0);
#define YAW_ZERO 4442
#define PITCH_ZERO 7308

#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_GREEN_OFF()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_RED_OFF()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET)
#define LED_GREEN_ON()    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_RED_ON()      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET)
#define PREPARE_TIME 2000//启动时间
#define ABS(x) x>0?x:-x

void stopCMFL(void);

WorkState_e WorkState = PREPARE_STATE;
uint16_t prepare_time = 0;

int16_t CMFLIntensity = 0;
int16_t yawIntensity = 0;		
int16_t pitchIntensity = 0;

float yawRealAngle = 0.0;
float pitchRealAngle = 0.0;

int FLcount=0;
int FLflag=1;
int rotateFlag=0;
extern float FLAngleTarget;
float FLRealAngle = 0.0;
extern int shootFlag;
extern Shoot_State_e ShootState;
void preventLock(){
	static int count=0;
	static int reversed=0;
	static int time_count=0;
	if(CMFLRx.RotateSpeed > 20||CMFLRx.RotateSpeed < -20)count+=1;
	else count=0;
	if(count>=250)CMFLIntensity = -3000,reversed = 1;
	if(reversed == 1)time_count += 1;
	else time_count = 0;
	if(time_count >= 100)reversed = 0,count = 0,CMFLIntensity = 0;
}
void stopCMFL(){
	if(CMFLRx.RotateSpeed>20||CMFLRx.RotateSpeed<-20)CMFLIntensity = -CMFLRx.RotateSpeed*2;
	else CMFLIntensity = 0;
}
//拨弹电机的控制
void ControlCMFL(void)
{	
	FLRealAngle=CMFLRx.angle*360/8191;
	while(FLAngleTarget<0)FLAngleTarget+=360;
	while(FLAngleTarget>360)FLAngleTarget-=360;
	if(CMFLRx.RotateSpeed>5&&FLRealAngle>340)rotateFlag=1;
	else if(CMFLRx.RotateSpeed>0&&FLRealAngle<20&&rotateFlag==1){
		rotateFlag=0;
		FLcount+=1;
	}
	if(FLcount==3)shootFlag=1,FLflag=1,FLcount=0;
	if(FLflag==1)CMFLIntensity = ProcessPitchPID(FLAngleTarget,FLRealAngle,CMFLRx.RotateSpeed );//-gYroXs
	else CMFLIntensity = 300;
}
void calculateGMMotor(){
	static int i=0;
	static int pitch_zero=0;
	if(i==0)pitch_zero=GMPITCHRx.angle,i=1;
	yawRealAngle = -(GMYAWRx.angle - YAW_ZERO) * 360 / 8192.0f;
	NORMALIZE_ANGLE180(yawRealAngle);		
	MINMAX(yawAngleTarget, -120,120);
	
	pitchRealAngle = -(GMPITCHRx.angle - pitch_zero) * 360 / 8192.0f;
	NORMALIZE_ANGLE180(pitchRealAngle);
	MINMAX(pitchAngleTarget, -90,120);
}
//控制云台YAW轴,PITCH轴
void ControlGMMotor(void)
{	
	calculateGMMotor();
	yawIntensity = -ProcessYawPID(yawAngleTarget, yawRealAngle,0);
	pitchIntensity = -ProcessPitchPID(pitchAngleTarget,pitchRealAngle,0);
}
//慢速复位云台到零点（暂未使用）
void resetGMMotor(){
	static float yawAngel,pitchAngel;
	can1_type = 1;
	
	calculateGMMotor();
	if(prepare_time==1)yawAngel=yawRealAngle,pitchAngel=pitchRealAngle;
	yawAngleTarget=(0-yawAngel)*prepare_time/PREPARE_TIME+yawAngel;
	pitchAngleTarget=(0-pitchAngel)*prepare_time/PREPARE_TIME+pitchAngel;
	
	yawIntensity = -ProcessYawPID(yawAngleTarget, yawRealAngle,0);
	pitchIntensity = -ProcessPitchPID(pitchAngleTarget,pitchRealAngle,0);
	setGMMotor();
}
//状态机切换
void WorkStateFSM(void)
{
	switch (WorkState)
	{
		case PREPARE_STATE:
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(prepare_time<PREPARE_TIME) prepare_time++;//,resetGMMotor();
			if(prepare_time == PREPARE_TIME)//开2秒进入正常模式
			{
				WorkState = NORMAL_STATE;
				prepare_time = 0;
			}
		}break;
		case NORMAL_STATE:
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case STOP_STATE://紧急停止
		{
			if (inputmode == REMOTE_INPUT)
			{
				WorkState = PREPARE_STATE;
				RemoteTaskInit();
			}
		}break;
	}
}
//底盘电机CAN信号控制
void setCMMotor()
{
	CanTxMsgTypeDef pData;
	CMGMMOTOR_CAN.pTxMsg = &pData;
	
	CMGMMOTOR_CAN.pTxMsg->StdId = CM_TXID;
	CMGMMOTOR_CAN.pTxMsg->ExtId = 0;
	CMGMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	CMGMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CMGMMOTOR_CAN.pTxMsg->DLC = 0x08;
	CMGMMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(CMFLIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)CMFLIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[2] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[3] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[4] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[5] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[6] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can1_update == 1 && can1_type == 0)
	{
		//CAN通信前关中断
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		//CAN通信后开中断，防止中断影响CAN信号发送
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}

//云台电机CAN信号控制
void setGMMotor(void)
{
	CanTxMsgTypeDef pData;
	CMGMMOTOR_CAN.pTxMsg = &pData;
	
	CMGMMOTOR_CAN.pTxMsg->StdId = GM_TXID;
	CMGMMOTOR_CAN.pTxMsg->ExtId = 0;
	CMGMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	CMGMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CMGMMOTOR_CAN.pTxMsg->DLC = 0x08;
//	pitchIntensity=ChassisSpeedRef.forward_back_ref*90;
//	yawIntensity=0;
	CMGMMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(pitchIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)pitchIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(yawIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)yawIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[4] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[5] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[6] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}


//主控制循环
void controlLoop()
{
	WorkStateFSM();
	
	if(WorkState != STOP_STATE)
	{
		ControlGMMotor();
//		if(ChassisSpeedRef.forward_back_ref>=200) CMFLIntensity = 8000;
		setGMMotor();
		
//		ControlCMFL();
		if(shootFlag==1)stopCMFL();
		setCMMotor();
//		if(CMFLIntensity == 1500)preventLock();
	}
}

//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		//主循环在时间中断中启动
		controlLoop();
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == htim7.Instance)
	{
		rc_cnt++;
		if (rc_update)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame) WorkState = PREPARE_STATE;
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
				rc_first_frame = 1;
			}
			rc_update = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	if(GPIO_Pin&GPIO_PIN_4){
		shootFlag=1;
	}
	
//PE4口中断函数，单发控制
}
