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

///*通过define使一套程序使用多台车*/
#define INFANTRY_1
//#define INFANTRY_4
//#define INFANTRY_5   

#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_GREEN_OFF()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_RED_OFF()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET)
#define LED_GREEN_ON()    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_RED_ON()      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET)

WorkState_e WorkState = PREPARE_STATE;
uint16_t prepare_time = 0;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

int16_t CMFLIntensity = 0, CMFRIntensity = 0, CMBLIntensity = 0, CMBRIntensity = 0;
int16_t yawIntensity = 0;		
int16_t pitchIntensity = 0;

//底盘PID初始化
void CMControlInit(void)
{
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}

//单个底盘电机的控制，下同
void ControlCMFL(void)
{			
	CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
											 + ChassisSpeedRef.left_right_ref*0.075 
											 + ChassisSpeedRef.rotate_ref*0.075;	
	CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;
			
			
	CM1SpeedPID.fdb = CMFLRx.RotateSpeed;

	CM1SpeedPID.Calc(&CM1SpeedPID);
	CMFLIntensity = CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output;
}

void ControlCMFR(void)
{			
	CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
										 + ChassisSpeedRef.left_right_ref*0.075 
										 + ChassisSpeedRef.rotate_ref*0.075;
	CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;
			
			
	CM2SpeedPID.fdb = CMFRRx.RotateSpeed;

	CM2SpeedPID.Calc(&CM2SpeedPID);
	CMFRIntensity = CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output;
}

void ControlCMBL(void)
{		
	CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 + ChassisSpeedRef.rotate_ref*0.075;
	CM3SpeedPID.ref = 160 * CM3SpeedPID.ref;
			
			
	CM3SpeedPID.fdb = CMBLRx.RotateSpeed;

	CM3SpeedPID.Calc(&CM3SpeedPID);
	CMBLIntensity = CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output;
}

void ControlCMBR(void)
{		
	CM4SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 + ChassisSpeedRef.rotate_ref*0.075;
	CM4SpeedPID.ref = 160 * CM4SpeedPID.ref;
			
			
	CM4SpeedPID.fdb = CMBRRx.RotateSpeed;

	CM4SpeedPID.Calc(&CM4SpeedPID);
	CMBRIntensity = CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output;
}

//状态机切换
void WorkStateFSM(void)
{
	switch (WorkState)
	{
		case PREPARE_STATE:
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(prepare_time<5000) prepare_time++;
			if(prepare_time == 3000) GYRO_RST();//开机三秒复位陀螺仪
			if(prepare_time == 5000)//开机五秒进入正常模式
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
	CMGMMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(CMFRIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)CMFRIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[4] = (uint8_t)(CMBLIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[5] = (uint8_t)CMBLIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[6] = (uint8_t)(CMBRIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[7] = (uint8_t)CMBRIntensity;

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
void setGMMotor()
{
	CanTxMsgTypeDef pData;
	CMGMMOTOR_CAN.pTxMsg = &pData;
	
	CMGMMOTOR_CAN.pTxMsg->StdId = GM_TXID;
	CMGMMOTOR_CAN.pTxMsg->ExtId = 0;
	CMGMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	CMGMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CMGMMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	CMGMMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(yawIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)yawIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(pitchIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)pitchIntensity;
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

#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)

#ifdef INFANTRY_5
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(8.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(40.0, 0.0, 15.0, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(30.0, 0.0, 5, 10000.0, 10000.0, 10000.0, 4000.0);
//手动标定0点
#define yaw_zero 2163//2200
#define pitch_zero 3275
#endif
#ifdef INFANTRY_4
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(8.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(8.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(40.0, 0.0, 15.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(70.0, 0.0, 5, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 2806//2840
#define pitch_zero 5009 
#endif
#ifdef INFANTRY_1
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(8.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(40.0, 0.0, 15.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(25.0, 0.0, 5, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 1150//100
#define pitch_zero 6400
#endif

float yawRealAngle = 0.0;
float pitchRealAngle = 0.0;
float gap_angle = 0.0;
//底盘跟随云台旋转控制
void ControlRotate(void)
{
	gap_angle  = (GMYAWRx.angle - yaw_zero) * 360 / 8192.0f;
  NORMALIZE_ANGLE180(gap_angle);	
	
	if(WorkState == NORMAL_STATE) 
	{
		CMRotatePID.ref = 0;
		CMRotatePID.fdb = gap_angle;
		CMRotatePID.Calc(&CMRotatePID);   
		ChassisSpeedRef.rotate_ref = CMRotatePID.output * 13 + rotate_forward * 90 + ChassisSpeedRef.forward_back_ref * 0.01 + ChassisSpeedRef.left_right_ref * 0.01;
	}
}

//控制云台YAW轴
void ControlYaw(void)
{
	uint16_t yawZeroAngle = yaw_zero;
			
	yawRealAngle = (GMYAWRx.angle - yawZeroAngle) * 360 / 8192.0f;
	NORMALIZE_ANGLE180(yawRealAngle);
			
	if(WorkState == NORMAL_STATE) 
	{
		yawRealAngle = -ZGyroModuleAngle;
	}
							
	yawIntensity = ProcessYawPID(yawAngleTarget, yawRealAngle, -gYroZs);
	
	ControlRotate();
}

//控制云台pitch轴
void ControlPitch(void)
{
	uint16_t pitchZeroAngle = pitch_zero;
				
	pitchRealAngle = -(GMPITCHRx.angle - pitchZeroAngle) * 360 / 8192.0;
	NORMALIZE_ANGLE180(pitchRealAngle);

	MINMAX(pitchAngleTarget, -9.0f, 32);
				
	pitchIntensity = ProcessPitchPID(pitchAngleTarget,pitchRealAngle,-gYroXs);
}

//主控制循环
void controlLoop()
{
	WorkStateFSM();
	
	if(WorkState == NORMAL_STATE )
	{
		//ControlYaw();
		ControlPitch();
		
		setGMMotor();
		
		ControlCMFL();
		ControlCMFR();
		ControlCMBL();
		ControlCMBR();
		
		setCMMotor();
		
		PlateMotorTask();
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
	else if (htim->Instance == htim10.Instance)  //10ms，处理上位机数据，优先级不高
	{
		#ifdef DEBUG_MODE
		zykProcessData();
		#endif
	}
}