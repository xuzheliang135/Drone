/**
  ******************************************************************************
  * File Name          : UpperTask.c
  * Description        : 上位机处理任务，进行串口调试
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef DEBUG_MODE
//--------------------底层接收驱动部分-------------------//
uint8_t data;
uint8_t buf[REC_LEN];
uint16_t RX_STA=0;
void zykReceiveData(uint8_t data);
void ctrlUartRxCpltCallback()
{
	zykReceiveData(data);
	HAL_UART_AbortReceive((&CTRL_UART));
	if(HAL_UART_Receive_DMA(&CTRL_UART, &data, 1) != HAL_OK)
	{
		Error_Handler();
		printf( "CtrlUart error" );
	} 
}

void ctrlUartInit(){
	if(HAL_UART_Receive_DMA(&CTRL_UART, &data, 1) != HAL_OK){
		Error_Handler();
		printf( "InitCtrlUart error" );
	} 
}

void zykReceiveData(uint8_t data)
{
		if((RX_STA&0x8000)==0)
		{
			if(RX_STA&0x4000)
			{
				if(data!=0x0a)
				{
						RX_STA=0;
				}
				else 
				{
					RX_STA|=0x8000;	
					buf[RX_LEN]='\0';   
				}
			}
			else 
			{	
				if(data==0x0d)RX_STA|=0x4000;
				else
				{
					buf[RX_STA&0X3FFF]=data ;
					RX_STA++;
					if(RX_STA>(REC_LEN-1))RX_STA=0; 
				}		 
			}
		}
}

//--------------------数据解析协议部分-------------------//
uint8_t ComProtocal(char*rxbuf,char*head,char*end,char* separater,char dataout[][15])
{
    uint8_t headlength,endlength,datalength,totallength;
    uint8_t i=0;
    char temp[50]="";
    char*splitchar;
    headlength=strlen(head);
    endlength=strlen(end);
    totallength=strlen(rxbuf);
		datalength=totallength-headlength-endlength;
    strncpy(temp,rxbuf,headlength);
    temp[headlength]='\0';
    if(strcmp(temp,head))
    {
        return 0;
    }
    strncpy(temp,rxbuf+totallength-endlength,endlength);
    temp[endlength]='\0';
    if(strcmp(temp,end))
    {
        return 0;
    }
    strncpy(temp,rxbuf+headlength,datalength);
    temp[datalength]='\0';

    splitchar=strtok((char*)temp,separater);
    while(splitchar!=NULL)
    {
        sprintf(dataout[i++],"%s",splitchar);
        splitchar=strtok(NULL,separater);
    }
    return i;
}

//--------------------任务循环部分-------------------//
void zykProcessData()
{	
	  //printf("ok");
		if(RX_DONE)
		{
		char data[10][15];
		//printf(buf);
		/////////// GM CONTROL ////////////////
		if(strcmp(buf,"U")==0)
		{
			printf("UP\r\n");
			pitchAngleTarget+=5;
		}
		else if(strcmp(buf,"D")==0)
		{
			printf("DOWN\r\n");
			pitchAngleTarget-=5;
		}
		else if(strcmp(buf,"OK")==0)
		{
			printf("OK\r\n");
		}
		if(strcmp(buf,"L")==0)
		{
			printf("LEFT\r\n");
			yawAngleTarget+=5;
		}
		else if(strcmp(buf,"R")==0)
		{
			printf("RIGHT\r\n");
			yawAngleTarget-=5;
		}
		/////////// GM PID
		else if(ComProtocal(buf,"#GMYPP","$","@",data))
		{
			float p=atof(data[0]);
			yawPositionPID.kp=p;
			printf("Yaw position P change to %f\r\n",yawPositionPID.kp);
		}
		else if(ComProtocal(buf,"#GMYPI","$","@",data))
		{
			float p=atof(data[0]);
			yawPositionPID.ki=p;
			printf("Yaw position I change to %f\r\n",yawPositionPID.ki);
		}
		else if(ComProtocal(buf,"#GMYPD","$","@",data))
		{
			float p=atof(data[0]);
			yawPositionPID.kd=p;
			printf("Yaw position D change to %f\r\n",yawPositionPID.kd);
		}
		else if(ComProtocal(buf,"#GMYSP","$","@",data))
		{
			float p=atof(data[0]);
			yawSpeedPID.kp=p;
			printf("Yaw speed P change to %f\r\n",yawSpeedPID.kp);
		}
		else if(ComProtocal(buf,"#GMYSI","$","@",data))
		{
			float p=atof(data[0]);
			yawSpeedPID.ki=p;
			printf("Yaw speed I change to %f\r\n",yawSpeedPID.ki);
		}
		else if(ComProtocal(buf,"#GMYSD","$","@",data))
		{
			float p=atof(data[0]);
			yawSpeedPID.kd=p;
			printf("Yaw speed D change to %f\r\n",yawSpeedPID.kd);
		}
				/////////// GM PID ￡¨pitch￡?
		else if(ComProtocal(buf,"#GMPPP","$","@",data))
		{
			float p=atof(data[0]);
			pitchPositionPID.kp=p;
			printf("Pitch position P change to %f\r\n",pitchPositionPID.kp);
		}
		else if(ComProtocal(buf,"#GMPPI","$","@",data))
		{
			float p=atof(data[0]);
			pitchPositionPID.ki=p;
			printf("Pitch position I change to %f\r\n",pitchPositionPID.ki);
		}
		else if(ComProtocal(buf,"#GMPPD","$","@",data))
		{
			float p=atof(data[0]);
			pitchPositionPID.kd=p;
			printf("Pitch position D change to %f\r\n",pitchPositionPID.kd);
		}
		else if(ComProtocal(buf,"#GMPSP","$","@",data))
		{
			float p=atof(data[0]);
			pitchSpeedPID.kp=p;
			printf("Pitch speed P change to %f\r\n",pitchSpeedPID.kp);
		}
		else if(ComProtocal(buf,"#GMPSI","$","@",data))
		{
			float p=atof(data[0]);
			pitchSpeedPID.ki=p;
			printf("Pitch speed I change to %f\r\n",pitchSpeedPID.ki);
		}
		else if(ComProtocal(buf,"#GMPSD","$","@",data))
		{
			float p=atof(data[0]);
			pitchSpeedPID.kd=p;
			printf("Pitch speed D change to %f\r\n",pitchSpeedPID.kd);
		}
		///////////////////UPPER
		else if(strcmp(buf,"RD1")==0)
		{
			float realSpeed2=-gYroZs;
			printf("#DATA%.2f@%.2f@%.2f$",yawPositionPID.output,realSpeed2,yawRealAngle);
		}
		else if(strcmp(buf,"RD2")==0)
		{
			float realSpeed2=-gYroXs;
			printf("#DATA%.2f@%.2f@%.2f$",pitchPositionPID.output,realSpeed2,pitchRealAngle);
		}
		strcpy(buf,"\0");
		RX_STA=0;
	}
}
#endif