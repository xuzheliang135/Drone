#ifndef __JUDGE_H__
#define __JUDGE_H__
#include "includes.h"

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
	
typedef enum
{
	ONLINE,
	OFFLINE
}JudgeState_e;

typedef struct 
{
		uint8_t bulletType;
		uint8_t bulletFreq;
		float bulletSpeed;
}extShootData_t;

typedef struct 
{
    float chassisVolt;
		float chassisCurrent;
		float chassisPower;
    float chassisPowerBuffer;
		uint16_t shooterHeat0;
    uint16_t shooterHeat1;
}extPowerHeatData_t;

extern extShootData_t extShootData;
extern extPowerHeatData_t extPowerHeat;

#endif
