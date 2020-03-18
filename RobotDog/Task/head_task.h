#ifndef HEAD_TASK_H
#define HEAD_TASK_H

#include "stm32f4xx.h"

struct Head_t
{
	uint8_t IDs[3];
	int position[3];
	int bias_position[3];
	float x;
	float y;
	float z;
	float delta_x;
	float delta_y;
	float delta_z;
	float angle_1;	//踝关节电机
	float angle_2;	//小腿关节
	float angle_3;	//大腿关节
	enum
	{
		Work,	// 遥控器直接控制头部末端位置
		Head_Stop,	// 遥控器无法控制头部
		Stable	// 遥控器控制身体姿态，同时头部稳定在原位
	} workstate;
};

extern struct Head_t head;

void HeadInit(void);
void HeadCalc(void);
void HeadChange(void);
void HeadServoSendData(void);

#endif
