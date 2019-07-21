#ifndef BODY_TASK_H
#define BODY_TASK_H

#include "stm32f4xx.h"

#define BIG_LEG_LENGTH 50
#define SMALL_LEG_LENGTH 50
#define LONG_LENGTH 125.4f
#define SHORT_LENGTH 113.9f
#define SHORT_LONG_ANGLE 42.249f
#define DIAGNONAL_LENGTH 84.7f

#define ANGLE_TO_ENCODER 3.413f
#define RAD_TO_DEGREE 57.596f;

struct Leg_t
{
	uint8_t IDs[3];
	int position[3];
	int bias_position[3];
	int coefficient_1;
	int coefficient_2;
	float x;
	float y;
	float z;
	float delta_x;
	float delta_y;
	float delta_z;
	float angle_1;	//踝关节电机
	float angle_2;	//小腿关节
	float angle_3;	//大腿关节
	uint8_t count;
	enum
	{
		Down,
		Up,
	} state,last_state;
};

struct Body_t
{
	int vx;
	int vy;
	int rotate;
	float yaw;
	float pitch;
	float roll;
	enum
	{
		Attitude,
		Walk,
		Stop,
		Body_Stable
	} workstate;
};

extern struct Body_t body;

void BodyInit(void);
void LegChange(void);
void ServoSendData(void);
void BodyChange(void);

#endif
