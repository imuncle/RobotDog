#include "body_task.h"
#include "scs009.h"
#include "remote.h"
#include "math.h"
#include "mpu6050.h"

struct Leg_t FR_Leg;
struct Leg_t FL_Leg;
struct Leg_t BL_Leg;
struct Leg_t BR_Leg;

struct Body_t body;

// 静止站立状态时12个舵机的位置
int init_position[12] = {550, 641, 796,
						 525, 491, 79,
						 600, 864, 766,
						 274, 342, 207};

int Abs(float a)
{
	return (a>=0)?a:-a;
}

void BodyInit(void)
{
	// 初始化右前腿
	FR_Leg.IDs[0] = 1;
	FR_Leg.IDs[1] = 2;
	FR_Leg.IDs[2] = 3;
	FR_Leg.bias_position[0] = init_position[0];
	FR_Leg.bias_position[1] = init_position[1];
	FR_Leg.bias_position[2] = init_position[2];
	FR_Leg.coefficient_1 = 1;
	FR_Leg.coefficient_2 = -1;
	FR_Leg.x = 0.0f;
	FR_Leg.y = 0.0f;
	FR_Leg.z = 86.6f;
	
	// 初始化左前腿
	FL_Leg.IDs[0] = 4;
	FL_Leg.IDs[1] = 5;
	FL_Leg.IDs[2] = 6;
	FL_Leg.bias_position[0] = init_position[3];
	FL_Leg.bias_position[1] = init_position[4];
	FL_Leg.bias_position[2] = init_position[5];
	FL_Leg.coefficient_1 = 1;
	FL_Leg.coefficient_2 = 1;
	FL_Leg.x = 0.0f;
	FL_Leg.y = 0.0f;
	FL_Leg.z = 86.6f;
	
	// 初始化左后腿
	BL_Leg.IDs[0] = 10;
	BL_Leg.IDs[1] = 11;
	BL_Leg.IDs[2] = 12;
	BL_Leg.bias_position[0] = init_position[9];
	BL_Leg.bias_position[1] = init_position[10];
	BL_Leg.bias_position[2] = init_position[11];
	BL_Leg.coefficient_1 = -1;
	BL_Leg.coefficient_2 = 1;
	BL_Leg.x = 0.0f;
	BL_Leg.y = 0.0f;
	BL_Leg.z = 86.6f;
	
	// 初始化右后腿
	BR_Leg.IDs[0] = 7;
	BR_Leg.IDs[1] = 8;
	BR_Leg.IDs[2] = 9;
	BR_Leg.bias_position[0] = init_position[6];
	BR_Leg.bias_position[1] = init_position[7];
	BR_Leg.bias_position[2] = init_position[8];
	BR_Leg.coefficient_1 = -1;
	BR_Leg.coefficient_2 = -1;
	BR_Leg.x = 0.0f;
	BR_Leg.y = 0.0f;
	BR_Leg.z = 86.6f;
	
	body.workstate = Stop;
}

// 根据腿末端坐标计算腿部三个舵机的位置
void LegCalc(struct Leg_t* leg)
{
	leg->x =  leg->delta_x;
	leg->y =  leg->delta_y;
	leg->z =  86.6f + leg->delta_z;
	float L;
	L = sqrt(leg->x * leg->x + leg->y * leg->y + leg->z * leg->z);
	leg->angle_1 = asin(leg->y/L);
	leg->angle_2 = acos(L/100) - asin(leg->x/L);
	leg->angle_3 = acos(1 - L*L/5000);
	leg->angle_1 *= RAD_TO_DEGREE;
	leg->angle_2 *= RAD_TO_DEGREE;
	leg->angle_3 *= RAD_TO_DEGREE;
	leg->angle_3 = 180 - leg->angle_3;
	leg->position[0] = leg->bias_position[0] + leg->coefficient_1 * leg->angle_1 * ANGLE_TO_ENCODER;
	leg->position[1] = leg->bias_position[1] + leg->coefficient_2 * leg->angle_2 * ANGLE_TO_ENCODER;
	leg->position[2] = leg->bias_position[2] + leg->coefficient_2 * leg->angle_3 * ANGLE_TO_ENCODER;
}

void LegChange(void)
{
	LegCalc(&FR_Leg);
	LegCalc(&FL_Leg);
	LegCalc(&BL_Leg);
	LegCalc(&BR_Leg);
}

// 机器人姿态解析，根据机器人的yaw、pitch、roll计算四条腿末端坐标
void AttitudeParse(void)
{
	float yaw_angle = 42.249f - body.yaw;
	yaw_angle /= RAD_TO_DEGREE;
	float yaw_delta_x = 56.95f - 84.7f*sin(yaw_angle);
	float yaw_delta_y = 84.7f * cos(yaw_angle) - 62.7f;
	
	float pitch_angle = 90.0f - Abs(body.pitch)/2;
	pitch_angle -= 53.489f;
	pitch_angle /= RAD_TO_DEGREE;
	float tmp_length = sqrt(22210.76f*(1-cos(body.pitch/57.596f)));
	float pitch_delta_x = tmp_length * cos(pitch_angle);
	float pitch_delta_z = tmp_length * sin(pitch_angle);
	if(body.pitch < 0)
	{
		pitch_delta_x = -pitch_delta_x;
		pitch_delta_z = -pitch_delta_z;
	}
	
	tmp_length = sqrt(20834.785f*(1-cos(body.roll/57.596f)));
	pitch_angle = 90.0f - Abs(body.roll)/2;
	pitch_angle -= 56.084f;
	pitch_angle /= RAD_TO_DEGREE;
	float roll_delta_y = tmp_length * cos(pitch_angle);
	float roll_delta_z = tmp_length * sin(pitch_angle);
	if(body.roll < 0)
	{
		roll_delta_y = -roll_delta_y;
		roll_delta_z = -roll_delta_z;
	}
	
	FR_Leg.delta_x = yaw_delta_x - pitch_delta_x;
	FR_Leg.delta_y = -yaw_delta_y + roll_delta_y;
	FR_Leg.delta_z = pitch_delta_z - roll_delta_z;
	
	FL_Leg.delta_x = -yaw_delta_x - pitch_delta_x;
	FL_Leg.delta_y = -yaw_delta_y + roll_delta_y;
	FL_Leg.delta_z = pitch_delta_z + roll_delta_z;
	
	BL_Leg.delta_x = -yaw_delta_x - pitch_delta_x;
	BL_Leg.delta_y = yaw_delta_y + roll_delta_y;
	BL_Leg.delta_z = -pitch_delta_z + roll_delta_z;
	
	BR_Leg.delta_x = yaw_delta_x - pitch_delta_x;
	BR_Leg.delta_y = yaw_delta_y + roll_delta_y;
	BR_Leg.delta_z = -pitch_delta_z - roll_delta_z;
}

void ClearLegData(struct Leg_t * leg)
{
	leg->delta_x = 0;
	leg->delta_y = 0;
	leg->delta_z = 0;
}

// 机器人单条腿的行走运动
void Move(struct Leg_t * leg)
{
	uint8_t cycle = 36;
	int StepLength = 2*body.vx;
	int LR_StepLength = 2*body.vy;
	int R_StepLength = 2*body.rotate;
	// 设置遥控器死区
	if(Abs(body.vx) < 2 && Abs(body.vy) < 2 && Abs(body.rotate) < 2)
	{
		leg->delta_x = 0;
		leg->count = 0;
		return;
	}
	if(leg->state != leg->last_state) leg->count = 0;
	leg->count++;
	if(leg->state == Up)
	{
		leg->delta_x = -StepLength+leg->count*2*StepLength/cycle;
		leg->delta_y = -LR_StepLength+leg->count*2*LR_StepLength/cycle;
		if(leg == &FR_Leg || leg == &FL_Leg)
		{
			leg->delta_y += -R_StepLength+leg->count*2*R_StepLength/cycle;
		}
		else
		{
			leg->delta_y -= -R_StepLength+leg->count*2*R_StepLength/cycle;
		}
	}
	else if(leg->state == Down)
	{
		leg->delta_x = StepLength-leg->count*2*StepLength/(3*cycle);
		leg->delta_y = LR_StepLength-leg->count*2*LR_StepLength/(3*cycle);
		if(leg == &FR_Leg || leg == &FL_Leg)
		{
			leg->delta_y += R_StepLength-leg->count*2*R_StepLength/(3*cycle);
		}
		else
		{
			leg->delta_y -= R_StepLength-leg->count*2*R_StepLength/(3*cycle);
		}
	}
}

// 依次控制四条腿的行走状态
void ListLeg(void)
{
	static int count = 0;
	if(Abs(body.vx) < 2 && Abs(body.vy) < 2 && Abs(body.rotate) < 2)
	{
		count = 0;
		FR_Leg.state = Down;
		BL_Leg.state = Down;
		FL_Leg.state = Down;
		BR_Leg.state = Down;
		FR_Leg.delta_z = 0;
		BL_Leg.delta_z = 0;
		FL_Leg.delta_z = 0;
		BR_Leg.delta_z = 0;
		return;
	}
	count ++;
	uint8_t cycle = 36;
	FR_Leg.last_state = FR_Leg.state;
	BL_Leg.last_state = BL_Leg.state;
	FL_Leg.last_state = FL_Leg.state;
	BR_Leg.last_state = BR_Leg.state;
	if(count<cycle)
	{
		FR_Leg.state = Up;
		BL_Leg.state = Down;
		FL_Leg.state = Down;
		BR_Leg.state = Down;
		if(count < cycle/2)
		{
			FR_Leg.delta_z = -count*40/cycle;
		}
		else
		{
			FR_Leg.delta_z = -(cycle-count)*40/cycle;
		}
	}
	else if(count < 2 * cycle)
	{
		FR_Leg.state = Down;
		BL_Leg.state = Up;
		FL_Leg.state = Down;
		BR_Leg.state = Down;
		if(count < cycle*3/2)
		{
			BL_Leg.delta_z = -(count-cycle)*40/cycle;
		}
		else
		{
			BL_Leg.delta_z = -(2*cycle-count)*40/cycle;
		}
	}
	else if(count < 3 * cycle)
	{
		FR_Leg.state = Down;
		BL_Leg.state = Down;
		FL_Leg.state = Up;
		BR_Leg.state = Down;
		if(count < cycle*5/2)
		{
			FL_Leg.delta_z = -(count-2*cycle)*40/cycle;
		}
		else
		{
			FL_Leg.delta_z = -(3*cycle-count)*40/cycle;
		}
	}
	else if(count < 4 * cycle)
	{
		FR_Leg.state = Down;
		BL_Leg.state = Down;
		FL_Leg.state = Down;
		BR_Leg.state = Up;
		if(count < cycle*7/2)
		{
			BR_Leg.delta_z = -(count-3*cycle)*40/cycle;
		}
		else
		{
			BR_Leg.delta_z = -(4*cycle-count)*40/cycle;
		}
	}
	else
	{
		count = 0;
	}
}

// 机器人四条腿控制函数入口
void BodyChange(void)
{
	if(body.workstate == Stop)
	{
		body.yaw = 0;
		body.pitch = 0;
		body.roll = 0;
		body.vx = 0;
		body.vy = 0;
		body.rotate = 0;
		
		ClearLegData(&FR_Leg);
		ClearLegData(&FL_Leg);
		ClearLegData(&BL_Leg);
		ClearLegData(&BR_Leg);
	}
	else if(body.workstate == Attitude)
	{
		body.yaw = remote.value.right_x - 16;
		body.pitch = remote.value.right_y - 16;
		body.roll = remote.value.left_x - 16;
		if(body.pitch > 10) body.pitch = 10;
		if(body.pitch < -10) body.pitch = -10;
		if(body.roll > 10) body.roll = 10;
		if(body.roll < -10) body.roll = -10;
		AttitudeParse();
	}
	else if(body.workstate == Walk)
	{
		body.vx = remote.value.right_y - 16;
		body.vy = remote.value.right_x - 16;
		body.rotate = remote.value.left_x - 16;
		ListLeg();
		Move(&FR_Leg);
		Move(&FL_Leg);
		Move(&BL_Leg);
		Move(&BR_Leg);
	}
	else if(body.workstate == Body_Stable)
	{
		body.pitch = -mpu6500.angle.pitch;
		body.roll = mpu6500.angle.roll;
		if(body.pitch > 5) body.pitch = 5;
		if(body.pitch < -5) body.pitch = -5;
		if(body.roll > 5) body.roll = 5;
		if(body.roll < -5) body.roll = -5;
		AttitudeParse();
	}
}

// 控制舵机
void ServoSendData(void)
{
	snycWrite(FR_Leg.IDs, 3, 0x2A, FR_Leg.position);
	snycWrite(FL_Leg.IDs, 3, 0x2A, FL_Leg.position);
	snycWrite(BL_Leg.IDs, 3, 0x2A, BL_Leg.position);
	snycWrite(BR_Leg.IDs, 3, 0x2A, BR_Leg.position);
}
