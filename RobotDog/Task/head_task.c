#include "head_task.h"
#include "body_task.h"
#include "remote.h"
#include "math.h"
#include "scs009.h"

struct Head_t head;

int head_init_position[3] = {505, 500, 518};

void HeadInit(void)
{
	head.IDs[0] = 13;
	head.IDs[1] = 14;
	head.IDs[2] = 15;
	head.bias_position[0] = head_init_position[0];
	head.bias_position[1] = head_init_position[1];
	head.bias_position[2] = head_init_position[2];
	head.x = 70.71f;
	head.y = 0.0f;
	head.z = 0.0f;
	
	head.workstate = Head_Stop;
}

void HeadChange(void)
{
	if(head.workstate == Work)
	{
		head.delta_x = remote.value.right_y - 16;
		head.delta_y = remote.value.right_x - 16;
		head.delta_z = remote.value.left_x - 16;
	}
	else if(head.workstate == Head_Stop)
	{
		head.delta_x = 0;
		head.delta_y = 0;
		head.delta_z = 0;
	}
	else if(head.workstate == Stable)
	{
		float Length = sqrt(27852.72f*(1-cos(body.yaw/57.596f)));
		float theta = 90 - body.yaw/2;
		theta /= RAD_TO_DEGREE;
		head.delta_y = -Length * sin(theta);
		head.delta_x = -Length * cos(theta);
		if(body.yaw < 0)
		{
			head.delta_y = -head.delta_y;
		}
		Length = sqrt(29529.36f*(1-cos(body.pitch/57.596f)));
		theta = 90 - body.pitch/2;
		theta /= RAD_TO_DEGREE;
		head.delta_x -= Length * cos(theta);
		head.delta_z = -Length * sin(theta);
		if(body.pitch < 0)
		{
			head.delta_z = -head.delta_z;
		}
	}
}

void HeadCalc(void)
{
	float L;
	head.x = 70.71f + head.delta_x;
	head.y = head.delta_y;
	head.z = head.delta_z;
	L = sqrt(head.x * head.x + head.y * head.y + head.z * head.z);
	head.angle_1 = asin(head.y/L);
	head.angle_2 = asin(head.z/L)+acos(L/100);
	head.angle_3 = acos(1 - L*L/5000);
	head.angle_1 *= RAD_TO_DEGREE;
	head.angle_2 *= RAD_TO_DEGREE;
	head.angle_3 *= RAD_TO_DEGREE;
	head.angle_2 = 90 - head.angle_2;
	head.angle_3 = 90 - head.angle_3;
	head.position[0] = head.bias_position[0] + head.angle_1 * ANGLE_TO_ENCODER;
	head.position[1] = head.bias_position[1] + head.angle_2 * ANGLE_TO_ENCODER;
	head.position[2] = head.bias_position[2] - head.angle_3 * ANGLE_TO_ENCODER;
}

void HeadServoSendData(void)
{
	snycWrite(head.IDs, 3, 0x2A, head.position);
}
