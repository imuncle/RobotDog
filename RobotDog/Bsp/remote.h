#ifndef REMOTE_H
#define REMOTE_H

#include "nrf24l01.h"

struct RemoteValue_t
{
	int left_x;
	int right_x;
	int right_y;
	int key_1;
	int key_2;
	int key_3;
	int key_4;
	int key_5;
	int key_6;
};

struct Remote_t
{
	struct RemoteValue_t value;
	struct RemoteValue_t last_value;
	uint8_t state;
};

extern struct Remote_t remote;

void RemoteReceive(void);

#endif
