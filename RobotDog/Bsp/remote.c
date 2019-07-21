#include "remote.h"

struct Remote_t remote;
uint8_t tmp_buf[9] = {0};

uint8_t rx_state;

void RemoteReceive(void)
{
	rx_state = NRF24L01_RxPacket(tmp_buf);
	remote.last_value = remote.value;
	remote.value.left_x = tmp_buf[0];
	remote.value.right_x = tmp_buf[1];
	remote.value.right_y = tmp_buf[2];
	remote.value.key_1 = tmp_buf[3];
	remote.value.key_2 = tmp_buf[4];
	remote.value.key_3 = tmp_buf[5];
	remote.value.key_4 = tmp_buf[6];
	remote.value.key_5 = tmp_buf[7];
	remote.value.key_6 = tmp_buf[8];
	
	if(remote.value.left_x != 0 || remote.value.right_x != 0 || remote.value.right_y != 0)
	{
		remote.state = 1;
	}
	else
	{
		remote.state = 0;
	}
}
