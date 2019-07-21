#include "scs009.h"
#include "usart.h"

void snycWrite(uint8_t ID[], uint8_t IDN, uint8_t MemAddr, int *nDat)
{
	uint8_t i;
	uint8_t nLen = 2;
	uint8_t tmp_position[2];
	uint8_t mesLen = ((nLen+1)*IDN+4);
	uint8_t checkSum = 0;
	uint8_t bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = 0x83;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	HAL_UART_Transmit(&huart2, bBuf, 7, 10);

	checkSum = 0xfe + mesLen + 0x83 + MemAddr + nLen;
	for(i=0; i<IDN; i++){
		HAL_UART_Transmit(&huart2, ID+i, 1, 10);
		tmp_position[0] = nDat[i] >> 8;
		tmp_position[1] = nDat[i];
		HAL_UART_Transmit(&huart2, tmp_position, nLen, 10);
		checkSum += ID[i];
//		for(j=0; j<nLen; j++){
//			checkSum += nDat[j];
//		}
		checkSum += tmp_position[0];
		checkSum += tmp_position[1];
	}
	checkSum = ~checkSum;
	HAL_UART_Transmit(&huart2, &checkSum, 1, 10);
}

void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
	uint8_t i;
	uint8_t msgLen = 2;
	uint8_t bBuf[6];
	uint8_t CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		HAL_UART_Transmit(&huart2, bBuf, 6, 10);
		
	}else{
		bBuf[3] = msgLen;
		HAL_UART_Transmit(&huart2, bBuf, 5, 10);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		HAL_UART_Transmit(&huart2, nDat, nLen, 10);
	}
	CheckSum = ~CheckSum;
	HAL_UART_Transmit(&huart2, &CheckSum, 1, 10);
}
