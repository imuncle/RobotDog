#include "nrf24l01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xb0,0x43,0x10,0x10,0x01};
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xb0,0x43,0x10,0x10,0x01};

uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t byte)
{
  uint8_t d_read,d_send=byte;
  if(HAL_SPI_TransmitReceive(hspi,&d_send,&d_read,1,0xFF)!=HAL_OK)
  {
    d_read=0xFF;
  }
  return d_read; 
}

uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,uint8_t_ctr;
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_RESET);
	status=SPIx_ReadWriteByte(&hspi_NRF24L01,reg);
	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hspi_NRF24L01,*pBuf++);
  }
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_SET);
	return status;
}

uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_RESET);
	status=SPIx_ReadWriteByte(&hspi_NRF24L01,reg);
	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspi_NRF24L01,0XFF);
  }
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_SET);
	return status;
}

int NRF24L01_Check(void)
{
	uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	uint8_t i;
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+NRF24L01_TX_ADDR,buf,5);
	NRF24L01_Read_Buf(NRF24L01_TX_ADDR,buf,5);
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	
	if(i!=5) return 1;
	return 0;
}

uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_RESET);
	status =SPIx_ReadWriteByte(&hspi_NRF24L01,reg);
	SPIx_ReadWriteByte(&hspi_NRF24L01,value);
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_SET);
	return status;
}

uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_RESET);
	SPIx_ReadWriteByte(&hspi_NRF24L01,reg);
	reg_val=SPIx_ReadWriteByte(&hspi_NRF24L01,0XFF);
	HAL_GPIO_WritePin(GPIO_CSN, GPIO_CSN_PIN, GPIO_PIN_SET);
	return reg_val;
}

uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	HAL_GPIO_WritePin(GPIO_CE, GPIO_CE_PIN, GPIO_PIN_RESET);
	NRF24L01_Write_Buf(NRF24L01_WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	HAL_GPIO_WritePin(GPIO_CE, GPIO_CE_PIN, GPIO_PIN_SET);
	
	while(HAL_GPIO_ReadPin(GPIO_IRQ, GPIO_IRQ_PIN) != 0);
	
	sta = NRF24L01_Read_Reg(NRF24L01_STATUS);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_STATUS,sta);
	if(sta&NRF24L01_MAX_TX)
	{
		NRF24L01_Write_Reg(NRF24L01_FLUSH_TX,0xff);
		return NRF24L01_MAX_TX;
	}
	if(sta&NRF24L01_TX_OK) return NRF24L01_TX_OK;
	return 0xFF;
}

uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;
	sta = NRF24L01_Read_Reg(NRF24L01_STATUS);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_STATUS,sta);
	if(sta&NRF24L01_RX_OK)
	{
		NRF24L01_Read_Buf(NRF24L01_RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);
		NRF24L01_Write_Reg(NRF24L01_FLUSH_RX,0xff);
		return 0;
	}
	return sta;
}

void NRF24L01_TX_Mode(void)
{
	HAL_GPIO_WritePin(GPIO_CE, GPIO_CE_PIN, GPIO_PIN_RESET);
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+NRF24L01_TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+NRF24L01_RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_EN_AA,0x01);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_EN_RXADDR,0x01);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_SETUP_RETR,0xff);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_RF_CH,40);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_RF_SETUP,0x0f);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_CONFIG,0x0e);
	
	HAL_GPIO_WritePin(GPIO_CE, GPIO_CE_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
}

void NRF24L01_RX_Mode(void)
{
	HAL_GPIO_WritePin(GPIO_CE, GPIO_CE_PIN, GPIO_PIN_RESET);
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_CONFIG,0x0f);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_EN_AA,0x01);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_EN_RXADDR,0x01);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_RF_CH,40);
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_RF_SETUP,0x0f);
	
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+NRF24L01_RX_PW_P0,RX_PLOAD_WIDTH);

	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+NRF24L01_RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);
	
	HAL_GPIO_WritePin(GPIO_CE, GPIO_CE_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
}
