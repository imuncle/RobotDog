#include "soft_i2c.h"

#define IIC_SDA_H() HAL_GPIO_WritePin(IIC_GPIO, SDA, GPIO_PIN_SET)
#define IIC_SDA_L() HAL_GPIO_WritePin(IIC_GPIO, SDA, GPIO_PIN_RESET)
#define IIC_SCL_H() HAL_GPIO_WritePin(IIC_GPIO, SCL, GPIO_PIN_SET)
#define IIC_SCL_L() HAL_GPIO_WritePin(IIC_GPIO, SCL, GPIO_PIN_RESET)
#define IIC_SDA_Read() HAL_GPIO_ReadPin(IIC_GPIO, SDA)

void IIC_Delay(uint16_t time)
{
	uint16_t count;
	count = time * 6;
	while(count --);
}

void IIC_SDA_Out(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IIC_GPIO, &GPIO_InitStruct);
}

void IIC_SDA_In(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IIC_GPIO, &GPIO_InitStruct);
}

void IIC_Start(void)
{
	IIC_SDA_Out();
	IIC_SDA_H();
	IIC_SCL_H();
	IIC_Delay(1);
	IIC_SDA_L();
	IIC_Delay(1);
	IIC_SCL_L();
}

void IIC_Stop(void)
{
	IIC_SDA_Out();
	IIC_SCL_L();
	IIC_SDA_L();
	IIC_Delay(1);
	IIC_SCL_H();
	IIC_SDA_H();
	IIC_Delay(1);
}

void IIC_Ack(uint8_t re)					     
{
	IIC_SDA_Out();
	if(re)
	   IIC_SDA_H();
	else
	   IIC_SDA_L();
	IIC_SCL_H();
	IIC_Delay(1);
	IIC_SCL_L();
	IIC_Delay(1);
}

int IIC_WaitAck(void)
{
	uint16_t Out_Time=1000;
    
    IIC_SDA_H();
	IIC_SDA_In();
	IIC_Delay(1);
	IIC_SCL_H();
	IIC_Delay(1);
	while(IIC_SDA_Read())
	{
		if(--Out_Time)
		{
			IIC_Stop();
            return 0xff;
		}
	}
	IIC_SCL_L();
    return 0;
}

void IIC_WriteBit(uint8_t Temp)
{
	uint8_t i;
	IIC_SDA_Out();
	IIC_SCL_L();
	for(i=0;i<8;i++)
	{
		if(Temp&0x80)
		{
			IIC_SDA_H();
		}
		else
		{
			IIC_SDA_L();
		}
		Temp<<=1;
		IIC_Delay(1);
		IIC_SCL_H();
		IIC_Delay(1);
		IIC_SCL_L();
	}
}

uint8_t IIC_ReadBit(void)
{
	uint8_t i,Temp=0;
	IIC_SDA_In();
	for(i=0;i<8;i++)
	{
		IIC_SCL_L();
		IIC_Delay(1);
		IIC_SCL_H();
		Temp<<=1;
		if(IIC_SDA_Read())
		   Temp++;
		IIC_Delay(1);
	}
	IIC_SCL_L();
	return Temp;
}

int IIC_WriteData(uint8_t dev_addr,uint8_t reg_addr,uint8_t data)
{
	IIC_Start();
    
	IIC_WriteBit(dev_addr);
	if(IIC_WaitAck() == 0xff)
    {
        return 0xff;
    }
    
	IIC_WriteBit(reg_addr);
	if(IIC_WaitAck() == 0xff)
    {
        return 0xff;
    }

    IIC_WriteBit(data);
    if(IIC_WaitAck() == 0xff)
    {
        return 0xff;
    }

	IIC_Stop();
    return 0;
}

int IIC_ReadData(uint8_t dev_addr, uint8_t reg_addr, uint8_t * pdata, uint8_t count)
{
	uint8_t i;
	IIC_Start();
	
    IIC_WriteBit(dev_addr);
	if(IIC_WaitAck() == 0xff)
    {
        return 0xff;
    }
    
    IIC_WriteBit(reg_addr);
	if(IIC_WaitAck() == 0xff)
    {
        return 0xff;
    }
	
    IIC_Start();
    
    IIC_WriteBit(dev_addr+1);
	if(IIC_WaitAck() == 0xff)
    {
        return 0xff;
    }
    
    for(i=0;i<(count-1);i++)
    {
        *pdata=IIC_ReadBit();
        IIC_Ack(0);
        pdata++;
    }

    *pdata=IIC_ReadBit();
    IIC_Ack(1); 
    
    IIC_Stop(); 
    
    return 0;    
}
