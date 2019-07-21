#ifndef NRF24L01_H
#define NRF24L01_H

#include "spi.h"

#define hspi_NRF24L01 hspi1

#define GPIO_CE GPIOB
#define GPIO_CE_PIN GPIO_PIN_6
#define GPIO_CSN GPIOC
#define GPIO_CSN_PIN GPIO_PIN_12
#define GPIO_IRQ GPIOD
#define GPIO_IRQ_PIN GPIO_PIN_2

#define NRF24L01_READ_REG        0x00
#define NRF24L01_WRITE_REG       0x20
#define NRF24L01_RD_RX_PLOAD     0x61
#define NRF24L01_WR_TX_PLOAD     0xA0
#define NRF24L01_FLUSH_TX        0xE1
#define NRF24L01_FLUSH_RX        0xE2

#define NRF24L01_TX_ADDR         0x10
#define NRF24L01_RX_ADDR_P0      0x0A

#define NRF24L01_CONFIG          0x00
#define NRF24L01_EN_AA           0x01
#define NRF24L01_EN_RXADDR       0x02
#define NRF24L01_SETUP_RETR      0x04
#define NRF24L01_RF_CH           0x05
#define NRF24L01_RF_SETUP        0x06
#define NRF24L01_STATUS          0x07

#define NRF24L01_MAX_TX  	0x10
#define NRF24L01_TX_OK   	0x20
#define NRF24L01_RX_OK   	0x40

#define NRF24L01_RX_PW_P0        0x11

#define TX_ADR_WIDTH    5
#define RX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  32
#define RX_PLOAD_WIDTH  32

int NRF24L01_Check(void);
void NRF24L01_TX_Mode(void);
void NRF24L01_RX_Mode(void);
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);

#endif
