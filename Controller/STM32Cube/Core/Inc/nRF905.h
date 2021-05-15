/*
 * nRF905.h
 *
 *  Created on: 10 Dec 2020
 *      Author: 61416
 */

#ifndef INC_NRF905_H_
#define INC_NRF905_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#define NRF905_SPI_PORT 	   hspi2

#define NRF905_PWR_PORT		   GPIOB
#define NRF905_PWR_PIN         GPIO_PIN_15

#define NRF905_TXEN_PORT       GPIOB
#define NRF905_TXEN_PIN        GPIO_PIN_13

#define NRF905_TRX_CE_PORT     GPIOB
#define NRF905_TRX_CE_PIN      GPIO_PIN_14

#define NRF905_CSN_PORT        GPIOB
#define NRF905_CSN_PIN         GPIO_PIN_2

#define NRF905_CD_PORT         GPIOB
#define NRF905_CD_PIN          GPIO_PIN_1

#define NRF905_DR_PORT         GPIOA
#define NRF905_DR_PIN          GPIO_PIN_12

extern SPI_HandleTypeDef NRF905_SPI_PORT;

typedef enum
{
    US = 0,             /** 908.42Mhz */
    EUROPE = 1,         /** 868.42MHz */
    AFRICA  = 2,         /** 868.42MHz */
    CHINA = 3,          /** 868.42MHz */
    HK = 4,             /** 919.82MHz */
    JAPAN = 5,          /** 853.42MHz */
    AUSTRALIA = 6,      /** 921.42MHz */
    NEW_ZEALAND = 7,    /** 921.42MHz */
    BRASIL = 8,         /** 921.42MHz */
    RUSSIA = 9,         /** 896.00MHz */
} nrf905_freq_type;

extern void nRF905_Init();
extern void nRF905_WriteConfig(unsigned char *conf_buf);
extern void nRF905_WriteConfigFreq(nrf905_freq_type freq_band);
extern void nRF905_ReadConfig(unsigned char *conf_buf);
extern void nRF905_RX(unsigned char *TxRxBuffer);
extern void nRF905_RXAddress(unsigned char *TxRxBuf, unsigned char *RxAddress);
extern void nRF905_TXAddress(unsigned char *TxRxBuf, unsigned char *TxAddress);
extern void nRF905_TX(unsigned char *TxRxBuf);

void TxPacket(unsigned char *TxAddress, unsigned char *TxRxBuf);
void set_tx(void);
void set_rx(void);
unsigned int check_ready(void);
void RxPacket(unsigned char *TxRxBuffer);

#endif /* INC_NRF905_H_ */
