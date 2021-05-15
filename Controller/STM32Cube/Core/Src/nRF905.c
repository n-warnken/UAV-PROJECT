/*
 * nRF905.c
 *
 *  Created on: 10 Dec 2020
 *      Author: 61416
 */

#include "nRF905.h"

#define WC		0x00
#define RC		0x10
#define WTP		0x20
#define RTP		0x21
#define WTA		0x22
#define RTA		0x23
#define RRP		0x24

unsigned char config_info_buf[10] = {
		0x76,                   //CH_NO,868.4MHZ
		0x0E,                   //output power 10db, resend disable, Current Normal operation
		0x44,                   //4-byte address
		0x20,0x20,              //receive or send data length 32 bytes
		0xCC,0xCC,0xCC,0xCC,    //receiving address
		0x58,                   //CRC enable,8bit CRC,external clock disable,16MHZ Oscillator
};

unsigned int freq_tab[10] = {
    0x13e,
    0x076,
    0x076,
    0x076,
    0x177,
    0x02b,
    0x17f,
    0x17f,
    0x17f,
    0x100,
};

void nRF905_Init() {

	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF905_PWR_PORT, NRF905_PWR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_PIN_SET);

}

void nRF905_WriteConfig(unsigned char *conf_buf) {

	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&NRF905_SPI_PORT, (uint8_t *) WC, 1, 1000);
//	for (int i = 0; i < 10; i++) {
//
//		HAL_SPI_Transmit(&NRF905_SPI_PORT, (uint8_t *) conf_buf[i], 1, 1000);
//	}
	HAL_SPI_Transmit(&NRF905_SPI_PORT, conf_buf, 10, 1000);
	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET);

}

void nRF905_WriteConfigFreq(nrf905_freq_type freq_band) {

	if (freq_band < 10) {

		config_info_buf[0] = (unsigned char)freq_tab[freq_band];
		if (freq_tab[freq_band] & 0x100) {

			config_info_buf[1] |= 0x01;
		} else {

			config_info_buf[1] &= ~0x01;
		}
	}

	nRF905_WriteConfig(config_info_buf);
}

void nRF905_ReadConfig(unsigned char *conf_buf) {

	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&NRF905_SPI_PORT, (uint8_t *) RC, 1, 1000);
//	for (int i = 0; i < 10; i++) {
//
//		conf_buf[i] = HAL_SPI_Transmit(&NRF905_SPI_PORT, 0x00, 1, 1000);
//	}
	HAL_SPI_Receive(&NRF905_SPI_PORT, conf_buf, 10, 1000);

	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET);

}

void nRF905_RX(unsigned char *TxRxBuffer) {

	set_rx();
	while(check_ready() == 0);
	HAL_Delay(1);
	RxPacket(TxRxBuffer);
	HAL_Delay(1);
}

void nRF905_RXAddress(unsigned char *TxRxBuf, unsigned char *RxAddress) {

	if((config_info_buf[5] != RxAddress[0]) ||
			(config_info_buf[6] != RxAddress[1]) ||
			(config_info_buf[7] != RxAddress[2]) ||
			(config_info_buf[8] != RxAddress[3])) {

		config_info_buf[5] = RxAddress[0];
		config_info_buf[6] = RxAddress[1];
		config_info_buf[7] = RxAddress[2];
		config_info_buf[8] = RxAddress[3];

		nRF905_WriteConfig(config_info_buf);
	}

	set_rx();			// Set nRF905 in Rx mode
	while (check_ready() == 0);
	HAL_Delay(1);
	RxPacket(TxRxBuf);
	HAL_Delay(1);
}

void nRF905_TXAddress(unsigned char *TxRxBuf, unsigned char *TxAddress) {

	set_tx();
	HAL_Delay(1);
	// Send data by nRF905
	TxPacket(TxAddress, TxRxBuf);
}

void nRF905_TX(unsigned char *TxRxBuf) {

	set_tx();
	HAL_Delay(1);
	// Send data by nRF905
	TxPacket(config_info_buf+5, TxRxBuf);
}

void TxPacket(unsigned char *TxAddress, unsigned char *TxRxBuf) {

	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_RESET);	// Write payload command
	HAL_SPI_Transmit(&NRF905_SPI_PORT, (uint8_t *) WTP, 1, 1000);
//	for (int i = 0; i < 32; i++) {
//		// Write 32 bytes Tx data
//		HAL_SPI_Transmit(&NRF905_SPI_PORT, TxRxBuf[i], 1, 1000);
//	}
	HAL_SPI_Transmit(&NRF905_SPI_PORT, TxRxBuf, 32, 1000);
	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	// Spi enable for write a spi command
	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_RESET);	// Write address command
	HAL_SPI_Transmit(&NRF905_SPI_PORT, (uint8_t *) WTA, 1, 1000);

	// Write 4 bytes address
//	for (int i = 0; i < 4; i++){
//
//		HAL_SPI_Transmit(&NRF905_SPI_PORT, TxAddress[i], 1, 1000);
//	}
	HAL_SPI_Transmit(&NRF905_SPI_PORT, TxAddress, 4, 1000);
	// Spi disable
	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET);
	// Set TRX_CE high,start Tx data transmission, CE pulse
	HAL_GPIO_WritePin(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, GPIO_PIN_RESET);
}

void set_tx(void) {

	HAL_GPIO_WritePin(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_PIN_SET);

	HAL_Delay(1);
}

void set_rx(void) {

	HAL_GPIO_WritePin(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF905_TXEN_PORT, NRF905_TXEN_PIN, GPIO_PIN_RESET);

	HAL_Delay(1);
}

unsigned int check_ready(void) {

	if (HAL_GPIO_ReadPin(NRF905_DR_PORT, NRF905_DR_PIN) == GPIO_PIN_SET) {
		return 1;
	} else {

		return 0;
	}
}

void RxPacket(unsigned char *TxRxBuffer) {

	HAL_GPIO_WritePin(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&NRF905_SPI_PORT, (uint8_t *) RRP, 1, 1000);
	HAL_Delay(1);
//	for (int i = 0; i < 32; i++) {
//		TxRxBuffer[i] = HAL_SPI_Transmit(&NRF905_SPI_PORT, 0, 1, 1000);
//		HAL_Delay(1);
//	}
	HAL_SPI_Receive(&NRF905_SPI_PORT, TxRxBuffer, 32, 1000);

	HAL_GPIO_WritePin(NRF905_CSN_PORT, NRF905_CSN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NRF905_TRX_CE_PORT, NRF905_TRX_CE_PIN, GPIO_PIN_SET);

}





