/*
 * hal_iss.c
 *
 *  Created on: Nov 28, 2020
 *      Author: 61416
 */

#include "hal_iss.h"
#include "stm32f4xx_hal.h"

uint32_t iss_eventcounter_val[4];
uint32_t iss_lasttime_val[4];
uint32_t iss_delay_val[4];

void hal_iss_init(void) {

	iss_eventcounter_val[0] = 0;
	iss_eventcounter_val[1] = 0;
	iss_eventcounter_val[2] = 0;
	iss_eventcounter_val[3] = 0;
	iss_lasttime_val[0] = 0;
	iss_lasttime_val[1] = 0;
	iss_lasttime_val[2] = 0;
	iss_lasttime_val[3] = 0;
	iss_delay_val[0] = 10;
	iss_delay_val[1] = 10;
	iss_delay_val[2] = 10;
	iss_delay_val[3] = 10;

}

void hal_iss_synchronsier(unsigned char signal_source_index) {

	if (iss_lasttime_val[signal_source_index] + iss_delay_val[signal_source_index]
		< HAL_GetTick()) {
		iss_eventcounter_val[signal_source_index] += 1;
		}
	iss_lasttime_val[signal_source_index] = HAL_GetTick();
}

uint32_t hal_iss_eventcounter_read(unsigned char signal_source_index) {

	return iss_eventcounter_val[signal_source_index];
}

uint32_t hal_iss_lasttimer_read(unsigned char signal_source_index) {

	return iss_lasttime_val[signal_source_index];
}

void hal_iss_eventcounter_reset(unsigned char signal_source_index) {

	iss_eventcounter_val[signal_source_index] = 0;
}

void hal_iss_lasttimer_reset(unsigned char signal_source_index) {

	iss_lasttime_val[signal_source_index] = 0;
}

void hal_iss_delaytimer_ctrl(unsigned char signal_source_index, int delay_value) {

	iss_delay_val[signal_source_index] = delay_value;
}
