/*
 * hal_iss.h
 *
 *  Created on: Nov 28, 2020
 *      Author: 61416
 */

#ifndef INC_HAL_ISS_H_
#define INC_HAL_ISS_H_

#include <stdint.h>

#define HAL_ISS_SOURCE_1 0
#define HAL_ISS_SOURCE_2 1
#define HAL_ISS_SOURCE_3 2
#define HAL_ISS_SOURCE_4 3

void hal_iss_init(void);
void hal_iss_synchronsier(unsigned char signal_source_index);
uint32_t hal_iss_eventcounter_read(unsigned char signal_source_index);
uint32_t hal_iss_lasttimer_read(unsigned char signal_source_index);
void hal_iss_eventcounter_reset(unsigned char signal_source_index);
void hal_iss_lasttimer_reset(unsigned char signal_source_index);
void hal_iss_delaytimer_ctrl(unsigned char signal_source_index, int delay_value);


#endif /* INC_HAL_ISS_H_ */
