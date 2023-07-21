/*
 * t12tip.h
 *
 *  Created on: Jun 1, 2023
 *      Author: RadoslawBorczuch
 */

#ifndef INC_T12TIP_H_
#define INC_T12TIP_H_

#include "main.h"
#include "stdbool.h"
#include "stm32f0xx_hal.h"

#define PWM_timer htim14
#define PWM_Chanel TIM_CHANNEL_1


void PWM_generation(uint8_t percentage);
uint32_t adc_read(uint32_t channel);


#endif /* INC_T12TIP_H_ */
