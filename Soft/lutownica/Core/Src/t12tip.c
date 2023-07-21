/*
 * t12tip.c
 *
 *  Created on: Jun 1, 2023
 *      Author: RadoslawBorczuch
 */


#include "t12tip.h"
extern TIM_HandleTypeDef htim14;
extern ADC_HandleTypeDef hadc;



uint32_t adc_read(uint32_t channel)
{
	uint32_t value;
	ADC_ChannelConfTypeDef adc_ch;
	adc_ch.Channel = channel;
	adc_ch.Rank = 1;
	adc_ch.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc, &adc_ch);

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 10);
	value =HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	return value;
}

void PWM_generation(uint8_t percentage){
	HAL_TIM_PWM_Start(&PWM_timer, PWM_Chanel);
	if (percentage > 100) percentage=100;
	TIM14->CCR1 = percentage*655;
}




