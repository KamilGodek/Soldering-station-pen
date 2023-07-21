/*
 * gui.c
 *
 *  Created on: 23 kwi 2023
 *      Author: Jaroslaw
 */
#include "gui.h"


void MainScrean(uint16_t *setTemp, uint16_t currnetTemp,uint8_t step, bool *screanState, bool status,uint8_t power){
	if(HAL_GPIO_ReadPin(SW_BACK_GPIO_Port, SW_BACK_Pin)==GPIO_PIN_RESET){

		*setTemp -= step;
		*setTemp = (*setTemp>400)?0:*setTemp;
		Buzz();
	}

	if(HAL_GPIO_ReadPin(SW_NEXT_GPIO_Port, SW_NEXT_Pin)==GPIO_PIN_RESET){
		*setTemp += step;
		*setTemp = (*setTemp>400)?400:*setTemp;
		Buzz();
	}
	if(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET){
		*screanState=true;
		Buzz();
		while(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET);
	}
	DrawMainScrean(*setTemp,currnetTemp,status,power);
}



void DrawMainScrean(uint16_t setTemp, uint16_t currnetTemp,bool status,uint8_t power){
	ssd1306_Fill(Black);
	PrintCurrentTemperature(currnetTemp);
	PrintSetTemperature(setTemp);
	PrintDeviceStatus(status);
	PrintPower(power);
	ssd1306_UpdateScreen();
}

void PrintSetTemperature(uint16_t setTemp){
	char stringSetTemperature[6] ;
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("SET:", Font_7x10, 1);
	stringSetTemperature[0] = '0' + ((int) setTemp % 1000 / 100);
	stringSetTemperature[1] = '0' + ((int) setTemp % 100 / 10);
	stringSetTemperature[2] = '0' + ((int) setTemp % 10);
	stringSetTemperature[3] = '*';
	stringSetTemperature[4] = 'C';
	stringSetTemperature[5] = '\0';
	ssd1306_SetCursor(28, 0);
	ssd1306_WriteString(stringSetTemperature, Font_7x10, 1);
}

void PrintCurrentTemperature(uint16_t currnetTemp){
	char stringCurrentTemperature[6];
	stringCurrentTemperature[0] = '0' + ((int) currnetTemp % 1000 / 100);
	stringCurrentTemperature[1] = '0' + ((int) currnetTemp % 100 / 10);
	stringCurrentTemperature[2] = '0' + ((int) currnetTemp % 10);
	stringCurrentTemperature[3] = '*';
	stringCurrentTemperature[4] = 'C';
	stringCurrentTemperature[5] = '\0';
	ssd1306_SetCursor(73, 2);
	ssd1306_WriteString(stringCurrentTemperature, Font_11x18, 1);
}

void PrintPower(uint8_t power){
	char stringPower[4] ;
	ssd1306_SetCursor(73, 22);
	ssd1306_WriteString("POW:", Font_7x10, 1);
	stringPower[0] = '0' + ((int) power % 100 / 10);
	stringPower[1] = '0' + ((int) power % 10);
	stringPower[2] = 'W';
	stringPower[3] = '\0';
	ssd1306_SetCursor(102, 22);
	ssd1306_WriteString(stringPower, Font_7x10, 1);
}

void PrintDeviceStatus(bool status){
	ssd1306_SetCursor(0, 22);
	if(status==true)ssd1306_WriteString("WORKING", Font_7x10, 1);
	if(status==false)ssd1306_WriteString("RDY2WORK", Font_7x10, 1);

}

void Buzz(){
  HAL_GPIO_WritePin(Haptic_GPIO_Port, Haptic_Pin, GPIO_PIN_SET);
  HAL_Delay(20);
  HAL_GPIO_WritePin(Haptic_GPIO_Port, Haptic_Pin, GPIO_PIN_RESET);
  HAL_Delay(20);
}

void MenuScrean(bool *screanState, uint8_t *step,uint8_t *menuState){
	ssd1306_Fill(Black);
	if(HAL_GPIO_ReadPin(SW_NEXT_GPIO_Port, SW_NEXT_Pin)==GPIO_PIN_RESET){
		*menuState=*menuState+1;
		//*menuState = (*menuState>2)?0:*menuState;
		if(*menuState>2)*menuState=0;
		Buzz();
		while(HAL_GPIO_ReadPin(SW_NEXT_GPIO_Port, SW_NEXT_Pin)==GPIO_PIN_RESET);
	}
	if(HAL_GPIO_ReadPin(SW_BACK_GPIO_Port, SW_BACK_Pin)==GPIO_PIN_RESET){

		*menuState=*menuState-1;
		//*menuState = (*menuState>2)?2:*menuState;
		if(*menuState>2)*menuState=2;
		Buzz();
		while(HAL_GPIO_ReadPin(SW_BACK_GPIO_Port, SW_BACK_Pin)==GPIO_PIN_RESET);
	}

	switch (*menuState){
	case 0:
		ssd1306_SetCursor(11,0);
		ssd1306_WriteString("< Home screen >", Font_7x10, 1);
		if(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET){
			Buzz();
			*screanState=false;
			while(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET);
		}
		break;
	case 1:
			ssd1306_SetCursor(11,0);
			ssd1306_WriteString("< Select Step >", Font_7x10, 1);

				//Select_Step(*step);

		break;
	}
	ssd1306_UpdateScreen();

}

//void BackToMainScrean (bool *screanState){
//	ssd1306_SetCursor(11,0);
//	ssd1306_WriteString("< Home screen >", Font_7x10, 1);
//	if(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET){
//		Buzz();
//		*screanState=false;
//		while(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET);
//	}
//}
//
//void Select_Step(uint8_t *step){
//	uint8_t state =0;
//	uint8_t *step=1;
//	while(1){
//	if(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET){
//		state=state+1;
//		if(state>2)state=0;
//		Buzz();
//		while(HAL_GPIO_ReadPin(SW_SELECT_GPIO_Port, SW_SELECT_Pin)==GPIO_PIN_RESET);
//	}
//
//
//	switch(state){
//	case 0:
//		ssd1306_SetCursor(64-24,22);
//		ssd1306_WriteString("<  1  >", Font_7x10, 1);
//		step=1;
//	break;
//	case 1:
//		ssd1306_SetCursor(64-24,22);
//		ssd1306_WriteString("<  5  >", Font_7x10, 1);
//		step=5;
//	break;
//	case 2:
//		ssd1306_SetCursor(64-24,22);
//		ssd1306_WriteString("<  10  >", Font_7x10, 1);
//		step=10;
//	break;
//	}
//	}
//
//}




