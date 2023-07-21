/*
 * gui.h
 *
 *  Created on: 23 kwi 2023
 *      Author: Jaroslaw
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include "main.h"
#include "ssd1306.h"
#include "stdbool.h"


void MainScrean(uint16_t *setTemp, uint16_t currnetTemp,uint8_t step, bool *menuState, bool status,uint8_t power);
void DrawMainScrean(uint16_t setTemp, uint16_t currnetTemp,bool status,uint8_t power);
//void DrawStepLayer(uint8_t one, uint8_t five, uint8_t ten);
//void DrawSettingsLayer(uint8_t scroll);
void PrintPower(uint8_t power);
void PrintCurrentTemperature(uint16_t currnetTemp);
void PrintSetTemperature(uint16_t setTemp);
void PrintDeviceStatus(bool status);
void Buzz(void);
//void PrintSettings(uint8_t settings);
void MenuScrean(bool *screanState, uint8_t *step,uint8_t *menuState);
void BackToMainScrean (bool *screanState);
uint8_t Select_Step();

#endif /* INC_GUI_H_ */
