#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"                  // Device header


void ADC_Voltage_Init(void);
uint16_t ADC_GetValue(void);
float ADC_GetVoltage(void);

#endif