#include "adc.h"

#define VOLTAGE_DIVIDER_RATIO ((10.0f + 10.0f) / 10.0f)

void ADC_Voltage_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//不使用外部触发，那就是软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);
	
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);//复位ADC1的校准状态
	while(ADC_GetResetCalibrationStatus(ADC1));//等待复位校准完成
	ADC_StartCalibration(ADC1);//启动ADC1的校准
	while(ADC_GetCalibrationStatus(ADC1));//等待校准完成
	
}

uint16_t ADC_GetValue(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//启动软件触发
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);//等待转化完毕
	return ADC_GetConversionValue(ADC1);//返回读取的数据
}

float ADC_GetVoltage(void)
{
	uint16_t adc_value = ADC_GetValue();
	float voltage = (float)adc_value * 3.3f / 4096.0f;
	return voltage * VOLTAGE_DIVIDER_RATIO;
}