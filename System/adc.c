#include "adc.h"

#define VOLTAGE_DIVIDER_RATIO ((10.0f + 10.0f) / 10.0f)

void ADC_Voltage_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//Analog input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ ScanConvMode= DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//No external trigger (software trigger)
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);

	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);//Reset ADC1 calibration
	while (ADC_GetResetCalibrationStatus(ADC1));//Wait reset calibration done
	ADC_StartCalibration(ADC1);//Start ADC1 calibration
	while (ADC_GetCalibrationStatus(ADC1));//Wait calibration done

}

/**
  * @brief  获取ADC转换值
  * @param  无
  * @retval ADC转换值
  */
uint16_t ADC_GetValue(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//Start software trigger
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);//Wait conversion complete
	return ADC_GetConversionValue(ADC1);//Return converted value
}
/**
  * @brief  将ADC转换值转化为ADC电压值
  * @param  无
  * @retval ADC电压值
  */
float ADC_GetVoltage(void)
{
	uint16_t adc_value = ADC_GetValue();
	// 计算电压值，假设使用3.3V参考电压和12位ADC分辨率（4096），并考虑电压分压器的影响
	float voltage = (float)adc_value * 3.3f / 4096.0f;
	return voltage * VOLTAGE_DIVIDER_RATIO;
}