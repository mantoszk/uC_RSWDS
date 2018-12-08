#include "psd.h"

bool PSD_init()
{
	psd_raw_values = malloc(number_of_sensors * sizeof(uint16_t));

	if (psd_raw_values == NULL)
	{
		trace(CRITICAL_MESSAGE, "PSD - memory allocation failed");
		return false;
	}
	trace(POSITIVE_MESSAGE, "PSD - memory allocation succeeded");
	return true;
}

bool PSD_free()
{
	if (psd_raw_values == NULL)
	{
		trace(CRITICAL_MESSAGE, "PSD - memory isn't initialized");
		return false;
	}
	free(psd_raw_values);
	trace(POSITIVE_MESSAGE, "PSD - memory allocation succeeded");
	return true;
}

void PSD_read()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) psd_raw_values, number_of_sensors);
	HAL_Delay(20);
	HAL_ADC_Stop_DMA(&hadc1);
	trace(POSITIVE_MESSAGE, "PSD - data read out");
}

uint16_t PSD_small_1_value()
{
	return psd_raw_values[0];
}

uint16_t PSD_small_2_value()
{
	return psd_raw_values[1];
}

uint16_t PSD_big_value()
{
	return psd_raw_values[2];
}

uint16_t PSD_small_1_distance_cm()
{
	return (uint16_t) ((a_small_1 / psd_raw_values[0]) - b_small_1);
}

uint16_t PSD_small_2_distance_cm()
{
	return (uint16_t) ((a_small_2 / psd_raw_values[1]) - b_small_2);
}

uint16_t PSD_big_distance_cm()
{
	return (uint16_t) ((a_big / psd_raw_values[2]) - b_big);
}

void PSD_calibrate_small_1(float a, float b)
{
	a_small_1 = a;
	b_small_1 = b;
}
void PSD_calibrate_small_2(float a, float b)
{
	a_small_2 = a;
	b_small_2 = b;
}
void PSD_calibrate_big(float a, float b)
{
	a_big = a;
	b_big = b;
}
