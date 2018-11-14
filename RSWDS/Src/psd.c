#include "psd.h"

bool PSD_init() {
	psd_raw_values = malloc(number_of_sensors * sizeof(uint16_t));

	if (psd_raw_values == NULL) {
		printf("psd malloc failed\r\n");
		return false;
	}
	printf("psd malloc succeeded\r\n");
	return true;
}

bool PSD_free() {
	if (psd_raw_values == NULL) {
		printf("psd memory isn't initilized\r\n");
		return false;
	}
	free(psd_raw_values);
	printf("psd free succeeded\r\n");
	return true;
}

void PSD_read() {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) psd_raw_values, number_of_sensors);
	HAL_Delay(20);
	HAL_ADC_Stop_DMA(&hadc1);
}

uint16_t PSD_short_value() {
	return psd_raw_values[0];
}

uint16_t PSD_short2_value() {
	return psd_raw_values[1];
}

uint16_t PSD_long_value() {
	return psd_raw_values[2];
}
