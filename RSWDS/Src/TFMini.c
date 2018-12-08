#include "TFMini.h"

bool lidar_init() {
	lidar_raw_values = malloc(number_of_characters * sizeof(uint8_t));

	if (lidar_raw_values == NULL) {
		trace(CRITICAL_MESSAGE, "Lidar - memory allocation failed");
		return false;
	}
	trace(POSITIVE_MESSAGE, "Lidar - memory allocation succeeded");
	return true;
}

bool lidar_free() {
	if (lidar_raw_values == NULL) {
		trace(CRITICAL_MESSAGE, "Lidar - memory isn't initialized");
		return false;
	}
	free(lidar_raw_values);
	trace(POSITIVE_MESSAGE, "Lidar - memory freed");
	return true;
}

bool lidar_read() {
	HAL_UART_Receive_DMA(&huart6, lidar_raw_values, number_of_characters);
	HAL_Delay(20);
	HAL_UART_DMAStop(&huart6);

	if (lidar_raw_values[0] != 0x59 || lidar_raw_values[1] != 0x59 || lidar_raw_values[7] != 0x00) {
		trace(CRITICAL_MESSAGE, "Lidar - corrupted data frame");
		lidar_raw_values[2] = 0;
		lidar_raw_values[3] = 0;
		return false;
	}

	uint16_t checksum = 0;
	uint8_t number_without_checksum = number_of_characters - 1;

	for (uint8_t i = 0; i < number_without_checksum; ++i)
		checksum += lidar_raw_values[i];

	if ((checksum & 0xFF) != lidar_raw_values[number_without_checksum]) {
		trace(CRITICAL_MESSAGE, "Lidar - checksum incorrect");
		lidar_raw_values[2] = 0;
		lidar_raw_values[3] = 0;
		return false;
	}

	trace(POSITIVE_MESSAGE, "Lidar - reading data successful");
	return true;
}

uint16_t lidar_distance_cm() {
	return lidar_raw_values[3] << 8 | lidar_raw_values[2];
}

float lidar_distance_m() {
	return (lidar_raw_values[3] << 8 | lidar_raw_values[2]) / 100.0f;
}

uint16_t lidar_strength() {
	return lidar_raw_values[5] << 8 | lidar_raw_values[4];
}

