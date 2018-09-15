#include "timer.h"

bool stopwatch_init() {
	stopwatch = malloc(number_of_stopwatches * sizeof(uint16_t));
	is_stopwatch_on = malloc(number_of_stopwatches * sizeof(bool));
	stopwatch_lock_time = malloc(number_of_stopwatches * sizeof(uint16_t));

	if (stopwatch == NULL || is_stopwatch_on == NULL
			|| stopwatch_lock_time == NULL) {
		printf("stopwatch malloc failed\r\n");
		return false;
	}

	for(uint8_t i = 0; i<number_of_stopwatches; ++i)
	{
		stopwatch[i] = 0;
		is_stopwatch_on[i] = false;
		stopwatch_lock_time[i] = 0;
	}

	printf("stopwatch malloc succeeded\r\n");
	return true;
}

bool stopwatch_free() {
	if (stopwatch == NULL && is_stopwatch_on == NULL
			&& stopwatch_lock_time == NULL) {
		printf("stopwatch memory isn't initilized\r\n");
		return false;
	}

	if (stopwatch != NULL)
		free(stopwatch);

	if (is_stopwatch_on != NULL)
		free(is_stopwatch_on);

	if (stopwatch_lock_time != NULL)
		free(stopwatch_lock_time);

	printf("stopwatch free succeeded\r\n");
	return true;
}

void stopwatch_on(uint8_t _stopwatch_number) {
	is_stopwatch_on[_stopwatch_number] = true;
	stopwatch[_stopwatch_number] = 0;
}

void stopwatch_off(uint8_t _stopwatch_number) {
	is_stopwatch_on[_stopwatch_number] = false;
}

bool is_locked(uint8_t _stopwatch_number) {
	return is_stopwatch_on[_stopwatch_number];
}

void set_stopwatch_blocking_time(uint8_t _stopwatch_number, uint16_t _blocking_time) {
	stopwatch_lock_time[_stopwatch_number] = _blocking_time;
}

void stopwatch_update() {
	for (uint8_t i = 0; i < number_of_stopwatches; ++i) {
		if ((is_stopwatch_on[i] == true)
				&& (stopwatch[i] < stopwatch_lock_time[i]))
			++stopwatch[i];
		else
			stopwatch_off(i);
	}
}
