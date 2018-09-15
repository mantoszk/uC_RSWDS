#ifndef TIMER_H_
#define TIMER_H_

#include "defines.h"

static const uint8_t number_of_stopwatches = 2;
uint16_t *stopwatch;
uint16_t *stopwatch_lock_time;
bool *is_stopwatch_on;

bool stopwatch_init();
bool stopwatch_free();
bool is_locked(uint8_t _stopwatch_number);

void stopwatch_on(uint8_t _stopwatch_number);
void stopwatch_off(uint8_t _stopwatch_number);

void set_stopwatch_blocking_time(uint8_t _stopwatch_number, uint16_t _blocking_time);
void stopwatch_update();

#endif /* TIMER_H_ */
