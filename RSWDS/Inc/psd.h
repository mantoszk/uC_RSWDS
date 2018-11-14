#ifndef PSD_H_
#define PSD_H_
#include "defines.h"
#include "adc.h"

static const uint8_t number_of_sensors = 3;
uint16_t *psd_raw_values;

bool PSD_init();
bool PSD_free();
void PSD_read();

uint16_t PSD_short_value();
uint16_t PSD_short2_value();
uint16_t PSD_long_value();
#endif
