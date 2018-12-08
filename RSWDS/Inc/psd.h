#ifndef PSD_H_
#define PSD_H_
#include "defines.h"
#include "adc.h"
#include "debug.h"

static const uint8_t number_of_sensors = 3;
uint16_t *psd_raw_values;

static float a_small_1 = 0.0f;
static float b_small_1 = 0.0f;

static float a_small_2 = 0.0f;
static float b_small_2 = 0.0f;

static float a_big = 0.0f;
static float b_big = 0.0f;

bool PSD_init();
bool PSD_free();
void PSD_read();

uint16_t PSD_small_1_value();
uint16_t PSD_small_2_value();
uint16_t PSD_big_value();
uint16_t PSD_small_1_distance_cm();
uint16_t PSD_small_2_distance_cm();
uint16_t PSD_big_distance_cm();

void PSD_calibrate_small_1(float a, float b);
void PSD_calibrate_small_2(float a, float b);
void PSD_calibrate_big(float a, float b);

#endif
