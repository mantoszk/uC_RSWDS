#ifndef TFMINI_H_
#define TFMINI_H_
#include "defines.h"
#include "usart.h"
#include "debug.h"

static number_of_characters = 9;
uint8_t *lidar_raw_values;

bool lidar_init();
bool lidar_free();
bool lidar_read();
uint16_t lidar_distance_cm();
float lidar_distance_m();
uint16_t lidar_strength();

#endif
