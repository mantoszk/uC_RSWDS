#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "defines.h"
#include "i2c.h"
#include "vl53l0x/vl53l0x_api.h"
#include "debug.h"

static VL53L0X_Dev_t dev_VL53L0X;
static VL53L0X_DEV Dev0 = &dev_VL53L0X;
static int	VL53L0X_status = 0;

static VL53L0X_RangingMeasurementData_t VL53L0X_RangingData;

bool my_VL53L0X_init();
bool my_VL53L0X_read();
uint16_t my_VL53L0X_distance_mm();
float my_VL53L0X_distance_m();

#endif
