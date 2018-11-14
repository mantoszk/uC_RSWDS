#ifndef VL53L1X_H_
#define VL53L1X_H_

#include "defines.h"
#include "i2c.h"
#include "vl53l1x/vl53l1_api.h"

#define interruptModeQuestionMark 1 /* If interruptModeQuestionMark = 1 then device working in interrupt mode, else device working in polling mode */

static VL53L1_Dev_t dev_VL53L1X;
static VL53L1_DEV Dev1 = &dev_VL53L1X;
static int VL53L1X_callback_counter = 0;
static int VL53L1X_status = 0;

static VL53L1_RangingMeasurementData_t VL53L1X_RangingData;

bool my_VL53L1X_init();
bool my_VL53L1X_read();
uint16_t my_VL53L1X_distance_mm();
float my_VL53L1X_distance_m();

#endif
