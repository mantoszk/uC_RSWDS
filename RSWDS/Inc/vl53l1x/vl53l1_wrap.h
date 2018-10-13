#include "vl53l1x/vl53l1_api.h"

typedef enum{
	HIGH_SPEED2=0,
	HIGH_ACCURACY2,
	LONG_RANGE2
}RangingConfig_ee ;

uint8_t VL_single_measurement_blocking2(VL53L1_DEV dev, int16_t* distance);
uint8_t VL_init_parameters2(VL53L1_Dev_t *dev, RangingConfig_ee rangingConfig);
uint8_t VL_init_sensor_set_addres2(VL53L1_DEV dev, uint8_t addr);
void VL53L1X_Measurement2(void);
void Measurement2(void);
