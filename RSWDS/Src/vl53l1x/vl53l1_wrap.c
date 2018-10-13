#include "vl53l1x/vl53l1_wrap.h"
#include "i2c.h"
#include "vl53l1x/vl53l1_tuning_parm_defaults.h"

uint8_t VL_init_parameters2(VL53L1_Dev_t *dev, RangingConfig_ee rangingConfig) {
	int status;
	/*

	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	 FixPoint1616_t signalLimit = (FixPoint1616_t) (0.25 * 65536);
	 FixPoint1616_t sigmaLimit = (FixPoint1616_t) (18 * 65536);
	 uint32_t timingBudget = 33000;
	 uint8_t preRangeVcselPeriod = 14;
	 uint8_t finalRangeVcselPeriod = 10;
	 */

	status = VL53L1_StaticInit(dev);
	if (status) {
		return status;
	}
	VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_LONG);
	VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 50000);
	VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 500);
	VL53L1_PerformRefSpadManagement(dev);
	VL53L1_StartMeasurement(dev);
	/*
	 status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount,
	 &isApertureSpads);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_SetDeviceMode(dev, mode);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_SetLimitCheckEnable(dev,
	 VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_SetLimitCheckEnable(dev,
	 VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
	 if (status) {
	 return status;
	 }

	 switch (rangingConfig) {
	 case LONG_RANGE:
	 signalLimit = (FixPoint1616_t) (0.1 * 65536);
	 sigmaLimit = (FixPoint1616_t) (60 * 65536);
	 timingBudget = 33000;
	 preRangeVcselPeriod = 18;
	 finalRangeVcselPeriod = 14;
	 break;
	 case HIGH_ACCURACY:
	 signalLimit = (FixPoint1616_t) (0.25 * 65536);
	 sigmaLimit = (FixPoint1616_t) (18 * 65536);
	 timingBudget = 200000;
	 preRangeVcselPeriod = 14;
	 finalRangeVcselPeriod = 10;
	 break;
	 case HIGH_SPEED:
	 signalLimit = (FixPoint1616_t) (0.25 * 65536);
	 sigmaLimit = (FixPoint1616_t) (20 * 65536);
	 timingBudget = 20000;
	 preRangeVcselPeriod = 14;
	 finalRangeVcselPeriod = 10;
	 break;
	 default:
	 return status;
	 }

	 status = VL53L0X_SetLimitCheckValue(dev,
	 VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_SetLimitCheckValue(dev,
	 VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, timingBudget);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
	 preRangeVcselPeriod);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
	 finalRangeVcselPeriod);
	 if (status) {
	 return status;
	 }

	 status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
	 if (status) {
	 return status;
	 }
	 */
	return 0;
}

uint8_t VL_single_measurement_blocking2(VL53L1_DEV dev, int16_t* distance) {
	uint8_t status = 0;
	VL53L1_RangingMeasurementData_t pomiar;

	status = VL53L1_GetRangingMeasurementData(dev, &pomiar);

	if (pomiar.RangeMilliMeter < 5000)
		*distance = pomiar.RangeMilliMeter;

	return status;
}

uint8_t VL_init_sensor_set_addres2(VL53L1_DEV dev, uint8_t addr) {

	uint8_t status = 0;
	//HAL_GPIO_WritePin(dev->GPIOx_reset, dev->GPIO_pin_reset, ENABLE);

	uint32_t time = 0;

	for (; time < 10000;) {
		time++;
	}

	status |= VL53L1_DataInit(dev);

	status |= VL53L1_SetDeviceAddress(dev, addr);
	if (status) {
		return status;
	}

	dev->I2cDevAddr = addr;

	return status;
}
