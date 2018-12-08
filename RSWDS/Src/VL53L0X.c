#include "VL53L0X.h"

bool my_VL53L0X_init() {
	Dev0->I2cDevAddr = 0x52;
	Dev0->i2c_handle = &hi2c1;
	Dev0->comms_speed_khz = 400;

	VL53L0X_status = VL53L0X_WaitDeviceBooted(Dev0);
	VL53L0X_status = VL53L0X_DataInit(Dev0);
	VL53L0X_status = VL53L0X_StaticInit(Dev0);

	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	VL53L0X_status = VL53L0X_PerformRefCalibration(Dev0, &VhvSettings, &PhaseCal);
	VL53L0X_status = VL53L0X_PerformRefSpadManagement(Dev0, &refSpadCount, &isApertureSpads);

	VL53L0X_status = VL53L0X_SetDeviceMode(Dev0, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	VL53L0X_status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev0, 33000);
	VL53L0X_status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(Dev0, 200);

	FixPoint1616_t signalLimit = (FixPoint1616_t) (0.1 * 65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t) (60 * 65536);
	VL53L0X_status = VL53L0X_SetLimitCheckValue(Dev0, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
	VL53L0X_status = VL53L0X_SetLimitCheckValue(Dev0, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);

	VL53L0X_status = VL53L0X_SetVcselPulsePeriod(Dev0, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_status = VL53L0X_SetVcselPulsePeriod(Dev0, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	if (VL53L0X_status != VL53L0X_ERROR_NONE) {
		trace(CRITICAL_MESSAGE, "VL53L0X_StartMeasurement failed");
		return false;
	}

	return true;
}

bool my_VL53L0X_read() {
	VL53L0X_status = VL53L0X_PerformSingleRangingMeasurement(Dev0, &VL53L0X_RangingData);
	VL53L0X_ClearInterruptMask(Dev0, 0);

	if (VL53L0X_status == VL53L0X_ERROR_NONE) {
		trace(POSITIVE_MESSAGE, "VL53L0X - reading data successful");
		return true;
	} else {
		trace(CRITICAL_MESSAGE, "VL53L0X - reading data unsuccessful");
		return false;
	}
}

uint16_t my_VL53L0X_distance_mm() {
	if (VL53L0X_RangingData.RangeMilliMeter < 3000)
		return VL53L0X_RangingData.RangeMilliMeter;
	else
		return 0;
}

float my_VL53L0X_distance_m() {
	if (VL53L0X_RangingData.RangeMilliMeter < 3000)
		return VL53L0X_RangingData.RangeMilliMeter / 1000.0f;
	else
		return 0;
}

