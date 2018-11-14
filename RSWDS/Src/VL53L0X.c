#include "VL53L0X.h"

bool my_VL53L0X_init() {
	Dev0->I2cDevAddr = 0x52;
	Dev0->i2c_handle = &hi2c1;
	Dev0->comms_speed_khz = 400;

	VL53L0X_status = VL53L0X_WaitDeviceBooted(Dev0);
	VL53L0X_status = VL53L0X_DataInit(Dev0);
	VL53L0X_status = VL53L0X_StaticInit(Dev0);
	VL53L0X_status = VL53L0X_SetDeviceMode(Dev0, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	//VL53L0X_status = VL53L0X_SetDistanceMode(Dev0, VL53L1_DISTANCEMODE_LONG);
	VL53L0X_status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev0, 33000);
	VL53L0X_status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(Dev0, 200);

	FixPoint1616_t signalLimit = (FixPoint1616_t) (0.1 * 65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t) (60 * 65536);

	VL53L0X_status = VL53L0X_SetLimitCheckValue(Dev0, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);

	VL53L0X_status = VL53L0X_SetLimitCheckValue(Dev0, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);

	VL53L0X_status = VL53L0X_SetVcselPulsePeriod(Dev0, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);

	VL53L0X_status = VL53L0X_SetVcselPulsePeriod(Dev0, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	if (VL53L0X_status) {
		printf("VL53L0_StartMeasurement failed \r\n");
		return false;
	}

	return true;
}

bool my_VL53L0X_read() {
	VL53L0X_status = VL53L0X_PerformSingleRangingMeasurement(Dev0, &VL53L0X_RangingData);
	if (VL53L0X_status == 0 && VL53L0X_RangingData.RangeMilliMeter < 3000) {
			printf("VL53L0X: %d,%d,%.2f,%.2f\r\n", VL53L0X_RangingData.RangeStatus, VL53L0X_RangingData.RangeMilliMeter,
					VL53L0X_RangingData.SignalRateRtnMegaCps / 65536.0, VL53L0X_RangingData.AmbientRateRtnMegaCps / 65336.0);
			VL53L0X_ClearInterruptMask(Dev0, 0);
			return true;
	}

	VL53L0X_ClearInterruptMask(Dev0, 0);
	return false;
}

uint16_t my_VL53L0X_distance_mm()
{
	return VL53L0X_RangingData.RangeMilliMeter;
}

float my_VL53L0X_distance_m()
{
	return VL53L0X_RangingData.RangeMilliMeter/1000.0f;
}

