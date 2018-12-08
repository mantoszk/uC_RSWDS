#include <VL53L1.h>

bool my_VL53L1_init() {

	Dev1->I2cDevAddr = 0x52;
	Dev1->I2cHandle = &hi2c3;
	Dev1->comms_speed_khz = 400;

	VL53L1_status = VL53L1_WaitDeviceBooted(Dev1);
	VL53L1_status = VL53L1_DataInit(Dev1);
	VL53L1_status = VL53L1_StaticInit(Dev1);
	VL53L1_status = VL53L1_SetPresetMode(Dev1, VL53L1_PRESETMODE_AUTONOMOUS);
	VL53L1_status = VL53L1_SetDistanceMode(Dev1, VL53L1_DISTANCEMODE_LONG);
	VL53L1_status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev1, 70000);
	VL53L1_status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev1, 200);
	VL53L1_status = VL53L1_StartMeasurement(Dev1);

	if (VL53L1_status) {
		trace(CRITICAL_MESSAGE, "VL53L1_StartMeasurement failed");
		return false;
	}

	return true;
}

bool my_VL53L1_read() {
	if (interruptModeQuestionMark == 0) {
		__WFI();

		if (VL53L1_callback_counter != 0) {
			VL53L1_callback_counter = 0;
			VL53L1_status = VL53L1_GetRangingMeasurementData(Dev1, &VL53L1_RangingData);
			VL53L1_status = VL53L1_ClearInterruptAndStartMeasurement(Dev1);
		}
	} else {
		VL53L1_status = VL53L1_WaitMeasurementDataReady(Dev1);
		if (!VL53L1_status) {
			VL53L1_status = VL53L1_GetRangingMeasurementData(Dev1, &VL53L1_RangingData);
			VL53L1_status = VL53L1_ClearInterruptAndStartMeasurement(Dev1);
		}
	}

	if (VL53L1_status == VL53L1_ERROR_NONE) {
		trace(POSITIVE_MESSAGE, "VL53L1 - reading data successful");
		return true;
	} else{
		trace(CRITICAL_MESSAGE, "VL53L1 - reading data unsuccessful");
		return false;
	}
}

uint16_t my_VL53L1_distance_mm() {
	if (VL53L1_RangingData.RangeMilliMeter < 5000)
		return VL53L1_RangingData.RangeMilliMeter;
	else
		return 0;
}

float my_VL53L1_distance_m() {
	if (VL53L1_RangingData.RangeMilliMeter < 5000)
		return VL53L1_RangingData.RangeMilliMeter / 1000;
	else
		return 0;
}
