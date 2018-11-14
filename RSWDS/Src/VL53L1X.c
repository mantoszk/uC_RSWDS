#include "VL53L1X.h"

bool my_VL53L1X_init() {

	Dev1->I2cDevAddr = 0x52;
	Dev1->I2cHandle = &hi2c3;
	Dev1->comms_speed_khz = 400;

	VL53L1X_status = VL53L1_WaitDeviceBooted(Dev1);
	VL53L1X_status = VL53L1_DataInit(Dev1);
	VL53L1X_status = VL53L1_StaticInit(Dev1);
	VL53L1X_status = VL53L1_SetPresetMode(Dev1, VL53L1_PRESETMODE_AUTONOMOUS);
	VL53L1X_status = VL53L1_SetDistanceMode(Dev1, VL53L1_DISTANCEMODE_LONG);
	VL53L1X_status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev1, 70000);
	VL53L1X_status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev1, 200);
	VL53L1X_status = VL53L1_StartMeasurement(Dev1);

	if (VL53L1X_status) {
		printf("VL53L1_StartMeasurement failed \r\n");
		return false;
	}

	return true;
}

bool my_VL53L1X_read() {
	if (interruptModeQuestionMark == 0) {
		__WFI();

		if (VL53L1X_callback_counter != 0) {
			VL53L1X_callback_counter = 0;
			VL53L1X_status = VL53L1_GetRangingMeasurementData(Dev1, &VL53L1X_RangingData);
			if (VL53L1X_status == 0) {
				if (VL53L1X_RangingData.RangeMilliMeter < 5000)
					printf("VL53L1X: %d,%d,%.2f,%.2f\r\n", VL53L1X_RangingData.RangeStatus, VL53L1X_RangingData.RangeMilliMeter,
							VL53L1X_RangingData.SignalRateRtnMegaCps / 65536.0, VL53L1X_RangingData.AmbientRateRtnMegaCps / 65336.0);
			}
			VL53L1X_status = VL53L1_ClearInterruptAndStartMeasurement(Dev1);
		}
	} else {
		VL53L1X_status = VL53L1_WaitMeasurementDataReady(Dev1);
		if (!VL53L1X_status) {
			VL53L1X_status = VL53L1_GetRangingMeasurementData(Dev1, &VL53L1X_RangingData);
			if (VL53L1X_status == 0) {
				if (VL53L1X_RangingData.RangeMilliMeter < 5000)
					printf("VL53L1X: %d,%d,%.2f,%.2f\r\n", VL53L1X_RangingData.RangeStatus, VL53L1X_RangingData.RangeMilliMeter,
							(VL53L1X_RangingData.SignalRateRtnMegaCps / 65536.0), VL53L1X_RangingData.AmbientRateRtnMegaCps / 65336.0);
			}
			VL53L1X_status = VL53L1_ClearInterruptAndStartMeasurement(Dev1);
		}
	}
}

uint16_t my_VL53L1X_distance_mm() {
	return VL53L1X_RangingData.RangeMilliMeter;
}

float my_VL53L1X_distance_m() {
	return VL53L1X_RangingData.RangeMilliMeter / 1000;
}