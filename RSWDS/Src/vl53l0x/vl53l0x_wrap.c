/*
 * vl53l0x_stm_api.c
 *
 *  created on: 25.05.2017
 *      author: the
 */

#include "vl53l0x/vl53l0x_wrap.h"
#include "i2c.h"
#include "gpio_map.h"

/*!
 * Ustawia parametry czujnika, takze mode, kt�ry decyduje czy continuos czy single
 *
 */
uint8_t VL_init_parameters(VL53L0X_Dev_t *dev, RangingConfig_e rangingConfig, VL53L0X_DeviceModes mode) {

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;

	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	if (Status == VL53L0X_ERROR_NONE) {
		// printf ("Call of VL53L0X_StaticInit\n");
		Status = VL53L0X_StaticInit(dev); // Device Initialization
		//print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		// printf ("Call of VL53L0X_PerformRefCalibration\n");
		Status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal); // Device Initialization
		//print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		// printf ("Call of VL53L0X_PerformRefSpadManagement\n");
		Status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads); // Device Initialization
		//printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
		// print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {

		// no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
		// printf ("Call of VL53L0X_SetDeviceMode\n");
		Status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
		// print_pal_error(Status);
	}

	// Enable/Disable Sigma and Signal check

	/*   if (Status == VL53L0X_ERROR_NONE) {
	 Status = VL53L0X_SetSequenceStepEnable(pMyDevice,VL53L0X_SEQUENCESTEP_DSS, 1);
	 }*/

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(dev,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckEnable(dev,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(dev,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t) (0.1 * 65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetLimitCheckValue(dev,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t) (60 * 65536));
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 33000);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(dev,
		VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	}
	if (Status == VL53L0X_ERROR_NONE) {
		Status = VL53L0X_SetVcselPulsePeriod(dev,
		VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	}

	return Status;
}

uint8_t VL_set_interrupt(VL53L0X_DEV dev, uint8_t state, VL53L0X_InterruptPolarity polar) {
	if (state) {
		return VL53L0X_SetGpioConfig(dev, 0, VL53L0X_DEVICEMODE_SINGLE_RANGING,
		VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, polar);
	} else {
		return VL53L0X_SetGpioConfig(dev, 0, VL53L0X_DEVICEMODE_SINGLE_RANGING,
		VL53L0X_GPIOFUNCTIONALITY_OFF, polar);
	}
}

uint8_t VL_get_measurement_if_ready(VL53L0X_DEV dev, int16_t* distance) {
	VL53L0X_RangingMeasurementData_t pomiar;
	uint8_t ready;
	VL53L0X_GetMeasurementDataReady(dev, &ready);

	if (ready) {

		VL53L0X_GetRangingMeasurementData(dev, &pomiar);

		VL53L0X_ClearInterruptMask(dev, 0);

		if (pomiar.RangeMilliMeter > 3000) {
			*distance = -1;
		} else {
			*distance = pomiar.RangeMilliMeter;
		}
		return 0;
	}
	return 1;

}

uint8_t VL_get_measurement(VL53L0X_DEV dev, int16_t* distance) {
	VL53L0X_RangingMeasurementData_t pomiar;
	uint8_t state = 0;

	state |= VL53L0X_GetRangingMeasurementData(dev, &pomiar);

	state |= VL53L0X_ClearInterruptMask(dev, 0);

	if (pomiar.RangeMilliMeter > 3000) {
		*distance = -1;
	} else {
		*distance = pomiar.RangeMilliMeter;
	}
	return state;

}

uint8_t VL_start_measurement_non_blocking(VL53L0X_DEV dev) {
	return VL53L0X_StartMeasurement(dev);

}

uint8_t VL_stop_measurment(VL53L0X_DEV dev) {
	return VL53L0X_StopMeasurement(dev);
}

//jesli -1 to out of range, blokuje
uint8_t VL_single_measurement_blocking(VL53L0X_DEV dev, int16_t* distance) {
	uint8_t status = 0;
	VL53L0X_RangingMeasurementData_t pomiar;
	//pojedynczy pomiar
	status = VL53L0X_PerformSingleRangingMeasurement(dev, &pomiar);

	/*if(pomiar.RangeMilliMeter>3000){
	 *distance=-1;
	 }else{
	 *distance=pomiar.RangeMilliMeter;
	 }*/
	if (pomiar.RangeMilliMeter < 3000)
		*distance = pomiar.RangeMilliMeter;
	return status;
}

void VL_reset_on(VL53L0X_DEV dev) {
	//HAL_GPIO_WritePin(dev->GPIOx_reset, dev->GPIO_pin_reset, DISABLE);

}

uint8_t VL_init_sensor_set_addres(VL53L0X_DEV dev, uint8_t addr) {

	uint8_t status = 0;
	//wylacz reset ---trzeba zapewnic swoje!!, jak trzeba pare po kolei odpalac zeby kazdemu nadac inny adres to
	//science the shit out of this
	//HAL_GPIO_WritePin(dev->GPIOx_reset, dev->GPIO_pin_reset, ENABLE);
	//odczekaj chwile

	uint32_t time = 0;

	for (; time < 10000;) {
		time++;
	}

	status |= VL53L0X_DataInit(dev);

	status |= VL53L0X_SetDeviceAddress(dev, addr);
	if (status) {
		return status;
	}

	dev->I2cDevAddr = addr;

	return status;
}

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t NewDatReady = 0;
	uint32_t LoopNb;

	// Wait until it finished
	// use timeout to avoid deadlock
	if (Status == VL53L0X_ERROR_NONE) {
		LoopNb = 0;
		do {
			Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
			if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
				break;
			}
			LoopNb = LoopNb + 1;
			VL53L0X_PollingDelay(Dev);
		} while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

		if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
			Status = VL53L0X_ERROR_TIME_OUT;
		}
	}

	return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint32_t StopCompleted = 0;
	uint32_t LoopNb;

	// Wait until it finished
	// use timeout to avoid deadlock
	if (Status == VL53L0X_ERROR_NONE) {
		LoopNb = 0;
		do {
			Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
			if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
				break;
			}
			LoopNb = LoopNb + 1;
			VL53L0X_PollingDelay(Dev);
		} while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

		if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
			Status = VL53L0X_ERROR_TIME_OUT;
		}

	}

	return Status;
}

VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice) {
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData = &RangingMeasurementData;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	if (Status == VL53L0X_ERROR_NONE) {
		//printf ("Call of VL53L0X_StaticInit\n");
		Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
		// StaticInit will set interrupt by default
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		//  printf ("Call of VL53L0X_PerformRefCalibration\n");
		Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal); // Device Initialization
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		//  printf ("Call of VL53L0X_PerformRefSpadManagement\n");
		Status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); // Device Initialization
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {

		// printf ("Call of VL53L0X_SetDeviceMode\n");
		Status = VL53L0X_SetDeviceMode(pMyDevice,
		VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		//	printf ("Call of VL53L0X_StartMeasurement\n");
		Status = VL53L0X_StartMeasurement(pMyDevice);
		print_pal_error(Status);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		uint32_t measurement;
		uint32_t no_of_measurements = 32;

		uint16_t* pResults = (uint16_t*) malloc(sizeof(uint16_t) * no_of_measurements);

		for (measurement = 0; measurement < no_of_measurements; measurement++) {

			Status = WaitMeasurementDataReady(pMyDevice);

			if (Status == VL53L0X_ERROR_NONE) {
				Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);

				*(pResults + measurement) = pRangingMeasurementData->RangeMilliMeter;
				//  printf("In loop measurement %d: %d\n", measurement, pRangingMeasurementData->RangeMilliMeter);

				// Clear the interrupt
				VL53L0X_ClearInterruptMask(pMyDevice,
				VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
				VL53L0X_PollingDelay(pMyDevice);
			} else {
				break;
			}
		}

		if (Status == VL53L0X_ERROR_NONE) {
			for (measurement = 0; measurement < no_of_measurements; measurement++) {
				// printf("measurement %d: %d\n", measurement, *(pResults + measurement));
			}
		}

		free(pResults);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		//   printf ("Call of VL53L0X_StopMeasurement\n");
		Status = VL53L0X_StopMeasurement(pMyDevice);
	}

	if (Status == VL53L0X_ERROR_NONE) {
		// printf ("Wait Stop to be competed\n");
		Status = WaitStopCompleted(pMyDevice);
	}

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_ClearInterruptMask(pMyDevice,
		VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

	return Status;
}

void VL53L0X_Measurement(void) {
	/* VL53L0X definitions  ---------------------------------------------------------*/
	int ans = 0;
	uint8_t stan = 0;
	//VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t MyDevice;
	VL53L0X_Dev_t *pMyDevice = &MyDevice;
	//VL53L0X_Version_t Version;
	//VL53L0X_Version_t *pVersion = &Version;
	//VL53L0X_DeviceInfo_t DeviceInfo;
	int16_t wynik = 0;
	//uint16_t errors = 0;

	/*--------VL5130 START--------*/

	//MyDevice.GPIOx_reset = sensor[0].gpio_port;
	//MyDevice.GPIO_pin_reset = sensor[0].gpio_pin;
	MyDevice.I2cDevAddr = 0x52;
	MyDevice.i2c_handle = &hi2c1;
	stan = VL_init_sensor_set_addres(&MyDevice, 0x52);
	stan = VL_init_parameters(&MyDevice, LONG_RANGE,
	VL53L0X_DEVICEMODE_SINGLE_RANGING);
	HAL_Delay(25);
	ans = VL_single_measurement_blocking(&MyDevice, &wynik);
	printf("%8d:%1d\r\n", wynik, ans);
	//HAL_GPIO_WritePin(MyDevice.GPIOx_reset, MyDevice.GPIO_pin_reset, ENABLE); /* vl53l0x */
	HAL_Delay(25);
	//HAL_GPIO_WritePin(MyDevice.GPIOx_reset, MyDevice.GPIO_pin_reset, DISABLE); /* vl53l0x */

	/*--------VL5130 END--------*/

}

