/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "TFMini.h"
#include "psd.h"
#include "timer.h"
#include "vl53l0x/vl53l0x_wrap.h"
#include "vl53l1x/vl53l1_wrap.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	return len;
}

#define interruptModeQuestionMark 0 /* If interruptModeQuestionMark = 1 then device working in interrupt mode, else device working in polling mode */
VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;
VL53L0X_Dev_t dev2;
VL53L0X_DEV Dev2 = &dev2;
int status, status2, ting = 0;
volatile int VL53L0X_callback_counter, VL53L1X_callback_counter;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == VL53L0X_INT_Pin) {
		++VL53L0X_callback_counter;
	}

	if (GPIO_Pin == VL53L1X_INT_Pin) {
		++VL53L1X_callback_counter;
	}
}

void AutonomousLowPowerRangingTest(void) {

	static VL53L1_RangingMeasurementData_t RangingData;
	if (ting == 0) {
		status = VL53L1_WaitDeviceBooted(Dev);
		status = VL53L1_DataInit(Dev);
		status = VL53L1_StaticInit(Dev);
		status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_AUTONOMOUS);
		status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
		status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 70000);
		status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 400);
		HAL_Delay(100);
		status = VL53L1_StartMeasurement(Dev);

		if (status) {
			printf("VL53L1_StartMeasurement failed \r\n");
			while (1)
				;
		}
	}

	if (interruptModeQuestionMark == 0) {
		//do // interrupt mode
		//{
		__WFI();

		if (VL53L1X_callback_counter != 0) {
			VL53L1X_callback_counter = 0;
			status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
			if (status == 0) {
				printf("VL53L1X: %d,%d,%.2f,%.2f\r\n", RangingData.RangeStatus, RangingData.RangeMilliMeter,
						RangingData.SignalRateRtnMegaCps / 65536.0, RangingData.AmbientRateRtnMegaCps / 65336.0);
			}
			status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
		}
		//} while (1);
	} else {
		do // polling mode
		{
			status = VL53L1_WaitMeasurementDataReady(Dev);
			if (!status) {
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if (status == 0) {
					printf("VL53L1X: %d,%d,%.2f,%.2f\r\n", RangingData.RangeStatus, RangingData.RangeMilliMeter,
							(RangingData.SignalRateRtnMegaCps / 65536.0), RangingData.AmbientRateRtnMegaCps / 65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
				break;
			}
		} while (1);
	}

//  return status;
}

void AutonomousLowPowerRangingTest2(void) {

	static VL53L0X_RangingMeasurementData_t RangingData;

	status2 = VL53L0X_WaitDeviceBooted(Dev2);
	status2 = VL53L0X_DataInit(Dev2);
	status2 = VL53L0X_StaticInit(Dev2);
	status2 = VL53L0X_SetDeviceMode(Dev2, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	//status = VL53L0X_SetDistanceMode(Dev2, VL53L1_DISTANCEMODE_LONG);
	status2 = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev2, 33000);
	//status2 = VL53L0X_SetInterMeasurementPeriodMilliSeconds(Dev2, 100);

	FixPoint1616_t signalLimit = (FixPoint1616_t) (0.1 * 65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t) (60 * 65536);

	status2 = VL53L0X_SetLimitCheckValue(Dev2,
	VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);

	status2 = VL53L0X_SetLimitCheckValue(Dev2,
	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);

	status2 = VL53L0X_SetVcselPulsePeriod(Dev2, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);

	status2 = VL53L0X_SetVcselPulsePeriod(Dev2, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	status2 = VL53L0X_StartMeasurement(Dev2);

	HAL_Delay(100);

	if (status2) {
		printf("VL53L0_StartMeasurement failed \r\n");
		while (1)
			;
	}

	if (interruptModeQuestionMark == 0) {

		__WFI();

		status2 = VL53L0X_PerformSingleRangingMeasurement(Dev2, &RangingData);
		if (status2 == 0) {
			printf("VL53L0X: %d,%d,%.2f,%.2f\r\n", RangingData.RangeStatus, RangingData.RangeMilliMeter, RangingData.SignalRateRtnMegaCps / 65536.0,
					RangingData.AmbientRateRtnMegaCps / 65336.0);
		}

//  return status;
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USART6_UART_Init();
	MX_I2C3_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("Program has been started\r\n");
	PSD_init();
	lidar_init();
	stopwatch_init();

	set_stopwatch_blocking_time(0, 0);
	set_stopwatch_blocking_time(1, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	dev.I2cDevAddr = 0x52;
	dev.I2cHandle = &hi2c3;
	dev.comms_speed_khz = 400;
	dev2.I2cDevAddr = 0x52;
	dev2.i2c_handle = &hi2c1;
	dev2.comms_speed_khz = 400;

	uint8_t byteData;
	uint16_t wordData;
	VL53L1_RdByte(Dev, 0x010F, &byteData);
	printf("VL53L1X Model_ID: %02X\n\r", byteData);
	VL53L1_RdByte(Dev, 0x0110, &byteData);
	printf("VL53L1X Module_Type: %02X\n\r", byteData);
	VL53L1_RdWord(Dev, 0x010F, &wordData);
	printf("VL53L1X: %02X\n\r", wordData);

	while (1) {
		AutonomousLowPowerRangingTest();
		AutonomousLowPowerRangingTest2();
		HAL_Delay(300);
	}

	/*
	 while (1) {
	 if (is_locked(0) == false) {
	 PSD_read();
	 stopwatch_on(0);
	 }

	 if (is_locked(1) == false) {
	 lidar_read();
	 stopwatch_on(1);
	 }

	 //printf("values: %d, %d\r\n", PSD_short_value(), PSD_long_value());
	 //printf("distance: %d cm\r\n", lidar_distance_cm());
	 //printf("strength: %d\r\n", lidar_strength());


	 stopwatch_update();
	 HAL_Delay(0);
	 /* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/*
	 }
	 PSD_free();
	 lidar_free();
	 stopwatch_free();
	 /* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
