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
#include <VL53L1.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "TFMini.h"
#include "psd.h"
#include "VL53L0X.h"
#include "VL53L1.h"
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
uint8_t received = 0;
uint8_t read_flag = 0;
uint8_t laser_flag = 0;
uint8_t single_sensor = 0;
uint8_t trace_mode = 0;
uint8_t output_level = 0;
uint8_t psd_level = 0;

uint16_t temp_data[9];
uint8_t message_buff[9];
uint8_t message_size = 0;
int message_buff2[50];
int message_size2 = 0;

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == VL53L1X_INT_Pin)
	{
		++VL53L1_callback_counter;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		uint8_t buffer[50];
		uint16_t size = 0;

		if (trace_mode == 1)
		{
			switch (received)
			{
			case 49: //1
				trace_mode = 2;
				break;
			case 50: //2
				trace_mode = 3;
				break;
			case 51: //3
				trace_mode = 4;
				break;
			}
		}

		if (output_level == 1)
		{
			switch (received)
			{
			case 49: //1
				output_level = 2;
				break;
			case 50: //2
				output_level = 3;
				break;
			}
		}

		if (psd_level == 1)
		{
			switch (received)
			{
			case 49: //1
				psd_level = 2;
				break;
			case 50: //2
				psd_level = 3;
				break;
			}
		}

		if (trace_mode == 0 && output_level == 0 && psd_level == 0)
		{
			switch (received)
			{
			case 122: //z
				read_flag = 1;
				break;
			case 99: //c
				read_flag = 2;
				break;
			case 120: //x
				++laser_flag;
				break;
			case 113: //q
				single_sensor = 1;
				break;
			case 119: //w
				single_sensor = 2;
				break;
			case 101: //e
				single_sensor = 3;
				break;
			case 114: //r
				single_sensor = 4;
				break;
			case 116: //t
				single_sensor = 5;
				break;
			case 121: //y
				single_sensor = 6;
				break;
			case 106: //j
				trace_mode = 1;
				printf("TRACE_LEVEL:\r\n1. CRITICAL_MESSAGE\r\n 2. POSITIVE_MESSAGE\r\n 3. ONLY_OUTPUT\r\n");
				break;
			case 107: //k
				output_level = 1;
				printf("OUTPUT_TYPE:\r\n1. OUTPUT_HUMAN\r\n 2. OUTPUT_MACHINE\r\n");
				break;
			case 108: //l
				psd_level = 1;
				printf("PSD_OUTPUT:\r\n1. PSD_RAW\r\n 2. PSD_SI\r\n");
				break;
			default:
				size = sprintf(buffer, "Odebrano nieznany znak: %c\n\r", received);
				break;
			}
		}
		HAL_UART_Transmit_IT(&huart2, buffer, size);
		HAL_UART_Receive_IT(&huart2, &received, 1);
	}
}

void sequential_read(uint8_t samples_amount, uint8_t delay_time_ms)
{
	int i, j;
	for (j = 0; j < samples_amount; ++j)
	{

		HAL_GPIO_WritePin(VL53L0X_TRIGGER_GPIO_Port, VL53L0X_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time_ms);
		my_VL53L0X_init();
		my_VL53L0X_read();
		temp_data[5] = my_VL53L0X_distance_mm();
		HAL_GPIO_WritePin(VL53L0X_TRIGGER_GPIO_Port, VL53L0X_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_time_ms);

		HAL_GPIO_WritePin(VL53L1X_TRIGGER_GPIO_Port, VL53L1X_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time_ms);
		my_VL53L1_init();
		my_VL53L1_read();
		temp_data[6] = my_VL53L1_distance_mm();
		HAL_GPIO_WritePin(VL53L1X_TRIGGER_GPIO_Port, VL53L1X_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_time_ms);

		HAL_GPIO_WritePin(SHARP_SMALL_1_TRIGGER_GPIO_Port,
		SHARP_SMALL_1_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time_ms);
		PSD_read();
		if (psd_output == PSD_RAW)
		{
			temp_data[1] = PSD_small_1_value();
		}
		else if (psd_output == PSD_SI)
		{
			temp_data[1] = PSD_small_1_distance_cm();
		}
		HAL_GPIO_WritePin(SHARP_SMALL_1_TRIGGER_GPIO_Port,
		SHARP_SMALL_1_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_time_ms);

		HAL_GPIO_WritePin(SHARP_SMALL_2_TRIGGER_GPIO_Port,
		SHARP_SMALL_2_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time_ms);
		PSD_read();
		if (psd_output == PSD_RAW)
		{
			temp_data[2] = PSD_small_2_value();
		}
		else if (psd_output == PSD_SI)
		{
			temp_data[2] = PSD_small_2_distance_cm();
		}
		HAL_GPIO_WritePin(SHARP_SMALL_2_TRIGGER_GPIO_Port,
		SHARP_SMALL_2_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_time_ms);

		HAL_GPIO_WritePin(SHARP_BIG_TRIGGER_GPIO_Port, SHARP_BIG_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time_ms * 2);
		PSD_read();
		if (psd_output == PSD_RAW)
		{
			temp_data[3] = PSD_big_value();
		}
		else if (psd_output == PSD_SI)
		{
			temp_data[3] = PSD_big_distance_cm();
		}
		HAL_GPIO_WritePin(SHARP_BIG_TRIGGER_GPIO_Port, SHARP_BIG_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_time_ms * 2);

		HAL_GPIO_WritePin(TFMini_TRIGGER_GPIO_Port, TFMini_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time_ms);
		lidar_read();
		temp_data[4] = lidar_distance_cm();
		HAL_GPIO_WritePin(TFMini_TRIGGER_GPIO_Port, TFMini_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_time_ms);

		for (i = 0; i < 8; ++i)
			temp_data[8] += temp_data[i];

		temp_data[8] &= 0xFF;

		if (output_type == OUTPUT_MACHINE)
		{

			message_size = sprintf(message_buff, "%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n", temp_data[0], temp_data[1], temp_data[2], temp_data[3],
					temp_data[4], temp_data[5], temp_data[6], temp_data[7], temp_data[8]);
			HAL_UART_Transmit_IT(&huart2, message_buff, message_size);

		}
		else if (output_type == OUTPUT_HUMAN)
		{
			message_size2 = sprintf(message_buff2, "%04d %04d %04d %04d %04d %04d %04d %04d %04d\r\n", temp_data[0], temp_data[1], temp_data[2], temp_data[3],
					temp_data[4], temp_data[5], temp_data[6], temp_data[7], temp_data[8]);
			HAL_UART_Transmit_IT(&huart2, message_buff2, message_size2);
		}
	}
}

void simultaneous_read(uint8_t samples_amount, uint8_t delay_time_ms)
{
	int i, j;

	for (j = 0; j < samples_amount; ++j)
	{

		HAL_GPIO_WritePin(VL53L0X_TRIGGER_GPIO_Port, VL53L0X_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VL53L1X_TRIGGER_GPIO_Port, VL53L1X_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SHARP_SMALL_1_TRIGGER_GPIO_Port,
		SHARP_SMALL_1_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SHARP_SMALL_2_TRIGGER_GPIO_Port,
		SHARP_SMALL_2_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SHARP_BIG_TRIGGER_GPIO_Port, SHARP_BIG_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TFMini_TRIGGER_GPIO_Port, TFMini_TRIGGER_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time_ms);

		if (j == 0)
		{
			my_VL53L0X_init();
			my_VL53L1_init();
		}

		PSD_read();
		lidar_read();
		my_VL53L0X_read();
		my_VL53L1_read();

		HAL_Delay(delay_time_ms);
		HAL_GPIO_WritePin(VL53L0X_TRIGGER_GPIO_Port, VL53L0X_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(VL53L1X_TRIGGER_GPIO_Port, VL53L1X_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SHARP_SMALL_1_TRIGGER_GPIO_Port,
		SHARP_SMALL_1_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SHARP_SMALL_2_TRIGGER_GPIO_Port,
		SHARP_SMALL_2_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SHARP_BIG_TRIGGER_GPIO_Port, SHARP_BIG_TRIGGER_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(TFMini_TRIGGER_GPIO_Port, TFMini_TRIGGER_Pin, GPIO_PIN_SET);

		if (psd_output == PSD_RAW)
		{
			temp_data[1] = PSD_small_1_distance_cm();
			temp_data[2] = PSD_small_2_distance_cm();
			temp_data[3] = PSD_big_distance_cm();
		}
		else if (psd_output == PSD_SI)
		{
			temp_data[1] = PSD_small_1_value();
			temp_data[2] = PSD_small_2_value();
			temp_data[3] = PSD_big_value();
		}
		temp_data[4] = lidar_distance_cm();
		temp_data[5] = my_VL53L0X_distance_mm();
		temp_data[6] = my_VL53L1_distance_mm();

		for (i = 0; i < 8; ++i)
			temp_data[8] += temp_data[i];

		temp_data[8] &= 0xFF;

		if (output_type == OUTPUT_MACHINE)
		{
			message_size = sprintf(message_buff, "%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n", temp_data[0], temp_data[1], temp_data[2], temp_data[3],
					temp_data[4], temp_data[5], temp_data[6], temp_data[7], temp_data[8]);
			HAL_UART_Transmit_IT(&huart2, message_buff, message_size);

		}
		else if (output_type == OUTPUT_HUMAN)
		{
			message_size = sprintf(message_buff, "%04d %04d %04d %04d %04d %04d %04d %04d %04d\r\n", temp_data[0], temp_data[1], temp_data[2], temp_data[3],
					temp_data[4], temp_data[5], temp_data[6], temp_data[7], temp_data[8]);
			HAL_UART_Transmit_IT(&huart2, message_buff2, message_size2);
		}
	}
}

void measure_psd_small_1()
{
	uint16_t temp_data;

	HAL_GPIO_WritePin(SHARP_SMALL_1_TRIGGER_GPIO_Port, SHARP_SMALL_1_TRIGGER_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	PSD_read();

	if (psd_output == PSD_RAW)
	{
		temp_data = PSD_small_1_value();
	}
	else if (psd_output == PSD_SI)
	{
		temp_data = PSD_small_1_distance_cm();
	}

	HAL_GPIO_WritePin(SHARP_SMALL_1_TRIGGER_GPIO_Port, SHARP_SMALL_1_TRIGGER_Pin, GPIO_PIN_SET);

	if (output_type == OUTPUT_HUMAN)
	{
		uint16_t message_buff[10];
		uint16_t message_size = sprintf(message_buff, "%04d\r\n", temp_data);
	}
	else if (output_type == OUTPUT_MACHINE)
	{
		uint8_t message_buff;
		uint8_t message_size = sprintf(message_buff, "%02\r\n", temp_data);
	}
}

void measure_psd_small_2()
{
	uint16_t temp_data;

	HAL_GPIO_WritePin(SHARP_SMALL_2_TRIGGER_GPIO_Port, SHARP_SMALL_2_TRIGGER_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	PSD_read();

	if (psd_output == PSD_RAW)
	{
		temp_data = PSD_small_2_value();
	}
	else if (psd_output == PSD_SI)
	{
		temp_data = PSD_small_2_distance_cm();
	}

	HAL_GPIO_WritePin(SHARP_SMALL_2_TRIGGER_GPIO_Port, SHARP_SMALL_2_TRIGGER_Pin, GPIO_PIN_SET);

	if (output_type == OUTPUT_HUMAN)
	{
		uint16_t message_buff[10];
		uint16_t message_size = sprintf(message_buff, "%04d\r\n", temp_data);
	}
	else if (output_type == OUTPUT_MACHINE)
	{
		uint8_t message_buff;
		uint8_t message_size = sprintf(message_buff, "%02\r\n", temp_data);
	}
}

void measure_psd_big()
{
	uint16_t temp_data;

	HAL_GPIO_WritePin(SHARP_BIG_TRIGGER_GPIO_Port, SHARP_BIG_TRIGGER_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	PSD_read();

	if (psd_output == PSD_RAW)
	{
		temp_data = PSD_big_value();
	}
	else if (psd_output == PSD_SI)
	{
		temp_data = PSD_big_distance_cm();
	}

	HAL_GPIO_WritePin(SHARP_BIG_TRIGGER_GPIO_Port, SHARP_BIG_TRIGGER_Pin, GPIO_PIN_SET);

	if (output_type == OUTPUT_HUMAN)
	{
		uint16_t message_buff[10];
		uint16_t message_size = sprintf(message_buff, "%04d\r\n", temp_data);
	}
	else if (output_type == OUTPUT_MACHINE)
	{
		uint8_t message_buff;
		uint8_t message_size = sprintf(message_buff, "%02\r\n", temp_data);
	}
}

void measure_TFMini()
{
	HAL_GPIO_WritePin(TFMini_TRIGGER_GPIO_Port, TFMini_TRIGGER_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	lidar_read();
	uint16_t temp_data = lidar_distance_cm();

	HAL_GPIO_WritePin(TFMini_TRIGGER_GPIO_Port, TFMini_TRIGGER_Pin, GPIO_PIN_SET);

	if (output_type == OUTPUT_HUMAN)
	{
		uint16_t message_buff[10];
		uint16_t message_size = sprintf(message_buff, "%04d\r\n", temp_data);
	}
	else if (output_type == OUTPUT_MACHINE)
	{
		uint8_t message_buff;
		uint8_t message_size = sprintf(message_buff, "%02\r\n", temp_data);
	}
}
void measure_VL53L0X()
{
	HAL_GPIO_WritePin(VL53L0X_TRIGGER_GPIO_Port, VL53L0X_TRIGGER_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	my_VL53L0X_init();
	my_VL53L0X_read();
	uint16_t temp_data = my_VL53L0X_distance_mm();

	HAL_GPIO_WritePin(VL53L0X_TRIGGER_GPIO_Port, VL53L0X_TRIGGER_Pin, GPIO_PIN_SET);

	if (output_type == OUTPUT_HUMAN)
	{
		uint16_t message_buff[10];
		uint16_t message_size = sprintf(message_buff, "%04d\r\n", temp_data);
	}
	else if (output_type == OUTPUT_MACHINE)
	{
		uint8_t message_buff;
		uint8_t message_size = sprintf(message_buff, "%02\r\n", temp_data);
	}
}
void measure_VL53L1()
{
	HAL_GPIO_WritePin(VL53L1X_TRIGGER_GPIO_Port, VL53L1X_TRIGGER_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);

	my_VL53L1_init();
	my_VL53L1_read();
	uint16_t temp_data = my_VL53L1_distance_mm();

	HAL_GPIO_WritePin(VL53L1X_TRIGGER_GPIO_Port, VL53L1X_TRIGGER_Pin, GPIO_PIN_SET);
	if (output_type == OUTPUT_HUMAN)
	{
		uint16_t message_buff[10];
		uint16_t message_size = sprintf(message_buff, "%04d\r\n", temp_data);
	}
	else if (output_type == OUTPUT_MACHINE)
	{
		uint8_t message_buff;
		uint8_t message_size = sprintf(message_buff, "%02\r\n", temp_data);
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
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
	MX_USART1_UART_Init();
	MX_SPI3_Init();
	/* USER CODE BEGIN 2 */
	set_trace_level_output_only();
	set_output_type_human();
	set_psd_value_raw();

	PSD_init();
	lidar_init();
	HAL_UART_Receive_IT(&huart2, &received, 1);
	trace(POSITIVE_MESSAGE, "Program has been started");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	temp_data[0] = 105; //frame header
	temp_data[7] = 0; //temp sonar val

	while (1)
	{

		if (read_flag == 1) //z
		{
			sequential_read(10, 50);

			HAL_Delay(20);
			read_flag = 0;
			message_size2 = sprintf(message_buff2, "--------------------------------------------\r\n");
			HAL_UART_Transmit_IT(&huart2, message_buff2, message_size2);
		}
		else if (read_flag == 2) //c
		{
			simultaneous_read(10, 50);

			HAL_Delay(20);
			read_flag = 0;
			message_size2 = sprintf(message_buff2, "--------------------------------------------\r\n");
			HAL_UART_Transmit_IT(&huart2, message_buff2, message_size2);
		}

		if (laser_flag != 0) //x
		{
			HAL_GPIO_TogglePin(LASER_TRIGGER_GPIO_Port, LASER_TRIGGER_Pin);
			--laser_flag;
		}

		if (single_sensor == 1) //q
		{
			trace(POSITIVE_MESSAGE, "PSD_SMALL_1:");
			measure_psd_small_1();
			single_sensor = 0;
		}

		else if (single_sensor == 2) //w
		{
			trace(POSITIVE_MESSAGE, "PSD_SMALL_2:");
			measure_psd_small_2();
			single_sensor = 0;
		}

		else if (single_sensor == 3) //e
		{
			trace(POSITIVE_MESSAGE, "PSD_BIG:");
			measure_psd_big();
			single_sensor = 0;
		}

		else if (single_sensor == 4) //r
		{
			trace(POSITIVE_MESSAGE, "TFMINI:");
			measure_TFMini();
			single_sensor = 0;
		}

		else if (single_sensor == 5) //t
		{
			trace(POSITIVE_MESSAGE, "VL53L0X:");
			measure_VL53L0X();
			single_sensor = 0;
		}

		else if (single_sensor == 6) //y
		{
			trace(POSITIVE_MESSAGE, "VL53L1:");
			measure_VL53L1();
			single_sensor = 0;
		}

		if (trace_mode == 2)
		{
			set_trace_level_critical();
			trace(POSITIVE_MESSAGE, "TRACE MODE SET TO CRITICAL_MESSAGE");
			trace_mode = 0;
		}
		else if (trace_mode == 3)
		{
			set_trace_level_positive();
			trace(POSITIVE_MESSAGE, "TRACE MODE SET TO POSITIVE_MESSAGE");
			trace_mode = 0;
		}
		else if (trace_mode == 4)
		{
			set_trace_level_output_only();
			trace(POSITIVE_MESSAGE, "TRACE MODE SET TO OUTPUT_ONLY");
			trace_mode = 0;
		}

		if (output_level == 2)
		{
			set_output_type_human();
			trace(POSITIVE_MESSAGE, "OUTPUT TYPE SET TO HUMAN");
			output_level = 0;
		}
		else if (output_level == 3)
		{
			set_output_type_machine();
			trace(POSITIVE_MESSAGE, "OUTPUT TYPE SET TO MACHINE");
			output_level = 0;
		}

		if (psd_level == 2)
		{
			set_psd_value_raw();
			trace(POSITIVE_MESSAGE, "PSD OUTPUT SET TO RAW");
			psd_level = 0;
		}
		else if (psd_level == 3)
		{
			set_psd_value_si();
			trace(POSITIVE_MESSAGE, "PSD OUTPUT SET TO SI");
			psd_level = 0;
		}

		HAL_Delay(10);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	PSD_free();
	lidar_free();
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
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
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
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
