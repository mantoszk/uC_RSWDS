/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SHARP_SMALL_1_ADC_Pin GPIO_PIN_0
#define SHARP_SMALL_1_ADC_GPIO_Port GPIOC
#define SHARP_SMALL_2_ADC_Pin GPIO_PIN_1
#define SHARP_SMALL_2_ADC_GPIO_Port GPIOC
#define SHARP_BIG_ADC_Pin GPIO_PIN_2
#define SHARP_BIG_ADC_GPIO_Port GPIOC
#define PC_DEBUG_TX_Pin GPIO_PIN_2
#define PC_DEBUG_TX_GPIO_Port GPIOA
#define PC_DEBUG_RX_Pin GPIO_PIN_3
#define PC_DEBUG_RX_GPIO_Port GPIOA
#define VL53L1X_INT_Pin GPIO_PIN_4
#define VL53L1X_INT_GPIO_Port GPIOA
#define VL53L1X_INT_EXTI_IRQn EXTI4_IRQn
#define VL53L0X_INT_Pin GPIO_PIN_5
#define VL53L0X_INT_GPIO_Port GPIOA
#define VL53L0X_INT_EXTI_IRQn EXTI9_5_IRQn
#define SHARP_BIG_TRIGGER_Pin GPIO_PIN_6
#define SHARP_BIG_TRIGGER_GPIO_Port GPIOA
#define SHARP_SMALL_2_TRIGGER_Pin GPIO_PIN_7
#define SHARP_SMALL_2_TRIGGER_GPIO_Port GPIOA
#define VL53L1X_TRIGGER_Pin GPIO_PIN_13
#define VL53L1X_TRIGGER_GPIO_Port GPIOB
#define TFMini_TX_Pin GPIO_PIN_6
#define TFMini_TX_GPIO_Port GPIOC
#define TFMini_RX_Pin GPIO_PIN_7
#define TFMini_RX_GPIO_Port GPIOC
#define VL53L1X_SDA_Pin GPIO_PIN_9
#define VL53L1X_SDA_GPIO_Port GPIOC
#define VL53L1X_SCL_Pin GPIO_PIN_8
#define VL53L1X_SCL_GPIO_Port GPIOA
#define ROTATION_TX_Pin GPIO_PIN_9
#define ROTATION_TX_GPIO_Port GPIOA
#define ROTATION_RX_Pin GPIO_PIN_10
#define ROTATION_RX_GPIO_Port GPIOA
#define VL53L0X_TRIGGER_Pin GPIO_PIN_11
#define VL53L0X_TRIGGER_GPIO_Port GPIOA
#define LASER_TRIGGER_Pin GPIO_PIN_12
#define LASER_TRIGGER_GPIO_Port GPIOA
#define SONAR_SCK_Pin GPIO_PIN_10
#define SONAR_SCK_GPIO_Port GPIOC
#define SONAR_MISO_Pin GPIO_PIN_11
#define SONAR_MISO_GPIO_Port GPIOC
#define SONAR_MOSI_Pin GPIO_PIN_12
#define SONAR_MOSI_GPIO_Port GPIOC
#define SHARP_SMALL_1_TRIGGER_Pin GPIO_PIN_6
#define SHARP_SMALL_1_TRIGGER_GPIO_Port GPIOB
#define TFMini_TRIGGER_Pin GPIO_PIN_7
#define TFMini_TRIGGER_GPIO_Port GPIOB
#define VL53L0X_SCL_Pin GPIO_PIN_8
#define VL53L0X_SCL_GPIO_Port GPIOB
#define VL53L0X_SDA_Pin GPIO_PIN_9
#define VL53L0X_SDA_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
