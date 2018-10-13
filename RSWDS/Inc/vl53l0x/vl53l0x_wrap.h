/*
 * vl53l0x_stm_api.h
 *
 *  Created on: 25.05.2017
 *      Author: The
 */

#ifndef API_VL_VL53L0X_STM_API_H_
#define API_VL_VL53L0X_STM_API_H_

#include "vl53l0x_api.h"

/*!
 * Tryby mierzenia
 *
 */
typedef enum{
	HIGH_SPEED=0,
	HIGH_ACCURACY,
	LONG_RANGE
}RangingConfig_e ;


/*!
 * Startuje pomiar, nie czeka na wynik. Jesli continous to startuje pomiary.
 */
uint8_t VL_start_measurement_non_blocking(VL53L0X_DEV dev);


uint8_t VL_stop_measurment(VL53L0X_DEV dev);

/*!
 * Jesli sa nowe gotowe dane to zwraca 0,w distance sďż˝ dane, jeďż˝li nie wykryto obiektu to -1 w distance
 */
uint8_t VL_get_measurement_if_ready(VL53L0X_DEV dev, int16_t* distance);


/*!
 * Pobiera pomiar
 */
uint8_t VL_get_measurement(VL53L0X_DEV dev, int16_t* distance);


/*!
 * Ustawia parametry czujnika, takze mode, ktďż˝ry decyduje czy continuos czy single
 *
 */
uint8_t VL_init_parameters(VL53L0X_Dev_t *dev, RangingConfig_e rangingConfig,VL53L0X_DeviceModes mode);



/*!
 * Ustawia parametry czujnika, takze mode, ktďż˝ry decyduje czy continuos czy single
 *
 */
uint8_t VL_init_parameters(VL53L0X_Dev_t *dev, RangingConfig_e rangingConfig,VL53L0X_DeviceModes mode);

//jesli -1 to out of range, blokuje
uint8_t VL_single_measurement_blocking(VL53L0X_DEV dev, int16_t* distance);

//inicjuje czujnik z podanym adresem, reset ustawiany jest przez pin ustawiony w strukturze VL53L0X_DEV
uint8_t VL_init_sensor_set_addres(VL53L0X_DEV dev,uint8_t addr);


/*!
 * Ustawia czujnik w tryb reset - wyďż˝ďż˝cza brutalnie, potem trzeba od nowa inicjowac
 *
 */
void VL_reset_on(VL53L0X_DEV dev);

/*!
 * Ustawia tryb przerwania na czujniku, 0/1 - wlaczone/wylaczone
 * w srodku funkcji mozna zmienic jakim zboczem ma byc okreslane przerwanie
 */
uint8_t VL_set_interrupt(VL53L0X_DEV dev,uint8_t state,VL53L0X_InterruptPolarity polar);


VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev);
VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev);
VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice);


void VL53L0X_Measurement(void);


#endif /* API_VL_VL53L0X_STM_API_H_ */
