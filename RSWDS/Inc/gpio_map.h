/*
 * gpio_map.h
 *
 *  Created on: 23.09.2018
 *      Author: piotr
 */

#ifndef SENSOR_GPIO_MAP_H_
#define SENSOR_GPIO_MAP_H_

#define SENSORS_I2C_N 1 /**< number of I2C sensors*/

/*!
 Structure which represents stepper motor
 */
typedef struct {
	/*!
	  \name DIR
	 */
	/*\{*/
	GPIO_TypeDef *gpio_port;		/**< Direction GPIO_port */
	uint16_t gpio_pin;			/**< Direction GPIO_pin */
	/*\}*/
} sensor_gpio_t;

const sensor_gpio_t sensor[SENSORS_I2C_N] = {

		{}

};

#endif /* SENSOR_GPIO_MAP_H_ */
