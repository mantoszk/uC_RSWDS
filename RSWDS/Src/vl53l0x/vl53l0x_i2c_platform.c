/*
 * vl53l0x_i2c_platform.c
 *
 *  Created on: 16.02.2017
 *      Author: The
 */


#include "vl53l0x/vl53l0x_i2c_platform.h"

uint8_t _i2c_buffer[64];

int32_t VL53L0X_write_byte(I2C_HandleTypeDef* i2c_handle,uint8_t address,  uint8_t index, uint8_t   data){

	_i2c_buffer[0]=index;
	_i2c_buffer[1]=data;

	return HAL_I2C_Master_Transmit(i2c_handle,address,_i2c_buffer,2,10);

}

int32_t VL53L0X_read_byte(I2C_HandleTypeDef* i2c_handle,uint8_t address,  uint8_t index, uint8_t  *pdata){

	if(HAL_I2C_Master_Transmit(i2c_handle,address,&index,1,10)){
		return 1;
	}

	return HAL_I2C_Master_Receive(i2c_handle,address|1,pdata,1,10);



}

int32_t VL53L0X_read_dword(I2C_HandleTypeDef* i2c_handle,uint8_t address, uint8_t index, uint32_t *pdata) {
	if (HAL_I2C_Master_Transmit(i2c_handle, address, &index, 1, 10)) {
		return 1;
	}
	if (HAL_I2C_Master_Receive(i2c_handle, address | 1, _i2c_buffer, 4, 10)) {
		return 1;
	}

	*pdata = ((uint32_t) _i2c_buffer[0] << 24)
			+ ((uint32_t) _i2c_buffer[1] << 16)
			+ ((uint32_t) _i2c_buffer[2] << 8) + (uint32_t) _i2c_buffer[3];
	return 0;
}


int32_t VL53L0X_write_word(I2C_HandleTypeDef* i2c_handle,uint8_t address,  uint8_t index, uint16_t  data){


    _i2c_buffer[0] = index;
    _i2c_buffer[1] = data >> 8;
    _i2c_buffer[2] = data & 0x00FF;

	return HAL_I2C_Master_Transmit(i2c_handle,address,_i2c_buffer,3,10);
}


int32_t VL53L0X_read_word(I2C_HandleTypeDef* i2c_handle,uint8_t address,  uint8_t index, uint16_t *pdata){
	if (HAL_I2C_Master_Transmit(i2c_handle, address, &index, 1, 10)) {
		return 1;
	}
	if (HAL_I2C_Master_Receive(i2c_handle, address | 1, _i2c_buffer, 2, 10)) {
		return 1;
	}

	 *pdata = ((uint16_t)_i2c_buffer[0]<<8) + (uint16_t)_i2c_buffer[1];
	return 0;

}


int32_t VL53L0X_read_multi(I2C_HandleTypeDef* i2c_handle,uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count){
	if (HAL_I2C_Master_Transmit(i2c_handle, address, &index, 1, 10)) {
		return 1;
	}
	if (HAL_I2C_Master_Receive(i2c_handle, address | 1, pdata, count, 10)) {
		return 1;
	}

	return 0;


}


int32_t VL53L0X_write_multi(I2C_HandleTypeDef* i2c_handle,uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count){
    _i2c_buffer[0] = index;
    memcpy(&_i2c_buffer[1], pdata, count);

	return HAL_I2C_Master_Transmit(i2c_handle,address,_i2c_buffer,count+1,10);

}

