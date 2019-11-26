/*
 * pcf8574.c
 *
 *  Created on: Dec 30, 2014
 *      Author: peter
 */

#include "pcf8574.h"

PCF8574_RESULT PCF8574_Init(PCF8574_HandleTypeDef* handle) {

	handle->PCF_I2C_ADDRESS &= 0x07;

	/*if (handle->i2c.State == HAL_I2C_STATE_RESET) {
		handle->i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		handle->i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		handle->i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
		handle->i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
		handle->i2c.Init.OwnAddress1 = 0xFE;
		if (HAL_I2C_Init(&handle->i2c) != HAL_OK) {
			handle->errorCallback(PCF8574_ERROR);
			return PCF8574_ERROR;
		}
	}*/
	return PCF8574_OK;
}

PCF8574_RESULT PCF8574_DeInit(PCF8574_HandleTypeDef* handle) {
	HAL_I2C_DeInit(&handle->i2c);
	return PCF8574_OK;
}

PCF8574_RESULT PCF8574_Write(PCF8574_HandleTypeDef* handle, uint8_t val) {
	if (HAL_I2C_Master_Transmit(&handle->i2c,
			(handle->PCF_I2C_ADDRESS << 1) | PCF8574_I2C_ADDRESS_MASK, &val, 1,
			handle->PCF_I2C_TIMEOUT) != HAL_OK) {
		//handle->errorCallback(PCF8574_ERROR);
		return PCF8574_ERROR;
	}

	/*if (HAL_I2C_Master_Transmit_DMA(&handle->i2c,
			(handle->PCF_I2C_ADDRESS << 1) | PCF8574_I2C_ADDRESS_MASK, &val, 1) != HAL_OK) {
		handle->errorCallback(PCF8574_ERROR);
		//<return PCF8574_ERROR;
	}*/

	//HAL_I2C_Master_Transmit_DMA(&hi2c2, 0xA4, (uint8_t*)ai2cBuffer, 2);
	return PCF8574_OK;
}

PCF8574_RESULT PCF8574_Read(PCF8574_HandleTypeDef* handle, uint8_t* val) {
	if (HAL_I2C_Master_Receive(&handle->i2c,
			(handle->PCF_I2C_ADDRESS << 1) | PCF8574_I2C_ADDRESS_MASK, val, 1,
			handle->PCF_I2C_TIMEOUT) != HAL_OK) {
		return PCF8574_ERROR;
	}
	return PCF8574_OK;
}
