/*
 * pcf8574.h
 *
 *  Created on: Dec 30, 2014
 *      Author: peter
 */

#ifndef INC_PCF8574_H_
#define INC_PCF8574_H_

#include "stm32f1xx_hal.h"

/** @file	pcf8574.h
 * @brief	In order to use this you have to create a PCF8574_HandleTypeDef variable (e.g. "pcf").
 * Then you will set the the address based on the configuration of your chip (pins A0, A1, A2) ( pcf.PCF_I2C_ADDRESS ) (0 to 7),
 * timeout ( pcf.PCF_I2C_TIMEOUT ) (e.g. 1000 (=1 sec)),
 * I2C instance to use ( pcf.i2c.Instance ) (e.g. I2C1 or I2C2 ...),
 * speed of the communication ( pcf.i2c.Init.ClockSpeed ) (e.g. 100 000 (=100kHz)).
 *
 * Example:
 * example.c
 * example_msp.c
 */

/**
 * Provides possible return values for the functions
 */
typedef enum{
	PCF8574_OK,		/**< Everything went OK */
	PCF8574_ERROR	/**< An error occured */
} PCF8574_RESULT;

/** @def PCF8574_I2C_ADDRESS_MASK - Pulled from the datasheet
 */
#define PCF8574_I2C_ADDRESS_MASK	0x40

/**
 * PCF8574 handle structure which wraps all the necessary variables together in
 * order to simplify the communication with the chip
 */
typedef struct{
	uint8_t				PCF_I2C_ADDRESS;	/**< address of the chip you want to communicate with */
	uint32_t			PCF_I2C_TIMEOUT;	/**< timeout value for the communication in milliseconds */
	I2C_HandleTypeDef 	i2c;				/**< I2C_HandleTypeDef structure */
	void				(*errorCallback)(PCF8574_RESULT);
} PCF8574_HandleTypeDef;

/** @var PCF8574_Type0Pins[8] - characterization of pins for hardware of type 0
 */
extern uint32_t PCF8574_Type0Pins[];

/**
 * Initializes the I2C for communication
 * @param	handle - a pointer to the PCF8574 handle
 * @return	whether the function was successful or not
 */
PCF8574_RESULT PCF8574_Init(PCF8574_HandleTypeDef* handle);

/**
 * Deinitializes the I2C
 * @param	handle - a pointer to the PCF8574 handle
 * @return	whether the function was successful or not
 */
PCF8574_RESULT PCF8574_DeInit(PCF8574_HandleTypeDef* handle);

/**
 * Writes a given value to the port of PCF8574
 * @param	handle - a pointer to the PCF8574 handle
 * @param	val - a value to be written to the port
 * @return	whether the function was successful or not
 */
PCF8574_RESULT PCF8574_Write(PCF8574_HandleTypeDef* handle, uint8_t val);

/**
 * Reads the current state of the port of PCF8574
 * @param	handle - a pointer to the PCF8574 handle
 * @param	val - a pointer to the variable that will be assigned a value from the chip
 * @return	whether the function was successful or not
 */
PCF8574_RESULT PCF8574_Read(PCF8574_HandleTypeDef* handle, uint8_t* val);

#endif /* INC_PCF8574_H_ */
