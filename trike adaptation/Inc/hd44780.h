/*
 * hd44780.h
 *
 *  Created on: Feb 20, 2014
 *      Author: Peter
 */

#ifndef HD44780_H_
#define HD44780_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "pcf8574.h"

/**
 * @file	hd44780.h
 * @brief	Header file for communication with the HD44780 LCD driver.
 * To use it you will have to create a variable of type LCD_PCF8574_HandleTypeDef (e.g. "lcd") and then
 * set the I2C address based on the address pins on your PCF8574 (0-7) (lcd.pcf8574.PCF_I2C_ADDRESS),
 * set the I2C timeout (in milliseconds) (lcd.pcf8574.PCF_I2C_TIMEOUT),
 * set the I2C instance (e.g. I2C1 or I2C2) (lcd.pcf8574.i2c.Instance),
 * set the I2C clock speed (in Hertz) (lcd.pcf8574.i2c.Init.ClockSpeed),
 * set the number of lines (has to be type of LCD_NUMBER_OF_LINES) (lcd.NUMBER_OF_LINES),
 * set the interface type (has to be type of LCD_TYPE) (lcd.type).
 *
 * Example:
 * example.c
 * example_msp.c
 */

/** LCD Interface possibilities
 */
typedef enum{
	PCF8574,	/*!< Use PCF8574 I2C IO expander as the interface */
	GPIO		/*!< Use GPIO pins directly */
} LCD_INTERFACE;

/** Possible return values for the functions
 */
typedef enum{
	LCD_OK,		/*!< Everything went OK */
	LCD_ERROR	/*!< An error occured */
} LCD_RESULT;

/** Type of hardware to use
 */
typedef enum{
	TYPE0,
	TYPE1,
	TYPE2
} LCD_TYPE;


/** Number of lines on your LCD
 */
typedef enum{
	NUMBER_OF_LINES_1=0,
	NUMBER_OF_LINES_4=3,
	NUMBER_OF_LINES_2=1
} LCD_NUMBER_OF_LINES;

/**
 * Structure that hold all the required variables in
 * order to simplify the communication process
 */
typedef struct{
	LCD_NUMBER_OF_LINES		NUMBER_OF_LINES;	/**< Number of lines on your LCD */
	uint8_t 				D;
	uint8_t 				C;
	uint8_t 				B;
	char 					lcdbuf[2][16];		/**< Buffer for the LCD */
	int 					x, oldx, y, oldy;
	uint8_t 				state;				/**< Holds current state of the PCF8574 expander */
	uint32_t*				pins;				/**< Array of pins based on your hardware (wiring) */
	LCD_TYPE				type;				/**< Type of hardware you want to use */
	PCF8574_HandleTypeDef 	pcf8574;			/**< PCF8574_HandleTypeDef for communication with PCF8574 */
	void					(*errorCallback)(LCD_RESULT);
} LCD_PCF8574_HandleTypeDef;

/** @def INTERFACE - Selector for the type of interface you want to use (has to be a type of LCD_INTERFACE) */
#define LCD_INTERFACE_SELECTOR			PCF8574

/** Enumeration of the LCD pins */
typedef enum{
	LCD_PIN_D4=0,
	LCD_PIN_D5=1,
	LCD_PIN_D6=2,
	LCD_PIN_D7=3,
	LCD_PIN_RS=4,
	LCD_PIN_RW=5,
	LCD_PIN_E=6,
	LCD_PIN_LED=7
} LCD_PIN;

/** Used to specify the direction in certain LCD operations */
typedef enum{
	DIRECTION_LEFT=0,
	DIRECTION_RIGHT=1
} LCD_DIRECTION;

/**  */
typedef enum{
	DIRECTION_INCREMENT=1,
	DIRECTION_DECREMENT=2
} LCD_DIRECTION_INC_DEC;

/**  */
typedef enum{
	SHIFT_YES=1,
	SHIFT_NO=0
} LCD_SHIFT;

#if LCD_INTERFACE_SELECTOR==PCF8574

/**
 *	LCD initialization function
 *	@param	handle - a pointer to the LCD handle
 *	@return	whether the function was successful or not
 */
LCD_RESULT LCD_Init(LCD_PCF8574_HandleTypeDef* handle);

/**
 * LCD deinitialization function
 * @param	handle - a pointer to the LCD handle
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_DeInit(LCD_PCF8574_HandleTypeDef* handle);

/**
 * Sends a command to the HD44780 controller
 * @param	handle - a pointer to the LCD handle
 * @param	cmd - a command you want to send
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_WriteCMD(LCD_PCF8574_HandleTypeDef* handle, uint8_t cmd);

/**
 * Sends data to the HD44780 controller
 * @param	handle - a pointer to the LCD handle
 * @param	data - data you want to send
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_WriteDATA(LCD_PCF8574_HandleTypeDef* handle, uint8_t data);

/**
 * Gets the state of the busy flag
 * @param	handle - a pointer to the LCD handle
 * @param	flag - a pointer to a variable that will contain the state of the flag
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_GetBusyFlag(LCD_PCF8574_HandleTypeDef* handle,uint8_t* flag);

/**
 * Writes lower 4bits of data to the data bus of the controller
 * @param	handle - a pointer to the LCD handle
 * @param	data - data you want to put on the data bus (lower 4bits)
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_WriteToDataBus(LCD_PCF8574_HandleTypeDef* handle, uint8_t data);

/**
 * Clears the LCD
 * @param	handle - a pointer to the LCD handle
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_ClearDisplay(LCD_PCF8574_HandleTypeDef* handle);

/**
 * Writes a string to the LCD
 * @param	handle - a pointer to the LCD handle
 * @param	s - string you want to write to the LCD
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_WriteString(LCD_PCF8574_HandleTypeDef* handle, char *s);

/**
 * Sets the location of the memory pointer in the controller (used to control other operations (for example where to write a string))
 * @param	handle - a pointer to the LCD handle
 * @param	x - x-coordinate of the location
 * @param	y - y-coordinate of the location
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_SetLocation(LCD_PCF8574_HandleTypeDef* handle, uint8_t x, uint8_t y);

/**
 * Turns ON the display
 * @param	handle - a pointer to the LCD handle
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_DisplayON(LCD_PCF8574_HandleTypeDef* handle);

/**
 * Turns OFF the display
 * @param	handle - a pointer to the LCD handle
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_DisplayOFF(LCD_PCF8574_HandleTypeDef* handle);

/**
 * Turns ON the cursor
 * @param	handle - a pointer to the LCD handle
 * @param	blink - if you want the cursor to blink set this to 1, else 0
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_CursorON(LCD_PCF8574_HandleTypeDef* handle, uint8_t blink);

/**
 * Turns OFF the cursor
 * @param	handle - a pointer to the LCD handle
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_CursorOFF(LCD_PCF8574_HandleTypeDef* handle);

/**
 * Shifts the cursor in the specified direction certain number of steps
 * @param	handle - a pointer to the LCD handle
 * @param	direction - specifies the direction
 * @param	steps - specifies how many positions to shift the cursor by
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_ShiftCursor(LCD_PCF8574_HandleTypeDef* handle, LCD_DIRECTION direction,uint8_t steps);

/**
 * Shifts the contents of the LCD
 * @param	handle - a pointer to the LCD handle
 * @param	direction - directions of the shift
 * @param	steps - how many positions to shift the contents by
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_ShiftDisplay(LCD_PCF8574_HandleTypeDef* handle, uint8_t direction, uint8_t steps);

/**
 * Writes a number to the LCD
 * @param	handle - a pointer to the LCD handle
 * @param	n - a number you want to write to the LCD
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_WriteNumber(LCD_PCF8574_HandleTypeDef* handle, unsigned long n, uint8_t base);


LCD_RESULT LCD_WriteFloat(LCD_PCF8574_HandleTypeDef* handle, double number, uint8_t digits);

/**
 * Sets the mode by which data is written to the LCD
 * @param	handle - a pointer to the LCD handle
 * @param	direction
 * @param	shift
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_EntryModeSet(LCD_PCF8574_HandleTypeDef* handle, LCD_DIRECTION_INC_DEC direction,LCD_SHIFT shift);

/**
 * Creates a custom character at the given address
 * @param	handle - a pointer to the LCD handle
 * @param	pattern - pointer to the bit pattern of the character
 * @param	address - an address to which the character will be written
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_CustomChar(LCD_PCF8574_HandleTypeDef* handle, uint8_t *pattern,uint8_t address);

/**
 * Writes the current state to the PCF8574 expander
 * @param	handle - a pointer to the LCD handle
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_I2C_WriteOut(LCD_PCF8574_HandleTypeDef* handle);

/**
 * Controls the state of the LCD backlight
 * @param	handle - a pointer to the LCD handle
 * @param	on - set it to 1 if you want to turn the backlight on, else 0
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_StateLEDControl(LCD_PCF8574_HandleTypeDef* handle, uint8_t on);

/**
 * Rewrites a bit in the state variable with the value specified
 * @param	handle - a pointer to the LCD handle
 * @param	value - value of the bit (0 or 1)
 * @param	pin - pin which you want to write to
 * @return	whether the function was successful or not
 */
LCD_RESULT LCD_StateWriteBit(LCD_PCF8574_HandleTypeDef* handle, uint8_t value, LCD_PIN pin);

/**
 * Waits until the busy flag is reset
 * @param	handle - a pointer to the LCD handle
 */
void LCD_WaitForBusyFlag(LCD_PCF8574_HandleTypeDef* handle);

#endif

#endif /* HD44780_H_ */
