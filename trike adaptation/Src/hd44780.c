/*
 *  * hd44780.c
 *
 *  Created on: Feb 20, 2014
 *      Author: Peter
 */

#include "hd44780.h"

uint32_t PCF8574_Type0Pins[8] = { 4, 5, 6, 7, 0, 1, 2, 3 };
uint8_t LCDerrorFlag = 0;

void LCD_WaitForBusyFlag(LCD_PCF8574_HandleTypeDef* handle) {
	uint8_t flag;
	LCD_GetBusyFlag(handle, &flag);
	//uint32_t startTick=HAL_GetTick();
	//while (flag == 1 && HAL_GetTick()-startTick<handle->pcf8574.PCF_I2C_TIMEOUT) {
	//	LCD_GetBusyFlag(handle, &flag);
	//}
	return;
}

LCD_RESULT LCD_I2C_WriteOut(LCD_PCF8574_HandleTypeDef* handle) {
	if (!LCDerrorFlag) {
		if (PCF8574_Write(&handle->pcf8574, handle->state) != PCF8574_OK) {
			//handle->errorCallback(LCD_ERROR);
			LCDerrorFlag = 1;
			return LCD_ERROR;
		}
		return LCD_OK;
	}
	return LCD_ERROR;
}

LCD_RESULT LCD_StateLEDControl(LCD_PCF8574_HandleTypeDef* handle, uint8_t on) {
	return LCD_StateWriteBit(handle, on & 1, LCD_PIN_LED);
}

LCD_RESULT LCD_StateWriteBit(LCD_PCF8574_HandleTypeDef* handle, uint8_t value,
		LCD_PIN pin) {

	if (value) {
		handle->state |= 1 << handle->pins[pin];
	} else {
		handle->state &= ~(1 << handle->pins[pin]);
	}
	return LCD_I2C_WriteOut(handle);
}

LCD_RESULT LCD_Init(LCD_PCF8574_HandleTypeDef* handle) {
	handle->D = 1;
	handle->B = 0;
	handle->C = 0;
	if (handle->type == TYPE0) {
		handle->pins = PCF8574_Type0Pins;
	} else {
		//handle->errorCallback(LCD_ERROR);
		return LCD_ERROR;	// no type of subinterface was specified
	}
	if (PCF8574_Init(&handle->pcf8574) != PCF8574_OK) {
		//handle->errorCallback(LCD_ERROR);
		return LCD_ERROR;
	}

	//HAL_Delay(50);
	LCD_StateWriteBit(handle, 0, LCD_PIN_RS);
	LCD_StateWriteBit(handle, 0, LCD_PIN_RW);
	LCD_StateWriteBit(handle, 0, LCD_PIN_E);

	LCD_WriteToDataBus(handle, 3);

	LCD_StateWriteBit(handle, 1, LCD_PIN_E);
	HAL_Delay(1);
	LCD_StateWriteBit(handle, 0, LCD_PIN_E);
	HAL_Delay(5);

	LCD_WriteToDataBus(handle, 3);

	LCD_StateWriteBit(handle, 1, LCD_PIN_E);
	HAL_Delay(1);
	LCD_StateWriteBit(handle, 0, LCD_PIN_E);
	HAL_Delay(1);

	LCD_WriteToDataBus(handle, 3);

	LCD_StateWriteBit(handle, 1, LCD_PIN_E);
	HAL_Delay(1);
	LCD_StateWriteBit(handle, 0, LCD_PIN_E);
	HAL_Delay(1);

	LCD_WriteToDataBus(handle, 2);

	LCD_StateWriteBit(handle, 1, LCD_PIN_E);
	HAL_Delay(1);
	LCD_StateWriteBit(handle, 0, LCD_PIN_E);
	HAL_Delay(1);

	uint8_t cmd = 0;
	cmd = cmd | (handle->NUMBER_OF_LINES << 3);
	cmd = cmd | (1 << 5);

	LCD_WriteCMD(handle, cmd);	// setting interface

	cmd = 0;
	cmd = cmd | (1 << 3);
	cmd = cmd | (handle->C << 1);
	cmd = cmd | handle->B;

	LCD_WriteCMD(handle, cmd);	// setting display/cursor

	LCD_ClearDisplay(handle);

	LCD_EntryModeSet(handle, DIRECTION_INCREMENT, SHIFT_NO);

	LCD_DisplayON(handle);

	LCD_StateLEDControl(handle, 1);	// LED power on

	return LCD_OK;

}

LCD_RESULT LCD_WriteToDataBus(LCD_PCF8574_HandleTypeDef* handle, uint8_t data) {
	if ((data & 1) == 1) {
		handle->state |= 1 << handle->pins[LCD_PIN_D4];
	} else {
		handle->state &= ~(1 << handle->pins[LCD_PIN_D4]);
	}

	if ((data & 2) == 2) {
		handle->state |= 1 << handle->pins[LCD_PIN_D5];
	} else {
		handle->state &= ~(1 << handle->pins[LCD_PIN_D5]);
	}

	if ((data & 4) == 4) {
		handle->state |= 1 << handle->pins[LCD_PIN_D6];
	} else {
		handle->state &= ~(1 << handle->pins[LCD_PIN_D6]);
	}

	if ((data & 8) == 8) {
		handle->state |= 1 << handle->pins[LCD_PIN_D7];
	} else {
		handle->state &= ~(1 << handle->pins[LCD_PIN_D7]);
	}

	return LCD_I2C_WriteOut(handle);
}

LCD_RESULT LCD_GetBusyFlag(LCD_PCF8574_HandleTypeDef* handle, uint8_t* flag) {

	LCD_StateWriteBit(handle, 0, LCD_PIN_E);
	LCD_StateWriteBit(handle, 0, LCD_PIN_RS);
	LCD_StateWriteBit(handle, 1, LCD_PIN_RW);

	LCD_StateWriteBit(handle, 1, LCD_PIN_E);

	//PCF8574_Read(&handle->pcf8574, flag);

	//*flag &= 1 << handle->pins[LCD_PIN_D7];
	//*flag >>= handle->pins[LCD_PIN_D7];

	LCD_StateWriteBit(handle, 0, LCD_PIN_E);

	LCD_StateWriteBit(handle, 1, LCD_PIN_E);

	//uint8_t flag2;
	//PCF8574_Read(&handle->pcf8574, &flag2);

	LCD_StateWriteBit(handle, 0, LCD_PIN_E);
	LCD_StateWriteBit(handle, 0, LCD_PIN_RW);

	return LCD_OK;

}

LCD_RESULT LCD_WriteCMD(LCD_PCF8574_HandleTypeDef* handle, uint8_t cmd) {
	if (!LCDerrorFlag) {
		LCD_StateWriteBit(handle, 0, LCD_PIN_E);
		LCD_StateWriteBit(handle, 0, LCD_PIN_RS);

		LCD_WriteToDataBus(handle, cmd >> 4);
		LCD_StateWriteBit(handle, 1, LCD_PIN_E);
		LCD_StateWriteBit(handle, 0, LCD_PIN_E);

		LCD_WriteToDataBus(handle, cmd);
		LCD_StateWriteBit(handle, 1, LCD_PIN_E);
		LCD_StateWriteBit(handle, 0, LCD_PIN_E);

		LCD_WaitForBusyFlag(handle);

		return LCD_OK;
	} return LCD_ERROR;

}

LCD_RESULT LCD_WriteDATA(LCD_PCF8574_HandleTypeDef* handle, uint8_t data) {

	LCD_StateWriteBit(handle, 0, LCD_PIN_E);
	LCD_StateWriteBit(handle, 1, LCD_PIN_RS);

	LCD_WriteToDataBus(handle, data >> 4);
	LCD_StateWriteBit(handle, 1, LCD_PIN_E);
	LCD_StateWriteBit(handle, 0, LCD_PIN_E);

	LCD_WriteToDataBus(handle, data);
	LCD_StateWriteBit(handle, 1, LCD_PIN_E);
	LCD_StateWriteBit(handle, 0, LCD_PIN_E);

	LCD_WaitForBusyFlag(handle);

	return LCD_OK;

}

LCD_RESULT LCD_SetLocation(LCD_PCF8574_HandleTypeDef* handle, uint8_t x,
		uint8_t y) {

	uint8_t add = 0x40 * y + x;
	uint8_t cmd = 1 << 7;
	cmd = cmd | add;
	return LCD_WriteCMD(handle, cmd);

}

LCD_RESULT LCD_WriteString(LCD_PCF8574_HandleTypeDef* handle, char *s) {
	int i = 0;

	if (s != 0) {

		while (i < 80 && s[i] != 0) {
			LCD_WaitForBusyFlag(handle);
			LCD_WriteDATA(handle, s[i]);
			i++;
		}
	}
	return LCD_OK;
}

LCD_RESULT LCD_ClearDisplay(LCD_PCF8574_HandleTypeDef* handle) {
	return LCD_WriteCMD(handle, 1);
}

LCD_RESULT LCD_DisplayON(LCD_PCF8574_HandleTypeDef* handle) {
	handle->D = 1;
	uint8_t cmd = 0;
	cmd = cmd | (1 << 3);
	cmd = cmd | (handle->D << 2);
	cmd = cmd | (handle->C << 1);
	cmd = cmd | handle->B;
	return LCD_WriteCMD(handle, cmd);
}

LCD_RESULT LCD_DisplayOFF(LCD_PCF8574_HandleTypeDef* handle) {
	handle->D = 0;
	uint8_t cmd = 0;
	cmd = cmd | (1 << 3);
	cmd = cmd | (handle->D << 2);
	cmd = cmd | (handle->C << 1);
	cmd = cmd | handle->B;
	return LCD_WriteCMD(handle, cmd);
}

LCD_RESULT LCD_CursorON(LCD_PCF8574_HandleTypeDef* handle, uint8_t blink) {
	handle->C = 1;
	blink &= 1;
	handle->B = blink;
	uint8_t cmd = 0;
	cmd = cmd | (1 << 3);
	cmd = cmd | (handle->D << 2);
	cmd = cmd | (handle->C << 1);
	cmd = cmd | handle->B;
	return LCD_WriteCMD(handle, cmd);
}

LCD_RESULT LCD_CursorOFF(LCD_PCF8574_HandleTypeDef* handle) {
	handle->C = 0;
	uint8_t cmd = 0;
	cmd = cmd | (1 << 3);
	cmd = cmd | (handle->D << 2);
	cmd = cmd | (handle->C << 1);
	cmd = cmd | handle->B;
	return LCD_WriteCMD(handle, cmd);
}

LCD_RESULT LCD_ShiftCursor(LCD_PCF8574_HandleTypeDef* handle, uint8_t direction,
		uint8_t steps) {
	direction &= 1;

	uint8_t cmd = 0;
	cmd |= 1 << 4;
	cmd |= direction << 2;

	int i = 0;
	for (i = 0; i < steps; i++) {
		if (LCD_WriteCMD(handle, cmd) != LCD_OK) {
			//handle->errorCallback(LCD_ERROR);
			return LCD_ERROR;
		}
	}
	return LCD_OK;
}

LCD_RESULT LCD_ShiftDisplay(LCD_PCF8574_HandleTypeDef* handle,
		uint8_t direction, uint8_t steps) {
	direction &= 1;

	uint8_t cmd = 0;
	cmd |= 1 << 4;
	cmd |= 1 << 3;
	cmd |= direction << 2;

	int i = 0;
	for (i = 0; i < steps; i++) {
		if (LCD_WriteCMD(handle, cmd) != LCD_OK) {
			//handle->errorCallback(LCD_ERROR);
			return LCD_ERROR;
		}
	}
	return LCD_OK;
}

LCD_RESULT LCD_WriteNumber(LCD_PCF8574_HandleTypeDef* handle, unsigned long n,
		uint8_t base) {

	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char *str = &buf[sizeof(buf) - 1];

	*str = '\0';

	// prevent crash if called with base == 1
	if (base < 2)
		base = 10;

	do {
		unsigned long m = n;
		n /= base;
		char c = m - base * n;
		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);
	return LCD_WriteString(handle, str);
}

LCD_RESULT LCD_WriteFloat(LCD_PCF8574_HandleTypeDef* handle, double number,
		uint8_t digits) {
	// Handle negative numbers
	if (number < 0.0) {
		LCD_WriteString(handle,"-");
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	double rounding = 0.5;
	for (uint8_t i = 0; i < digits; ++i)
		rounding /= 10.0;

	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long int_part = (unsigned long) number;
	double remainder = number - (double) int_part;
	LCD_WriteNumber(handle,int_part,10);

	// Print the decimal point, but only if there are digits beyond
	if (digits > 0) {
		LCD_WriteString(handle,".");
	}

	// Extract digits from the remainder one at a time
	while (digits-- > 0) {
		remainder *= 10.0;
		int toPrint = (int)(remainder);
		LCD_WriteNumber(handle,toPrint,10);
		remainder -= toPrint;
	}
	return LCD_OK;
}

LCD_RESULT LCD_EntryModeSet(LCD_PCF8574_HandleTypeDef* handle,
		LCD_DIRECTION_INC_DEC direction, LCD_SHIFT shift) {

	uint8_t cmd = 0;
	cmd |= 1 << 2;
	cmd |= direction << 1;
	cmd |= shift;

	return LCD_WriteCMD(handle, cmd);

}

LCD_RESULT LCD_CustomChar(LCD_PCF8574_HandleTypeDef* handle, uint8_t *pattern,
		uint8_t address) {
	uint8_t a = 0;
	int i = 0;
	a = 8 * address;
	LCD_WriteCMD(handle, a | 0x40);
	for (i = 0; i < 8; i++) {
		LCD_WriteDATA(handle, pattern[i]);
	}
	return LCD_OK;
}
