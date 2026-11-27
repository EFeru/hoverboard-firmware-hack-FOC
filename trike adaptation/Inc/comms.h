/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef COMMS_H
#define COMMS_H

#include "stm32f1xx_hal.h"

#if defined(DEBUG_SERIAL_PROTOCOL)

enum types {UINT8_T,UINT16_T,UINT32_T,INT8_T,INT16_T,INT32_T,INT,FLOAT};
#define typename(x) _Generic((x), \
    uint8_t:    UINT8_T, \
    uint16_t:   UINT16_T, \
    uint32_t:   UINT32_T, \
    int8_t:     INT8_T, \
    int16_t:    INT16_T, \
    int32_t:    INT32_T, \
    int:        INT, \
    float:      FLOAT)

#define PARAM_SIZE(param) sizeof(param) / sizeof(parameter_entry)
#define COMMAND_SIZE(command) sizeof(command) / sizeof(command_entry)

#define SIZEP(x) ((char*)(&(x) + 1) - (char*)&(x))
#define ADD_PARAM(var) typename(var),&var


int32_t extToInt(uint8_t index,int32_t value);
int8_t  setParamValInt(uint8_t index, int32_t newValue);
int8_t  setParamValExt(uint8_t index, int32_t newValue);
int32_t intToExt(uint8_t index,int32_t value);
int32_t getParamValInt(uint8_t index);
int32_t getParamValExt(uint8_t index);

int8_t initParamVal(uint8_t index);
int8_t incrParamVal(uint8_t index);

int8_t saveAllParamVal();
int16_t getParamInitInt(uint8_t index);
int32_t getParamInitExt(uint8_t index);
int8_t printCommandHelp(uint8_t index);
int8_t printParamHelp(uint8_t index);
int8_t printAllParamHelp();
int8_t printParamVal();
int8_t printParamDef(uint8_t index);
int8_t printAllParamDef();
void printError(uint8_t errornum );
int8_t watchParamVal(uint8_t index);

int8_t findCommand(uint8_t *userCommand, uint32_t len);
int8_t findParam(uint8_t *userCommand, uint32_t len);
void handle_input(uint8_t *userCommand, uint32_t len);
void process_debug();


typedef struct debug_command_struct debug_command;
struct debug_command_struct {
  uint8_t semaphore;
  uint8_t error;
  int8_t command_index;
  int8_t param_index;
  int32_t param_value;
};

typedef struct command_entry_struct command_entry;
struct command_entry_struct {
  const uint8_t type;
  const char *name;
  int8_t (*callback_function0)();
  int8_t (*callback_function1)(uint8_t index);
  int8_t (*callback_function2)(uint8_t index,int32_t value);
  const char *help;
};

typedef struct parameter_entry_struct parameter_entry;
struct parameter_entry_struct {
  const uint8_t type;
  const char *name;
  const uint8_t datatype;
  void *valueL;
  void *valueR;
  const uint16_t addr;
  const int32_t init;
  const uint8_t initFormat;
  const int32_t min;
  const int32_t max;
  const uint8_t div;
  const uint8_t mul;
  const uint8_t fix;
  void (*callback_function)();
  const char *help;
};

#endif  // DEBUG_SERIAL_PROTOCOL
#endif  // COMMS_H
