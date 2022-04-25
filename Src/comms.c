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

// Includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "BLDC_controller.h"
#include "util.h"
#include "comms.h"

#if defined(DEBUG_SERIAL_PROTOCOL)
#if defined(DEBUG_SERIAL_PROTOCOL) && (defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3))

#ifdef CONTROL_ADC
  #define RAW_MIN 0
  #define RAW_MAX 4095
#else
  #define RAW_MIN -1000
  #define RAW_MAX 1000
#endif


#define MAX_PARAM_WATCH 15

extern ExtY rtY_Left;                   /* External outputs */
extern ExtU rtU_Left;                   /* External inputs */
extern P    rtP_Left;

extern ExtY rtY_Right;                  /* External outputs */
extern ExtU rtU_Right;                      /* External inputs */
extern P    rtP_Right;


extern InputStruct input1[];            // input structure
extern InputStruct input2[];            // input structure

extern uint16_t VirtAddVarTab[NB_OF_VAR];
extern int16_t speedAvg;                      // average measured speed
extern int16_t speedAvgAbs;                   // average measured speed in absolute
extern uint8_t ctrlModReqRaw;
extern int16_t batVoltageCalib;
extern int16_t board_temp_deg_c;
extern int16_t left_dc_curr;
extern int16_t right_dc_curr;
extern int16_t dc_curr;
extern int16_t cmdL; 
extern int16_t cmdR; 



enum commandTypes {READ,WRITE};
// Function0 - Function with 0 parameter
// Function1 - Function with 1 parameter (e.g. GET PARAM)
// Function2 - Function with 2 parameter (e.g. SET PARAM XXXX)
const command_entry commands[] = {
  // Type   ,Name      ,Function0         ,Function1       ,Function2      ,Help     
    {READ   ,"GET"     ,printAllParamDef  ,printParamDef   ,NULL           ,"Get Parameter/Variable"},
    {READ   ,"HELP"    ,printAllParamHelp ,printParamHelp  ,NULL           ,"Command/Parameter/Variable Help"},
    {READ   ,"WATCH"   ,NULL              ,watchParamVal   ,NULL           ,"Toggle Parameter/Variable Watch"},
    {WRITE  ,"SET"     ,NULL              ,NULL            ,setParamValExt ,"Set Parameter"},
    {WRITE  ,"INIT"    ,NULL              ,initParamVal    ,NULL           ,"Init Parameter from EEPROM or CONFIG.H"},
    {WRITE  ,"SAVE"    ,saveAllParamVal   ,NULL            ,NULL           ,"Save Parameters to EEPROM"},
};

enum paramTypes {PARAMETER,VARIABLE};
const parameter_entry params[] = {
  // CONTROL PARAMETERS
  // Type       ,Name                 ,Datatype ,ValueL ptr                  ,ValueR                    ,EEPRM Addr ,Init              Int/Ext ,Min    ,Max    ,Div             ,Mul  ,Fix   ,Callback Function  ,Help text
    {PARAMETER  ,"CTRL_MOD"           ,ADD_PARAM(ctrlModReqRaw)              ,NULL                      ,0          ,CTRL_MOD_REQ      ,0      ,1      ,3      ,0               ,0    ,0     ,NULL               ,"Ctrl mode 1:VLT 2:SPD 3:TRQ"},
    {PARAMETER  ,"CTRL_TYP"           ,ADD_PARAM(rtP_Left.z_ctrlTypSel)      ,&rtP_Right.z_ctrlTypSel   ,0          ,CTRL_TYP_SEL      ,0      ,0      ,2      ,0               ,0    ,0     ,NULL               ,"Ctrl type 0:COM 1:SIN 2:FOC"},
    {PARAMETER  ,"I_MOT_MAX"          ,ADD_PARAM(rtP_Left.i_max)             ,&rtP_Right.i_max          ,1          ,I_MOT_MAX         ,1      ,1      ,40     ,A2BIT_CONV      ,0    ,4     ,NULL               ,"Max phase current A"},
    {PARAMETER  ,"N_MOT_MAX"          ,ADD_PARAM(rtP_Left.n_max)             ,&rtP_Right.n_max          ,2          ,N_MOT_MAX         ,1      ,10     ,2000   ,0               ,0    ,4     ,NULL               ,"Max motor RPM"},
    {PARAMETER  ,"FI_WEAK_ENA"        ,ADD_PARAM(rtP_Left.b_fieldWeakEna)    ,&rtP_Right.b_fieldWeakEna ,0          ,FIELD_WEAK_ENA    ,0      ,0      ,1      ,0               ,0    ,0     ,NULL               ,"Enable field weak"},
  	{PARAMETER  ,"FI_WEAK_HI"         ,ADD_PARAM(rtP_Left.r_fieldWeakHi)     ,&rtP_Right.r_fieldWeakHi  ,0          ,FIELD_WEAK_HI     ,1      ,0      ,1500   ,0               ,0    ,4     ,Input_Lim_Init     ,"Field weak high RPM"},
	  {PARAMETER  ,"FI_WEAK_LO"         ,ADD_PARAM(rtP_Left.r_fieldWeakLo)     ,&rtP_Right.r_fieldWeakLo  ,0          ,FIELD_WEAK_LO     ,1      ,0      ,1000   ,0               ,0    ,4     ,Input_Lim_Init     ,"Field weak low RPM"},
    {PARAMETER  ,"FI_WEAK_MAX"        ,ADD_PARAM(rtP_Left.id_fieldWeakMax)   ,&rtP_Right.id_fieldWeakMax,0          ,FIELD_WEAK_MAX    ,1      ,0      ,20     ,A2BIT_CONV      ,0    ,4     ,NULL               ,"Field weak max current A(FOC)"},
    {PARAMETER  ,"PHA_ADV_MAX"        ,ADD_PARAM(rtP_Left.a_phaAdvMax)       ,&rtP_Right.a_phaAdvMax    ,0          ,PHASE_ADV_MAX     ,1      ,0      ,55     ,0               ,0    ,4     ,NULL               ,"Max Phase Adv angle Deg(SIN)"},     
  // INPUT PARAMETERS
  // Type       ,Name                 ,ValueL ptr                            ,ValueR                    ,EEPRM Addr ,Init              Int/Ext ,Min    ,Max    ,Div             ,Mul  ,Fix   ,Callback Function  ,Help text
    {VARIABLE   ,"IN1_RAW"            ,ADD_PARAM(input1[0].raw)              ,NULL                      ,0          ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input1 raw"},        
    {PARAMETER  ,"IN1_TYP"            ,ADD_PARAM(input1[0].typ)              ,NULL                      ,3          ,0                 ,0      ,0      ,3      ,0               ,0    ,0     ,0                  ,"Input1 type"},        
    {PARAMETER  ,"IN1_MIN"            ,ADD_PARAM(input1[0].min)              ,NULL                      ,4          ,RAW_MIN           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input1 min"},        
    {PARAMETER  ,"IN1_MID"            ,ADD_PARAM(input1[0].mid)              ,NULL                      ,5          ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input1 mid"},
    {PARAMETER  ,"IN1_MAX"            ,ADD_PARAM(input1[0].max)              ,NULL                      ,6          ,RAW_MAX           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input1 max"},        
    {VARIABLE   ,"IN1_CMD"            ,ADD_PARAM(input1[0].cmd)              ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,0                  ,"Input1 cmd"},        
    
    {VARIABLE   ,"IN2_RAW"            ,ADD_PARAM(input2[0].raw)              ,NULL                      ,0          ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input2 raw"},   
    {PARAMETER  ,"IN2_TYP"            ,ADD_PARAM(input2[0].typ)              ,NULL                      ,7          ,0                 ,0      ,0      ,3      ,0               ,0    ,0     ,0                  ,"Input2 type"},        
    {PARAMETER  ,"IN2_MIN"            ,ADD_PARAM(input2[0].min)              ,NULL                      ,8          ,RAW_MIN           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input2 min"},        
    {PARAMETER  ,"IN2_MID"            ,ADD_PARAM(input2[0].mid)              ,NULL                      ,9          ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input2 mid"},
    {PARAMETER  ,"IN2_MAX"            ,ADD_PARAM(input2[0].max)              ,NULL                      ,10         ,RAW_MAX           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Input2 max"},
    {VARIABLE   ,"IN2_CMD"            ,ADD_PARAM(input2[0].cmd)              ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,0                  ,"Input2 cmd"},
#if defined(PRI_INPUT1) && defined(PRI_INPUT2) && defined(AUX_INPUT1) && defined(AUX_INPUT2)  
  // Type       ,Name                 ,ValueL ptr                            ,ValueR                    ,EEPRM Addr ,Init              Int/Ext ,Min    ,Max    ,Div             ,Mul  ,Fix   ,Callback Function  ,Help text
    {VARIABLE   ,"AUX_IN1_RAW"        ,ADD_PARAM(input1[1].raw)              ,NULL                      ,0          ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input1 raw"},        
    {PARAMETER  ,"AUX_IN1_TYP"        ,ADD_PARAM(input1[1].typ)              ,NULL                      ,11         ,0                 ,0      ,0      ,3      ,0               ,0    ,0     ,0                  ,"Aux. input1 type"},        
    {PARAMETER  ,"AUX_IN1_MIN"        ,ADD_PARAM(input1[1].min)              ,NULL                      ,12         ,RAW_MIN           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input1 min"},        
    {PARAMETER  ,"AUX_IN1_MID"        ,ADD_PARAM(input1[1].mid)              ,NULL                      ,13         ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input1 mid"},
    {PARAMETER  ,"AUX_IN1_MAX"        ,ADD_PARAM(input1[1].max)              ,NULL                      ,14         ,RAW_MAX           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input1 max"},        
    {VARIABLE   ,"AUX_IN1_CMD"        ,ADD_PARAM(input1[1].cmd)              ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,0                  ,"Aux. input1 cmd"},        
    
    {VARIABLE   ,"AUX_IN2_RAW"        ,ADD_PARAM(input2[1].raw)              ,NULL                      ,0          ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input2 raw"},        
    {PARAMETER  ,"AUX_IN2_TYP"        ,ADD_PARAM(input2[1].typ)              ,NULL                      ,15         ,0                 ,0      ,0      ,3      ,0               ,0    ,0     ,0                  ,"Aux. input2 type"},        
    {PARAMETER  ,"AUX_IN2_MIN"        ,ADD_PARAM(input2[1].min)              ,NULL                      ,16         ,RAW_MIN           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input2 min"},        
    {PARAMETER  ,"AUX_IN2_MID"        ,ADD_PARAM(input2[1].mid)              ,NULL                      ,17         ,0                 ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input2 mid"},
    {PARAMETER  ,"AUX_IN2_MAX"        ,ADD_PARAM(input2[1].max)              ,NULL                      ,18         ,RAW_MAX           ,0      ,RAW_MIN,RAW_MAX,0               ,0    ,0     ,0                  ,"Aux. input2 max"},
    {VARIABLE   ,"AUX_IN2_CMD"        ,ADD_PARAM(input2[1].cmd)              ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,0                  ,"Aux. input2 cmd"},
#endif  
  // FEEDBACK
  // Type       ,Name                 ,Datatype, ValueL ptr                  ,ValueR                    ,EEPRM Addr ,Init              Int/Ext ,Min    ,Max    ,Div             ,Mul  ,Fix   ,Callback Function  ,Help text
    {VARIABLE   ,"DC_CURR"            ,ADD_PARAM(dc_curr)                    ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Total DC Link current A *100"},
    {VARIABLE   ,"RDC_CURR"           ,ADD_PARAM(right_dc_curr)              ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Right DC Link current A *100"},
    {VARIABLE   ,"LDC_CURR"           ,ADD_PARAM(left_dc_curr)               ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Left DC Link current A *100"},
    {VARIABLE   ,"CMDL"               ,ADD_PARAM(cmdL)                       ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Left Motor Command"},
    {VARIABLE   ,"CMDR"               ,ADD_PARAM(cmdR)                       ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Right Motor Command"},
    {VARIABLE   ,"SPD_AVG"            ,ADD_PARAM(speedAvg)                   ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Motor Measured Avg RPM"},
    {VARIABLE   ,"SPDL"               ,ADD_PARAM(rtY_Left.n_mot)             ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Left Motor Measured RPM"},
    {VARIABLE   ,"SPDR"               ,ADD_PARAM(rtY_Right.n_mot)            ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Right Motor Measured RPM"},
    {VARIABLE   ,"RATE"               ,0       , NULL                        ,NULL                      ,0          ,RATE              ,0      ,0      ,0      ,0               ,0    ,4     ,NULL               ,"Rate *10"},
    {VARIABLE   ,"SPD_COEF"           ,0       , NULL                        ,NULL                      ,0          ,SPEED_COEFFICIENT ,0      ,0      ,0      ,0               ,10   ,14    ,NULL               ,"Speed Coefficient *10"},
    {VARIABLE   ,"STR_COEF"           ,0       , NULL                        ,NULL                      ,0          ,STEER_COEFFICIENT ,0      ,0      ,0      ,0               ,10   ,14    ,NULL               ,"Steer Coefficient *10"},
    {VARIABLE   ,"BATV"               ,ADD_PARAM(batVoltageCalib)            ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Calibrated Battery voltage *100"},       
    {VARIABLE   ,"TEMP"               ,ADD_PARAM(board_temp_deg_c)           ,NULL                      ,0          ,0                 ,0      ,0      ,0      ,0               ,0    ,0     ,NULL               ,"Calibrated Temperature °C *10"},       

};


const char *errors[9] = {
  "Command not found", // Err1
  "Parameter not found", // Err2
  "This command cannot be used with a Variable", // Err3
  "Value not in range", // Err4
  "Value expected", // Err5
  "Start of line expected", // Err6
  "End of line expected", // Err7
  "Parameter expected", // Err8
  "Uncaught error" // Err9
  "Watch list is full" // Err10
};

debug_command command;
int8_t watchParamList[MAX_PARAM_WATCH] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; 

// Set Param with Value from external format
int8_t setParamValExt(uint8_t index, int32_t value) {   
  int8_t ret = 0;
  // check min and max before conversion to internal values
  if (IN_RANGE(value,params[index].min,params[index].max)){
    ret = setParamValInt(index,extToInt(index,value));
    printParamDef(index);
  }else{
    printError(4); // Error - Value out of range
  }
  return ret;
}

// Set Param with value from internal format
int8_t setParamValInt(uint8_t index, int32_t newValue) {
  int32_t oldValue = getParamValInt(index);
  if (oldValue != newValue){ 
    // if value is different, beep, cast and assign new value
    switch (params[index].datatype){
      case UINT8_T:
        if (params[index].valueL != NULL) *(uint8_t*)params[index].valueL = newValue;
        if (params[index].valueR != NULL) *(uint8_t*)params[index].valueR = newValue;
        break;
      case UINT16_T:
        if (params[index].valueL != NULL) *(uint16_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(uint16_t*)params[index].valueR = newValue;
        break;
      case UINT32_T:
        if (params[index].valueL != NULL) *(uint32_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(uint32_t*)params[index].valueR = newValue;
        break;
      case INT8_T:
        if (params[index].valueL != NULL) *(int8_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(int8_t*)params[index].valueR = newValue;
        break;
      case INT16_T:
        if (params[index].valueL != NULL) *(int16_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(int16_t*)params[index].valueR = newValue;
        break;
      case INT32_T:
        if (params[index].valueL != NULL) *(int32_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(int32_t*)params[index].valueR = newValue;
        break;
    }

    // Beep if value was modified
    beepShort(5);
  }

  // Run callback function if assigned
  if (params[index].callback_function) (*params[index].callback_function)();
  return 1;
}

// Get Parameter Internal value and translate to external 
int32_t getParamValExt(uint8_t index) {
  return intToExt(index,getParamValInt(index));
}

// Get Parameter Internal Value
int32_t getParamValInt(uint8_t index) {
  int32_t value = 0;

  int8_t countVar = 0;
  if (params[index].valueL != NULL) countVar++;
  if (params[index].valueR != NULL) countVar++;

  if (countVar > 0){
    // Read Left and Right values and calculate average 
    // If left and right have to be summed up, DIV field could be adapted to multiply by 2
    // Cast to parameter datatype
    switch (params[index].datatype){
      case UINT8_T:
        if (params[index].valueL != NULL) value += *(uint8_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(uint8_t*)params[index].valueR;
        break;
      case UINT16_T:
        if (params[index].valueL != NULL) value += *(uint16_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(uint16_t*)params[index].valueR;
        break;
      case UINT32_T:
        if (params[index].valueL != NULL) value += *(uint32_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(uint32_t*)params[index].valueR;
        break;
      case INT8_T:
        if (params[index].valueL != NULL) value += *(int8_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(int8_t*)params[index].valueR;
        break;
      case INT16_T:
        if (params[index].valueL != NULL) value += *(int16_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(int16_t*)params[index].valueR;
        break;
      case INT32_T:
        if (params[index].valueL != NULL) value += *(int32_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(int32_t*)params[index].valueR;
        break;
      default:
        value = 0;
    }

    // Divide by number of values provided for the parameter
    value /= countVar;
  }else{
    // No variable was provided, return init value that might contain a macro
    value = params[index].init;
  }

  return value;
}

// Add or remove parameter from watch list
int8_t watchParamVal(uint8_t index){
  int8_t i,found = 0;
  for(i=0;i < MAX_PARAM_WATCH && watchParamList[i]>-1;i++){
    if (watchParamList[i] == index) found = 1;
    if (found) watchParamList[i] = (i < MAX_PARAM_WATCH-1)?watchParamList[i+1]:-1;
  }
  if (!found){
    if (watchParamList[i] == -1){
      watchParamList[i] = index;
      return 1;
    }
    printError(10);
    return 0;
  }
  return 1;
}

// Print value for all parameters with watch flag
int8_t printParamVal(){
  int8_t i = 0; 
  for(i=0;i < MAX_PARAM_WATCH && watchParamList[i]>-1;i++){
    printf("%s:%li ",params[watchParamList[i]].name,getParamValExt(watchParamList[i]));
  }
  if (i>0) printf("\r\n");
  return 1;
}

// Print help for Command
int8_t printCommandHelp(uint8_t index){
  printf("? %s:\"%s\"\r\n",commands[index].name,commands[index].help);
  return 1;
}

// Print help for parameter
int8_t printParamHelp(uint8_t index){
  printf("? %s:\"%s\" ",params[index].name,params[index].help);
  if (params[index].type == PARAMETER) printf("[min:%li max:%li]",params[index].min,params[index].max);
  printf("\r\n");
  return 1;
}

// Print help for all parameters
int8_t printAllParamHelp(){
  printf("? Commands\r\n");
  for(int i=0;i<COMMAND_SIZE(commands);i++)
    printCommandHelp(i);
  printf("?\r\n");

  printf("? Parameters\r\n");
  for(int i=0;i<PARAM_SIZE(params);i++){
    if (params[i].type == PARAMETER) printParamHelp(i);
  }
  printf("?\r\n");

  printf("? Variables\r\n");
  for(int i=0;i<PARAM_SIZE(params);i++){
    if (params[i].type == VARIABLE) printParamHelp(i);
  }
  printf("?\r\n");

  return 1;
}

// Print definition(name,value,initial value, min, max) for parameter
int8_t printParamDef(uint8_t index){
  printf("# name:\"%s\" value:%li init:%li min:%li max:%li\r\n",
         params[index].name,     // Parameter Name
         getParamValExt(index),  // Parameter Value translated to external format
         getParamInitExt(index), // Parameter Init Value translated to external format
         params[index].min,      // Parameter Min Value with External format 
         params[index].max);     // Parameter Max Value with External format
  return 1;
}

// Print definition(name,value,initial value, min, max) for all parameters
int8_t printAllParamDef(){
  for(int i=0;i<PARAM_SIZE(params);i++) printParamDef(i);
  return 1;
}

void printError(uint8_t errornum ){
  printf("! Err%i:\"%s\"\r\n",errornum,errors[errornum-1]);
}

// Function to increment a value
// Get Parameter in External format, check max value, increment, set Parameter
// Not used in the protocol yet 
int8_t incrParamVal(uint8_t index) {
  // This should be used only if min and max values are known
  if (params[index].min == params[index].max) return 0;
  
  uint32_t value = getParamValExt(index);
  if (value < params[index].max){
    return setParamValExt(index,value + 1);
  }else{
    return setParamValExt(index,(int32_t) params[index].min);
  } 
}

// Get internal Parameter value and save it to EEprom for all paraemeter with an address assigned 
int8_t saveAllParamVal() {
  HAL_FLASH_Unlock();
  EE_WriteVariable(VirtAddVarTab[0] , (uint16_t)FLASH_WRITE_KEY);
  for(int i=0;i<PARAM_SIZE(params);i++){ 
    // Only Parameters with eeprom address can be saved
    if (params[i].addr){
      EE_WriteVariable(VirtAddVarTab[params[i].addr] , (uint16_t)getParamValInt(i));    
    }
  }
  HAL_FLASH_Lock();
  return 1;
}

// Translate from Internal to External format
int32_t intToExt(uint8_t index,int32_t value){
  // Multiply for small number
  if(params[index].mul) value *= params[index].mul;
  // Divide to translate to external format
  if(params[index].div) value /= params[index].div;
  // Shift to translate to external format
  if(params[index].fix) value >>= params[index].fix;
  return value;
}

// Translate from External to Internal Format
int32_t extToInt(uint8_t index,int32_t value){
  // Multiply to translate to internal format
  if(params[index].div) value *= params[index].div;
  // Shift to translate to internal format
  if (params[index].fix) value <<= params[index].fix;
  // Divide for small number
  if(params[index].mul) value /= params[index].mul;
  return value;
}

// Get Parameter Init value(EEPROM or init/config.h) and translate to external format
int32_t getParamInitExt(uint8_t index) {
  return intToExt(index,getParamInitInt(index));
}

// Get Parameter value with EEprom data if address is avalaible, init/config.h value otherwise
int16_t getParamInitInt(uint8_t index){
  if (params[index].addr){
    // if EEPROM address is specified, init from EEPROM address
    uint16_t writeCheck, readVal;
    
    HAL_FLASH_Unlock();
    EE_ReadVariable(VirtAddVarTab[0], &writeCheck);
    EE_ReadVariable(VirtAddVarTab[params[index].addr] , &readVal);
    HAL_FLASH_Lock();
    
    // EEPROM was written, use stored value
    if (writeCheck == FLASH_WRITE_KEY){
      return readVal;
    }else{
      // Use init value from array
      if (params[index].initFormat){
        // Init Value is in External format (e.g. PHA_ADV_MAX is 25 deg)
        return extToInt(index,params[index].init);
      }else{
        return params[index].init;
      }
    }
  }else{
    if (params[index].initFormat){
      // Init Value is in External format (e.g. PHA_ADV_MAX is 25 deg)
      return extToInt(index,params[index].init);
    }else{
      return params[index].init;
    }
  }
}


// initialize Parameter value with EEprom data if address is avalaible, init/config.h value otherwise
int8_t initParamVal(uint8_t index) {
  int8_t ret = 0;
  ret = setParamValInt(index,(int32_t) getParamInitInt(index));
  printParamDef(index);
  return ret;
}

// Find command in commands array and return index
int8_t findCommand(uint8_t *userCommand, uint32_t len){
  for(int index=0;index<COMMAND_SIZE(commands);index++){
    uint8_t command_len = strlen(commands[index].name);
    if (command_len < len){
      if (memcmp(userCommand,commands[index].name,command_len)==0){
        return index;
      }
    }
  }
  return -1; // Not found
}

// Find parameter in params array and return index
int8_t findParam(uint8_t *userCommand, uint32_t len){
  for(int index=0;index<PARAM_SIZE(params);index++){
    uint8_t param_len = strlen(params[index].name);
    if (param_len < len){
      if (memcmp(userCommand,params[index].name,param_len)==0){
        return index;
      }
    }
  }
  return -1; // Not found
}

// Parse and save the command to be executed
void handle_input(uint8_t *userCommand, uint32_t len)
{

  // If there is already an unprocessed command, exit
  if (command.semaphore == 1) return;
  if (*userCommand != '$') return; // reject if first character is not $ 
  
  // Check end of line
  userCommand+=len-1; // Go to last char
  if (*userCommand != '\n' && *userCommand != '\r'){
    command.error = 7; // Error - End of line expected
    return;
  }
  userCommand-=len-1; // Come back
  userCommand++; // Skip $

  int8_t  cindex = -1;
  int8_t  pindex = -1;
  uint8_t size   = 0;

  // Find Command
  cindex = findCommand(userCommand,len);
  if (cindex == -1){
    // Error - Command not found
    command.error = 1;
    return;
  }

  // Skip command characters
  size = strlen(commands[cindex].name);
  {len-=size;userCommand+=size;}
  // Skip if space
  if (*userCommand == 0x20){len-=1;userCommand+=1;}

  if (*userCommand == '\n' || *userCommand == '\r'){
    if (commands[cindex].callback_function0 != NULL){
      // Command without parameter
      command.semaphore = 1;
      command.command_index = cindex;
      command.param_index   = -1;
      command.param_value   = 0;
    }else{
      command.error = 8; // Error - Parameter expected
    }
    return;
  }

  // Find parameter
  pindex = findParam(userCommand,len);
  if (pindex == -1){
    // Error - Parameter not found
    command.error = 2;
    return;
  }

  // Skip parameter characters
  size = strlen(params[pindex].name);
  {len-=size;userCommand+=size;}
  // Skip if space
  if (*userCommand == 0x20){len-=1;userCommand+=1;}
   
  if (commands[cindex].type == WRITE && params[pindex].type == VARIABLE){
    // Error - This command cannot be used with a Variable
    command.error = 3;
    return;
  }
  
  if (commands[cindex].callback_function1 != NULL){
    if (*userCommand == '\n' || *userCommand == '\r'){
      // Command with parameter
      command.semaphore = 1;
      command.command_index = cindex;
      command.param_index   = pindex;
      command.param_value   = 0;
    }else{
      command.error = 7; // Error - End of line expected
    }
    return;
  }
  
  int32_t value = 0;
  int8_t  sign  = 1;
  int8_t  count = 0;

  // Read sign
  if (*userCommand == '-'){len-=1;userCommand+=1;sign =-1;} 
  // Read value
  for (value=0; (unsigned)*userCommand-'0'<10; userCommand++){
    value = 10*value+(*userCommand-'0');
    count++;
    // Error - Value out of range
    if (value>MAX_int16_T){command.error = 4;return;}
  }

  if (count == 0){
    // Error - Value required
    command.error = 5;
    return;
  }
      
  // Apply sign
  value*= sign;

  // Command with parameter and value
  if (commands[cindex].callback_function2 != NULL){
    if (*userCommand == '\n' || *userCommand == '\r'){
      command.semaphore = 1;
      command.command_index = cindex;
      command.param_index   = pindex;
      command.param_value   = value;
    }else{
      command.error = 7; // Error - End of line expected
    }
    return;
  }

  // Uncaught error
  command.error = 9;

}

void process_debug()
{
  
  // Print parameters from watch list
  printParamVal();

  // Show Error if any
  if(command.error> 0){
    printError(command.error);
    command.error = 0;
    return;
  }

  // Nothing to do
  if (command.semaphore == 0) return;

  int8_t ret = 0;
  if (commands[command.command_index].callback_function0 != NULL && 
      command.param_index == -1){
    // This function needs no parameter
    ret = (*commands[command.command_index].callback_function0)();
    if (ret==1){printf("OK\r\n");}
    command.semaphore = 0;
    return;
  }

  if (commands[command.command_index].callback_function1 != NULL &&
      command.param_index != -1){
    // This function needs only a parameter
    ret = (*commands[command.command_index].callback_function1)(command.param_index);
    if (ret==1){printf("OK\r\n");}
    command.semaphore = 0;
    return;
  }  

  if (commands[command.command_index].callback_function2 != NULL && 
      command.param_index != -1){
    // This function needs an additional parameter
    ret = (*commands[command.command_index].callback_function2)(command.param_index,command.param_value);
    if (ret==1){printf("OK\r\n");}
    command.semaphore = 0;
  }
}

#endif
#endif  // DEBUG_SERIAL_PROTOCOL

