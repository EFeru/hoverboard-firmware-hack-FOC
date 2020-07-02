#include "defines.h"
#include "config.h"
#include "bipropellantProtocolMachine.h"

#include "comms.h"
#include "util.h"
#include "stm32f1xx_hal.h"
#include "bldc.h"
#include "setup.h"

#include <string.h>
#include <stdlib.h>

#include "bipropellantProtocolStructs.h"

#if defined(USART2_ENABLE)
    PROTOCOL_STAT sUSART2;
#endif
#if defined(USART3_ENABLE)
    PROTOCOL_STAT sUSART3;
#endif

extern uint32_t timeoutCnt;

extern int main_ascii_init(PROTOCOL_STAT *s); // from ascii_proto_funcs.c


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x09 enable

static char protocol_enable = 0;

void fn_enable ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            protocol_enable = bldc_getMotorsEnable();
            break;

        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            if ( bldc_getMotorsEnable() != protocol_enable) {
                // clear speeds to zero every time it is changed
                PWMData.pwm[0] = 0;
                PWMData.pwm[1] = 0;
            }
            bldc_setMotorsEnable(protocol_enable);
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x02 HallData

/* see hallinterrupts.h */

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x03 SpeedData

PROTOCOL_SPEED_DATA SpeedData = {
    {0, 0},

    600, // max power (PWM)
    -600,  // min power
    40 // minimum mm/s which we can ask for
};


void fn_SpeedData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x0D PWMData and 0x0E PWMData.pwm

PROTOCOL_PWM_DATA PWMData = {
    .pwm[0] = 0,
    .pwm[1] = 0,
    .speed_max_power =  600,
    .speed_min_power = -600,
    .speed_minimum_pwm = 40 // guard value, below this set to zero
};

extern int pwms[2];

void fn_PWMData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            ctrlModReq = 1; // 1 = VOLTAGE mode (default), 2 = SPEED mode, 3 = TORQUE mode.
            timeoutCnt = 0;
            break;
    }

    fn_defaultProcessing(s, param, cmd, msg);

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            for (int i = 0; i < 2; i++) {
                if (((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] > ((PROTOCOL_PWM_DATA*) (param->ptr))->speed_max_power) {
                    ((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] = ((PROTOCOL_PWM_DATA*) (param->ptr))->speed_max_power;
                }
                if (((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] < ((PROTOCOL_PWM_DATA*) (param->ptr))->speed_min_power) {
                    ((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] = ((PROTOCOL_PWM_DATA*) (param->ptr))->speed_min_power;
                }
                if ((((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] > 0) && (((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] < ((PROTOCOL_PWM_DATA*) (param->ptr))->speed_minimum_pwm)) {
                    ((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] = 0;
                }
                if ((((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] < 0) && (((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] > -((PROTOCOL_PWM_DATA*) (param->ptr))->speed_minimum_pwm)) {
                    ((PROTOCOL_PWM_DATA*) (param->ptr))->pwm[i] = 0;
                }
            }
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x21 BuzzerData

PROTOCOL_BUZZER_DATA BuzzerData = {
    .buzzerFreq = 0,
    .buzzerPattern = 0,
    .buzzerLen = 0,
};

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

void fn_BuzzerData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
        /*
            ((PROTOCOL_BUZZER_DATA*) (param->ptr))->buzzerFreq       = buzzerFreq;
            ((PROTOCOL_BUZZER_DATA*) (param->ptr))->buzzerLen        = buzzerLen;
            ((PROTOCOL_BUZZER_DATA*) (param->ptr))->buzzerPattern    = buzzerPattern;
            */
            break;
    }

    fn_defaultProcessing(s, param, cmd, msg);

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
        /*
            buzzerFreq      = ((PROTOCOL_BUZZER_DATA*) (param->ptr))->buzzerFreq;
            buzzerLen       = ((PROTOCOL_BUZZER_DATA*) (param->ptr))->buzzerLen;
            buzzerPattern   = ((PROTOCOL_BUZZER_DATA*) (param->ptr))->buzzerPattern;
            */
            break;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x08 Electrical Params

extern int16_t curDC_max;
extern int16_t curL_DC;
extern int16_t curR_DC;

void fn_electrictalParams ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            if(param != NULL && param->ptr != NULL && param->len == sizeof(ELECTRICAL_PARAMS))
            {
                ELECTRICAL_PARAMS *measured = (ELECTRICAL_PARAMS*) param->ptr;
                memset(measured, 0, sizeof(ELECTRICAL_PARAMS));
                measured->bat_raw = adc_buffer.batt1;
                measured->batteryVoltage = (float)batVoltageFixdt / (float)0xFFFF;
                measured->board_temp_raw = adc_buffer.temp;
                measured->dcCurLim = curDC_max;
                measured->motors[0].dcAmps = curL_DC;
                measured->motors[1].dcAmps = curR_DC;
            }
            break;
    }
    fn_defaultProcessingReadOnly(s, param, cmd, msg);
}


////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
int setup_protocol(PROTOCOL_STAT *s) {

    protocol_GetTick = HAL_GetTick;
    protocol_Delay = HAL_Delay;
    protocol_SystemReset =HAL_NVIC_SystemReset;


    int errors = 0;

    #if defined(USART2_ENABLE)

      extern int USART2_DMA_send(unsigned char *data, int len);

      errors += protocol_init(&sUSART2);

      sUSART2.send_serial_data=USART2_DMA_send;
      sUSART2.send_serial_data_wait=USART2_DMA_send;
      sUSART2.timeout1 = 500;
      sUSART2.timeout2 = 100;
      sUSART2.allow_ascii = 1;

    #endif

    #if defined(USART3_ENABLE)

      extern int USART3_DMA_send(unsigned char *data, int len);

      errors += protocol_init(&sUSART3);

      sUSART3.send_serial_data=USART3_DMA_send;
      sUSART3.send_serial_data_wait=USART3_DMA_send;
      sUSART3.timeout1 = 500;
      sUSART3.timeout2 = 100;
      sUSART3.allow_ascii = 1;

    #endif

    // initialise ascii protocol functions
    main_ascii_init(s);



//        errors += setParamVariable( s, 0x02, UI_NONE, (void *)&HallData, sizeof(HallData) );

        errors += setParamVariable( s, 0x03, UI_NONE, &SpeedData, sizeof(SpeedData) );
        setParamHandler( s, 0x03, fn_SpeedData );

        setParamHandler( s, 0x08, fn_electrictalParams );

        errors += setParamVariable( s, 0x09, UI_CHAR, &protocol_enable, sizeof(protocol_enable) );
        setParamHandler( s, 0x09, fn_enable );

        errors += setParamVariable( s, 0x0D, UI_NONE, &PWMData, sizeof(PWMData) );
        setParamHandler( s, 0x0D, fn_PWMData );

        errors += setParamVariable( s, 0x0E, UI_2LONG, &(PWMData.pwm), sizeof(PWMData.pwm) );
        setParamHandler( s, 0x0E, fn_PWMData );

        errors += setParamVariable( s, 0x21, UI_NONE, &BuzzerData, sizeof(BuzzerData) );
        setParamHandler( s, 0x21, fn_BuzzerData );

    return errors;

}
