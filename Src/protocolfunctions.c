#include "defines.h"
#include "config.h"
#include "protocolfunctions.h"

#include "comms.h"

#include "stm32f1xx_hal.h"


#include <string.h>
#include <stdlib.h>

#include "control_structures.h"

#if defined(SERIAL_USART2_IT)
    PROTOCOL_STAT sUSART2;
#endif
#if defined(SERIAL_USART3_IT) && !defined(CONTROL_SENSOR)
    PROTOCOL_STAT sUSART3;
#endif

extern volatile uint32_t input_timeout_counter; // global variable for input timeout
extern uint32_t timeout;


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x09 enable

extern uint8_t enable; // global variable for motor enable
extern void init_PID_control(); // from main
extern int main_ascii_init(PROTOCOL_STAT *s); // from ascii_proto_funcs.c

//////////////////////////////////////////////
// make values safe before we change enable...

char protocol_enable = 0;
void fn_enable ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            protocol_enable = enable;
            break;

        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
            if (!protocol_enable) {
                // assume we will enable,
                // set wanted posn to current posn, else we may rush into a wall
//                PosnData.wanted_posn_mm[0] = HallData[0].HallPosn_mm;
//                PosnData.wanted_posn_mm[1] = HallData[1].HallPosn_mm;

                // clear speeds to zero
                SpeedData.wanted_speed_mm_per_sec[0] = 0;
                SpeedData.wanted_speed_mm_per_sec[1] = 0;
                PWMData.pwm[0] = 0;
                PWMData.pwm[1] = 0;
//                init_PID_control();
            }
            enable = protocol_enable;
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}


#ifdef CONTROL_SENSOR
////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x01 sensor_data

extern SENSOR_DATA sensor_data[2];

// used to send only pertinent data, not the whole structure
PROTOCOL_SENSOR_FRAME sensor_copy[2];

void fn_SensorData ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
            // copy just sensor input data
            memcpy(&sensor_copy[0], &sensor_data[0].complete, sizeof(sensor_copy[0]));
            memcpy(&sensor_copy[1], &sensor_data[1].complete, sizeof(sensor_copy[1]));
            break;
    }
    fn_defaultProcessingReadOnly(s, param, cmd, msg);
}

#endif

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
//            control_type = CONTROL_TYPE_SPEED;
//            input_timeout_counter = 0;
            break;
    }
    fn_defaultProcessing(s, param, cmd, msg);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x04 Position

POSN Position;

void fn_Position ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
        /*
            ((POSN*) (param->ptr))->LeftAbsolute = HallData[0].HallPosn_mm;
            ((POSN*) (param->ptr))->LeftOffset = HallData[0].HallPosn_mm - HallData[0].HallPosn_mm_lastread;
            ((POSN*) (param->ptr))->RightAbsolute = HallData[1].HallPosn_mm;
            ((POSN*) (param->ptr))->RightOffset = HallData[1].HallPosn_mm - HallData[1].HallPosn_mm_lastread;
            */
            break;
    }

    fn_defaultProcessing(s, param, cmd, msg);

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
        /*
            HallData[0].HallPosn_mm_lastread = ((POSN*) (param->ptr))->LeftAbsolute;
            HallData[1].HallPosn_mm_lastread = ((POSN*) (param->ptr))->RightAbsolute;
            */
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x05 PositionIncr

POSN_INCR PositionIncr;

void fn_PositionIncr ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {

    fn_defaultProcessing(s, param, cmd, msg);

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
/*            // if switching to control type POSITION,
            if ((control_type != CONTROL_TYPE_POSITION) || !enable) {
                control_type = CONTROL_TYPE_POSITION;
                // then make sure we won't rush off somwehere strange
                // by setting our wanted posn to where we currently are...
                fn_enable( s, param, PROTOCOL_CMD_READVALRESPONSE, msg); // TODO: I don't like calling this with a param entry which does not fit to the handler..
            }

            enable = 1;
            input_timeout_counter = 0;

            // increment our wanted position
            PosnData.wanted_posn_mm[0] += ((POSN_INCR*) (param->ptr))->Left;
            PosnData.wanted_posn_mm[1] += ((POSN_INCR*) (param->ptr))->Right;
            */
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x06 PosnData

PROTOCOL_POSN_DATA PosnData = {
    {0, 0},

    200, // max pwm in posn mode
    70, // min pwm in posn mode
};


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x07 RawPosition

PROTOCOL_POSN RawPosition;

void fn_RawPosition ( PROTOCOL_STAT *s, PARAMSTAT *param, unsigned char cmd, PROTOCOL_MSG3full *msg ) {
    switch (cmd) {
        case PROTOCOL_CMD_READVAL:
        case PROTOCOL_CMD_SILENTREAD:
        /*
            ((PROTOCOL_POSN*) (param->ptr))->LeftAbsolute = HallData[0].HallPosn;
            ((PROTOCOL_POSN*) (param->ptr))->LeftOffset = HallData[0].HallPosn - HallData[0].HallPosn_lastread;
            ((PROTOCOL_POSN*) (param->ptr))->RightAbsolute = HallData[1].HallPosn;
            ((PROTOCOL_POSN*) (param->ptr))->RightOffset = HallData[1].HallPosn - HallData[1].HallPosn_lastread;
            */
            break;
    }

    fn_defaultProcessing(s, param, cmd, msg);

    switch (cmd) {
        case PROTOCOL_CMD_WRITEVAL:
        case PROTOCOL_CMD_READVALRESPONSE:
        /*
            HallData[0].HallPosn_lastread = ((PROTOCOL_POSN*) (param->ptr))->LeftAbsolute;
            HallData[1].HallPosn_lastread = ((PROTOCOL_POSN*) (param->ptr))->RightAbsolute;
            */
            break;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x0A disablepoweroff

extern uint8_t disablepoweroff;


////////////////////////////////////////////////////////////////////////////////////////////
// Variable & Functions for 0x0B debug_out

extern uint8_t debug_out;



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
//            control_type = CONTROL_TYPE_PWM;
            timeout= 0;
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
extern uint16_t buzzerLen;

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
// Variable & Functions for 0x80 to 0xA0 FlashContent

// from main.c
extern void change_PID_constants();
extern void init_PID_control();

//extern volatile ELECTRICAL_PARAMS electrical_measurements;





////////////////////////////////////////////////////////////////////////////////////////////
// initialize protocol and register functions
int setup_protocol(PROTOCOL_STAT *s) {

    protocol_GetTick = HAL_GetTick;
    protocol_Delay = HAL_Delay;
    protocol_SystemReset =HAL_NVIC_SystemReset;


    int errors = 0;

    #if defined(SERIAL_USART2_IT)
      // initialise, even if CONTROL_SENSOR, as we may switch vuia software to use this....
      extern int USART2_IT_send(unsigned char *data, int len);

      errors += protocol_init(&sUSART2);

      sUSART2.send_serial_data=USART2_IT_send;
      sUSART2.send_serial_data_wait=USART2_IT_send;
      sUSART2.timeout1 = 500;
      sUSART2.timeout2 = 100;
      sUSART2.allow_ascii = 1;

    #endif

    #if defined(SERIAL_USART3_IT) && !defined(CONTROL_SENSOR)

      extern int USART3_IT_send(unsigned char *data, int len);

      errors += protocol_init(&sUSART3);

      sUSART3.send_serial_data=USART3_IT_send;
      sUSART3.send_serial_data_wait=USART3_IT_send;
      sUSART3.timeout1 = 500;
      sUSART3.timeout2 = 100;
      sUSART3.allow_ascii = 1;

    #endif


    // initialise ascii protocol functions
    main_ascii_init(s);


    #ifdef CONTROL_SENSOR
        errors += setParamVariable( s, 0x01, UI_NONE, &sensor_copy, sizeof(sensor_copy) );
        setParamHandler( s, 0x01, fn_SensorData );
    #endif

//        errors += setParamVariable( s, 0x02, UI_NONE, (void *)&HallData, sizeof(HallData) );

        errors += setParamVariable( s, 0x03, UI_NONE, &SpeedData, sizeof(SpeedData) );
        setParamHandler( s, 0x03, fn_SpeedData );

        errors += setParamVariable( s, 0x04, UI_NONE, &Position, sizeof(Position) );
        setParamHandler( s, 0x04, fn_Position );



        errors += setParamVariable( s, 0x06, UI_NONE, &PosnData, sizeof(PosnData) );

        errors += setParamVariable( s, 0x07, UI_NONE, &RawPosition, sizeof(RawPosition) );
        setParamHandler( s, 0x07, fn_RawPosition );

//        errors += setParamVariable( s, 0x08, UI_NONE, (void *)&electrical_measurements, sizeof(ELECTRICAL_PARAMS) );

        errors += setParamVariable( s, 0x09, UI_CHAR, &protocol_enable, sizeof(enable) );
        setParamHandler( s, 0x09, fn_enable );

//        errors += setParamVariable( s, 0x0A, UI_CHAR, &disablepoweroff, sizeof(disablepoweroff) );

        errors += setParamVariable( s, 0x0B, UI_CHAR, &debug_out, sizeof(debug_out) );


        errors += setParamVariable( s, 0x0D, UI_NONE, &PWMData, sizeof(PWMData) );
        setParamHandler( s, 0x0D, fn_PWMData );

        errors += setParamVariable( s, 0x0E, UI_2LONG, &(PWMData.pwm), sizeof(PWMData.pwm) );
        setParamHandler( s, 0x0E, fn_PWMData );

        errors += setParamVariable( s, 0x21, UI_NONE, &BuzzerData, sizeof(BuzzerData) );
        setParamHandler( s, 0x21, fn_BuzzerData );

    return errors;

}


void consoleLog(char *message) {
    #ifdef DEBUG_SOFTWARE_SERIAL
        if (debug_out) protocol_send_text(&sSoftwareSerial, message, PROTOCOL_SOM_NOACK);
    #endif

    #ifdef SERIAL_USART2_IT
        if (debug_out) protocol_send_text(&sUSART2, message, PROTOCOL_SOM_NOACK);
    #endif

    #if defined(SERIAL_USART3_IT) && !defined(CONTROL_SENSOR)
        if (debug_out) protocol_send_text(&sUSART3, message, PROTOCOL_SOM_NOACK);
    #endif
}
