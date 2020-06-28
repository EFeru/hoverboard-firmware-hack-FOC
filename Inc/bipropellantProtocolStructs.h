/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
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

////////////////////////////////////////////////////////////////////
// structures used in main repo which may mirror protocol structures
////////////////////////////////////////////////////////////////////

// Define to prevent recursive inclusion
#ifndef BIPROPELLANTPROTOCOLSTRUCTS_H
#define BIPROPELLANTPROTOCOLSTRUCTS_H

#include "protocol.h"

//// control structures used in firmware

// used in main
extern PROTOCOL_SPEED_DATA SpeedData;

extern PROTOCOL_PWM_DATA PWMData;


#pragma pack(push, 4)  // int32_t and float are 4 byte each
typedef struct tag_HALL_DATA_STRUCT{
    int32_t HallPosn; // 90 per revolution
    int32_t HallSpeed; // speed part calibrated to speed demand value

    float HallPosnMultiplier; // m per hall segment

    int32_t HallPosn_lastread; // posn offset set via protocol in raw value
    int32_t HallPosn_mm; // posn in mm
    int32_t HallPosn_mm_lastread; // posn offset set via protocol in mm
    int32_t HallSpeed_mm_per_s; // speed in m/s

    uint32_t HallTimeDiff;
    uint32_t HallSkipped;
} HALL_DATA_STRUCT;
#pragma pack(pop)

#pragma pack(push, 4) // all used types (float and int) are 4 bytes

typedef struct tag_MOTOR_ELECTRICAL{
        float dcAmps;
        float dcAmpsAvgAcc;
        float dcAmpsAvg;
        int r1;
        int r2;
        int q;

        int dcAmpsx100;

        int pwm_limiter;
        int pwm_requested;
        int pwm_actual;

        unsigned int limiter_count;
} MOTOR_ELECTRICAL;
#pragma pack(pop)

#pragma pack(push, 4) // all used types (float and int) are 4 bytes
typedef struct tag_ELECTRICAL_PARAMS{
    int bat_raw;
    float batteryVoltage;

    int board_temp_raw;
    float board_temp_filtered;
    float board_temp_deg_c;

    int charging;

    int dcCurLim; // amps*100
    int dc_adc_limit; // limit expressed in terms of ADC units.

    MOTOR_ELECTRICAL motors[2];

} ELECTRICAL_PARAMS;
#pragma pack(pop)

#endif