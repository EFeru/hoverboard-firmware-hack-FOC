#pragma once

#include "protocol.h"

int setup_protocol(PROTOCOL_STAT *s);


extern PROTOCOL_STAT sSoftwareSerial;
extern PROTOCOL_STAT sUSART2;
extern PROTOCOL_STAT sUSART3;

void consoleLog(char *message);