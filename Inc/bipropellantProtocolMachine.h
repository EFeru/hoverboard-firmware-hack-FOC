// Define to prevent recursive inclusion
#ifndef BIPROPELLANTPROTOCOLMACHINE_H
#define BIPROPELLANTPROTOCOLMACHINE_H

#include "protocol.h"

int setup_protocol(PROTOCOL_STAT *s);

extern PROTOCOL_STAT sUSART2;
extern PROTOCOL_STAT sUSART3;

#endif