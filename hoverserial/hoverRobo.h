//#define DEBUG_HOVER_RX

#define HOVER_SERIAL_BAUD   9600       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)

#include <SoftwareSerial.h>
//SoftwareSerial HoverSerial(2,3);        // RX, TX
SoftwareSerial oHoverSerial(9,8); // eHposer Arduino Mini


typedef struct{
   int16_t steer;
   int16_t speed;
   uint32_t crc;
} Serialcommand;
Serialcommand oCmd;

typedef struct{
   int16_t iSpeedL; // 100* km/h
   int16_t iSpeedR; // 100* km/h
   uint16_t iHallSkippedL;
   uint16_t iHallSkippedR;
   uint16_t iTemp;  // °C
   uint16_t iVolt;  // 100* V
   int16_t iAmpL;  // 100* A
   int16_t iAmpR;  // 100* A
   uint32_t crc;
} SerialFeedback;
SerialFeedback oFeedback;

uint32_t crc32_for_byte(uint32_t r) 
{
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

void setupHover() 
{
  oHoverSerial.begin(HOVER_SERIAL_BAUD);
}


void HoverSend(int16_t iSteer,int16_t iSpeed)
{
/*  #ifdef DEBUG_HOVER
  Serial.print("Hover.Send() iSpeed = ");Serial.println(iSpeed);
  #endif
  */
  
  oCmd.steer = iSteer;
  oCmd.speed = iSpeed;

  uint32_t crc = 0;
  crc32((const void *)&oCmd, sizeof(Serialcommand)-4,   &crc);
  oCmd.crc = crc;
  
  oHoverSerial.write((uint8_t *) &oCmd, sizeof(oCmd)); 
}

int iFailedRec = 0;
boolean HoverReceive()
{
  //while (oHoverSerial.available()) {Serial.print(" ");Serial.print(oHoverSerial.read(),HEX);}return false;

  if (oHoverSerial.available()<  sizeof(SerialFeedback))
    return false;

  SerialFeedback oNew;
  byte* p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
    *p++ = oHoverSerial.read();;

  uint32_t crc = 0;
  crc32((const void *)&oNew, sizeof(SerialFeedback)-4,   &crc);

#ifdef DEBUG_HOVER_RX
  char sBuff[10];
  p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
  {
    sprintf(sBuff," %02x",p[i]);
    Serial.print(sBuff);
  }
  Serial.print(" ?= ");Serial.println(crc,HEX);
#endif

  if (oNew.crc == crc)
  {
    memcpy(&oFeedback,&oNew,sizeof(SerialFeedback));
    #ifdef DEBUG_HOVER
      if (iFailedRec) Serial.println();
      Serial.print("speedL: ");Serial.print(oNew.iSpeedL);
      Serial.print("\tspeedR: ");Serial.print(oNew.iSpeedR);
      Serial.print("\tskippedL: ");Serial.print(oNew.iHallSkippedL);
      Serial.print("\tskippedR: ");Serial.print(oNew.iHallSkippedR);
      Serial.print("\t°C: ");Serial.print(oNew.iTemp);
      Serial.print("\tU: ");Serial.print(0.01 * (float)oNew.iVolt);
      Serial.print("\tlA: ");Serial.print(0.1 * (float)oNew.iAmpL);
      Serial.print("\trA: ");Serial.println(0.1 * (float)oNew.iAmpR);
      iFailedRec = 0;
    #endif

    return true;    
  }

#ifdef DEBUG_HOVER_RX
  Serial.print("X");
  while (oHoverSerial.available()) {Serial.print(" ");Serial.print(oHoverSerial.read(),HEX);}Serial.println("\t:-(");
#else
  while (oHoverSerial.available()) oHoverSerial.read();   // empty garbage
  #ifdef DEBUG_HOVER
    Serial.print("X");
    iFailedRec++;
  #endif
#endif
  return false;
}
