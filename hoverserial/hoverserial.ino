#define DEBUG_HOVER

//#include "hoverFeru.h"  // uncomment if SERIAL_ROBO is not active in config.h or you have still the original Feru firmware on the controller :-)
#include "hoverRobo.h"

#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing

void setup() 
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  setupHover();
}

unsigned long iTimeSend = 0;
int iTestMax = SPEED_MAX_TEST;
int iTest = 0;

void loop() 
{
  unsigned long timeNow = millis();

  // Check for new received data
  HoverReceive();

  // Send commands
  if (iTimeSend > timeNow) 
    return;
    
  iTimeSend = timeNow + TIME_SEND;
  HoverSend(0, SPEED_MAX_TEST - 2*abs(iTest));
  //Send(0, (timeNow % 10000 > 2000) ? SPEED_MAX_TEST : 0);

  // Calculate test command signal
  iTest += 10;
  if (iTest > iTestMax) iTest = -iTestMax;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}
