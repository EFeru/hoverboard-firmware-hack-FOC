// *******************************************************************
//  Arduino Nano 3.3V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Left Sensor cable (long wired cable)
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// • Option 2: Serial on Right Sensor cable (short wired cable) - recommended, so the ADCs on the other cable are still available
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   9600       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
//SoftwareSerial HoverSerial(2,3);        // RX, TX
SoftwareSerial HoverSerial(9,8); // eHposer Arduino Mini

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback oHoverFeedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setupHover() 
{
  HoverSerial.begin(HOVER_SERIAL_BAUD);
}

// ########################## SEND ##########################
void HoverSend(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
bool HoverReceive()
{
  // Check for new data availability in the Serial buffer
  if (!HoverSerial.available()) return false;
  
  incomingByte    = HoverSerial.read();                                 // Read the incoming byte
  bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;   // Construct the start frame    

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print(incomingByte);
    return false;
  #endif      
  
  // Copy received data
  if (bufStartFrame == START_FRAME) 
  {                     // Initialize if new data is detected
    p     = (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
    *p++  = incomingByte;
    idx   = 2;  
  } 
  else if (idx >= 2 && idx < sizeof(SerialFeedback)) 
  {  // Save the new received data
    *p++  = incomingByte; 
    idx++;
  } 

  bool bRet = false;
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) 
  {    
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
  
    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) 
    {
      bRet = true;
      // Copy the new data
      memcpy(&oHoverFeedback, &NewFeedback, sizeof(SerialFeedback));

#ifdef DEBUG_HOVER
      // Print data to built-in Serial
      Serial.print("1: ");   Serial.print(oHoverFeedback.cmd1);
      Serial.print("\t2: ");  Serial.print(oHoverFeedback.cmd2);
      Serial.print("\t3: ");  Serial.print(oHoverFeedback.speedR_meas);
      Serial.print("\t4: ");  Serial.print(oHoverFeedback.speedL_meas);
      Serial.print("\t5: ");  Serial.print(oHoverFeedback.batVoltage);
      Serial.print("\t6: ");  Serial.print(oHoverFeedback.boardTemp);
      Serial.print("\t7: ");  Serial.println(oHoverFeedback.cmdLed);
#endif
    }
    else
    {
#ifdef DEBUG_HOVER
      Serial.println("Non-valid data skipped");
#endif
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  
  // Update previous states
  incomingBytePrev  = incomingByte;
  return bRet;
}
