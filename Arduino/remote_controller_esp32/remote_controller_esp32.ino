/**
 * This Software is based on the https://github.com/ZZ-Cat/CRSFforArduino Repo
 * 
 * @brief Example of how to read rc channels from a receiver.
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * 
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU Affero General Public License v3.0
 * This example is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#include "CRSFforArduino.hpp"
#define USE_SERIAL_PLOTTER 0

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

//#include <SoftwareSerial.h>
//SoftwareSerial HoverSerial(34,35);        // RX, TX
#define RXD2 16
#define TXD2 17

#define RXD1 12
#define TXD1 13



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
SerialFeedback Feedback;
SerialFeedback NewFeedback;

CRSFforArduino *crsf = nullptr;

int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
const char *rcChannelNames[] = {
    "A",
    "E",
    "T",
    "R",
    "Aux1",
    "Aux2",
    "Aux3",
    "Aux4",

    "Aux5",
    "Aux6",
    "Aux7",
    "Aux8",
    "Aux9",
    "Aux10",
    "Aux11",
    "Aux12"};

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void setup()
{
    // Initialise the serial port & wait for the port to open.
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println("Start");

    // Initialise CRSF for Arduino.
    crsf = new CRSFforArduino(&Serial2, 16, 17);

    Serial.println("CRSF new");
    
    if (!crsf->begin())
    {
        crsf->end();

        delete crsf;
        crsf = nullptr;

        Serial.println("CRSF for Arduino initialisation failed!");
        while (1)
        {
            delay(10);
        }
    }
    else
    {
      Serial.println("CRSF for Arduino initialised!");
    }

    rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;

    crsf->setRcChannelsCallback(onReceiveRcChannels);

      Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD1, TXD1);

      Serial.println("Serial1.begin");

    // Show the user that the sketch is ready.
    Serial.println("RC Channels Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    crsf->update();
}

int rcToHoverboard(int value) {
  return (value - 1500) * 2;
}

void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial1.write((uint8_t *) &Command, sizeof(Command)); 
}

void Receive()
{
    // Check for new data availability in the Serial buffer
    if (Serial1.available()) {
        incomingByte 	  = Serial1.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.println(incomingByte, HEX);
        Serial.print("start: ");
        Serial.println(bufStartFrame, HEX);
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    Serial.print("idx: ");
    Serial.println(idx);
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{   
    int speed = 0;
    int steer = 0;

    if (rcChannels->failsafe == false)
    {

        speed = -rcToHoverboard(crsf->rcToUs(crsf->getChannel(2)));
        steer = rcToHoverboard(crsf->rcToUs(crsf->getChannel(4)));



        /* Print RC channels every 100 ms. */
        unsigned long thisTime = millis();
        static unsigned long lastTime = millis();

        /* Compensate for millis() overflow. */
        if (thisTime < lastTime)
        {
            lastTime = thisTime;
        }

        if (thisTime - lastTime >= 100)
        {
            lastTime = thisTime;
#if USE_SERIAL_PLOTTER > 0
            for (int i = 1; i <= rcChannelCount; i++)
            {
                Serial.print(i);
                Serial.print(":");
                Serial.print(crsf->rcToUs(crsf->getChannel(i)));
                Serial.print("\t");
            }
            Serial.println();
#else
            Serial.print("RC Channels <");
            for (int i = 1; i <= rcChannelCount; i++)
            {
                Serial.print(rcChannelNames[i - 1]);
                Serial.print(": ");
                Serial.print(crsf->rcToUs(crsf->getChannel(i)));

                if (i < rcChannelCount)
                {
                    Serial.print(", ");
                }
            }
            Serial.println(">");
#endif
        }
    }

  Send(steer, speed);

}
