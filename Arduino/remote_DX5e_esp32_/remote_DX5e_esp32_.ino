// ########################## DEFINES ##########################
// #define DEBUG_RX 1           // Debug data. Prints to serial (comment-out to disable)
#define NUM_PINS 4              // Add new PPM Pins as idx to the Channel enum and to the PpmPins-Array as well!!
#define PPM_PIN_STEER 12
#define PPM_PIN_SPEED 14
#define PPM_PIN_ALIVE 27
#define PPM_PIN_KILLSW 26
#define PPM_REV_SIGNAL_FACTOR -1 // [1,-1] -1 = Invert Signal  
// #define NEW_PIN XX

// Adjustable Expo Factor (0.0 = Maximum non-linear response, 1.0 = Linear response)
#define FILTER_FACTOR_NOMRALIZED        0.4
/*
  Filter    Reaction      Smoothing
  0.1       Very slow     Very strongly smoothed
  0.3       Slow          Strongly smoothed
  0.5       Balanced      Moderate smoothing
  0.7       Fast          Slightly smoothed
  0.9       Very fast     Barely smoothed
  1.0       Direct        Not smoothed
*/
#define FILTER_DISABLED               0                                   // No filter input = output
#define FILTER_NON_LINEAR_EXPO_CURVE  1                                   // Non-Linear Exponential Curve
#define FILTER_EXPO_MOVING_AVG        2                                   // Exponential Moving Average
#define FILTER                        (FILTER_EXPO_MOVING_AVG)      // Selected Filter

#define NORMALIZE_FACTOR              1000
#define EXPO_FACTOR                   (1 - (FILTER_FACTOR_NOMRALIZED))
#define EMA_ALPHA                     (FILTER_FACTOR_NOMRALIZED * NORMALIZE_FACTOR) // EMA Alpha factor for Int calculation


#define PPM_MIN_PULSE_WIDTH      800    // Min Puls in us
#define PPM_MAX_PULSE_WIDTH     2000    // Max Puls in us
#define PPM_FRAME_INTERVAL     20000    // Frames (every 20ms)
#define PPM_FAILSAFE_VALUE       945    // Channel Throttle keep Trim middle | min value 931
#define PPM_KILL_SW             1550    // Channel 5 Switch // On ~1150us | Off ~1850us
#define PPM_RECEIVER_TIMEOUT      70    // Last Puls Alive Timeout in ms

#define UPPER_DEADBAND_MAX      1860
#define ZERO_DEADBAND_MAX       1510
#define ZERO_DEADBAND_MIN       1490
#define LOWER_DEADBAND_MIN      1150

#define HOVER_SPEED_MAX 1000
#define HOVER_SPEED_MIN -1000

// Hover Serial
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for Serial2 (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step

#define RXD2 16
#define TXD2 17

// PPM
enum CHANNEL_IDX
{
  STEER_CH = 0,
  SPEED_CH = 1,
  ALIVE_CH = 2,
  KILL_SW_CH = 3,
// NEW_PPM_PIN_CH = xx,
  NUM_OF_CHANELS
};

typedef struct{
  int                    pin = 0;
  volatile unsigned long pulseStartTime = 0;
  volatile unsigned long pulseEndTime = 0;
  volatile unsigned long ppmDuration = 0;
  volatile unsigned long lastPuls = 0;
  volatile bool          ppmSignalReceived = false;
} PpmData;

// Global variables
const int PpmPins[NUM_PINS] = {PPM_PIN_STEER, PPM_PIN_SPEED, PPM_PIN_ALIVE, PPM_PIN_KILLSW}; // Add new PPM pins to this array
const unsigned int PpmIntervallMs = PPM_FRAME_INTERVAL / 1000;
unsigned int errCode = 0;

PpmData ppmData[NUM_OF_CHANELS];

uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

const long long NormalizeCubic = NORMALIZE_FACTOR * NORMALIZE_FACTOR * NORMALIZE_FACTOR;

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

/*
*  *********** FUNCTIONS ***********
*/
void ISR_Handler(CHANNEL_IDX channel)
{
  unsigned long currentTime = micros();
  int idx = (int)channel;

   ppmData[idx].ppmSignalReceived = false;

  if (digitalRead(ppmData[idx].pin) == HIGH)
  {
    ppmData[idx].pulseStartTime = currentTime;
  }
  else
  { // Signal is LOW
    unsigned long pulseWidth = currentTime - ppmData[idx].pulseStartTime;
    // check pulse width and save as channel value
    if (pulseWidth >= PPM_MIN_PULSE_WIDTH && pulseWidth <= PPM_MAX_PULSE_WIDTH)
    {
      ppmData[idx].ppmDuration = pulseWidth;
      ppmData[idx].ppmSignalReceived = true;
      ppmData[idx].lastPuls = millis(); // last vaild puls
    }
  }
}
void IRAM_ATTR ppmIsrSteer()
{
  ISR_Handler(STEER_CH);
}

void IRAM_ATTR ppmIsrSpeed()
{
  ISR_Handler(SPEED_CH);
}

void IRAM_ATTR ppmIsrAlive()
{
  ISR_Handler(ALIVE_CH);
}

void IRAM_ATTR ppmIsrKillSw()
{
  ISR_Handler(KILL_SW_CH);
}

int ApplyFilter(int input)
{
  #if FILTER == 0
  return input;
  #elif FILTER == 1
  return ExpoFilter(input); // Non-Linear Exponential Curve
  #elif FILTER == 2 // Exponential Moving Average
  return EmaFilter(input);
  #endif  
}

int ExpoFilter(int input)
{
  float inputNorm = (float)input / 1000.0f; // [-1.0, 1.0]
  float output = 0.0f;
  // Calculate the cubic (input^3) part
  float inputCubed = input * input * input; // Ensure no overflow!
  // Apply the integer expo curve formula
  output = (EXPO_FACTOR * inputCubed) + ((1 - EXPO_FACTOR) * inputNorm);

  return (int)(output * 1000 + 0.5f);
}

int EmaFilter(int input)
{
  static int prevValue = 0;
  // EMA calculation
  int output = (EMA_ALPHA * input) + ((1000 - EMA_ALPHA) * prevValue);
  prevValue = (int)(output/1000);

  return prevValue;
}

bool isReceiverAlive()
{
  if ((millis() - ppmData[STEER_CH].lastPuls) >= PPM_RECEIVER_TIMEOUT)
  {
    errCode = 1;
    return false;
  }
  if ((millis() - ppmData[SPEED_CH].lastPuls) >= PPM_RECEIVER_TIMEOUT)
  {
    errCode = 2;
    return false;
  }
  /*
  if ((millis() - ppmData[ALIVE_CH].lastPuls) >= PPM_RECEIVER_TIMEOUT)
  {
    errCode = 3;
    return false;
  }
  if ((millis() - ppmData[KILL_SW_CH].lastPuls) >= PPM_RECEIVER_TIMEOUT)
  {
    errCode = 4;
    return false;
  }
  */
  if (ppmData[ALIVE_CH].ppmDuration <= PPM_FAILSAFE_VALUE)
  {
    errCode = 5;
    return false;
  }

  return true;
}

int GetHoverOutput(int channelIdx)
{
  int ppmValue = ppmData[channelIdx].ppmDuration;

  if(ppmValue >= UPPER_DEADBAND_MAX)
  {
    return HOVER_SPEED_MAX;
  }
  if(ppmValue <= LOWER_DEADBAND_MIN)
  {
    return HOVER_SPEED_MIN;
  }
  if(ppmValue >= ZERO_DEADBAND_MIN && ppmValue <= ZERO_DEADBAND_MAX)
  {
    return 0;
  }
  if(ppmValue > ZERO_DEADBAND_MAX)
  {
    return map(ppmValue, ZERO_DEADBAND_MAX, UPPER_DEADBAND_MAX, 1, HOVER_SPEED_MAX);
  }
  if(ppmValue < ZERO_DEADBAND_MIN)
  {
    return map(ppmValue, LOWER_DEADBAND_MIN, ZERO_DEADBAND_MIN, HOVER_SPEED_MIN, -1);
  }

  return 0;
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (Serial2.available()) {
        incomingByte 	  = Serial2.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
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
/*
*  *********** FUNCTIONs END ***********
*/

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD); // Initialise the serial port & wait for the port to open.
  while (!Serial)
  {
      ;
  }

  // PPM-Pins
  pinMode(PPM_PIN_STEER, INPUT_PULLDOWN);
  pinMode(PPM_PIN_SPEED, INPUT_PULLDOWN);
  pinMode(PPM_PIN_ALIVE, INPUT_PULLDOWN);
  pinMode(PPM_PIN_KILLSW, INPUT_PULLDOWN);
  
  // Init PPM-Data
  for (int i=0; i < NUM_PINS; i++)
  {
    ppmData[i].pin = PpmPins[i];
    ppmData[i].lastPuls = millis(); // set the first time to avoid timeout issues at the begining
  }

  // Interrupt PPM-Signal
  attachInterrupt(digitalPinToInterrupt(PPM_PIN_STEER), ppmIsrSteer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN_SPEED), ppmIsrSpeed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN_ALIVE), ppmIsrAlive, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN_KILLSW), ppmIsrKillSw, CHANGE);
  
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2); // Serial2 is HoverSerial

#ifdef DEBUG
  Serial.println("Hoverboard Serial v1.0");
  Serial.println("Start PPM Decoder!");
#endif
}

// ########################## LOOP ##########################
void loop(void)
{
  static unsigned long lastTime = 0;
  // Check for new received data
  Receive();

  // Send commands if Receiver is alive and not stdby
  if(!isReceiverAlive())
  {
    /*
      STOP BOARD
    */
    Send(0,0);
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.print("Receiver is DEAD!");
    Serial.print(" [Error: 0x");
    Serial.print(errCode);
    Serial.print("]"); 
    errCode = 0;
    Serial.println();
    Serial.println();
    delay(5);
    
    return;  
  }

  if (ppmData[KILL_SW_CH].ppmDuration >= PPM_KILL_SW)
  {
    /*
      STOP BOARD
    */
    Send(0,0);
    Serial.println();
    Serial.print("Receiver is in STANDBY!  [v:");
    Serial.print(ppmData[KILL_SW_CH].ppmDuration);
    Serial.print("]");
    Serial.println();
    Serial.println();
    delay(50);
    return;
  }
  
  if (millis() - lastTime > PpmIntervallMs)
  {
    lastTime = millis();
    
    int speed = PPM_REV_SIGNAL_FACTOR * ApplyFilter(GetHoverOutput(SPEED_CH));
    int steer = PPM_REV_SIGNAL_FACTOR * ApplyFilter(GetHoverOutput(STEER_CH));

    Send(steer, speed); // Send new Steer and Speed values to the Hoverboard

    /*
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(steer);
    Serial.println();
    */

#ifdef DEBUG_RX    
    Serial.print(ppmData[SPEED_CH].ppmDuration);
    Serial.print(" ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(ppmData[STEER_CH].ppmDuration);
    Serial.print("  ");
    Serial.print(steer);
    Serial.print(" A ");
    Serial.print(ppmData[ALIVE_CH].ppmDuration);
    Serial.print(" K ");
    Serial.print(ppmData[KILL_SW_CH].ppmDuration);

    Serial.println();
#endif
  }

  delay(1);
}

// ########################## END ##########################
