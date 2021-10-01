#include <AccelStepper.h>
#include <pt.h>

/*
  MACROS
*/

#define X_DIR 5 // X stepper, direction pin
#define Y_DIR 6 // Y
#define Z_DIR 7 // Z

#define X_STP 2 // X stepper, step pin
#define Y_STP 3 // Y
#define Z_STP 4 // Z

#define X_SPEED 1000 // X steps per second
#define Y_SPEED 1000 // Y
#define Z_SPEED 1000 // Z

#define X_ACCEL 5000.0 // X steps per second per second
#define Y_ACCEL 5000.0 // Y
#define Z_ACCEL 5000.0 // Z

#define TTLPIN 12 // ttl trigger pin
#define LEDPIN 13 // status led

#define BAUD_RATE 230400 // serial port rate
#define MAXBUFFERSIZE 64 // data read buffer size

/*
  Global Variables 
*/

// stepper motors parameters
typedef struct
{
  AccelStepper* _ptr;
  float _accel;
  float _speed;
  float _delta;
  float _distance;
  bool _running;
  bool _trigger;
  bool _sendUpdate;
} StepperMotorParam;

AccelStepper stepperX(AccelStepper::DRIVER, X_STP, X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STP, Y_DIR);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STP, Z_DIR);

// ptr, accel, speed, delta, position, distance, running, trigger, update
StepperMotorParam stepperArr[3]= {
  {NULL,0,0,0,0,false,false,false},
  {NULL,0,0,0,0,false,false,false},
  {NULL,0,0,0,0,false,false,false}
};

// data read buffer
char inBuffer[MAXBUFFERSIZE];
// flags
bool readInProgress= false;
bool messageToProcess= false;
bool anyMotorRunning= false;

unsigned long myTime= millis();

// =================================================================================
// =================================================================================
// =================================================================================

/*
  Update running flag.
*/
void updateRunningFlag()
{
  anyMotorRunning = stepperArr[0]._running || stepperArr[1]._running || stepperArr[2]._running;
}

/*
  Pause a motor.
*/
void pauseMotor(int idx)
{
  if(stepperArr[idx]._running && stepperArr[idx]._distance!=0)
  { 
      stepperArr[idx]._running=false;
      stepperArr[idx]._ptr->stop();
  }
  updateRunningFlag();
}

/*
  Resume a motor motion.
*/
void resumeMotor(int idx)
{
  if(!stepperArr[idx]._running && stepperArr[idx]._distance!=0)
  {
    stepperArr[idx]._running=true;
    stepperArr[idx]._ptr->move(stepperArr[idx]._distance);
  }
  updateRunningFlag();
}

/*
  Stop a motor.
*/
void stopMotor(int idx)
{
  stepperArr[idx]._ptr->stop();
  stepperArr[idx]._running=false;
  stepperArr[idx]._distance=0;
}

/*
  Send motor Position info.
*/
void sendPosition(int i)
{
  char tmpBuff[40];
  sprintf(tmpBuff, "<P%d,%ld,%ld,%ld>", i+1,stepperArr[i]._ptr->targetPosition(), stepperArr[i]._ptr->currentPosition(),stepperArr[i]._ptr->distanceToGo() );
  Serial.println(tmpBuff); 
}

/*
  Reset the motor zero position.
*/
void resetZero(int idx)
{
  stopMotor(idx);
  // reset the position (also set speed to 0)
  long offset;
  stepperArr[idx]._ptr->setCurrentPosition(offset);
  // set the proper speed
  stepperArr[idx]._ptr->setMaxSpeed(stepperArr[idx]._speed);
  sendPosition(idx);
}

/*
  Update run parameter for one motor.
*/
void updateRun(int idx, char* dir, float distance)
{
  distance = (strcmp(dir, "F")==0)? distance : -distance ;
  stepperArr[idx]._distance= distance;
  stepperArr[idx]._ptr->move(stepperArr[idx]._distance);
  stepperArr[idx]._running= true;

  updateRunningFlag();
}

/*
  Update setting for one motor
*/
void udpateSetting(char* setting, int idx, float value)
{ 
  if(stepperArr[idx]._running) stopMotor(idx);
  if (strcmp(setting, "SPEED") == 0)
  {
    stepperArr[idx]._speed= value;
    stepperArr[idx]._ptr->setMaxSpeed(stepperArr[idx]._speed);
  }
  else if (strcmp(setting, "ACCEL") == 0)
  {
    stepperArr[idx]._accel= value;
    stepperArr[idx]._ptr->setAcceleration(stepperArr[idx]._accel);
  }
  else if (strcmp(setting, "DELTA") == 0) stepperArr[idx]._delta= value;
  else if (strcmp(setting, "TTL") == 0) stepperArr[idx]._trigger= (value>0);

  updateRunningFlag();
}

// =================================================================================
// =================================================================================
// =================================================================================

/*
  Protothread for Serial port read
*/
static struct pt pt1;
static int protothreadReadSerial(struct pt *pt)
{
  static unsigned int index  = 0;
  static bool overflowed=false;
  // Start the protothread
  PT_BEGIN(pt);
  while(1) 
  {
    PT_WAIT_UNTIL(pt,Serial.available());
    // turn on the LED to indicate we are receiving
    digitalWrite(LEDPIN, HIGH);
    // read the a single character
    char inChar = Serial.read();
    
    if (inChar == '<') // message start
    {
      index = 0;
      readInProgress = true;
    }
    else if (inChar == '>') // message end
    {
      // terminate the string
      if(index<MAXBUFFERSIZE) inBuffer[index]='\0';
      else overflowed=true;
      // echo and update flag
      if(!overflowed)
      {
        Serial.print('<');
        Serial.print(inBuffer);
        Serial.println('>');
        messageToProcess=true;
      }
      else
      {
        messageToProcess= false;
        Serial.println("<BUFFEROVERFLOW>");
      }
      readInProgress = false;
      digitalWrite(LEDPIN, LOW);
      overflowed=false;
    }
    else if (readInProgress)
    {
      PT_WAIT_UNTIL(pt,!messageToProcess); // allow message processing
      if(index<MAXBUFFERSIZE)
      {
        inBuffer[index] = inChar;
        index++; 
      }
      else overflowed=true;
    }
  }
  // Stop the protothread
  PT_END(pt);
}

/*
  parse incoming message
*/
void parseMessage()
{
  char *strPtr;
  char *mode, *setting, *dir;
  int motorID;
  float value;
  float p_optional[3]={0,0,0};
  // MODE
  mode = strtok(inBuffer, ",");
  // SETTING
  setting = strtok(NULL, ",");
  // MOTORID
  strPtr = strtok(NULL, ",");
  motorID = atoi(strPtr);
  // VALUE
  strPtr = strtok(NULL, ","); 
  value = atof(strPtr);
  // DIR
  dir = strtok(NULL, ",");
  // param 1
  strPtr = strtok(NULL, ",");
  p_optional[0] = atof(strPtr);
  // param 2
  strPtr = strtok(NULL, ","); 
  p_optional[1] = atof(strPtr);
  // param 3
  strPtr = strtok(NULL, ",");
  p_optional[2] = atof(strPtr); 

  // overwrites parameters if RUN setting
  if (strcmp(setting, "RUN") == 0)
  {
    p_optional[0]= p_optional[1]= p_optional[2]= 999999.0;
  }
 
  // run the corresponding command
  if(strcmp(mode, "STOP") == 0)
  {
    for(int i=0;i<3;i++) stopMotor(i);
  }
  else if ((strcmp(mode, "SETTING") == 0) && (motorID>0 && motorID<4))
  {
    udpateSetting(setting, motorID-1, value);
  }
  else if (strcmp(mode, "RUN") == 0 && (strcmp(setting, "DIST") == 0))
  {
    if(motorID>10) // decompose the motorid
    {
      while(motorID>0)
      {
        int subMotorID = (motorID %10);
        if(subMotorID>0 && subMotorID<4) updateRun(subMotorID-1,dir,p_optional[subMotorID-1]); 
        motorID /= 10;
      }
    }
    else if(motorID>0 && motorID<4)
    {
      updateRun(motorID-1,dir,p_optional[motorID-1]);  
    }
  }
  else if (strcmp(mode, "PAUSE") == 0)
  {
    if(motorID>10)
    {
      while(motorID>0)
      {
        int subMotorID = (motorID %10);
        if(subMotorID>0 && subMotorID<4) pauseMotor(subMotorID-1);
        motorID /= 10;
      }
    }
    else if(motorID>0 && motorID<4)
    {
      pauseMotor(motorID-1);
    }
  }
  else if (strcmp(mode, "RESUME") == 0)
  {
    if(motorID>10)
    {
      while(motorID>0)
      {
        int subMotorID = (motorID %10);
        if(subMotorID>0 && subMotorID<4) resumeMotor(subMotorID-1);
        motorID /= 10;
      }
    }
    else if(motorID>0 && motorID<4) resumeMotor(motorID-1);
  }
  else if (strcmp(mode, "ZERO") == 0)
  {
    if(motorID>10)
    {
      while(motorID>0)
      {
        int subMotorID = (motorID %10);
        if(subMotorID>0 && subMotorID<4) resetZero(subMotorID-1);
        motorID /= 10;
      }
    }
    else if(motorID>0 && motorID<4) resetZero(motorID-1);
  }
}

/*
  ProtoThread for parsing message.
*/
static struct pt pt2;
static int protothreadParseMessage(struct pt *pt)
{
// Start the protothread
  PT_BEGIN(pt);
  while(1) 
  {
    PT_WAIT_UNTIL(pt,messageToProcess);
    parseMessage();
    messageToProcess=false;
  }
  // Stop the protothread
  PT_END(pt);
}

/*
  ProtoThread for moving motors.
*/
static struct pt pt3;
static int protothreadMoveMotors(struct pt *pt)
{
  // Start the protothread
  PT_BEGIN(pt);
  static bool doUpdate=false;
  while(1) 
  {
    PT_WAIT_UNTIL(pt,!readInProgress && !messageToProcess && anyMotorRunning);
    for(short i=0;i<3;i++)
    {
      if(stepperArr[i]._running)
      {
        // check for ttl
        if(stepperArr[i]._trigger && (digitalRead(TTLPIN)!=HIGH)) continue;
        // move one step
        stepperArr[i]._ptr->run();
        // update distance
        stepperArr[i]._distance = stepperArr[i]._ptr->distanceToGo();
        // Check if final position reached
        if (stepperArr[i]._distance == 0) 
        {
          stepperArr[i]._running=false;
          doUpdate=true;//force update
        }
        if(doUpdate) stepperArr[i]._sendUpdate=true;
      }
    }
    // send position updates
    if(doUpdate)
    {
      for(short i=0;i<3;i++)
      {
        if(stepperArr[i]._sendUpdate)
        {
          sendPosition(i);
          stepperArr[i]._sendUpdate=false;
        } 
      }
      myTime=millis();
    }
    doUpdate = (millis()-myTime > 1000);
    // update flags
    updateRunningFlag();
    PT_WAIT_UNTIL(pt,Serial.available()==0); // allow read if any
  }
  // Stop the protothread
  PT_END(pt);
}

/*
  Initial setup
*/
void setup()
{
  // init serial
  Serial.begin(BAUD_RATE);
  // TTL input
  pinMode(TTLPIN, INPUT);
  // LED output  
  pinMode(LEDPIN, OUTPUT);
  
  // init steppers
  stepperArr[0]._ptr= &stepperX;
  stepperArr[1]._ptr= &stepperY;
  stepperArr[2]._ptr= &stepperZ;
  stepperArr[0]._speed= X_SPEED;
  stepperArr[1]._speed= Y_SPEED;
  stepperArr[2]._speed= Z_SPEED;
  stepperArr[0]._accel= X_ACCEL;
  stepperArr[1]._accel= Y_ACCEL;
  stepperArr[2]._accel= Z_ACCEL;
  
  // update motors
  for(short idx=0; idx<3; idx++)
  {
    stepperArr[idx]._ptr->setMaxSpeed(stepperArr[idx]._speed);
    stepperArr[idx]._ptr->setAcceleration(stepperArr[idx]._accel);
  }
  
  // init protothreads
  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);
  
  // tell the PC we are ready
  Serial.println("<Arduino is ready>");
}

/*
  Main loop
*/
void loop()
{
  // execute protothreads
  protothreadReadSerial(&pt1);
  protothreadParseMessage(&pt2);
  protothreadMoveMotors(&pt3);
}
/* eof */
