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

#define LEDPIN 13 // status led

#define BAUD_RATE 230400 // serial port rate
#define MAXBUFFERSIZE 64 // data read buffer size
#define MAXQUEUESIZE 5 // message queue size

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
  float _position;
  float _distance;
  bool _running;
  bool _trigger;
} StepperMotorParam;

/*
  Print stepper motor values.
*/
void serialPrintStepperMotor(StepperMotorParam *p)
{
  Serial.print("accel: ");
  Serial.print(p->_accel);
  Serial.print(", speed: ");
  Serial.print(p->_speed);
  Serial.print(", delta: ");
  Serial.print(p->_delta);
  Serial.print(", position: ");
  Serial.print(p->_position);
  Serial.print(", distance: ");
  Serial.print(p->_distance);
  Serial.print(", running: ");
  Serial.print(p->_running);
  Serial.print(", trigger: ");
  Serial.println(p->_trigger);
}

AccelStepper stepperX(AccelStepper::DRIVER, X_STP, X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STP, Y_DIR);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STP, Z_DIR);

// ptr, accel, speed, delta, position, distance, running, trigger
StepperMotorParam stepperArr[3]= {
  {NULL,0,0,0,0,0,false,false},
  {NULL,0,0,0,0,0,false,false},
  {NULL,0,0,0,0,0,false,false}
};

// data read buffer
char inBuffer[MAXBUFFERSIZE];
// new message available
bool readInProgress=false;
bool messageToProcess = false;
bool anyMotorRunning=false;

/*
  Checks if a number contains a specific digit.
 */
bool containsDigit(int num, int d)
{
    while(num > 0)
    {   
      // digit matched
      if(num % 10 == d) return true;
      // move to next digit
      num = num / 10;
    }
    // not found
    return false;
}

/*
  Print stepper motor parameters.
*/
void printStepperMotor(int motorID)
{
  if(motorID>3 || motorID<1) Serial.println("Unknown motor ID");
  else
  {
    Serial.print("Motor");
    Serial.println(motorID);
    serialPrintStepperMotor(&stepperArr[motorID-1]);
  }
}

/*
  Pause a motor.
*/
void pauseMotor(int motorID)
{
  if(motorID>3 || motorID<1) Serial.println("Unknown motor ID");
  else if(stepperArr[motorID-1]._running && stepperArr[motorID-1]._distance>0)
  { 
      stepperArr[motorID-1]._running=false;
  }
  // update running flag
  anyMotorRunning = stepperArr[0]._running || stepperArr[1]._running || stepperArr[2]._running;
}

/*
  Resume a motor motion.
*/
void resumeMotor(int motorID)
{
  if(motorID>3 || motorID<1) Serial.println("Unknown motor ID");
  else if(!stepperArr[motorID-1]._running && stepperArr[motorID-1]._distance>0)
  {
    stepperArr[motorID-1]._running=true;
  }
  // update running flag
  anyMotorRunning = stepperArr[0]._running || stepperArr[1]._running || stepperArr[2]._running;
}

/*
  Update motor speed.
*/
void updateMotorSpeed(int motorID)
{
  if(motorID>3 || motorID<1) Serial.println("Unknown motor ID");
  else
  {
    stepperArr[motorID-1]._ptr->setMaxSpeed(stepperArr[motorID-1]._speed);
  }
}

/*
  Update motor acceleration.   
*/
void updateMotorAccel(int motorID)
{
  if(motorID>3 || motorID<1) Serial.println("Unknown motor ID");
  else
  {
    stepperArr[motorID-1]._ptr->setAcceleration(stepperArr[motorID-1]._accel);
  }
}

/*
  stop all steppers.
*/
void stopAllSteppers()
{
  for(short i=0;i<3;i++)
  {
    stepperArr[i]._ptr->stop();
    stepperArr[i]._running=false;
  }
}

/*
  Update run parameter for one motor.
*/
void updateRun(int motorID, char* dir, float distance)
{
  distance = (strcmp(dir, "F")==0)? distance : -distance ;
  
  if(motorID>3 || motorID<1) Serial.println("Unknown motor ID");
  else
  {
    Serial.print("Sending tomove command: ");
    Serial.println(stepperArr[motorID-1]._distance);
    stepperArr[motorID-1]._distance= distance;
    stepperArr[motorID-1]._ptr->move(stepperArr[motorID-1]._distance);
    stepperArr[motorID-1]._running= true;
  } 
  // update running flag
  anyMotorRunning = stepperArr[0]._running || stepperArr[1]._running || stepperArr[2]._running;
}

/*
  Update setting for one motor
*/
void udpateSetting(char* setting, int motorID, float value)
{
  if(motorID>3 || motorID<1) Serial.println("Unknown motor ID");
  else
  { 
      // reset position
      stepperArr[motorID-1]._position=0;
      if (strcmp(setting, "SPEED") == 0)
      {
        stepperArr[motorID-1]._speed= value;
        stepperArr[motorID-1]._ptr->setMaxSpeed(stepperArr[motorID-1]._speed);
      }
      else if (strcmp(setting, "ACCEL") == 0)
      {
        stepperArr[motorID-1]._accel= value;
        stepperArr[motorID-1]._ptr->setAcceleration(stepperArr[motorID-1]._accel);
      }
      else if (strcmp(setting, "DELTA") == 0) stepperArr[motorID-1]._delta= value;
      else if(strcmp(setting, "TTL") == 0) stepperArr[motorID-1]._trigger= (value>0);
  }
}

/*
 reply to PC
*/
void replyToPC(char* mode, char* setting, int motorID, float value, char* dir, float p1_optional, float p2_optional, float p3_optional)
{
    Serial.print("<mode: ");
    Serial.print(mode);
    Serial.print(", setting: ");
    Serial.print(setting);
    Serial.print(", motorID: ");
    Serial.print(motorID);
    Serial.print(", value: ");
    Serial.print(value);
    Serial.print(", direction: ");
    Serial.print(dir);
    Serial.print(", p1 optional: ");
    Serial.print(p1_optional);
    Serial.print(", p2 optional: ");
    Serial.print(p2_optional);
    Serial.print(", p3 optional: ");
    Serial.print(p3_optional);
    Serial.print(", Time: ");
    Serial.print(millis()/1000); // in seconds
    Serial.println(">");
}

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
      readInProgress = false;
      // terminate the string
      if(index<MAXBUFFERSIZE) inBuffer[index]='\0';
      else overflowed=true;
      
      if(!overflowed)
      {
        messageToProcess=true;
      }
      digitalWrite(LEDPIN, LOW);
      overflowed=false;
    }
    else if (readInProgress)
    {
      PT_WAIT_UNTIL(pt,!messageToProcess);
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

  // print the motorID and status
//  Serial.println("string");
//  Serial.println(motorID);
//  Serial.println("Status");
//  Serial.println(containsDigit(motorID,1)?1:0);
//  Serial.println(containsDigit(motorID,2)?1:0);
//  Serial.println(containsDigit(motorID,3)?1:0);
//  Serial.println("Status Distances");
//  for(short i=0;i<3;i++) Serial.println(stepperArr[i]._distance);
  
  // reply
  replyToPC(mode, setting, motorID, value, dir, p_optional[0], p_optional[1], p_optional[2]);

  // run the corresponding command
  if(strcmp(mode, "STOP") == 0)
  {
    stopAllSteppers();
  }
  else if (strcmp(mode, "SETTING") == 0)
  {
    udpateSetting(setting, motorID, value);
  }
  else if (strcmp(mode, "RUN") == 0 && (strcmp(setting, "DIST") == 0))
  {
    if(motorID>10)
    {
      while(motorID>0)
      {
        int subMotorID = (motorID %10);
        updateRun(subMotorID,dir,p_optional[subMotorID-1]); 
        motorID /= 10;
      }
    }
    else
    {
      updateRun(motorID,dir,p_optional[motorID-1]);  
    }
  }
  else if (strcmp(mode, "PAUSE") == 0)
  {
    if(motorID>10)
    {
      while(motorID>0)
      {
        int subMotorID = (motorID %10);
        pauseMotor(subMotorID);
        motorID /= 10;
      }
    }
    else
    {
      pauseMotor(motorID);
    }
  }
  else if (strcmp(mode, "RESUME") == 0)
  {
    if(motorID>10)
    {
      while(motorID>0)
      {
        int subMotorID = (motorID %10);
        resumeMotor(subMotorID);
        motorID /= 10;
      }
    }
    else
    {
      resumeMotor(motorID);
    }
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
  while(1) 
  {
    PT_WAIT_UNTIL(pt,!readInProgress && anyMotorRunning);
    for(short i=0;i<3;i++)
    {
      if(stepperArr[i]._running)
      {
        stepperArr[i]._ptr->run();
        // update distance
        stepperArr[i]._distance = stepperArr[i]._ptr->distanceToGo();
        // Check if final position reached
        if (stepperArr[i]._distance == 0) stepperArr[i]._running=false;
      }
      anyMotorRunning = stepperArr[0]._running || stepperArr[1]._running || stepperArr[2]._running;
    }
  }
  // Stop the protothread
  PT_END(pt);
}

/*
  Initial setup
*/
void setup()
{
  Serial.begin(BAUD_RATE);
  // flash LED
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  delay(500);
  digitalWrite(LEDPIN, LOW);
  delay(500);
  // init steppers
  
  stepperArr[0]._ptr = &stepperX;
  stepperArr[1]._ptr = &stepperY;
  stepperArr[2]._ptr = &stepperZ;
  stepperArr[0]._speed= X_SPEED;
  stepperArr[1]._speed= Y_SPEED;
  stepperArr[2]._speed= Z_SPEED;
  stepperArr[0]._accel= X_ACCEL;
  stepperArr[1]._accel= Y_ACCEL;
  stepperArr[2]._accel= Z_ACCEL;
  // update motors
  for(short i=1; i<4; i++)
  {
    updateMotorSpeed(i);
    updateMotorAccel(i);
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
//eof
