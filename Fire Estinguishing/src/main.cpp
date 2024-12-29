#include <Chassis.h>
#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

int flameSignal = 0;
const int FLAME_PIN = A11;  // connect flame sensor to pin A11
const int FAN_PIN = 20;     // fan module pin INB is connected to Romi board pin 20
const int FAN_SPEED = 255;  // fan speed can be between 0 to 255


// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// Declare the chassis object (with default values)
Chassis chassis(7.2, 1440, 13.7);

// A helper function for debugging
const int LED_PIN_EX1 = 12;
const int LED_PIN_EX2 = 18; // shown as pin A0 on the Romi board
// make sure to change the increments
// the lower the value the more gentle
float K_p = 0.3;

int intersectionCounter = 0;

void setLED(int pin, bool value)
{
    Serial.println("setLED()");
    digitalWrite(pin, value);
    pinMode(LED_PIN_EX1, OUTPUT);
    pinMode(LED_PIN_EX2, OUTPUT);
}

// Defines the robot states
enum ROBOT_STATE
{
    ROBOT_IDLE,
    ROBOT_DRIVE_FOR,
    ROBOT_LINING,
    TARGET_FOUND,
    ROBOT_EXIT_FIRESTATION,
};
ROBOT_STATE robotState = ROBOT_IDLE;

float baseSpeed = 10;
float whiteThreshold = 550;

// idle() stops the motors
void idle(void)
{
    Serial.println("idle()");
    setLED(LED_PIN_EX1, LOW);
    setLED(LED_PIN_EX2, LOW);

    chassis.idle();

    // set state to idle
    robotState = ROBOT_IDLE;
    Serial.println("/idle()");
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup()
{
    // This will initialize the Serial at a baud rate of 115200 for prints
    // Be sure to set your Serial Monitor appropriately
    Serial.begin(115200);

    // Initialize the chassis (which also initializes the motors)
    chassis.init();
    // Adjust the PID coefficients
    chassis.setMotorPIDcoeffs(10, 1);
    idle();

    // Initializes the IR decoder
    decoder.init();

    Serial.println("/setup()");
}

// Leaving fire department
void beginning(void){
  chassis.driveFor(30, 10, true);
  delay(500);
  chassis.turnFor(95, 40, true);
  delay(100);
  chassis.driveFor(-40, 10, true);
  delay(500);
  robotState = ROBOT_LINING;
}

void leaveFireState2(void){
  Serial.println("leaving fire state 2");
  chassis.driveFor(-20, 5, true);
  delay(100);
  chassis.turnFor(-85, 30, true);
  delay(100);
  chassis.driveFor(45, 10, true);
  delay(100);
  chassis.turnFor(90, 30, true);
  delay(100);
  chassis.driveFor(10, 10, true);
  delay(100);
  robotState = ROBOT_LINING;
}

void leaveFireState1(void){
    Serial.println("leaving fire state 1");
    chassis.driveFor(-50, 5, true);
    delay(1000);
    chassis.turnFor(-90, 20, true);
    delay(1000);
    chassis.driveFor(30, 5, true);
}


void readFlameSensor(){
  pinMode(FLAME_PIN, INPUT);
  flameSignal = analogRead(FLAME_PIN);
  delay(100);
  Serial.println((String)"flame sensor reading: "+ flameSignal);
}

// Turns on/off the fan based on flame sensor reading
void fanFunction(void){
    while(flameSignal < 970){
      chassis.setWheelSpeeds(0, 0);
      analogWrite(FAN_PIN, FAN_SPEED);
      Serial.println("Fan is turned on.");
      delay(100);
      readFlameSensor();
    }
    analogWrite(FAN_PIN, 0);
    Serial.println("Fan is turned off.");
    robotState = ROBOT_LINING;
}

/**
 * Turns on the fan if finds fire and leaves the fire site
 * If no fire, goes to next fire site
*/
boolean targetFound(void)
{
  readFlameSensor();
  delay(1000); 
  if (flameSignal < 970)
  {
    robotState = TARGET_FOUND;
    Serial.println("i found the fire! i'm turning on the fan!");
    fanFunction();
    delay(2000);
        if(intersectionCounter == 4){
            delay(2000);
            leaveFireState2();
        }
        if(intersectionCounter == 6){
            delay(2000);
            leaveFireState1();
        }
    return true;
  }
  else
  {
    if(intersectionCounter == 4){
            delay(2000);
            leaveFireState2();
    }
    if(intersectionCounter == 6){
            delay(2000);
            leaveFireState1();
    }
    return false;
  }
}

// A helper command to drive a set distance
void drive(float speed)
{
    setLED(LED_PIN_EX1, HIGH);
    robotState = ROBOT_DRIVE_FOR;

    // Calls chassis.setWheelSpeeds() to set the wheel speeds
    chassis.setWheelSpeeds(speed, speed);
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
    setLED(LED_PIN_EX2, HIGH);
    robotState = ROBOT_DRIVE_FOR;

    // Makes a call to chassis.turnFor()
    chassis.turnFor(ang, speed, false);
}

void handleLineFollowing(float baseSpeed)
{
    // Line following control
    int16_t leftADC = analogRead(LEFT_LINE_SENSE);
    int16_t rightADC = analogRead(RIGHT_LINE_SENSE);
    int16_t error = leftADC - rightADC;
    // defines an error
    int16_t turnEffort = K_p * error;
    chassis.setTwist(baseSpeed, turnEffort);
}

/**
 * Checks whether it is an intersection and increases the counter
*/
bool checkIntersectionEvent(uint16_t whiteThreshold)
{
    static bool prevIntersection = false;

    bool retVal = false;

    if (analogRead(LEFT_LINE_SENSE) <= whiteThreshold && analogRead(RIGHT_LINE_SENSE) <= whiteThreshold)
    {
        retVal = true;
        intersectionCounter += 1;
        Serial.println((String)"Intersection count: " + intersectionCounter);
    }

    return retVal;
}

/**
 * Handles intersection motion based on specific intersection counter number
 * Robot may turn, stop, or continue to line follow
*/
void handleIntersection(void)
{
    delay(500);
    if(intersectionCounter == 3){
        Serial.println("at count 3");
        chassis.driveFor(0,0);
        Serial.println("stop at count 3");
        delay(1000);
        chassis.turnFor(-90, 20, true);
        robotState = ROBOT_LINING;
    }
    if(intersectionCounter == 4){
        Serial.println("at count 4");
        chassis.driveFor(0,0);
        Serial.println("stop at count 4");
        delay(1000);
        chassis.turnFor(-90, 20, true);
        delay(1000);
        chassis.driveFor(40, 15, true);
        delay(1000);
        chassis.turnFor(-85, 20, true);
        delay(1000);
        chassis.driveFor(5, 15, true);
        delay(2000);
        targetFound();
    }

    if(intersectionCounter == 5){
        Serial.println("at count 5");
        chassis.driveFor(0,0);
        Serial.println("stop at count 5");
        delay(1000);
        chassis.turnFor(-75, 40, true);
        delay(1000);
        robotState = ROBOT_LINING;
    }

    if(intersectionCounter == 6){
        chassis.turnFor(-90, 20, true);
        delay(1000);
        chassis.driveFor(40, 15, true);
        delay(1000);
        chassis.turnFor(-90, 20, true);
        delay(1000);
        chassis.driveFor(40, 15, true);
        delay(2000);
        targetFound();
    }
    if(intersectionCounter == 11){
        Serial.println("at count 11");
        chassis.driveFor(0,0);
        Serial.println("stop at count 11");
        delay(1000);
        chassis.turnFor(-90, 30, true);
        delay(1000);
        chassis.driveFor(-30, 15, true);
        delay(1000);
        intersectionCounter = 0;
        Serial.println(intersectionCounter);
        beginning();

    }else {
        robotState = ROBOT_LINING;
    }
}

// Sets robot to line follow
void beginLineFollowing()
{
    robotState = ROBOT_LINING;
    setLED(LED_PIN_EX1, HIGH);
    setLED(LED_PIN_EX2, HIGH);
}

void handleMotionComplete()
{ }

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
    Serial.println("Key: " + String(keyPress));

    // Emergency stop for a robot
    if (keyPress == ENTER_SAVE)
    {
        idle();
    }
    switch (robotState)
    {
    case ROBOT_IDLE:
        if (keyPress == PLAY_PAUSE)
        {
            robotState = ROBOT_EXIT_FIRESTATION;
        }
        break;

    // Eespond to speed +/- commands (when in ROBOT_LINING state)
    case ROBOT_LINING:
        if (keyPress == VOLplus)
        {
            // add 5 on base speed if VOL+ is pressed
            baseSpeed += 5;
        }
        if (keyPress == VOLminus)
        {
            // subtract 5 on base speed if VOL- is pressed
            baseSpeed -= 5;
        }

    default:
        break;
    }
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
    // Checks for a key press on the remote
    int16_t keyPress = decoder.getKeyCode();
    if (keyPress >= 0)
        handleKeyPress(keyPress);

    // A basic state machine
    switch (robotState)
    {
    case ROBOT_DRIVE_FOR:
        if (chassis.checkMotionComplete())
            handleMotionComplete();
        break;

    // Case to handle line following
    case ROBOT_LINING:
        handleLineFollowing(baseSpeed);
        // Check/handle intersection
        if (checkIntersectionEvent(whiteThreshold))
        {
            handleIntersection();
        }

    default:
        break;

    case TARGET_FOUND:
        //idle();

    case ROBOT_EXIT_FIRESTATION:
        beginning();
        delay(100);
        beginLineFollowing();
    }

    
}