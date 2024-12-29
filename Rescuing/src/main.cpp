#include <Chassis.h>
#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>
#include <servo32u4.h>
#include <Rangefinder.h>

Servo32U4 servo;

#define SERVO_DOWN 1200
#define SERVO_UP 2200

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// TODO, Section IV.1 step4: Declare the chassis object (with default values)
Chassis chassis(7.2, 1440, 13.7);
// TODO, Section IV.2 step8: Adjust parameters to better match actual motion

// A helper function for debugging
const int LED_PIN_EX1 = 12;
const int LED_PIN_EX2 = 18; // shown as pin A0 on the Romi board
// make sure to change the increments
// the lower the value the more gentle
float K_p = 0.3;

int intersectionCounter = 0;

Rangefinder rangefinder(11, 4);
float distanceFront = 90;

void setLED(int pin, bool value)
{
    Serial.println("setLED()");
    digitalWrite(pin, value);
    pinMode(LED_PIN_EX1, OUTPUT);
    pinMode(LED_PIN_EX2, OUTPUT);
}

// Defines the robot states
// TODO, Section IV.3 step1: Define a state for line following
enum ROBOT_STATE
{
    ROBOT_IDLE,
    ROBOT_DRIVE_FOR,
    ROBOT_LINING,
    TARGET_FOUND,
    ROBOT_EXIT_FIRESTATION,
};
ROBOT_STATE robotState = ROBOT_IDLE;

// TODO, Section IV.3 step2: Define a baseSpeed
float baseSpeed = 10;
// TODO, Section IV.4 step1: Define a whiteThreshold
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

    // TODO, Section IV.1 step5: Initialize the chassis (which also initializes the motors)
    chassis.init();
    // TODO, Section IV.2 step1: Adjust the PID coefficients
    chassis.setMotorPIDcoeffs(10, 1);
    idle();

    // attach and initialize servo
    servo.attach();
    servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
    servo.writeMicroseconds(SERVO_DOWN);

    // Initializes the IR decoder
    decoder.init();

    // initializes the rangefinder
    rangefinder.init();

    Serial.println("/setup()");
}

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
  chassis.driveFor(-15, 5, true);
  delay(100);
  chassis.turnFor(-85, 30, true);
  delay(100);
  chassis.driveFor(60, 10, true);
  delay(100);
  chassis.turnFor(80, 30, true);
  delay(100);
  chassis.driveFor(10, 10, true);
  delay(100);
  robotState = ROBOT_LINING;
}

void leaveFireState1(void){
    Serial.println("leaving fire state 1");
    chassis.driveFor(-45, 15, true);
    delay(1000);
    chassis.turnFor(-85, 30, true);
    delay(1000);
    chassis.driveFor(60, 15, true);
}

void readSonar()
{                                            // read ultrasonic range sensor
  distanceFront = rangefinder.getDistance(); // distance in the front in cm
  delay(1000);
  Serial.println(distanceFront);
}

void rescueFunction(void){
    chassis.turnFor(180, 30, true);
    delay(1000);
    chassis.driveFor(-10, 25, true);
    delay(1000);
    // arm goes up
    servo.writeMicroseconds(SERVO_UP);
    delay(1000);
    chassis.driveFor(10, 25, true);
    delay(1000);
    chassis.turnFor(-180, 30, true);
    Serial.println("People rescued.");
    robotState = ROBOT_LINING;
}

boolean targetFound(void)
{
    delay(100);
    readSonar();
    delay(100);
    readSonar();
    if (distanceFront < 20 || distanceFront > 150) {
    // robotState = TARGET_FOUND;
    Serial.println("i found the people! i'm rescuing them!");
    rescueFunction();
    delay(2000);
        if(intersectionCounter == 4){
            delay(2000);
            leaveFireState2();
        }
        if(intersectionCounter == 6){
            delay(2000);
            leaveFireState1();
        }
        robotState = TARGET_FOUND;
        return true;
    }
  else
  {
    Serial.println("i didn't find anything.");
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
// At the start, it will take no arguments and we'll hardcode a motion
// TODO, Section IV.2 step4 (but not before!): Edit the function definition to accept a speed
// adjust the drive function to take in float speed argument
void drive(float speed)
{
    setLED(LED_PIN_EX1, HIGH);
    robotState = ROBOT_DRIVE_FOR;

    // TODO: In Section IV.1 step7 and IV.2 step2, add a call to chassis.setWheelSpeeds() to set the wheel speeds
    chassis.setWheelSpeeds(speed, speed);
    // TODO: In Section IV.2 step9, (optional) temporarily remove the call to setWheelSpeeds() and add a call to chassis.driveFor()
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
    setLED(LED_PIN_EX2, HIGH);
    robotState = ROBOT_DRIVE_FOR;

    // TODO, Section IV.2 step5: Make a call to chassis.turnFor()
    chassis.turnFor(ang, speed, false);
}

void handleLineFollowing(float baseSpeed)
{
    // TODO, Section IV.3 step5: Add line following control
    int16_t leftADC = analogRead(LEFT_LINE_SENSE);
    //Serial.println("leftADC: " + String(leftADC));
    int16_t rightADC = analogRead(RIGHT_LINE_SENSE);
    //Serial.println("rightADC: " + String(rightADC));
    int16_t error = leftADC - rightADC;
    // define an error
    int16_t turnEffort = K_p * error;
    chassis.setTwist(baseSpeed, turnEffort);
}

bool checkIntersectionEvent(uint16_t whiteThreshold)
{
    static bool prevIntersection = false;

    bool retVal = false;

    // TODO, Section IV.4 step2: Add logic to check for intersection
    if (analogRead(LEFT_LINE_SENSE) <= whiteThreshold && analogRead(RIGHT_LINE_SENSE) <= whiteThreshold)
    {
        retVal = true;
        intersectionCounter += 1;
        Serial.println((String)"Intersection count: " + intersectionCounter);
    }

    return retVal;
}

void handleIntersection(void)
{
    delay(500);
    if(intersectionCounter == 3){
        // turn right to go to site 2 curve
        Serial.println("at count 3");
        chassis.driveFor(0,0);
        Serial.println("stop at count 3");
        delay(1000);
        chassis.turnFor(-90, 30, true);
        robotState = ROBOT_LINING;
    }
    if(intersectionCounter == 4){
        // going into site 2
        Serial.println("at count 4");
        chassis.driveFor(0,0);
        Serial.println("stop at count 4");
        delay(1000);
        chassis.turnFor(-85, 30, true);
        delay(1000);
        chassis.driveFor(60, 25, true);
        delay(1000);
        chassis.turnFor(-85, 30, true);
        delay(1000);
        chassis.driveFor(2, 25, true);
        delay(2000);
        targetFound();
    }

    if(intersectionCounter == 5){
        Serial.println("at count 5");
        chassis.driveFor(0,0);
        Serial.println("stop at count 5");
        delay(1000);
        chassis.turnFor(-82, 40, true);
        delay(1000);
        robotState = ROBOT_LINING;
    }

    if(intersectionCounter == 6){
        // going into site 1
        chassis.turnFor(-90, 30, true);
        delay(1000);
        chassis.driveFor(60, 25, true);
        delay(1000);
        chassis.turnFor(-90, 30, true);
        delay(1000);
        chassis.driveFor(25, 25, true);
        delay(2000);
        targetFound();
    }

    if(intersectionCounter == 8){
        chassis.turnFor(-90, 30, true);
        delay(1000);
        chassis.driveFor(-20, 25, true);
        delay(1000);
        // drop off people at hospital
        servo.writeMicroseconds(SERVO_DOWN);
        delay(1000);
        chassis.driveFor(20, 25, true);
        delay(1000);
        chassis.turnFor(90, 30, true);
        delay(1000);
        chassis.driveFor(10, 25, true);
        robotState = ROBOT_LINING;
    }

    if(intersectionCounter == 11){
        // moves back into fire station
        Serial.println("at count 11");
        chassis.driveFor(0,0);
        Serial.println("stop at count 11");
        delay(1000);
        chassis.turnFor(-90, 30, true);
        delay(1000);
        chassis.driveFor(-30, 25, true);
        delay(1000);
        // restart like new
        intersectionCounter = 0;
        Serial.println(intersectionCounter);
        beginning();

    }else {
        robotState = ROBOT_LINING;
    }
}
void beginLineFollowing()
{
    robotState = ROBOT_LINING;
    setLED(LED_PIN_EX1, HIGH);
    setLED(LED_PIN_EX2, HIGH);
}

// TODO, Section IV.2 step7: Declare function handleMotionComplete(), which calls idle()
void handleMotionComplete()
{
    
}
// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
    Serial.println("Key: " + String(keyPress));

    // TODO, Section IV.1 step1: add "emergency stop"
    if (keyPress == ENTER_SAVE)
    {
        idle();
    }
    switch (robotState)
    {
    case ROBOT_IDLE:
        // TODO, Section IV.1 step2: Handle up arrow button
        // if (keyPress == UP_ARROW)
        // {
        //     drive(30);
        // }
        // // TODO, Section IV.2 step3: Handle the down, left and right arrow buttons
        // if (keyPress == DOWN_ARROW)
        // {
        //     drive(-30);
        // }
        // if (keyPress == RIGHT_ARROW)
        // {
        //     turn(-98, 50);
        // }
        // if (keyPress == LEFT_ARROW)
        // {
        //     turn(98, 50);
        // }
        // TODO, Section IV.3 step3: Respond to PLAY_PAUSE press
        if (keyPress == PLAY_PAUSE)
        {
            robotState = ROBOT_EXIT_FIRESTATION;
            //beginLineFollowing();
        }
        break;

    // TODO, Section IV.5 step1: respond to speed +/- commands (when in ROBOT_LINING state)
    // Speed control
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

    // Use the VOLplus and VOLminus keys on your IR remote
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
        // TODO, Section IV.2 step6: Uncomment to handle completed motion
        if (chassis.checkMotionComplete())
            handleMotionComplete();
        break;

    // TODO, Section IV.3 step4: Add a case to handle line following
    case ROBOT_LINING:
        handleLineFollowing(baseSpeed);
        // TODO, Section IV.4 step3: check/handle intersection
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

    // case ROBOT_LEAVE_FIRE2:
    //     leaveFireState2();

    }

    
}