
/*
  Walter-Lab01.ino
  Yao Xiong & Zhengyang Bi 2024/12/15

  This program will introduce using the stepper motor library to create motion algorithms for the robot.
  The motions will be go to angle, go to goal, move in a circle, square, figure eight and teleoperation (stop, forward, spin, reverse, turn)
  It will also include wireless commmunication for remote control of the robot by using a game controller or serial monitor.
  The primary functions created are
  moveCircle - given the diameter in inches, direction of clockwise or counterclockwise, the speed of the robot, move the robot in a circle with that diameter
  moveFigure8 - given the diameter in inches, use the moveCircle() function with direction input to create a Figure 8
  forward, reverse - both wheels move with same velocity, same direction
  pivot- one wheel stationary, one wheel moves forward or back, given the direction of to left or to right, the pivot angle in degrees, and speed in steps per second
  spin - both wheels move with same velocity opposite direction, given the direction of to left or to right, the spin angle in degrees, and speed in steps per second
  turn - both wheels move with same direction different velocity, give the direction of to left or to right, the turning time in seconds, and velocity difference in steps per second
  stop - both wheels stationary

  Interrupts
  https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  https://www.arduino.cc/en/Tutorial/CurieTimer1Interrupt
  https://playground.arduino.cc/code/timer1
  https://playground.arduino.cc/Main/TimerPWMCheatsheet
  http://arduinoinfo.mywikis.net/wiki/HOME

  Hardware Connections:
  Arduino pin mappings: https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet#pins
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182 

  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin
  digital pin 13 - enable LED on microcontroller

  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin

  INSTALL THE LIBRARY
  AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
  
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip
  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

//includew all necessary libraries
#include <Arduino.h>       //include for PlatformIO Ide
#include <AccelStepper.h>  //include the stepper motor library
#include <MultiStepper.h>  //include multiple stepper motor library
#include <math.h>

/**
// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "Portenta_H7_TimerInterrupt.h"

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "Portenta_H7_ISR_Timer.h"

// Timer
// These define's must be placed at the beginning before #include "Portenta_H7_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Timer
#define TIMER_INTERVAL_MS         100
#define HW_TIMER_INTERVAL_MS      50

#define TIMER_INTERVAL_0_5S           500L

// Init timer TIM12
Portenta_H7_Timer ITimer(TIM12);

// Init Portenta_H7_ISR_Timer
// Each Portenta_H7_ISR_Timer can service 16 different ISR-based timers
Portenta_H7_ISR_Timer ISR_Timer;

void TimerHandler()
{
  ISR_Timer.run();
}

*/

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define grnLED 6            //green LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = { 5, 6, 7 };  //array of LED pin numbers

//define motor pin numbers
#define stepperEnable 48  //stepper enable pin on stepStick
#define rtStepPin 50      //right stepper motor step pin
#define rtDirPin 51       // right stepper motor direction pin
#define ltStepPin 52      //left stepper motor step pin
#define ltDirPin 53       //left stepper motor direction pin

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  //create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   //create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;                                                 //create instance to control multiple steppers at the same time

#define stepperEnTrue false  //variable for enabling stepper motor
#define stepperEnFalse true  //variable for disabling stepper motor
#define max_speed 1500       //maximum stepper motor speed
#define max_accel 10000      //maximum motor acceleration

int pauseTime = 2500;  //time before robot moves
int stepTime = 500;    //delay time between high and low on step pin
int wait_time = 1000;  //delay for printing data

//define encoder pins
#define LEFT 0                        //left encoder
#define RIGHT 1                       //right encoder
const int ltEncoder = 18;             //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;             //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = { 0, 0 };  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = { 0, 0 };          //variable to hold encoder speed (left, right)
int accumTicks[2] = { 0, 0 };         //variable to hold accumulated ticks since last reset

#define TO_LEFT -1                    // direction variables to left
#define TO_RIGHT 1                    // direction variables to right

#define CLOCKWISE 1                    //direction variables for clockwise motion
#define COUNTERCLOCKWISE -1           //direction variables for counterclockwise motion

// Helper Functions
const float pi = 3.14159;               //pi
const float wheelRadius = 1.7;          //robot wheel radius in inches
const int stepsPerRevol = 800;          //robot wheel steps per revolution
const float robotWidth = 9.0;           //robot width in inches
const int defaultStepSpeed = 400;       //robot default speed in steps per second

const int PIDThreshold = 50;            //PID threshold
const int PIDkp = 1;                    //PID proportional gain
const int encoderRatio = 20;            //ratio between the encoder ticks and steps


// random move maximum values
const int maxTurnAngle = 90; // Maximum turn angle in degrees
const int maxDistanceMove = 12.0; // Maximum move distance in inches

// lidar constant values
#define FRONT 0
#define BANK 1
#define LEFT 2
#define RIGHT 3
#define numOfSens 4

uint16_t wait = 100;
int16_t ft_lidar = 8;
int16_t bk_lidar = 9;
int16_t lt_lidar = 10;
int16_t rt_lidar = 11;
int16_t lidar_pins[4] = {8,9,10,11 };






//interrupt function to count left encoder tickes
void LwheelSpeed() {
  encoder[LEFT]++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed() {
  encoder[RIGHT]++;  //count the right wheel encoder interrupts
}

void allOFF() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(leds[i], LOW);
  }
}

//function to set all stepper motor variables, outputs and LEDs
void init_stepper() {
  pinMode(rtStepPin, OUTPUT);                   //sets pin as output
  pinMode(rtDirPin, OUTPUT);                    //sets pin as output
  pinMode(ltStepPin, OUTPUT);                   //sets pin as output
  pinMode(ltDirPin, OUTPUT);                    //sets pin as output
  pinMode(stepperEnable, OUTPUT);               //sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  //turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);                   //set enable LED as output
  digitalWrite(enableLED, LOW);                 //turn off enable LED
  pinMode(redLED, OUTPUT);                      //set red LED as output
  pinMode(grnLED, OUTPUT);                      //set green LED as output
  pinMode(ylwLED, OUTPUT);                      //set yellow LED as output
  digitalWrite(redLED, HIGH);                   //turn on red LED
  digitalWrite(ylwLED, HIGH);                   //turn on yellow LED
  digitalWrite(grnLED, HIGH);                   //turn on green LED
  delay(pauseTime / 5);                         //wait 0.5 seconds
  digitalWrite(redLED, LOW);                    //turn off red LED
  digitalWrite(ylwLED, LOW);                    //turn off yellow LED
  digitalWrite(grnLED, LOW);                    //turn off green LED

  stepperRight.setMaxSpeed(max_speed);         //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);     //set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);          //set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);      //set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);           //add right motor to MultiStepper
  steppers.addStepper(stepperLeft);            //add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);  //turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);               //turn on enable LED
}

//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                            //print manager timer
  if (millis() - timer > 100) {                              //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                         //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                       //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];     //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT];  //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;   //clear the left encoder data buffer
    encoder[RIGHT] = 0;  //clear the right encoder data buffer
    timer = millis();    //record current time since program started
  }
}

/*this function will reset the encoders*/
void resetEncoder() {
  encoder[LEFT] = 0;   //clear the left encoder data buffer
  encoder[RIGHT] = 0;  //clear the right encoder data buffer
}

/*this function will stop the steppers*/
void stopSteppers() {
  stepperRight.stop();
  stepperLeft.stop();
}

/*
  This function performs PID control to adjust the position of the robot's wheels.
  Inputs:
    - leftDistance: Target distance for the left wheel.
    - rightDistance: Target distance for the right wheel.
  The function calculates the error between the target and actual encoder values,
  applies a proportional control to determine the necessary movement adjustments,
  and sets the wheel speeds accordingly. The steppers are then run to the computed positions.
  If the error is within a specified threshold, the function stops the steppers.
*/
void PIDControl(int leftDistance, int rightDistance) {
  // Calculate the error between the target and actual encoder values
  long leftDistanceError = abs(leftDistance) - encoder[LEFT]*encoderRatio;
  long rightDistanceError = abs(rightDistance) - encoder[RIGHT]*encoderRatio;

  // Apply proportional control
  if (abs(leftDistanceError) > PIDThreshold || abs(rightDistanceError) > PIDThreshold) {
    long outputLeft = leftDistanceError * PIDkp;
    long outputRight = rightDistanceError * PIDkp;


    // Set the wheel speeds
    if(leftDistanceError<0){
      stepperLeft.setSpeed(-stepperLeft.speed());
    }
    if(rightDistanceError<0){
      stepperRight.setSpeed(-stepperRight.speed());
    }
    
    // Run the steppers
    stepperLeft.move(outputLeft);
    stepperRight.move(outputRight);
    steppers.runSpeedToPosition();
  }
  stopSteppers();
}

/**
 * Converts a length in inches to a number of steps for the robot.
 * @param length the length in inches to convert
 * @return the number of steps for the robot to move the given length
 */

int length2Steps(double length) {
  return round(stepsPerRevol * length / (2 * pi * wheelRadius));
}


/**
 * Spins the robot about its center by rotating both wheels in opposite directions.
 * @param direction the direction of the spin (TO_LEFT or TO_RIGHT)
 * @param angle the angle of the spin in degrees
 * @param speed the speed of the spin in steps per second
 */

void spin(int direction, double angle, int speed) {
  if (direction != TO_LEFT && direction != TO_RIGHT && angle < 0) return;
  resetEncoder();

  // calculate the turn length, using the circle formula
  double stepperMoveLength = pi * robotWidth * angle / 360;
  int stepperMovePos = length2Steps(stepperMoveLength);

  if (direction == TO_RIGHT) {
    stepperLeft.move(stepperMovePos);
    stepperLeft.setSpeed(speed);

    stepperRight.move(-stepperMovePos);
    stepperRight.setSpeed(-speed);
  } else if (direction == TO_LEFT) {
    stepperLeft.move(-stepperMovePos);
    stepperLeft.setSpeed(-speed);

    stepperRight.move(stepperMovePos);
    stepperRight.setSpeed(speed);
  }

  steppers.runSpeedToPosition();

  // use Encoder PID Control
  if (direction == TO_RIGHT) {
    PIDControl(stepperMovePos, -stepperMovePos);
  } else if (direction == TO_LEFT) {
    PIDControl(-stepperMovePos, stepperMovePos);
  }
}


/**
 * Moves the robot forward by a specified distance at a specified speed.
 * @param distance the distance in inches to move the robot
 * @param speed the speed in steps per second to move the robot
 */
void forward(double distance, int speed) {
  resetEncoder();

  int stepperMovePos = length2Steps(distance);

  stepperLeft.move(stepperMovePos);   //move left wheel to relative position
  stepperRight.move(stepperMovePos);  //move right wheel to relative position

  stepperLeft.setSpeed(speed);   //set left motor speed
  stepperRight.setSpeed(speed);  //set right motor spee
  steppers.runSpeedToPosition();

  PIDControl(stepperMovePos, stepperMovePos);
}


/**
 * Moves the robot backward by a specified distance at a specified speed.
 * @param distance the distance in inches to move the robot backward
 * @param speed the speed in steps per second to move the robot backward
 * The function calls the forward() function with negative values of distance and speed.
 */

void reverse(double distance, int speed) {
  forward(-distance, -speed);
}


/**
 * Turns the robot to the absolute angle specified.
 * @param angle the absolute angle to turn to in degrees
 * @note Positive angles are to the left, negative angles are to the right
 */
void goToAngle(double angle) {
  Serial.println("goToAngle function");
  digitalWrite(grnLED, HIGH);  //turn on green LED
  // if angle larger than 0, turn left
  // if angle less than 0, turn right
  int direction = 0;
  if (angle > 0) {
    direction = TO_LEFT;
  } else if (angle < 0) {
    direction = TO_RIGHT;
  } else {
    return;
  }
  angle = abs(angle);
  spin(direction, angle, defaultStepSpeed);
}

/**
 * Moves the robot to the goal location (x, y) in inches.
 * @param x the x-coordinate of the goal in inches
 * @param y the y-coordinate of the goal in inches
 * @note The robot will first turn to the correct angle, then move forward to the goal
 */
void goToGoal(double x, double y) {
  Serial.println("goToGoal function");
  digitalWrite(redLED, LOW);   //turn off red LED
  digitalWrite(grnLED, HIGH);  //turn on green LED
  digitalWrite(ylwLED, HIGH);  //turn on yellow LED

  double distance = sqrt(pow(x, 2) + pow(y, 2));
  double angleRadian = atan(y / x);
  double angleDegree = angleRadian * 180.0 / pi;

  if (x > 0 && y > 0) {
    angleDegree = angleDegree;
  } else if (x < 0 && y > 0) {
    angleDegree = 90 + angleDegree;
  } else if (x < 0 && y < 0) {
    angleDegree = 180 + angleDegree;
  } else if (x > 0 && y < 0) {
    angleDegree = -angleDegree;
  }

  goToAngle(angleDegree);
  delay(500);
  forward(distance, defaultStepSpeed);
}


void randomWander(){
  digitalWrite(grnLED, HIGH);  //turn on green LED
  
  int randAngle = random(-maxTurnAngle, maxTurnAngle);
  int randDistance = random(maxDistanceMove);
  
  goToAngle(randAngle);
  forward(randDistance, defaultStepSpeed);

  // Serial.print("Random Values:\n\tAngle: ");
  // Serial.print(randAngle);
  // Serial.print("\tDistance: ");
  // Serial.println(randDistance);
}

void init_lidar(){
  for (int i = 0; i<numOfSens;i++){
    pinMode(lidar_pins[i], OUTPUT);
  }
}

//thie function will read the left or right sensor based upon input value
int readLidar(uint16_t side) {
  int16_t t = lidar_pins[side];
  int d; //distance to  object
  if (t == 0){
    // pulseIn() did not detect the start of a pulse within 1 second.
    //Serial.println("timeout");
    d = 100000; //no object detected
  }
  else if (t > 1850)  {
    //Serial.println("timeout");
    d = 100000; //no object detected
  }
  else  {
    // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
    d = (t - 1000) * 3 / 40;
 
    // Limit minimum distance to 0.
    if (d < 0) { d = 0; } 
  }
  //   Serial.print(d);
  // Serial.print(" cm, ");
  return d;
}

void avoidObstacle(){
  Serial.println(F("Obstacle Check"));
  // int distance = 0;
  // for (int i = 0; i<numOfSens;i++){
  //   distance = readLidar(i);
  //   if(distance <= 10){
  //     stopSteppers();
  //     return;
  //   }
  // }
  for (int i = 0;i<4;i++){
    Serial.print(i);
    Serial.print(": ");
    Serial.print(readLidar(i));
    Serial.print(" ");
  }
  Serial.println();

}


// MAIN
void setup() {
  int baudrate = 9600;  //serial monitor baud rate'
  init_stepper();       //set up stepper motor
  init_lidar();         //set up lidars
  
  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);  //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);  //init the interrupt mode for the right encoder
  
  // ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler); // Interval in microsecs
  // ISR_Timer.setInterval(TIMER_INTERVAL_0_5S,  avoidObstacle); // Timer for avoid obstacles
  
  randomSeed(analogRead(0)); //generate a new random number each time called

  Serial.begin(baudrate);  //start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime);  //always wait 2.5 seconds before the robot moves
}

void loop() {
  //uncomment each function one at a time to see what the code does
  // forward(24.0, defaultStepSpeed);
  // delay(1000);
  // reverse(24.0, defaultStepSpeed);
  // delay(1000);
  // spin(TO_LEFT, 90.0, defaultStepSpeed);
  // delay(1000);
  // spin(TO_RIGHT, 90.0, defaultStepSpeed);
  // delay(1000);

  // goToGoal(24.0, 24.0);
  // goToGoal(-36.0, -48.0);

  //Uncomment to read Encoder Data (uncomment to read on serial monitor)
  // print_encoder_data();   //prints encoder data

  // randomWander();

  delay(wait_time);  //wait to move robot or read data
}