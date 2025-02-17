/*
  Walter-Lab02.ino
  Yao Xiong & Zhengyang Bi 2024/1/17

  This program implements various autonomous behaviors for a wheeled robot using stepper motors and distance sensors.
  The robot features reactive behaviors including obstacle avoidance, runaway response, and following behavior.
  It uses a dual-core architecture (M7 and M4) for parallel processing of sensor data and motor control.

  The primary functions created are:
  goToAngle - turns robot to specified absolute angle in degrees
  goToGoal - moves robot to specified x,y coordinate in inches
  forward, reverse - both wheels move with same velocity, same direction
  spin - both wheels move with same velocity opposite direction
  smartWanderBehavior - random movement with obstacle avoidance
  runawayBehavior - moves away from detected obstacles
  followBehavior - maintains constant distance from detected obstacle
  collideBehavior - emergency stop when obstacle detected

  Sensor Integration:
  - 4 LIDAR sensors (front, back, left, right) for distance measurement
  - 2 wheel encoders for position tracking
  - PID control system for accurate movement

  Core Architecture:
  M4 Core - Handles sensor data collection and processing
  M7 Core - Manages movement control and behavioral algorithms
  Uses RPC (Remote Procedure Call) for inter-core communication

  Hardware Connections:
  Arduino pin mappings: https://docs.arduino.cc/tutorials/giga-r1-wifi/cheat-sheet#pins
  A4988 Stepper Motor Driver Pinout: https://www.pololu.com/product/1182

  Stepper Motor Control:
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - right stepper motor step pin
  digital pin 51 - right stepper motor direction pin
  digital pin 52 - left stepper motor step pin
  digital pin 53 - left stepper motor direction pin

  Status LEDs:
  digital pin 13 - enable LED on microcontroller
  digital pin 5 - red LED in series with 220 ohm resistor
  digital pin 6 - green LED in series with 220 ohm resistor
  digital pin 7 - yellow LED in series with 220 ohm resistor

  Sensors:
  digital pin 18 - left encoder pin
  digital pin 19 - right encoder pin
  digital pin 8 - front LIDAR
  digital pin 9 - back LIDAR
  digital pin 10 - left LIDAR
  digital pin 11 - right LIDAR

  Constants:
  Robot Physical Parameters:
  - Wheel radius: 1.7 inches
  - Robot width: 9.0 inches
  - Steps per revolution: 800

  Movement Parameters:
  - Default step speed: 300 steps/sec
  - Maximum speed: 1500 steps/sec
  - Maximum acceleration: 10000 steps/sec^2

  Sensor Parameters:
  - Maximum LIDAR distance: 40 cm
  - Obstacle threshold: 10 cm

  Required Libraries:
  - AccelStepper: https://www.airspayce.com/mikem/arduino/AccelStepper/
  Install via:
  Sketch->Include Library->Manage Libraries...->AccelStepper->Include
  OR
  Sketch->Include Library->Add .ZIP Library...->AccelStepper-1.53.zip

  See PlatformIO documentation for proper way to install libraries in Visual Studio
*/

// includew all necessary libraries
#include <Arduino.h>  //include for PlatformIO Ide
#include <math.h>
#include "RPC.h"
#include "mqtt.h"
#include "types.h"
#include "sensors.h"
#include "encoder.h"
#include "led.h"
#include "motors.h"
#include "behaviors.h"
#include "advancedBehavior.h"


/**
 * @brief Initializes the stepper motor and sets up interrupts for the left and right wheel encoders.
 *
 * This function performs the following tasks:
 * - Initializes the stepper motor by calling init_stepper().
 * - Attaches an interrupt to the left wheel encoder pin, triggering the LwheelSpeed function on any change.
 * - Attaches an interrupt to the right wheel encoder pin, triggering the RwheelSpeed function on any change.
 * - Delays execution for 1000 milliseconds to allow for setup stabilization.
 */
void setupM7() {
  init_stepper();  // set up stepper motor
  attachEncoders();
  delay(1000);

  connectWifi();
  connectMqtt();
  subscribeTopics();
}

void stateBehaviors() {
  switch (currentState){
    case STOP:
      stepperLeft.setSpeed(0);
      stepperRight.setSpeed(0);
      break;
    case MOVE_FORWARD:
      stepperLeft.setSpeed(defaultStepSpeed);
      stepperRight.setSpeed(defaultStepSpeed);
      break;
    case TURN_LEFT:
      stepperLeft.setSpeed(-defaultStepSpeed);
      stepperRight.setSpeed(defaultStepSpeed);
      break;
    case TURN_RIGHT:
      stepperLeft.setSpeed(defaultStepSpeed);
      stepperRight.setSpeed(-defaultStepSpeed);
      break;
    case MOVE_BACKWARD:
      stepperLeft.setSpeed(-defaultStepSpeed);
      stepperRight.setSpeed(-defaultStepSpeed);
      break;
    case SLAM_BEHAVIOR:
      SLAM();
      break;
    case MATRIX_PATH_PLANNING:

      break;
    default:
      stopMove();
      break;
  }
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}

/**
 * @brief Main loop function for behavior control.
 *
 * This function initializes the state of three LEDs (red, yellow, green) to off.
 * It then enters an infinite loop where it continuously executes the
 * smartFollowBehavior function. Other behaviors such as smartWanderBehavior
 * and runawayBehavior are commented out and can be enabled if needed.
 */
void loopM7() {
  allOFF();
  // currentState = STOP;
  currentState = MATRIX_PATH_PLANNING;
  

  while (true) {
    // smartWanderBehavior();
    // smartFollowBehavior();
    // runawayBehavior();
    // followWallBehavior();
    // goToGoalAvoidbs(72.0, 0.0);

    Position start, goal;
    start.x = 4; start.y = 0;
    goal.x = 0; goal.y = 4;
    PositionQueue path = matrixPathPlanning(start, goal);
    delay(10000);

    // mqttClient.poll();
    // publishData();
    // stateBehaviors();
  }
}

// set up the M4 to be the server for the sensors data
void setupM4() {
  setupSensors();
}

// loop for the M4 to read the sensors
void loopM4() {
  readSensors();
}

// MAIN
void setup() {
  int baudrate = 9600;        // serial monitor baud rate'
  randomSeed(analogRead(0));  // generate a new random number each time called
  Serial.begin(baudrate);     // start serial monitor communication
  Serial.println("Robot starting...Put ON TEST STAND");

  RPC.begin();
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    // if on M7 CPU, run M7 setup & loop
    setupM7();
    while (1)
      loopM7();
  } else {
    // if on M4 CPU, run M7 setup & loop
    setupM4();
    while (1)
      loopM4();
  }
}

void loop() {}