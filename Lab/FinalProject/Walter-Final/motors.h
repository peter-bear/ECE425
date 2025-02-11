#pragma once
#include <AccelStepper.h>  //include the stepper motor library
#include <MultiStepper.h>  //include multiple stepper motor library
#include "led.h"
#include <Arduino.h>

// define motor pin numbers
#define stepperEnable 48  // stepper enable pin on stepStick
#define rtStepPin 50      // right stepper motor step pin
#define rtDirPin 51       // right stepper motor direction pin
#define ltStepPin 52      // left stepper motor step pin
#define ltDirPin 53       // left stepper motor direction pin

#define stepperEnTrue false  // variable for enabling stepper motor
#define stepperEnFalse true  // variable for disabling stepper motor
#define max_speed 1500       // maximum stepper motor speed
#define max_accel 10000      // maximum motor acceleration

#define defaultStepSpeed 200  // default step speed


#define TO_LEFT -1  // direction variables to left
#define TO_RIGHT 1  // direction variables to right

#define CLOCKWISE 1          // direction variables for clockwise motion
#define COUNTERCLOCKWISE -1  // direction variables for counterclockwise motion

void init_stepper();
void stopMove();
int length2Steps(double length);
void spin(int direction, double angle, int speed);

void forward(double distance, int speed);
void reverse(double distance, int speed);


void goToAngle(double angle);
double getTurnAngle(double x, double y);
void goToGoal(double x, double y);

void turn(int direction, double timeDelay, int velocityDiff);

extern AccelStepper stepperRight;
extern AccelStepper stepperLeft;
extern MultiStepper steppers;