#pragma once

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "config.h"
#include "types.h"

// Stepper motor initialization and control
void init_stepper();
void stopMove();
void PIDControl(int leftDistance, int rightDistance);
int length2Steps(double length);

// Basic movement functions
void spin(int direction, double angle, int speed);
void forward(double distance, int speed);
void reverse(double distance, int speed);
void turn(int direction, double timeDelay, int velocityDiff);

// Navigation functions
void goToAngle(double angle);
double getTurnAngle(double x, double y);
void goToGoal(double x, double y);

// Encoder functions
void LwheelSpeed();
void RwheelSpeed();
void print_encoder_data();
void resetEncoder();

// External variables
extern AccelStepper stepperRight;
extern AccelStepper stepperLeft;
extern MultiStepper steppers;