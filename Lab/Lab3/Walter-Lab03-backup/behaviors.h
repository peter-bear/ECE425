#pragma once

#include "motors.h"
#include "sensors.h"

// LED control
void updateLEDs();
void allOFF();

// Basic behaviors
void collideBehavior();
void runawayBehavior();
void followBehavior();
void randomWander();
void printRandomValues(int randAngle, int randDistance);

// Smart behaviors
void smartWanderBehavior();
void smartFollowBehavior();
void moveAndFollowBehavior();

// Wall following behaviors
void followWallBehavior();
void followLeft();
void followRight();
void followCenter();