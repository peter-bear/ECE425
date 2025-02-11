#pragma once
#include "motors.h"
#include "types.h"
#include "sensors.h"
#include "led.h"
#include "encoder.h"
#include <Arduino.h>


#define runawayPropotion 0.5
#define followPropotion 0.25

#define OBSTACLE_THRESHOLD 10  // centimeters
#define OBSTACLE_MARGIN 2.0
#define FOLLOW_SPEED 200


// Constants for wall following behavior
#define WallFollowKp 3.0  // Proportional gain - adjust this between 1-10
#define WallFollowKd 0.1  // Derivative gain

#define TARGET_DISTANCE 5.0         // Target distance from wall (inches)
#define DEADBAND_INNER 4.0          // Inner deadband boundary (inches)
#define DEADBAND_OUTER 6.0          // Outer deadband boundary (inches)
#define FOLLOW_WALL_BASE_SPEED 300  // Base motor speed

#define WALL_THRESHOLD 35

enum ToGoalState {
  MOVING,
  AVOID_OBSTACLE_1,
  AVOID_OBSTACLE_2,
  AVOID_OBSTACLE_3,
};

void randomWander();
void collideBehavior();
void runawayBehavior();

void smartWanderBehavior();
void smartFollowBehavior();

void moveAndFollowBehavior();

void followCenter();
void followLeft();
void followRight();
void followWallBehavior();
void goToGoalAvoidbs(double x, double y);


void stateBehaviors();