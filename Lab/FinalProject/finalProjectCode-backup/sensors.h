#pragma once

#include "config.h"
#include "types.h"

// Basic sensor reading
int read_lidar(int pin);
int read_sonar(int pin);
void print_sensor_data(struct lidar data, struct sonar data2);
double cm2inch(int cm);

// Obstacle detection
bool isCloseObstacle();
bool checkFrontObstacle();
bool frontHasObstacle();
bool backHasObstacle();
bool leftHasObstacle();
bool rightHasObstacle();

// Wall detection
bool leftHasWall();
bool rightHasWall();
bool frontHasWall();
bool isLeftCorner();
bool isRightCorner();

// RPC functions
struct lidar read_lidars();
struct sonar read_sonars();

// Helper functions
bool isWithinObstacleMargin(double frontDistance);