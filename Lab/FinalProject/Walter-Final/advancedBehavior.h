#pragma once
#include "behaviors.h"
#include "positionQueue.h"
#include "mqtt.h"

#define MATRIX_SIZE_X 4
#define MATRIX_SIZE_Y 4

// the size for each grid in the matrix in inches
#define GRID_RATIO 18.0 
#define MOVE_DIRECTIONS 5

void debugPath(PositionQueue plannedPath);
PositionQueue matrixPathPlanning(Position start, Position goal);
void moveByPath(Position start, Position goal);
void moveOneStep(Position currentPosition, Position nextPosition);
void followRightAdvanced();
void followLeftAdvanced();
void followCenterAdvanced();
void followCenterByDistance(float distance);

void initializeBelif();
void gridLocalization();
void calculatePossiblePositions();
void sensorUpdateBelif();
void motionUpdateBelif();
void normalizeBelif();


extern int mapMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y];
extern int distanceMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y];
extern Position lidarDirections[LIDAR_NUM];
extern Position currentRobotPosition;
extern Position robotStartPosition;
extern Position robotGoalPosition;
extern PositionQueue plannedPath;
extern PositionQueue possiblePositions;
extern bool isLocalizing;
extern bool isCalculatingPosition;