#include "behaviors.h"
#include "positionQueue.h"

#define MATRIX_SIZE_X 5
#define MATRIX_SIZE_Y 5

void debugPath(PositionQueue plannedPath);
PositionQueue matrixPathPlanning(Position start, Position goal);
void moveByPath(Position start, Position goal);
void moveOneStep(Position currentPosition, Position nextPosition);
void SLAM();

extern int matrix[MATRIX_SIZE_X][MATRIX_SIZE_Y];
extern int distanceMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y];
extern Position lidarDirections[LIDAR_NUM];
extern Position currentRobotPosition;