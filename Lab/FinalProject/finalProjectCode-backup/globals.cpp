#include "types.h"

RobotState currentState;
volatile long encoder[2];
int lastSpeed[2];
int accumTicks[2];


// data used in M4 Core
struct lidar lidarData;
struct sonar sonarData;

unsigned long lastMeasureTime = 0;
double lastError = 0;

// lidar data used in M7 Main Core
struct lidar lidar_data_M7_read;