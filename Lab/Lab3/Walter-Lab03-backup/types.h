#pragma once
#include "RPC.h"

enum RobotState {
    NO_WALL,
    FOLLOWING_LEFT,
    FOLLOWING_RIGHT,
    FOLLOWING_CENTER,
    TOO_CLOSE,
    TOO_FAR,
    CLOSE_TO_RIGHT,
    CLOSE_TO_LEFT,
    TURNING_CORNER,
    RANDOM_WANDER,
    INSIDE_CORNER,
    COLLIDE_BEHAVIOR,
    RUNAWAY_BEHAVIOR,
    FOLLOW_BEHAVIOR
};

struct lidar {
    int front;
    int back;
    int left;
    int right;
    MSGPACK_DEFINE_ARRAY(front, back, left, right)
};

struct sonar {
    int left;
    int right;
    MSGPACK_DEFINE_ARRAY(left, right)
};

extern RobotState currentState;
extern volatile long encoder[2];
extern int lastSpeed[2];
extern int accumTicks[2];
extern struct lidar lidarData;
extern struct sonar sonarData;
extern unsigned long lastMeasureTime;
extern double lastError;
extern struct lidar lidar_data_M7_read;