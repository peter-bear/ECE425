#include "advancedBehavior.h"

void matrixPathPlanning(int** matrix, int start, int end, int size) {
}


// used to store the sensor data around the robot, front, back, left, right, 0 represent no obstacle, 1 represent obstacle
int obstacleAround[4] = { 0, 0, 0, 0 };

int* getObstacleData(struct lidar lidar_data) {
  int currentObstacleAround[4] = { 0, 0, 0, 0 };
  if (frontHasObstacle()) {
    currentObstacleAround[0] = 1;
  } else {
    currentObstacleAround[0] = 0;
  }

  if (backHasObstacle()) {
    currentObstacleAround[1] = 1;
  } else {
    currentObstacleAround[1] = 0;
  }

  if (leftHasObstacle()) {
    currentObstacleAround[2] = 1;
  } else {
    currentObstacleAround[2] = 0;
  }

  if (rightHasObstacle()) {
    currentObstacleAround[3] = 1;
  } else {
    currentObstacleAround[3] = 0;
  }
  return currentObstacleAround;
}

void exploreOpenSpaceStart() {
  // check the lidar data to see the space around the robot
  for (int i = 0; i < 4; i++) {
    if (obstacleAround[i] == 0) {
      // move the robot to the open space
      switch (i) {
        case 0:
          // move forward
          break;
        case 1:
          // turn 180 degrees
          spin(TURN_LEFT, 180, defaultStepSpeed);
          break;
        case 2:
          // turn 90 left
          spin(TURN_LEFT, 90, defaultStepSpeed);
          break;
        case 3:
          // turn 90 right
          spin(TURN_RIGHT, 90, defaultStepSpeed);
          break;
      }
      break;
    }
  }

  // move the robot to the open space
  stepperLeft.setSpeed(defaultStepSpeed);
  stepperRight.setSpeed(defaultStepSpeed);

  currentSLAMState = EXPLORING;
}

void exploring() {
  // check the lidar data to see if there is any obstacle around the robot
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  int* currentObstacleAround = getObstacleData(lidar_data);

  // check the obstacle data to see if there is any change
  for (int i = 0; i < 4; i++) {
    if (obstacleAround[i] != currentObstacleAround[i]) {
      // update the obstacle data
      obstacleAround[i] = currentObstacleAround[i];
      currentSLAMState = UPDATE_MAP;

      // data change, stop the robot
      stepperLeft.setSpeed(0);
      stepperRight.setSpeed(0);
      break;
    }
  }
}

void detect_obstacle() {
  // check the lidar data to see if there is any obstacle around the robot
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  int* currentObstacleAround = getObstacleData(lidar_data);

  // update the obstacle data
  for (int i = 0; i < 4; i++) {
    obstacleAround[i] = currentObstacleAround[i];
  }

  currentSLAMState = EXPLORE_START;
}

void updateMap() {
  // update the map

  currentSLAMState = DETECT_OBSTACLE;
}

void SLAM() {
  switch (currentSLAMState) {
    case DETECT_OBSTACLE:
      detect_obstacle();
      break;
    case EXPLORE_START:
      exploreOpenSpaceStart();
      break;
    case EXPLORING:
      exploring();
      break;
    case UPDATE_MAP:
      updateMap();
      break;
    default:
      break;
  }
}