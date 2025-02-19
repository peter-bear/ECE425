#include "advancedBehavior.h"

int mapMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 0, 99, 99, 0 }, { 0, 0, 0, 0 }, { 0, 99, 99, 0 }, { 0, 99, 0, 0 } };
int distanceMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 0, 99, 99, 0 }, { 0, 0, 0, 0 }, { 0, 99, 99, 0 }, { 0, 99, 0, 0 } };
double beliefMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y];

// move lidarDirections
Position lidarDirections[LIDAR_NUM] = {
  { 0, 1 },
  { 0, -1 },
  { -1, 0 },
  { 1, 0 }
};  // right left up down

Position moveDirections[MOVE_DIRECTIONS] = { { 0, 0 }, { 0, 1 }, { 0, -1 }, { -1, 0 }, { 1, 0 } };  // stay right left up down

// move posibilities for each direction stay, up, down, left, right
double movePosibilities[MOVE_DIRECTIONS] = {0.1, 0.225, 0.225, 0.225, 0.225};

float currentRobotDirection = 0.0;
Position currentRobotPosition = {-1, -1};

Position robotStartPosition;
Position robotGoalPosition;

// store the planned path
PositionQueue plannedPath;

// store the possible positions
PositionQueue possiblePositions;

// use degree to represent the directions
float getRobotDirection(int x, int y) {
  if (x == 0 && y == 1) {
    return 90;
  } else if (x == 0 && y == -1) {
    return -90;
  } else if (x == 1 && y == 0) {
    return 180;
  } else if (x == -1 && y == 0) {
    return 0.0;
  }
}

// implement wavefront algorithm to find the shortest path
// x represents the row, y represents the column
PositionQueue matrixPathPlanning(Position start, Position goal) {
  // initialize the queue
  PositionQueue queue = PositionQueue();
  PositionQueue path = PositionQueue();

  // initialize the distance matrix
  memcpy(distanceMatrix, mapMatrix, sizeof(mapMatrix));

  if (distanceMatrix[goal.x][goal.y] == 99) {
    Serial.println("The start position is not reachable!");
    return path;
  }

  // add the goal position to the queue
  queue.enqueue(goal.x, goal.y);
  distanceMatrix[goal.x][goal.y] = 1;

  // loop from the goal to use broadth first search
  while (!queue.isEmpty()) {
    Position current = queue.dequeue();
    for (int i = 0; i < LIDAR_NUM; i++) {
      Position next = { current.x + lidarDirections[i].x, current.y + lidarDirections[i].y };
      if (next.x >= 0 && next.x < MATRIX_SIZE_X && next.y >= 0 && next.y < MATRIX_SIZE_Y && distanceMatrix[next.x][next.y] == 0) {
        distanceMatrix[next.x][next.y] = distanceMatrix[current.x][current.y] + 1;
        queue.enqueue(next.x, next.y);
      }
    }
  }

  // check if the start position is reachable
  if (distanceMatrix[start.x][start.y] == 0) {
    Serial.println("The start position is not reachable!");
  } else {
    // find the shortest path
    Position current = start;
    while (current.x != goal.x || current.y != goal.y) {
      // loop through the lidarDirections
      for (int i = 0; i < LIDAR_NUM; i++) {
        Position next = { current.x + lidarDirections[i].x, current.y + lidarDirections[i].y };
        // check if the next position is reachable
        if (next.x >= 0 && next.x < MATRIX_SIZE_X && next.y >= 0 && next.y < MATRIX_SIZE_Y && distanceMatrix[next.x][next.y] == distanceMatrix[current.x][current.y] - 1) {
          path.enqueue(next.x, next.y);
          current = next;
          break;
        }
      }
    }
  }

  // debugPath(path);

  return path;
}

void debugPath(PositionQueue path) {
  // print the distanceMatrix
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      Serial.print(distanceMatrix[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }

  // print the path
  for (int i = 0; i < path.length(); i++) {
    Position current = path.getByIndex(i);
    Serial.print("(");
    Serial.print(current.x);
    Serial.print(", ");
    Serial.print(current.y);
    Serial.println(")");
  }
}

void followRightAdvanced() {
  currentState = FOLLOWING_RIGHT;
  updateLEDs();

  if (leftHasWall()) {
    followCenterAdvanced();
  }

  if (!rightHasWall()) {
    return;
  }

  // Calculate error (how far we are from desired distance)
  double error = lidar_data.right - TARGET_DISTANCE_CM;
  double error_diff = error - lastError;

  // Store current values for next iteration
  lastError = error;

  // If within deadband, drive straight
  if (lidar_data.right >= DEADBAND_INNER_CM && lidar_data.right <= DEADBAND_OUTER_CM) {
    currentState = INSIDE_DEADBAND;
    updateLEDs();
    stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
    stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
  } else {
    // Calculate speed adjustment based on error
    int speedAdjustment = (int)(WallFollowKp * error + WallFollowKd * error_diff);

    // If too far from wall (positive error)
    if (error > 0) {
      currentState = TOO_FAR;
      updateLEDs();

      // Turn towards wall - slow down right motor
      stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
      stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
    }
    // If too close to wall (negative error)
    else {
      currentState = TOO_CLOSE;
      updateLEDs();

      // Turn away from wall - slow down left motor
      stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
      stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
    }
  }
}

void followLeftAdvanced() {
  currentState = FOLLOWING_LEFT;
  updateLEDs();

  if (rightHasWall()) {
    followCenterAdvanced();
  }

  if (!leftHasWall()) {
    return;
  }

  // Calculate error (how far we are from desired distance)
  double error = lidar_data.left - TARGET_DISTANCE_CM;
  double error_diff = error - lastError;

  // Store current values for next iteration
  lastError = error;

  // If within deadband, drive straight
  if (lidar_data.left >= DEADBAND_INNER_CM && lidar_data.left <= DEADBAND_OUTER_CM) {
    currentState = INSIDE_DEADBAND;
    updateLEDs();
    stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
    stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
  } else {
    // Calculate speed adjustment based on error
    int speedAdjustment = (int)(WallFollowKp * error + WallFollowKd * error_diff);

    // If too far from wall (positive error)
    if (error > 0) {
      currentState = TOO_FAR;
      updateLEDs();

      // Turn towards wall - slow down left motor
      stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
      stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
    }
    // If too close to wall (negative error)
    else {
      currentState = TOO_CLOSE;
      updateLEDs();

      // Turn away from wall - slow down right motor
      stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
      stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
    }
  }
}

void followCenterAdvanced() {
  currentState = FOLLOWING_CENTER;
  updateLEDs();

  if (leftHasWall() && !rightHasWall()) {
    followLeft();
  } else if (!leftHasWall() && rightHasWall()) {
    followRight();
  } else if (!leftHasWall() && !rightHasWall()) {
    return;
  }

  // Calculate center error (positive means closer to left wall)
  float error = lidar_data.left - lidar_data.right;
  float error_diff = error - lastError;

  double speedAdjustment = 0.9 * error + WallFollowKd * error_diff;

  if (abs(error) <= 3) {
    speedAdjustment = 0;
  }

  stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - speedAdjustment);
  stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED + speedAdjustment);

  lastError = error;
}


void followCenterByDistance(float distance) {
  // reset the encoder value
  resetEncoder();

  // calculate the encoder value for the distance
  double distanceSteps = length2Steps(distance);
  long encoderValue = distanceSteps / encoderRatio;

  while (encoder[LEFT_ENCODER] < encoderValue) {
    stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
    stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);

    lidar_data = RPC.call("read_lidars").as<struct lidar>();

    if (leftHasWall() && !rightHasWall()) {
      followLeftAdvanced();
    } else if (!leftHasWall() && rightHasWall()) {
      followRightAdvanced();
    } else if (leftHasWall() && rightHasWall()) {
      followCenterAdvanced();
    }

    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

void moveOneStep(Position currentPosition, Position nextPosition) {
  float nextDirection = getRobotDirection(nextPosition.x - currentPosition.x, nextPosition.y - currentPosition.y);

  // turn the robot to the next direction
  float turnAngle = currentRobotDirection - nextDirection;
  // Serial.println(turnAngle);
  goToAngle(turnAngle);

  // move the robot to the next position
  followCenterByDistance(18.0);

  // update the current robot position and direction
  currentRobotDirection = nextDirection;
}

void moveByPath(Position start, Position goal) {
  currentRobotPosition = start;
  plannedPath = matrixPathPlanning(start, goal);
  for (int i = 0; i < plannedPath.length(); i++) {
    Position nextPosition = plannedPath.getByIndex(i);
    moveOneStep(currentRobotPosition, nextPosition);
    publishData();
    currentRobotPosition = nextPosition;
  }
  currentState = STOP;
}

void initializeBelif() {
  memset(beliefMatrix, 0, sizeof(beliefMatrix));
}

void normalizeBelif() {
  double sum = 0;
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      sum += beliefMatrix[i][j];
    }
  }
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      beliefMatrix[i][j] /= sum;
    }
  }
}

void motionUpdateBelif() {
  double new_belief[MATRIX_SIZE_X][MATRIX_SIZE_Y];
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      if(mapMatrix[i][j] == 99) {
        new_belief[i][j] = 0;
        continue;
      }

      // loop through all possible directions
      for(int k = 0; k < MOVE_DIRECTIONS; k++) {
        Position next = { i + moveDirections[k].x, j + moveDirections[k].y };
        if (next.x >= 0 && next.x < MATRIX_SIZE_X && next.y >= 0 && next.y < MATRIX_SIZE_Y && mapMatrix[next.x][next.y] != 99) {
          new_belief[i][j] += movePosibilities[k] * beliefMatrix[next.x][next.y];
        }
      }
    }
  }

  memcpy(beliefMatrix, new_belief, sizeof(beliefMatrix));
  normalizeBelif();
}

void sensorUpdateBelif() {
  // loop through all lidar directions and count the obstacle number
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  int sensorObstacleNum = leftHasWall() + rightHasWall() + frontHasWall() + backHasWall();

  double new_belief[MATRIX_SIZE_X][MATRIX_SIZE_Y];

  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      if(mapMatrix[i][j] == 99 || beliefMatrix[i][j] <= 0) {
        new_belief[i][j] = 0;
        continue;
      }

      // check the possible obstacle number
      int obstacleNum = 0;
      for(int k = 0; k < LIDAR_NUM; k++) {
        Position next = { i + lidarDirections[k].x, j + lidarDirections[k].y };
        // check if the next position is out of the map or is an obstacle
        if (next.x < 0|| next.x >= MATRIX_SIZE_X || next.y < 0 || next.y >= MATRIX_SIZE_Y || mapMatrix[next.x][next.y] == 99) {
          obstacleNum += 1;
        }
      }
      
      double sensorProbability = 0;
      // compare
      if (sensorObstacleNum == obstacleNum) {
        sensorProbability = 0.9;
      } else {
        sensorProbability = max(0.0, min(0.9 , abs(sensorObstacleNum - obstacleNum) * 0.9));
      }

      // update the belief matrix
      new_belief[i][j] = sensorProbability * beliefMatrix[i][j];
    }
  }

  memcpy(beliefMatrix, new_belief, sizeof(beliefMatrix));
  normalizeBelif();
}

bool isLocalizing = false;
bool isCalculatingPosition = false;

void calculatePossiblePositions(){
  isCalculatingPosition = true;
  double maxProbability = 0;

  // loop the matrix and get the highest probability position
  for(int i = 0; i < MATRIX_SIZE_X; i++){
    for(int j = 0; j < MATRIX_SIZE_Y; j++){
      if(beliefMatrix[i][j] > maxProbability){
        // clear the queue
        possiblePositions.clear();

        // update the max probability
        maxProbability = beliefMatrix[i][j];
        possiblePositions.enqueue(i, j);
      }else if(beliefMatrix[i][j] == maxProbability){
        possiblePositions.enqueue(i, j);
      }
    }
  }

  // only one possible position
  if(possiblePositions.length() == 1){
    currentRobotPosition = possiblePositions.getByIndex(0);
  }

  isCalculatingPosition = false;
}


void gridLocalization() {
  motionUpdateBelif();
  sensorUpdateBelif();
  calculatePossiblePositions();
}