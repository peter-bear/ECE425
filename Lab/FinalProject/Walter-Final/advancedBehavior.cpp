#include "advancedBehavior.h"

int mapMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 0, 99, 99, 0 }, { 0, 0, 0, 0 }, { 0, 99, 99, 0 }, { 0, 99, 0, 0 } };
int distanceMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 0, 99, 99, 0 }, { 0, 0, 0, 0 }, { 0, 99, 99, 0 }, { 0, 99, 0, 0 } };
double beliefMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 } };


int topoMatrix[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 9, 5, 1, 7 }, { 10, 15, 10, 15 }, { 10, 15, 10, 15 }, { 10, 15, 10, 15 } };

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


/**
 * @brief Plan a path from start to goal using broadth first search (wavefront algorithm).
 * @param start the start position
 * @param goal the goal position
 * @return the planned path
 * @note The path is a queue of positions, where the first position is the start position and the last position is the goal position.
 *       The path is the shortest path from start to goal, and the path is planned using broadth first search.
 */
PositionQueue matrixPathPlanning(Position start, Position goal) {
  // initialize the queue
  PositionQueue queue = PositionQueue();
  PositionQueue path = PositionQueue();

  // initialize the distance matrix
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      distanceMatrix[i][j] = mapMatrix[i][j];
    }
  }

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

/**
 * Prints the distance matrix and the path to the serial monitor for debugging purposes.
 * @param path the path to be printed
 */
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

/**
 * @brief Move the robot by following the right wall using advanced PD control.
 *
 * This function reads LIDAR data to determine the presence of walls on the left and right.
 * It then calculates the error between the current distance and the target distance and uses proportional-derivative control to adjust the motor speeds accordingly.
 * The algorithm handles different scenarios such as detecting corners and avoiding obstacles.
 * The function runs indefinitely in a loop, continuously adjusting the motor speeds to follow the right wall.
 * @note The function assumes the existence of certain global variables and functions:
 * - FOLLOW_WALL_BASE_SPEED: Constant for the base speed of the motors.
 * - currentState: Variable to store the current state of the robot.
 * - updateLEDs(): Function to update the LEDs based on the current state.
 * - RPC.call("read_lidars"): Function to read LIDAR data.
 * - leftHasWall(), rightHasWall(): Functions to check for walls.
 * - followLeft(), followRight(): Functions to follow the left or right wall.
 * - WallFollowKp, WallFollowKd: Constants for the proportional and derivative terms in the speed adjustment calculation.
 * - lastError: Variable to store the last error value.
 * - lidar_data: Struct to store LIDAR readings.
 */
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

/**
 * @brief Advanced wall following algorithm to follow the left wall using LIDAR data.
 * 
 * This function continuously reads LIDAR data to determine the presence of walls on the left and right.
 * It then calculates the error between the current distance and the target distance and uses proportional-derivative control to adjust the motor speeds accordingly.
 * The algorithm handles different scenarios such as detecting corners and avoiding obstacles.
 * The function runs indefinitely in a loop, continuously adjusting the motor speeds to follow the left wall.
 * 
 * @note The function assumes the existence of certain global variables and functions:
 * - FOLLOW_WALL_BASE_SPEED: Base speed for the motors.
 * - currentState: Variable to store the current state of the robot.
 * - updateLEDs(): Function to update the LEDs based on the current state.
 * - RPC.call("read_lidars"): Function to read LIDAR data.
 * - leftHasWall(), rightHasWall(): Functions to check for walls.
 * - followCenterAdvanced(): Function to follow the center path.
 * - TARGET_DISTANCE_CM: Desired distance from the wall.
 * - DEADBAND_INNER_CM, DEADBAND_OUTER_CM: Deadband range for distance control.
 * - WallFollowKp, WallFollowKd: Proportional and derivative gains for control.
 * - lastError: Previous error value for derivative control.
 */
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

/**
 * @brief Move the robot by following the center path using advanced PD control.
 *
 * This function reads LIDAR data to determine the presence of walls on the left, right,
 * and front. Depending on the presence of walls, it calls appropriate functions to
 * follow the left or right wall, or to turn around if a wall is detected in front.
 * The center error is calculated as the difference between the left and right LIDAR
 * readings. A speed adjustment is computed based on the error and its difference from
 * the last error, and the speeds of the stepper motors are adjusted accordingly.
 * The function runs indefinitely in a loop, continuously adjusting the motor speeds
 * to follow the center path.
 * @note The function assumes the existence of certain global variables and functions:
 * - FOLLOW_WALL_BASE_SPEED: Constant for the base speed of the motors.
 * - currentState: Variable to store the current state of the robot.
 * - updateLEDs(): Function to update the LEDs based on the current state.
 * - RPC.call("read_lidars"): Function to read LIDAR data.
 * - leftHasWall(), rightHasWall(), frontHasWall(): Functions to check for walls.
 * - followLeft(), followRight(): Functions to follow the left or right wall.
 * - WallFollowKd: Constant for the derivative term in the speed adjustment calculation.
 * - lastError: Variable to store the last error value.
 * - lidar_data: Struct to store LIDAR readings.
 */
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

/**
 * @brief Move the robot by a specified distance following the center path.
 *
 * This function first resets the encoder value and calculates the encoder value
 * for the specified distance. It then enters a loop where it continuously moves
 * the robot forward by following the center path with the followCenterAdvanced()
 * function until the encoder value reaches the calculated value.
 *
 * @param distance The distance to move the robot in inches.
 * @note The function assumes the existence of certain global variables and functions:
 * - resetEncoder(): Function to reset the encoder value.
 * - length2Steps(float length): Function to convert a distance in inches to steps.
 * - encoderRatio: Constant to convert steps to encoder value.
 * - encoder: Array to store the current encoder value.
 * - FOLLOW_WALL_BASE_SPEED: Constant for the base speed of the motors.
 * - stepperLeft, stepperRight: Instances of stepper motor control.
 * - followLeftAdvanced(), followRightAdvanced(), followCenterAdvanced():
 *   Functions to follow the left, right, or center path.
 */
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

/**
 * @brief Moves the robot from the current position to the next position in the planned path.
 *
 * @param currentPosition The current position of the robot.
 * @param nextPosition The position the robot should move to.
 *
 * This function first calculates the direction the robot needs to turn to face the next position.
 * It then turns the robot to that direction using the goToAngle() function.
 * After that, it moves the robot to the next position using the followCenterByDistance() function.
 * Finally, it updates the current robot position and direction.
 */
void moveOneStep(Position currentPosition, Position nextPosition) {
  float nextDirection = getRobotDirection(nextPosition.x - currentPosition.x, nextPosition.y - currentPosition.y);

  // turn the robot to the next direction
  float turnAngle = currentRobotDirection - nextDirection;
  goToAngle(turnAngle);

  // move the robot to the next position
  followCenterByDistance(18.0);

  // update the current robot position and direction
  currentRobotDirection = nextDirection;
}

/**
 * @brief Moves the robot by following the planned path from start to goal.
 * @param start the start position of the path
 * @param goal the goal position of the path
 * @note This function will call the matrixPathPlanning function to plan the path,
 *       and then call the moveOneStep function to move the robot to the next
 *       position until it reaches the goal position. It will also publish the
 *       data when the robot is moving.
 */
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

/**
 * Prints the belief matrix to the serial monitor for debugging purposes.
 */
void debugBelif() {
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      Serial.print(beliefMatrix[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

/**
 * Initializes the belief matrix by assigning equal probability to all positions with a 0 in the map matrix.
 * All other positions are assigned a probability of 0.
 */
void initializeBelif() {
  // count all the zeros in the map matrix
  int zeroCount = 0;
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      if (mapMatrix[i][j] == 0) {
        zeroCount += 1;
      }
    }
  }

  double zeroProbability = 1.0 / zeroCount;
  for (int i = 0; i < MATRIX_SIZE_X; i++) {
    for (int j = 0; j < MATRIX_SIZE_Y; j++) {
      if (mapMatrix[i][j] == 0) {
        beliefMatrix[i][j] = zeroProbability;
      } else {
        beliefMatrix[i][j] = 0;
      }
    }
  }
}

/**
 * Normalizes the belief matrix to ensure that the sum of all elements is 1.
 * This is necessary to ensure that the belief matrix represents a valid probability distribution.
 * The function loops through all the elements of the belief matrix, sums them up, and then divides each element by the sum.
 */
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

/**
 * Updates the belief matrix based on the motion model.
 * This function takes the motion probabilities and the current belief matrix, and updates the belief matrix by multiplying the motion probabilities with the current belief at each position.
 * The function then normalizes the belief matrix.
 */
void motionUpdateBelif() {
  double new_belief[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 } };
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

  // copy the new belief matrix to the old one
  for(int i = 0; i < MATRIX_SIZE_X; i++){
    for(int j = 0; j < MATRIX_SIZE_Y; j++){
      beliefMatrix[i][j] = new_belief[i][j];
    }
  }

  normalizeBelif();
}

/**
 * Updates the belief matrix based on sensor data.
 * This function takes the sensor data, counts the obstacle number, and compares it with the possible obstacle number.
 * The function then updates the belief matrix by multiplying the sensor probability with the current belief at each position.
 * Finally, the function normalizes the belief matrix.
 */
void sensorUpdateBelif() {
  // loop through all lidar directions and count the obstacle number
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  int sensorObstacleNum = leftHasWall() + rightHasWall() + frontHasWall() + backHasWall();

  double new_belief[MATRIX_SIZE_X][MATRIX_SIZE_Y] = { { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 } };

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
        sensorProbability = max(0.1, min(0.9 , exp(-abs(sensorObstacleNum - obstacleNum)) * 0.9));
      }

      // update the belief matrix
      new_belief[i][j] = sensorProbability * beliefMatrix[i][j];
    }
  }

  for(int i = 0; i < MATRIX_SIZE_X; i++){
    for(int j = 0; j < MATRIX_SIZE_Y; j++){
      beliefMatrix[i][j] = new_belief[i][j];
    }
  }

  normalizeBelif();
}

bool isLocalizing = false;
bool isCalculatingPosition = false;
bool findTheLocation = false;


/**
 * Calculates possible positions based on the probability matrix.
 * This function loops through the probability matrix, finds the highest probability position, and updates the possible positions queue.
 * If only one possible position is found, the robot is considered to be localized and the function ends.
 * The function also checks for incoming messages and stops if the STOP command is received.
 * The function publishes the robot position and lidar data after each iteration.
 */
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
    findTheLocation = true;
  }

  isCalculatingPosition = false;
}

/**
 * Explores the grid environment using lidar sensor data.
 * This function checks the robot's surroundings and makes decisions based on the presence or absence of walls in different directions.
 * It uses a series of if-else statements to determine the robot's next action, such as moving forward, spinning left or right, or turning at a corner.
 */
void exploreGrid() {
    lidar_data = RPC.call("read_lidars").as<struct lidar>();
    
    // 3 walls back
    if(leftHasWall() && rightHasWall() && !frontHasWall() && backHasWall()){
      // forward(GRID_RATIO, defaultStepSpeed);
      followCenterByDistance(GRID_RATIO);
    }
    // 3 walls front
    else if(leftHasWall() && rightHasWall() && frontHasWall() && !backHasWall()){
      spin(TO_LEFT, 180, defaultStepSpeed);
      // forward(GRID_RATIO, defaultStepSpeed);
      followCenterByDistance(GRID_RATIO);
    }
    // left 1 wall
    else if(leftHasWall() && !rightHasWall() && !frontHasWall() && !backHasWall()){
      // use random, either go forward or turn right
      if(random(0, 2) == 0){
        // forward(GRID_RATIO, defaultStepSpeed);
        followCenterByDistance(GRID_RATIO);
      }else{
        spin(TO_RIGHT, 90, defaultStepSpeed);
        // forward(GRID_RATIO, defaultStepSpeed);
        followCenterByDistance(GRID_RATIO);
      }
    }
    // right 1 wall
    else if(!leftHasWall() && rightHasWall() && !frontHasWall() && !backHasWall()){
      // use random, either go forward or turn left
      if(random(0, 2) == 0){
        // forward(GRID_RATIO, defaultStepSpeed);
        followCenterByDistance(GRID_RATIO);
      }else{
        spin(TO_LEFT, 90, defaultStepSpeed);
        // forward(GRID_RATIO, defaultStepSpeed);
        followCenterByDistance(GRID_RATIO);
      }  
    }
    // front 1 wall, T shape
    else if(!leftHasWall() && !rightHasWall() && frontHasWall() && !backHasWall()){
      // either turn left or right
      if(random(0, 2) == 0){
        spin(TO_LEFT, 90, defaultStepSpeed);
        // forward(GRID_RATIO, defaultStepSpeed);
        followCenterByDistance(GRID_RATIO);
      } else{
        spin(TO_RIGHT, 90, defaultStepSpeed);
        // forward(GRID_RATIO, defaultStepSpeed);
        followCenterByDistance(GRID_RATIO);
      }
    }
    // right corner
    else if(!leftHasWall() && rightHasWall() && frontHasWall() && !backHasWall()){
      spin(TO_LEFT, 90, defaultStepSpeed);
      // forward(GRID_RATIO, defaultStepSpeed);
      followCenterByDistance(GRID_RATIO);
    }
    // left corner
    else if(leftHasWall() && !rightHasWall() && frontHasWall() && !backHasWall()){
      spin(TO_RIGHT, 90, defaultStepSpeed);
      // forward(GRID_RATIO, defaultStepSpeed);
      followCenterByDistance(GRID_RATIO);
    }
    else {
      // forward(GRID_RATIO, defaultStepSpeed);
      followCenterByDistance(GRID_RATIO);
    }
}




/**
 * Grid localization function that uses lidar sensor data to update the belief matrix, explore the grid environment, and determine the robot's position.
 * It continues to run until the robot has found its position or the STOP state is received.
 * During each iteration, it polls for MQTT messages, updates the belief matrix based on the robot's motion and sensor data, and calculates the possible positions based on the belief matrix.
 * It then publishes the possible positions to the MQTT broker and moves the robot to explore the grid environment.
 */
void gridLocalization() {
  Serial.println("\n\n Grid Localization");

  while(!findTheLocation && currentState != STOP){
    mqttClient.poll();
    debugBelif();

    exploreGrid();
    motionUpdateBelif();
    sensorUpdateBelif();
    calculatePossiblePositions();
    publishData();
  }
  currentState = STOP;
  stopMove();
}


/**
 * Calculates the topology number based on the lidar sensor data.
 * It represents the status of the walls around the robot as a binary number.
 * The least significant bit is the front wall, the second least significant bit is the right wall, the third least significant bit is the back wall, and the most significant bit is the left wall.
 * The function returns the calculated topology number as an integer.
 */
int calculateTopoNum(){
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  int topoNum = 0;
  if(frontHasWall()){
    topoNum += 1;
  }
  if(leftHasWall()){
    topoNum += 8;
  }
  if(rightHasWall()){
    topoNum += 2;
  }
  if(backHasWall()){
    topoNum += 4;
  }
  return topoNum;
}


/**
 * Calculates possible positions based on the topology number.
 * This function clears the current list of possible positions and iterates over the topology matrix.
 * It enqueues all positions where the matrix value matches the calculated topology number, indicating potential robot locations.
 */

void calculateTopoPossiblePositions(){
  isCalculatingPosition = true;
  // loop the matrix and get the highest probability position
  possiblePositions.clear();

  for(int i = 0; i < MATRIX_SIZE_X; i++){
    for(int j = 0; j < MATRIX_SIZE_Y; j++){
      if(topoMatrix[i][j] == calculateTopoNum()){
        // clear the queue      
        possiblePositions.enqueue(i, j);
      }
    }
  }

  isCalculatingPosition = false;
}


/**
 * This function implements the topology localization algorithm. It first
 * calculates the topology number of the current position, then identifies the
 * possible positions in the topology matrix with the same topology number.
 * If only one possible position is found, the robot is considered to be
 * localized and the function ends. Otherwise, the robot moves to the next
 * position in the grid and repeats the process. The function also checks
 * for incoming messages and stops if the STOP command is received.
 * The function publishes the robot position and lidar data after each iteration.
 */
void topologyLocalization(){
  Serial.println("\n\n Topology Localization");

  while(!findTheLocation && currentState != STOP){
    mqttClient.poll();
  
    // calculate the topology number
    int topoNum = calculateTopoNum();

    // identify the locations
    calculateTopoPossiblePositions();

    // if only one possible position
    if(possiblePositions.length() == 1){
      currentRobotPosition = possiblePositions.getByIndex(0);
      findTheLocation = true;
    }else{
      exploreGrid();
    }
    
    publishData();
  }
  currentState = STOP;
  stopMove();
}