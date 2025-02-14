#include "behaviors.h"


// random move maximum values
const int maxTurnAngle = 90;       // Maximum turn angle in degrees
const int maxDistanceMove = 12.0;  // Maximum move distance in inches

// prints the sensor data
void print_sensor_data() {
  Serial.print("lidar: ");
  Serial.print(lidar_data.front);
  Serial.print(", ");
  Serial.print(lidar_data.back);
  Serial.print(", ");
  Serial.print(lidar_data.left);
  Serial.print(", ");
  Serial.print(lidar_data.right);
  Serial.println();
  Serial.print("sonar: ");
  Serial.print(sonar_data.left);
  Serial.print(", ");
  Serial.print(sonar_data.right);
  Serial.println();
}



/**
 * Prints the random values generated for the angle and distance.
 * @param randAngle the random angle generated
 * @param randDistance the random distance generated
 */
void printRandomValues(int randAngle, int randDistance) {
  Serial.print("Random Values:\n\tAngle: ");
  Serial.print(randAngle);
  Serial.print("\tDistance: ");
  Serial.println(randDistance);
}


/**
 * Generates a random angle and distance for the robot to move.
 * The robot will turn to the random angle, then move forward the random distance.
 */
void randomWander() {
  currentState = RANDOM_WANDER;
  updateLEDs();

  int randAngle = random(-maxTurnAngle, maxTurnAngle);
  int randDistance = random(maxDistanceMove);
  randDistance = length2Steps(randDistance);

  goToAngle(randAngle);

  stepperLeft.move(randDistance);
  stepperRight.move(randDistance);

  stepperLeft.setSpeed(defaultStepSpeed);
  stepperRight.setSpeed(defaultStepSpeed);
}


// converts cm to inches
double cm2inch(int cm) {
  return 0.393701 * cm;
}



// collide behavior
void collideBehavior() {
  currentState = COLLIDE_BEHAVIOR;
  updateLEDs();
  stopMove();
  delay(50);  // Small delay to prevent CPU hogging
}

// check if there is an obstacle in the front of the robot
bool frontHasObstacle() {
  return lidar_data.front < OBSTACLE_THRESHOLD;
}

// check if there is an obstacle in the back of the robot
bool backHasObstacle() {
  return lidar_data.back < OBSTACLE_THRESHOLD;
}

// check if there is an obstacle on the left of the robot
bool leftHasObstacle() {
  return lidar_data.left < OBSTACLE_THRESHOLD;
}

// check if there is an obstacle on the right of the robot
bool rightHasObstacle() {
  return lidar_data.right < OBSTACLE_THRESHOLD;
}

// check if there is an obstacle in front of the robot
bool checkFrontObstacle() {
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  return frontHasObstacle();
}

// check if there is an obstacle around the robot
bool isCloseObstacle() {
  // Read sensor data
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  sonar_data = RPC.call("read_sonars").as<struct sonar>();

  // Return true if any sensor detects a close obstacle
  return frontHasObstacle() || backHasObstacle() || leftHasObstacle() || rightHasObstacle();
  // return (sonar_data.left < OBSTACLE_THRESHOLD && sonar_data.left != 0) ||
  //      (sonar_data.right < OBSTACLE_THRESHOLD && sonar_data.right != 0);
}



// Runaway behavior
const int forwardDistance = 10;

/**
 * @brief Executes the runaway behavior for the robot.
 *
 * This function controls the robot's movement based on sensor data to avoid obstacles.
 * It reads lidar data, calculates the necessary turn angle, and moves the robot accordingly.
 *
 * The function performs the following steps:
 * 1. Turns on the yellow LED and turns off the red and green LEDs.
 * 2. Reads lidar data using an RPC call.
 * 3. Calculates the x and y distances in inches based on the lidar data.
 * 4. Determines the turn angle based on the presence of obstacles detected by the sensors.
 * 5. Moves the robot to the calculated angle and moves forward if the turn angle is valid.
 * 6. Stops the robot if the turn angle is invalid.
 *
 * The function uses the following helper functions:
 * - frontHasObstacle(): Checks if there is an obstacle in front of the robot.
 * - backHasObstacle(): Checks if there is an obstacle behind the robot.
 * - leftHasObstacle(): Checks if there is an obstacle to the left of the robot.
 * - rightHasObstacle(): Checks if there is an obstacle to the right of the robot.
 * - getTurnAngle(double x_inch, double y_inch): Calculates the turn angle based on x and y distances.
 * - goToAngle(double angle): Turns the robot to the specified angle.
 * - forward(double distance, int speed): Moves the robot forward by the specified distance at the given speed.
 * - stopMove(): Stops the robot's movement.
 */
void runawayBehavior() {
  currentState = RUNAWAY_BEHAVIOR;
  updateLEDs();

  lidar_data = RPC.call("read_lidars").as<struct lidar>();

  int x = lidar_data.front - lidar_data.back;
  int y = lidar_data.left - lidar_data.right;

  double x_inch = runawayPropotion * cm2inch(x);
  double y_inch = runawayPropotion * cm2inch(y);

  // print_sensor_data(lidar_data, sonar_data);

  double turnAngle = 0;

  if (!frontHasObstacle() && !backHasObstacle() && leftHasObstacle() && rightHasObstacle()) {
    turnAngle = 0;
  } else if (!leftHasObstacle() && !rightHasObstacle() && frontHasObstacle() && backHasObstacle()) {
    turnAngle = 90.0;
  } else if (frontHasObstacle() && backHasObstacle() && leftHasObstacle() && rightHasObstacle()) {
    turnAngle = 3600;
  } else if (!frontHasObstacle() && !backHasObstacle() && !leftHasObstacle() && !rightHasObstacle()) {
    return;
  } else {
    turnAngle = getTurnAngle(x_inch, y_inch);
  }

  if (turnAngle <= 360) {
    goToAngle(turnAngle);
    forward(forwardDistance, defaultStepSpeed);
  } else {
    stopMove();
  }
}




// Check if the robot is within the obstacle margin
bool isWithinObstacleMargin(double frontDistance) {
  return frontDistance < OBSTACLE_THRESHOLD + OBSTACLE_MARGIN && frontDistance > OBSTACLE_THRESHOLD - OBSTACLE_MARGIN;
}

/**
 * @brief Executes the follow behavior for a robot using LIDAR data.
 *
 * This function controls the robot to follow an object by maintaining a certain distance
 * from it. It uses LIDAR data to determine the distance to the object and adjusts the
 * robot's movement accordingly. The function continuously reads LIDAR data and moves
 * the robot forward or backward to maintain the desired distance.
 *
 * The function performs the following steps:
 * 1. Turns on the red and green LEDs and turns off the yellow LED.
 * 2. Reads the initial LIDAR data and calculates the front error in centimeters and inches.
 * 3. Enters an infinite loop where it:
 *    - Checks if the object is within the obstacle margin.
 *    - If the object is not within the obstacle margin and is within the maximum LIDAR distance:
 *      - Calculates the movement distance based on the proportional control.
 *      - Moves the robot forward if the front error is positive, or backward if the front error is negative.
 *    - Reads the updated LIDAR data and recalculates the front error.
 *    - Delays for 20 milliseconds before the next iteration.
 *
 * The function exits the loop and stops the behavior if the object is beyond the maximum LIDAR distance.
 */
void followBehavior() {
  currentState = FOLLOW_BEHAVIOR;
  updateLEDs();

  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  int front_error_cm = lidar_data.front - OBSTACLE_THRESHOLD;
  double front_error_inch = cm2inch(front_error_cm);

  while (true) {
    if (!isWithinObstacleMargin(lidar_data.front)) {
      if (lidar_data.front >= MAX_LIDAR_DISTANCE)
        break;

      double moveDistance = abs(followPropotion * front_error_inch);

      if (front_error_cm > 0) {
        forward(moveDistance, FOLLOW_SPEED);
      } else if (front_error_cm < 0) {
        reverse(moveDistance, FOLLOW_SPEED);
      }
    }

    lidar_data = RPC.call("read_lidars").as<struct lidar>();
    front_error_cm = lidar_data.front - OBSTACLE_THRESHOLD;
    front_error_inch = cm2inch(front_error_cm);
    delay(20);
  }
}


/**
 * @brief Executes the smart wander behavior for a robot.
 *
 * This function makes the robot move in a random direction and distance,
 * while continuously checking for obstacles. If an obstacle is detected,
 * the robot will stop, perform a collision avoidance behavior, and then
 * resume wandering.
 *
 * The function performs the following steps:
 * 1. Generates a random angle and distance for the robot to move.
 * 2. Converts the distance to steps for the stepper motors.
 * 3. Rotates the robot to the random angle.
 * 4. Moves the robot forward by the random distance.
 * 5. Sets the speed of the stepper motors to the default speed.
 * 6. Continuously runs the stepper motors and checks for obstacles.
 * 7. If an obstacle is detected, stops the motors, performs collision
 *    avoidance, and resumes wandering.
 * 8. Turns on the green LED while moving, and turns off other LEDs.
 * 9. Adds a small delay to allow sensor readings to be processed.
 */
void smartWanderBehavior() {

  randomWander();

  while (steppers.run()) {
    while (isCloseObstacle()) {
      // Obstacle detected - stop motors and turn on LED
      collideBehavior();
      delay(100);
      runawayBehavior();
    }
  }

  // Small delay to allow sensor readings to be processed
  delay(1);
}



/**
 * @brief Executes the smart follow behavior for a robot.
 *
 * This function makes the robot move in a random direction and distance,
 * while continuously checking for obstacles. If an obstacle is detected,
 * the robot will stop and execute the collide behavior, followed by the
 * follow behavior.
 *
 * The function performs the following steps:
 * 1. Generates a random angle and distance for the robot to move.
 * 2. Converts the distance to steps for the stepper motors.
 * 3. Rotates the robot to the generated angle.
 * 4. Moves the robot forward by the generated distance.
 * 5. Sets the speed of the stepper motors to the default speed.
 * 6. Continuously runs the stepper motors while checking for obstacles.
 * 7. If an obstacle is detected, stops the motors and executes the collide behavior.
 * 8. Turns on the green LED to indicate movement.
 * 9. Adds a small delay to allow sensor readings to be processed.
 */
void smartFollowBehavior() {
  randomWander();

  while (steppers.run()) {
    while (checkFrontObstacle()) {
      // Obstacle detected - stop motors and turn on LED
      collideBehavior();
      delay(100);
      followBehavior();
    }
  }

  // Small delay to allow sensor readings to be processed
  delay(1);
}

/**
 * @brief Controls the movement and obstacle-following behavior of the robot.
 *
 * This function initializes the LEDs and stepper motor speeds, then enters an infinite loop
 * where it continuously checks for close obstacles. If an obstacle is detected, it calls
 * the collideBehavior() and followBehavior() functions to handle the obstacle. Once the
 * obstacle is removed, it resets the LEDs and stepper motor speeds and continues moving
 * the steppers one step at a time with a small delay to allow sensor readings to be processed.
 */
void moveAndFollowBehavior() {
  // Set initial movement speeds
  stepperLeft.setSpeed(defaultStepSpeed);
  stepperRight.setSpeed(defaultStepSpeed);

  while (true) {
    while (isCloseObstacle()) {
      collideBehavior();
      followBehavior();
    }
    // Obstacle removed - reset movement
    stepperLeft.setSpeed(defaultStepSpeed);
    stepperRight.setSpeed(defaultStepSpeed);

    // Move the steppers one step at a time
    stepperLeft.runSpeed();
    stepperRight.runSpeed();

    // Small delay to allow sensor readings to be processed
    delay(1);
  }
}


double lastError = 0;  // For derivative calculation
// Convert inches to cm for lidar readings
const double DEADBAND_INNER_CM = DEADBAND_INNER * 2.54;
const double DEADBAND_OUTER_CM = DEADBAND_OUTER * 2.54;
const double TARGET_DISTANCE_CM = TARGET_DISTANCE * 2.54;

const bool bangBangControllorOn = true;


bool leftHasWall() {
  return lidar_data.left < WALL_THRESHOLD;
}

bool rightHasWall() {
  return lidar_data.right < WALL_THRESHOLD;
}

bool frontHasWall() {
  return lidar_data.front < WALL_THRESHOLD;
}

bool isLeftCorner() {
  return frontHasWall() && leftHasWall() && !rightHasWall();
}

bool isRightCorner() {
  return frontHasWall() && rightHasWall() && !leftHasWall();
}

void avoidObstacle() {
  collideBehavior();
  runawayBehavior();
}

/**
   * @brief Detects if the robot is at the left inner corner and updates the state accordingly.
   *
   * This function checks if the robot is at the left inner corner using the isLeftCorner() function.
   * If the robot is at the left inner corner, it updates the current state to INSIDE_CORNER,
   * updates the LEDs to reflect the new state, and then spins the robot 90 degrees to the right
   * at the default step speed.
   */
void detectLeftInnerCorner() {
  if (isLeftCorner()) {
    currentState = INSIDE_CORNER;
    updateLEDs();

    spin(TO_RIGHT, 90, defaultStepSpeed);
  }
}

/**
   * @brief Detects the left outer corner of the robot's path.
   *
   * This function performs a series of movements to detect the left outer corner.
   * It updates the current state to OUTSIDE_CORNER and updates the LEDs accordingly.
   * The robot moves forward, spins to the left, and moves forward again.
   * It then reads the LIDAR data to determine if there is a wall on the left side.
   *
   * @return true if there is a wall on the left side, false otherwise.
   */
bool detectLeftOuterCorner() {
  currentState = OUTSIDE_CORNER;
  updateLEDs();

  forward(2, FOLLOW_WALL_BASE_SPEED);
  spin(TO_LEFT, 90, FOLLOW_WALL_BASE_SPEED);
  forward(12, FOLLOW_WALL_BASE_SPEED);

  lidar_data = RPC.call("read_lidars").as<struct lidar>();

  return leftHasWall();
}

/**
 * @brief Function to follow the left wall using lidar data.
 * 
 * This function sets the speed of the left and right stepper motors to a base speed,
 * updates the current state to FOLLOWING_LEFT, and then enters an infinite loop to
 * continuously adjust the robot's path based on lidar readings.
 * 
 * The function performs the following steps:
 * 1. Sets the speed of both stepper motors to FOLLOW_WALL_BASE_SPEED.
 * 2. Updates the current state to FOLLOWING_LEFT and updates the LEDs.
 * 3. Enters an infinite loop to continuously read lidar data and adjust the robot's path.
 * 4. Checks if there is a wall on the right side and calls followCenter() if both sides have walls.
 * 5. Detects inner and outer corners on the left side.
 * 6. Calculates the error between the current distance to the left wall and the target distance.
 * 7. Adjusts the speed of the motors based on the error to maintain the desired distance from the wall.
 * 8. Updates the current state and LEDs based on the error.
 * 9. Runs the motors at the adjusted speeds.
 * 
 * The function uses a proportional-derivative (PD) controller to adjust the motor speeds
 * based on the error and its derivative.
 * 
 * @note This function contains an infinite loop and will not return.
 */
void followLeft() {
  stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
  stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);

  currentState = FOLLOWING_LEFT;
  updateLEDs();
  delay(1000);

  while (true) {
    lidar_data = RPC.call("read_lidars").as<struct lidar>();

    if (rightHasWall()) {
      // both left and right have walls
      followCenter();
    }

    // check if there is a corner on the left (inner corner)
    detectLeftInnerCorner();

    // check if there is a corner on the right (outer corner)
    if (!leftHasWall() && !detectLeftOuterCorner())
      return;

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
    } else if (lidar_data.left < DEADBAND_INNER_CM) {
      avoidObstacle();
    } else {
      // Calculate speed adjustment based on error
      int speedAdjustment = (int)(WallFollowKp * error + WallFollowKd * error_diff);

      // If too far from wall (positive error)
      if (error > 0) {
        currentState = TOO_FAR;
        updateLEDs();

        // Turn towards wall - slow down let motor
        stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
        stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
      }
      // If too close to wall (negative error)
      else {
        currentState = TOO_CLOSE;
        updateLEDs();

        // Turn away from wall - slow down rigt motor
        stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
        stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
      }
    }

    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}





/**
   * @brief Detects if the robot is at a right inner corner and updates its state accordingly.
   *
   * This function checks if the robot is at a right inner corner using the isRightCorner() function.
   * If a right inner corner is detected, it updates the current state to INSIDE_CORNER, updates the LEDs,
   * and makes the robot spin to the left by 90 degrees at the default step speed.
   */
void detectRightInnerCorner() {
  if (isRightCorner()) {
    currentState = INSIDE_CORNER;
    updateLEDs();

    spin(TO_LEFT, 90, defaultStepSpeed);
  }
}

/**
   * @brief Detects the right outer corner of the environment.
   *
   * This function performs a series of movements to detect the right outer corner.
   * It updates the current state to OUTSIDE_CORNER and updates the LEDs accordingly.
   * The robot moves forward a short distance, spins to the right by 90 degrees,
   * and then moves forward a longer distance. It then reads the LIDAR data to
   * determine if there is a wall on the right side.
   *
   * @return true if there is a wall on the right side, false otherwise.
   */
bool detectRightOuterCorner() {
  currentState = OUTSIDE_CORNER;
  updateLEDs();

  forward(2, FOLLOW_WALL_BASE_SPEED);
  spin(TO_RIGHT, 90, FOLLOW_WALL_BASE_SPEED);
  forward(12, FOLLOW_WALL_BASE_SPEED);

  lidar_data = RPC.call("read_lidars").as<struct lidar>();

  return rightHasWall();
}


/**
 * @brief Function to follow the right wall using LIDAR data and stepper motors.
 * 
 * This function sets the speed of the left and right stepper motors to a base speed
 * and enters a loop where it continuously reads LIDAR data to adjust the robot's
 * movement to follow the right wall. The function uses proportional control to
 * maintain a target distance from the wall and handles different scenarios such as
 * detecting corners and avoiding obstacles.
 * 
 * The function performs the following steps:
 * 1. Sets the initial speed of the stepper motors.
 * 2. Sets the current state to FOLLOWING_RIGHT and updates the LEDs.
 * 3. Enters an infinite loop to continuously read LIDAR data and adjust motor speeds.
 * 4. Checks if there is a wall on the left and calls followCenter() if both left and right have walls.
 * 5. Detects inner and outer corners on the right.
 * 6. Calculates the error between the current distance and the target distance.
 * 7. Applies proportional control to adjust motor speeds based on the error.
 * 8. Updates the current state and LEDs based on the error.
 * 9. Runs the stepper motors at the adjusted speeds.
 * 
 * @note This function assumes the existence of several global variables and functions:
 * - stepperLeft, stepperRight: Instances of stepper motor control.
 * - FOLLOW_WALL_BASE_SPEED: Base speed for the motors.
 * - currentState: Current state of the robot.
 * - updateLEDs(): Function to update the state of LEDs.
 * - lidar_data: Struct containing LIDAR data.
 * - TARGET_DISTANCE_CM: Desired distance from the wall.
 * - DEADBAND_INNER_CM, DEADBAND_OUTER_CM: Deadband range for distance control.
 * - WallFollowKp, WallFollowKd: Proportional and derivative gains for control.
 * - lastError: Previous error value for derivative control.
 * - leftHasWall(), rightHasWall(): Functions to check for walls on the left and right.
 * - detectRightInnerCorner(), detectRightOuterCorner(): Functions to detect corners.
 * - avoidObstacle(): Function to avoid obstacles.
 */
void followRight() {
  stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
  stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);

  currentState = FOLLOWING_RIGHT;
  updateLEDs();
  delay(1000);

  while (true) {


    lidar_data = RPC.call("read_lidars").as<struct lidar>();

    if (leftHasWall()) {
      // both left and right have walls
      followCenter();
    }

    // check if there is a corner on the right (inner corner)
    detectRightInnerCorner();

    // check if there is a corner on the right (outer corner)
    if (!rightHasWall() && !detectRightOuterCorner())
      return;

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
    }  // Outside deadband - apply proportional control
    else if (lidar_data.right < DEADBAND_INNER_CM) {
      avoidObstacle();
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
        stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
        stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
      }
    }

    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}






/**
 * @brief Function to follow the center path between two walls using LIDAR data.
 * 
 * This function sets the speed of the left and right stepper motors to a base speed
 * and continuously adjusts the speed based on the LIDAR readings to maintain a central
 * position between two walls. It updates the current state to FOLLOWING_CENTER and
 * updates the LEDs accordingly.
 * 
 * The function reads LIDAR data to determine the presence of walls on the left, right,
 * and front. Depending on the presence of walls, it calls appropriate functions to
 * follow the left or right wall, or to turn around if a wall is detected in front.
 * 
 * The center error is calculated as the difference between the left and right LIDAR
 * readings. A speed adjustment is computed based on the error and its difference from
 * the last error, and the speeds of the stepper motors are adjusted accordingly.
 * 
 * The function runs indefinitely in a loop, continuously adjusting the motor speeds
 * to follow the center path.
 * 
 * @note The function assumes the existence of certain global variables and functions:
 * - FOLLOW_WALL_BASE_SPEED: Base speed for the motors.
 * - currentState: Variable to store the current state of the robot.
 * - updateLEDs(): Function to update the LEDs based on the current state.
 * - RPC.call("read_lidars"): Function to read LIDAR data.
 * - leftHasWall(), rightHasWall(), frontHasWall(): Functions to check for walls.
 * - followLeft(), followRight(): Functions to follow the left or right wall.
 * - goToAngle(int angle): Function to turn the robot to a specified angle.
 * - WallFollowKd: Constant for the derivative term in the speed adjustment calculation.
 * - lastError: Variable to store the last error value.
 * - lidar_data: Struct to store LIDAR readings.
 */
void followCenter() {
  stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
  stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
  currentState = FOLLOWING_CENTER;
  updateLEDs();

  while (true) {

    lidar_data = RPC.call("read_lidars").as<struct lidar>();

    if (leftHasWall() && !rightHasWall()) {
      followLeft();
    } else if (!leftHasWall() && rightHasWall()) {
      followRight();
    } else if (!leftHasWall() && !rightHasWall()) {
      return;
    } else if (leftHasWall() && rightHasWall() && frontHasWall()) {
      goToAngle(180);
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

    stepperLeft.runSpeed();
    stepperRight.runSpeed();

    lastError = error;
  }
}



void followWallBehavior() {
  randomWander();
  Serial.println("followWallBehavior");

  while (steppers.run()) {
    lidar_data = RPC.call("read_lidars").as<struct lidar>();
    if (leftHasWall() && !rightHasWall() && lidar_data.left > DEADBAND_INNER_CM) {
      // only left has wall
      followLeft();
      break;
    } else if (!leftHasWall() && rightHasWall() && lidar_data.left > DEADBAND_INNER_CM) {
      // only right has wall
      followRight();
      break;
    } else if (leftHasWall() && rightHasWall()) {
      // both left and right have walls
      followCenter();
      break;
    } else if (frontHasWall()) {
      spin(TO_LEFT, 90, defaultStepSpeed);
      break;
    } else if (lidar_data.right < DEADBAND_INNER_CM || lidar_data.left < DEADBAND_INNER_CM) {
      avoidObstacle();
    }
  }
}


/**
 * @brief Moves the robot to a specified goal while avoiding obstacles.
 * 
 * This function calculates the angle and distance to the goal, then moves the robot
 * towards the goal while checking for obstacles using LIDAR data. If an obstacle is detected,
 * the robot will perform a series of maneuvers to avoid the obstacle and then continue towards
 * the goal.
 * 
 * @param x The x-coordinate of the goal.
 * @param y The y-coordinate of the goal.
 * 
 * The function uses the following states to manage the robot's movement:
 * - MOVING: The robot is moving towards the goal.
 * - AVOID_OBSTACLE_1: The robot detected an obstacle and is performing the first avoidance maneuver.
 * - AVOID_OBSTACLE_2: The robot is performing the second avoidance maneuver.
 * - AVOID_OBSTACLE_3: The robot is performing the third avoidance maneuver and will resume moving towards the goal.
 * 
 * The function uses LIDAR data to detect obstacles and encoder data to track the robot's movement.
 * It adjusts the robot's speed and direction based on the current state and obstacle detection.
 * 
 * The function also includes debug prints to the Serial monitor to indicate the current state and actions.
 */
void goToGoalAvoidbs(double x, double y) {

  int obsCount = 0;

  double angle = getTurnAngle(x, y);

  goToAngle(angle);

  double distanceSteps = length2Steps(sqrt(pow(x, 2) + pow(y, 2)));
  Serial.println(distanceSteps);

  int eCounts = 0;
  int avoidObsCount = 0;
  setStepperCounter(0);

  ToGoalState state = MOVING;

  while (eCounts * encoderRatio < distanceSteps) {
    lidar_data = RPC.call("read_lidars").as<struct lidar>();

    switch (state) {
      case MOVING:
        if (lidar_data.front < 10) {
          spin(TO_LEFT, 90, defaultStepSpeed);
          state = AVOID_OBSTACLE_1;
          setStepperCounter(0);
        } else {
          eCounts = encoder[LEFT_ENCODER];
          Serial.println("MOVING");
        }
        break;
      case AVOID_OBSTACLE_1:
        if (lidar_data.right > 30) {
          forward(2, defaultStepSpeed);
          spin(TO_RIGHT, 90, defaultStepSpeed);
          forward(14, defaultStepSpeed);
          state = AVOID_OBSTACLE_2;
          setStepperCounter(eCounts);
        } else {
          avoidObsCount = encoder[LEFT_ENCODER];
          Serial.println("AVOID 1");
        }
        break;
      case AVOID_OBSTACLE_2:
        if (lidar_data.right > 30) {
          forward(4, defaultStepSpeed);
          spin(TO_RIGHT, 90, defaultStepSpeed);
          forward(5, defaultStepSpeed);
          state = AVOID_OBSTACLE_3;
          setStepperCounter(0);
        } else {
          eCounts = encoder[LEFT_ENCODER];
          Serial.println("AVOID 2");
        }
        break;
      case AVOID_OBSTACLE_3:
        if (encoder[LEFT_ENCODER] >= avoidObsCount) {
          spin(TO_LEFT, 85, defaultStepSpeed);
          state = MOVING;
          eCounts += length2Steps(14) / encoderRatio;
          setStepperCounter(eCounts);
        } else {
          Serial.println("AVOID 3");
        }
        break;
      default:
        break;
    }

    stepperLeft.setSpeed(defaultStepSpeed);
    stepperRight.setSpeed(defaultStepSpeed);
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}