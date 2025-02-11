#include "motors.h"

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);  // create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);   // create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;


const float pi = 3.14159;       // pi
const float wheelRadius = 1.7;  // robot wheel radius in inches
const int stepsPerRevol = 800;  // robot wheel steps per revolution
const float robotWidth = 9.0;   // robot width in inches

// function to set all stepper motor variables, outputs and LEDs
void init_stepper() {
  pinMode(rtStepPin, OUTPUT);                   // sets pin as output
  pinMode(rtDirPin, OUTPUT);                    // sets pin as output
  pinMode(ltStepPin, OUTPUT);                   // sets pin as output
  pinMode(ltDirPin, OUTPUT);                    // sets pin as output
  pinMode(stepperEnable, OUTPUT);               // sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);  // turns off the stepper motor driver
  initLEDs();
  stepperRight.setMaxSpeed(max_speed);         // set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);     // set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);          // set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);      // set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);           // add right motor to MultiStepper
  steppers.addStepper(stepperLeft);            // add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);  // turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);               // turn on enable LED
}  // create instance to control multiple steppers at the same time



/*this function will stop the steppers*/
void stopMove() {
  stepperRight.stop();  // Stop right motor
  stepperLeft.stop();   // Stop left motor
}

/**
   * Converts a length in inches to a number of steps for the robot.
   * @param length the length in inches to convert
   * @return the number of steps for the robot to move the given length
   */

int length2Steps(double length) {
  return round(stepsPerRevol * length / (2 * pi * wheelRadius));
}


/**
 * Spins the robot about its center by rotating both wheels in opposite directions.
 * @param direction the direction of the spin (TO_LEFT or TO_RIGHT)
 * @param angle the angle of the spin in degrees
 * @param speed the speed of the spin in steps per second
 */
void spin(int direction, double angle, int speed) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  if (direction != TO_LEFT && direction != TO_RIGHT && angle < 0)
    return;

  // calculate the turn length, using the circle formula
  double stepperMoveLength = pi * robotWidth * angle / 360;
  int stepperMovePos = length2Steps(stepperMoveLength);

  if (direction == TO_RIGHT) {
    stepperLeft.move(stepperMovePos);
    stepperLeft.setSpeed(speed);

    stepperRight.move(-stepperMovePos);
    stepperRight.setSpeed(-speed);
  } else if (direction == TO_LEFT) {
    stepperLeft.move(-stepperMovePos);
    stepperLeft.setSpeed(-speed);

    stepperRight.move(stepperMovePos);
    stepperRight.setSpeed(speed);
  }

  steppers.runSpeedToPosition();
}



/**
 * Moves the robot forward by a specified distance at a specified speed.
 * @param distance the distance in inches to move the robot
 * @param speed the speed in steps per second to move the robot
 */
void forward(double distance, int speed) {
  int stepperMovePos = length2Steps(distance);
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  stepperLeft.move(stepperMovePos);   // move left wheel to relative position
  stepperRight.move(stepperMovePos);  // move right wheel to relative position

  stepperLeft.setSpeed(speed);   // set left motor speed
  stepperRight.setSpeed(speed);  // set right motor spee
  steppers.runSpeedToPosition();
}


/**
 * Moves the robot backward by a specified distance at a specified speed.
 * @param distance the distance in inches to move the robot backward
 * @param speed the speed in steps per second to move the robot backward
 * The function calls the forward() function with negative values of distance and speed.
 */

void reverse(double distance, int speed) {
  forward(-distance, -speed);
}




/**
 * Turns the robot to the absolute angle specified.
 * @param angle the absolute angle to turn to in degrees
 * @note Positive angles are to the left, negative angles are to the right
 */
void goToAngle(double angle) {
  // Serial.println("goToAngle function");

  // digitalWrite(grnLED, HIGH);  //turn on green LED
  // if angle larger than 0, turn left
  // if angle less than 0, turn right
  int direction = 0;
  if (angle > 0) {
    direction = TO_LEFT;
  } else if (angle < 0) {
    direction = TO_RIGHT;
  } else {
    return;
  }
  angle = abs(angle);
  spin(direction, angle, defaultStepSpeed);
}


/**
 * @brief Calculates the turn angle in degrees based on the given x and y coordinates.
 *
 * This function computes the angle in radians using the arctangent of y/x, then converts it to degrees.
 * It adjusts the angle based on the quadrant of the (x, y) point to ensure the correct angle is returned.
 *
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return The turn angle in degrees.
 */
double getTurnAngle(double x, double y) {
  double angleRadian = atan(y / x);

  double angleDegree = angleRadian * 180.0 / pi;

  if (x > 0 && y > 0) {
    angleDegree = angleDegree;
  } else if (x < 0 && y >= 0) {
    angleDegree = 180 + angleDegree;
  } else if (x < 0 && y < 0) {
    angleDegree = 180 + angleDegree;
  } else if (x > 0 && y < 0) {
    angleDegree = angleDegree;
  }

  return angleDegree;
}




/**
 * Moves the robot to the goal location (x, y) in inches.
 * @param x the x-coordinate of the goal in inches
 * @param y the y-coordinate of the goal in inches
 * @note The robot will first turn to the correct angle, then move forward to the goal
 */
void goToGoal(double x, double y) {
  // Serial.println("goToGoal function");
  digitalWrite(redLED, LOW);   // turn off red LED
  digitalWrite(grnLED, HIGH);  // turn on green LED
  digitalWrite(ylwLED, HIGH);  // turn on yellow LED

  double distance = sqrt(pow(x, 2) + pow(y, 2));
  double angleDegree = getTurnAngle(x, y);

  Serial.print("Angle: ");
  Serial.println(angleDegree);

  goToAngle(angleDegree);
  delay(1000);
  forward(distance, defaultStepSpeed);
}



/**
 * Turns the robot by moving each wheel a different distance at a different speed.
 * @param direction the direction of the turn (TO_LEFT or TO_RIGHT)
 * @param timeDelay the time in seconds to move each wheel
 * @param velocityDiff the difference in speed between the two wheels
 */
void turn(int direction, double timeDelay, int velocityDiff) {
  if (direction != TO_LEFT && direction != TO_RIGHT)
    return;
  int speedHigh = 500 + velocityDiff;
  int speedLow = 500;

  // calculate the distance for each wheel
  int distanceShort = round(speedLow * timeDelay);
  int distanceLong = round(speedHigh * timeDelay);

  if (direction == TO_RIGHT) {
    stepperLeft.move(distanceLong);
    stepperLeft.setSpeed(speedHigh);  // set left motor speed
    stepperRight.move(distanceShort);
    stepperRight.setSpeed(speedLow);  // set right motor speed
  } else if (direction == TO_LEFT) {
    stepperLeft.move(distanceShort);
    stepperLeft.setSpeed(speedLow);  // set left motor speed
    stepperRight.move(distanceLong);
    stepperRight.setSpeed(speedHigh);  // set right motor speed
  }

  steppers.runSpeedToPosition();
}