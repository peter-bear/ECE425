#include "behaviors.h"
#include <Arduino.h>

void updateLEDs() {
    digitalWrite(redLED, LOW);
    digitalWrite(ylwLED, LOW);
    digitalWrite(grnLED, LOW);

    switch (currentState) {
        case FOLLOWING_RIGHT:
            digitalWrite(redLED, HIGH);
            digitalWrite(ylwLED, HIGH);
            break;
        case FOLLOWING_LEFT:
            digitalWrite(grnLED, HIGH);
            digitalWrite(ylwLED, HIGH);
            break;
        case FOLLOWING_CENTER:
            digitalWrite(redLED, HIGH);
            digitalWrite(grnLED, HIGH);
            digitalWrite(ylwLED, HIGH);
            break;
        case RANDOM_WANDER:
            digitalWrite(grnLED, HIGH);
            break;
        case TOO_CLOSE:
        case CLOSE_TO_RIGHT:
            digitalWrite(ylwLED, HIGH);
            break;
        case TOO_FAR:
        case CLOSE_TO_LEFT:
            digitalWrite(redLED, HIGH);
            break;
        case INSIDE_CORNER:
            digitalWrite(redLED, HIGH);
            digitalWrite(grnLED, HIGH);
            break;
        case COLLIDE_BEHAVIOR:
            digitalWrite(redLED, HIGH);
            break;
        case RUNAWAY_BEHAVIOR:
            digitalWrite(ylwLED, HIGH);
            break;
        case FOLLOW_BEHAVIOR:
            digitalWrite(redLED, HIGH);
            digitalWrite(grnLED, HIGH);
            break;
    }
}

void allOFF() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(leds[i], LOW);
    }
}

void collideBehavior() {
    currentState = COLLIDE_BEHAVIOR;
    updateLEDs();
    stopMove();
    delay(50);
}

void runawayBehavior() {
    currentState = RUNAWAY_BEHAVIOR;
    updateLEDs();

    lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();

    int x = lidar_data_M7_read.front - lidar_data_M7_read.back;
    int y = lidar_data_M7_read.left - lidar_data_M7_read.right;

    double x_inch = runawayPropotion * cm2inch(x);
    double y_inch = runawayPropotion * cm2inch(y);

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

void followBehavior() {
    currentState = FOLLOW_BEHAVIOR;
    updateLEDs();

    lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();
    int front_error_cm = lidar_data_M7_read.front - OBSTACLE_THRESHOLD;
    double front_error_inch = cm2inch(front_error_cm);

    while (true) {
        if (!isWithinObstacleMargin(lidar_data_M7_read.front)) {
            if (lidar_data_M7_read.front >= MAX_LIDAR_DISTANCE)
                break;

            double moveDistance = abs(followPropotion * front_error_inch);

            if (front_error_cm > 0) {
                forward(moveDistance, FOLLOW_SPEED);
            } else if (front_error_cm < 0) {
                reverse(moveDistance, FOLLOW_SPEED);
            }
        }

        lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();
        front_error_cm = lidar_data_M7_read.front - OBSTACLE_THRESHOLD;
        front_error_inch = cm2inch(front_error_cm);
        delay(20);
    }
}

void randomWander() {
    currentState = RANDOM_WANDER;
    updateLEDs();

    int randAngle = random(-maxTurnAngle, maxTurnAngle);
    int randDistance = random(maxDistanceMove);
    randDistance = length2Steps(randDistance);

    stepperLeft.move(randDistance);
    stepperRight.move(randDistance);

    stepperLeft.setSpeed(defaultStepSpeed);
    stepperRight.setSpeed(defaultStepSpeed);
}

void printRandomValues(int randAngle, int randDistance) {
    Serial.print("Random Values:\n\tAngle: ");
    Serial.print(randAngle);
    Serial.print("\tDistance: ");
    Serial.println(randDistance);
}

void smartWanderBehavior() {
    randomWander();

    while (steppers.run()) {
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        digitalWrite(redLED, LOW);

        while (isCloseObstacle()) {
            collideBehavior();
            delay(100);
            runawayBehavior();
        }
    }
    delay(1);
}

void smartFollowBehavior() {
    randomWander();

    while (steppers.run()) {
        digitalWrite(grnLED, HIGH);
        digitalWrite(ylwLED, LOW);
        digitalWrite(redLED, LOW);

        while (checkFrontObstacle()) {
            collideBehavior();
            delay(100);
            followBehavior();
        }
    }
    delay(1);
}

void moveAndFollowBehavior() {
    stepperLeft.setSpeed(defaultStepSpeed);
    stepperRight.setSpeed(defaultStepSpeed);

    while (true) {
        while (isCloseObstacle()) {
            collideBehavior();
            followBehavior();
        }
        stepperLeft.setSpeed(defaultStepSpeed);
        stepperRight.setSpeed(defaultStepSpeed);
        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        delay(1);
    }
}

void followWallBehavior() {
    randomWander();

    while (steppers.run()) {
        lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();
        if (leftHasWall() && !rightHasWall()) {
            followLeft();
            break;
        } else if (!leftHasWall() && rightHasWall()) {
            followRight();
            break;
        } else if (leftHasWall() && rightHasWall()) {
            followCenter();
            break;
        }
    }
}

void followLeft() {
    stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
    stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
    currentState = FOLLOWING_LEFT;
    updateLEDs();
    delay(500);

    while (true) {
        unsigned long currentTime = millis();
        double deltaTime = (currentTime - lastMeasureTime) / 1000.0;
        double error = lidarData.left - TARGET_DISTANCE_CM;
        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;

        lastError = error;
        lastMeasureTime = currentTime;

        if (lidarData.left >= DEADBAND_INNER_CM && lidarData.left <= DEADBAND_OUTER_CM) {
            stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
            stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
        } else {
            int speedAdjustment = (int)(WallFollowKp * error + WallFollowKd * derivative);

            if (error > 0) {
                currentState = TOO_FAR;
                updateLEDs();
                stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
                stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
            } else {
                currentState = TOO_CLOSE;
                updateLEDs();
                stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
                stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
            }
        }

        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();

        if (rightHasWall()) {
            followCenter();
        }

        if (isLeftCorner()) {
            currentState = INSIDE_CORNER;
            updateLEDs();
            spin(TO_RIGHT, 90, defaultStepSpeed);
        }

        if (!leftHasWall()) {
            return;
        }
    }
}

void followRight() {
    stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
    stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
    currentState = FOLLOWING_RIGHT;
    updateLEDs();
    delay(500);

    while (true) {
        unsigned long currentTime = millis();
        double deltaTime = (currentTime - lastMeasureTime) / 1000.0;
        double error = lidar_data_M7_read.right - TARGET_DISTANCE_CM;
        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;

        lastError = error;
        lastMeasureTime = currentTime;

        if (lidar_data_M7_read.right >= DEADBAND_INNER_CM && lidar_data_M7_read.right <= DEADBAND_OUTER_CM) {
            stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
            stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
        } else {
            int speedAdjustment = (int)(WallFollowKp * error + WallFollowKd * derivative);

            if (error > 0) {
                currentState = TOO_FAR;
                updateLEDs();
                stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
                stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
            } else {
                currentState = TOO_CLOSE;
                updateLEDs();
                stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
                stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
            }
        }

        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();

        if (leftHasWall()) {
            followCenter();
        }

        if (isRightCorner()) {
            currentState = INSIDE_CORNER;
            updateLEDs();
            spin(TO_LEFT, 90, defaultStepSpeed);
        }

        if (!rightHasWall()) {
            return;
        }
    }
}

void followCenter() {
    stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
    stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
    currentState = FOLLOWING_CENTER;
    updateLEDs();
    delay(500);

    while (true) {
        unsigned long currentTime = millis();
        double deltaTime = (currentTime - lastMeasureTime) / 1000.0;
        double errorLeft = lidar_data_M7_read.left - TARGET_DISTANCE_CM;
        double errorRight = lidar_data_M7_read.right - TARGET_DISTANCE_CM;
        double centerError = errorLeft - errorRight;
        double derivative = deltaTime > 0 ? (centerError - lastError) / deltaTime : 0;

        lastError = centerError;
        lastMeasureTime = currentTime;

        if (abs(centerError) <= 1.0) {
            stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
            stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
        } else {
            int speedAdjustment = (int)(WallFollowKp * centerError + WallFollowKd * derivative);

            if (centerError > 0) {
                currentState = CLOSE_TO_RIGHT;
                updateLEDs();
                stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
                stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED);
            } else {
                currentState = CLOSE_TO_LEFT;
                updateLEDs();
                stepperLeft.setSpeed(FOLLOW_WALL_BASE_SPEED);
                stepperRight.setSpeed(FOLLOW_WALL_BASE_SPEED - abs(speedAdjustment));
            }
        }

        stepperLeft.runSpeed();
        stepperRight.runSpeed();
        lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();

        if (leftHasWall() && !rightHasWall()) {
            followLeft();
        } else if (!leftHasWall() && rightHasWall()) {
            followRight();
        } else if (!leftHasWall() && !rightHasWall()) {
            return;
        }
    }
}