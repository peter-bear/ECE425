#include "motors.h"
#include <Arduino.h>
#include <math.h>

AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);
MultiStepper steppers;

void init_stepper() {
    pinMode(rtStepPin, OUTPUT);
    pinMode(rtDirPin, OUTPUT);
    pinMode(ltStepPin, OUTPUT);
    pinMode(ltDirPin, OUTPUT);
    pinMode(stepperEnable, OUTPUT);
    digitalWrite(stepperEnable, stepperEnFalse);
    pinMode(enableLED, OUTPUT);
    digitalWrite(enableLED, LOW);
    pinMode(redLED, OUTPUT);
    pinMode(grnLED, OUTPUT);
    pinMode(ylwLED, OUTPUT);
    digitalWrite(redLED, HIGH);
    digitalWrite(ylwLED, HIGH);
    digitalWrite(grnLED, HIGH);
    delay(pauseTime / 5);
    digitalWrite(redLED, LOW);
    digitalWrite(ylwLED, LOW);
    digitalWrite(grnLED, LOW);

    stepperRight.setMaxSpeed(max_speed);
    stepperRight.setAcceleration(max_accel);
    stepperLeft.setMaxSpeed(max_speed);
    stepperLeft.setAcceleration(max_accel);
    steppers.addStepper(stepperRight);
    steppers.addStepper(stepperLeft);
    digitalWrite(stepperEnable, stepperEnTrue);
    digitalWrite(enableLED, HIGH);
}

void LwheelSpeed() {
    encoder[LEFT]++;
}

void RwheelSpeed() {
    encoder[RIGHT]++;
}

void print_encoder_data() {
    static unsigned long timer = 0;
    if (millis() - timer > 100) {
        lastSpeed[LEFT] = encoder[LEFT];
        lastSpeed[RIGHT] = encoder[RIGHT];
        accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];
        accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT];
        Serial.println("Encoder value:");
        Serial.print("\tLeft:\t");
        Serial.print(encoder[LEFT]);
        Serial.print("\tRight:\t");
        Serial.println(encoder[RIGHT]);
        Serial.println("Accumulated Ticks: ");
        Serial.print("\tLeft:\t");
        Serial.print(accumTicks[LEFT]);
        Serial.print("\tRight:\t");
        Serial.println(accumTicks[RIGHT]);
        encoder[LEFT] = 0;
        encoder[RIGHT] = 0;
        timer = millis();
    }
}

void resetEncoder() {
    encoder[LEFT] = 0;
    encoder[RIGHT] = 0;
}

void stopMove() {
    stepperRight.stop();
    stepperLeft.stop();
}

void PIDControl(int leftDistance, int rightDistance) {
    long leftDistanceError = abs(leftDistance) - encoder[LEFT] * encoderRatio;
    long rightDistanceError = abs(rightDistance) - encoder[RIGHT] * encoderRatio;

    if (abs(leftDistanceError) > PIDThreshold || abs(rightDistanceError) > PIDThreshold) {
        long outputLeft = leftDistanceError * PIDkp;
        long outputRight = rightDistanceError * PIDkp;

        if (leftDistanceError < 0) {
            stepperLeft.setSpeed(-stepperLeft.speed());
        }
        if (rightDistanceError < 0) {
            stepperRight.setSpeed(-stepperRight.speed());
        }

        stepperLeft.move(outputLeft);
        stepperRight.move(outputRight);
        steppers.runSpeedToPosition();
    }
    stopMove();
}

int length2Steps(double length) {
    return round(stepsPerRevol * length / (2 * pi * wheelRadius));
}

void spin(int direction, double angle, int speed) {
    if (direction != TO_LEFT && direction != TO_RIGHT && angle < 0)
        return;
    resetEncoder();

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

    if (direction == TO_RIGHT) {
        PIDControl(stepperMovePos, -stepperMovePos);
    } else if (direction == TO_LEFT) {
        PIDControl(-stepperMovePos, stepperMovePos);
    }
}

void forward(double distance, int speed) {
    resetEncoder();
    int stepperMovePos = length2Steps(distance);
    stepperLeft.setCurrentPosition(0);
    stepperRight.setCurrentPosition(0);

    stepperLeft.move(stepperMovePos);
    stepperRight.move(stepperMovePos);

    stepperLeft.setSpeed(speed);
    stepperRight.setSpeed(speed);
    steppers.runSpeedToPosition();

    PIDControl(stepperMovePos, stepperMovePos);
}

void reverse(double distance, int speed) {
    forward(-distance, -speed);
}

void goToAngle(double angle) {
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

void goToGoal(double x, double y) {
    digitalWrite(redLED, LOW);
    digitalWrite(grnLED, HIGH);
    digitalWrite(ylwLED, HIGH);

    double distance = sqrt(pow(x, 2) + pow(y, 2));
    double angleDegree = getTurnAngle(x, y);

    Serial.print("Angle: ");
    Serial.println(angleDegree);

    goToAngle(angleDegree);
    delay(1000);
    forward(distance, defaultStepSpeed);
}

void turn(int direction, double timeDelay, int velocityDiff) {
    if (direction != TO_LEFT && direction != TO_RIGHT)
        return;
    int speedHigh = 500 + velocityDiff;
    int speedLow = 500;

    int distanceShort = round(speedLow * timeDelay);
    int distanceLong = round(speedHigh * timeDelay);

    if (direction == TO_RIGHT) {
        stepperLeft.move(distanceLong);
        stepperLeft.setSpeed(speedHigh);
        stepperRight.move(distanceShort);
        stepperRight.setSpeed(speedLow);
    } else if (direction == TO_LEFT) {
        stepperLeft.move(distanceShort);
        stepperLeft.setSpeed(speedLow);
        stepperRight.move(distanceLong);
        stepperRight.setSpeed(speedHigh);
    }

    steppers.runSpeedToPosition();
}