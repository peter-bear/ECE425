#include "sensors.h"
#include "RPC.h"
#include <Arduino.h>

int read_lidar(int pin) {
    int16_t t = pulseIn(pin, HIGH);
    int d = MAX_LIDAR_DISTANCE;
    
    if (t != 0 && t <= 1850) {
        d = (t - 1000) * 3 / 40;
        if (d < 0) d = 0;
    }
    delay(10);
    return d;
}

int read_sonar(int pin) {
    float velocity = (331.5 + 0.6 * (float)(20)) * 100 / 1000000.0;
    uint16_t distance, pulseWidthUs;

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);
    pulseWidthUs = pulseIn(pin, HIGH);
    distance = pulseWidthUs * velocity / 2.0;
    
    if (distance < 0 || distance > 50) {
        distance = 0;
    }
    return distance;
}

void print_sensor_data(struct lidar data, struct sonar data2) {
    Serial.print("lidar: ");
    Serial.print(data.front);
    Serial.print(", ");
    Serial.print(data.back);
    Serial.print(", ");
    Serial.print(data.left);
    Serial.print(", ");
    Serial.print(data.right);
    Serial.println();
}

double cm2inch(int cm) {
    return 0.393701 * cm;
}

bool isCloseObstacle() {
    lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();
    return frontHasObstacle() || backHasObstacle() || leftHasObstacle() || rightHasObstacle();
}

bool checkFrontObstacle() {
    lidar_data_M7_read = RPC.call("read_lidars").as<struct lidar>();
    return frontHasObstacle();
}

bool frontHasObstacle() {
    return lidar_data_M7_read.front < OBSTACLE_THRESHOLD;
}

bool backHasObstacle() {
    return lidar_data_M7_read.back < OBSTACLE_THRESHOLD;
}

bool leftHasObstacle() {
    return lidar_data_M7_read.left < OBSTACLE_THRESHOLD;
}

bool rightHasObstacle() {
    return lidar_data_M7_read.right < OBSTACLE_THRESHOLD;
}

bool leftHasWall() {
    return lidar_data_M7_read.left < WALL_THRESHOLD;
}

bool rightHasWall() {
    return lidar_data_M7_read.right < WALL_THRESHOLD;
}

bool frontHasWall() {
    return lidar_data_M7_read.front < WALL_THRESHOLD;
}

bool isLeftCorner() {
    return frontHasWall() && leftHasWall() && !rightHasWall();
}

bool isRightCorner() {
    return frontHasWall() && rightHasWall() && !leftHasWall();
}

struct lidar read_lidars() {
    return lidarData;
}

struct sonar read_sonars() {
    return sonarData;
}

bool isWithinObstacleMargin(double frontDistance) {
    return frontDistance < OBSTACLE_THRESHOLD + OBSTACLE_MARGIN && 
           frontDistance > OBSTACLE_THRESHOLD - OBSTACLE_MARGIN;
}