#pragma once

#include <Arduino.h>

// LED Pins and Configuration
#define redLED 5
#define grnLED 6
#define ylwLED 7
#define enableLED 13
const int leds[3] = {5, 6, 7};

// Stepper Motor Pins
#define stepperEnable 48
#define rtStepPin 50
#define rtDirPin 51
#define ltStepPin 52
#define ltDirPin 53

// Stepper Motor Configuration
#define stepperEnTrue false
#define stepperEnFalse true
#define max_speed 1500
#define max_accel 10000
#define defaultStepSpeed 200

// Timing Constants
const int pauseTime = 2500;
const int stepTime = 500;
const int wait_time = 1000;

// Encoder Pins and Configuration
#define LEFT 0
#define RIGHT 1
const int ltEncoder = 18;
const int rtEncoder = 19;

// Direction Constants
#define TO_LEFT -1
#define TO_RIGHT 1
#define CLOCKWISE 1
#define COUNTERCLOCKWISE -1

// Robot Physical Parameters
const float pi = 3.14159;
const float wheelRadius = 1.7;    // inches
const int stepsPerRevol = 800;
const float robotWidth = 9.0;     // inches

// PID Control Parameters
const int PIDThreshold = 50;
const int PIDkp = 1;
const int encoderRatio = 20;

// Movement Parameters
const int maxTurnAngle = 90;
const int maxDistanceMove = 12;

// LIDAR Configuration
#define FRONT 0
#define BANK 1
#define LEFT 2
#define RIGHT 3
#define numOfSens 4

const int MAX_LIDAR_DISTANCE = 40;
const int OBSTACLE_THRESHOLD = 10;

// LIDAR Pins
const int16_t ft_lidar = 8;
const int16_t bk_lidar = 9;
const int16_t lt_lidar = 10;
const int16_t rt_lidar = 11;
const int16_t lidar_pins[4] = {8, 9, 10, 11};

// Sonar Pins
const int16_t leftSnr = 3;
const int16_t rightSnr = 4;

// Wall Following Parameters
const double WallFollowKp = 3.0;
const double WallFollowKd = 0.1;
const double TARGET_DISTANCE = 5.0;      // inches
const double DEADBAND_INNER = 4.0;       // inches
const double DEADBAND_OUTER = 6.0;       // inches
const int FOLLOW_WALL_BASE_SPEED = 300;

// Convert inches to cm
const double DEADBAND_INNER_CM = DEADBAND_INNER * 2.54;
const double DEADBAND_OUTER_CM = DEADBAND_OUTER * 2.54;
const double TARGET_DISTANCE_CM = TARGET_DISTANCE * 2.54;

const int WALL_THRESHOLD = 30;
const bool bangBangControllorOn = true;

// Other Constants
const double runawayPropotion = 0.5;
const double followPropotion = 0.25;
const double OBSTACLE_MARGIN = 2.0;
const double FOLLOW_SPEED = 200;
const int forwardDistance = 10;


// MQTT Configuration
#define SECRET_SSID "RHIT-OPEN"
#define SECRET_PASS ""
#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_PORT 1883

#define LIDAR_DATA_TOPIC "walter_lidar_data"
#define SONAR_DATA_TOPIC "walter_sonar_data"