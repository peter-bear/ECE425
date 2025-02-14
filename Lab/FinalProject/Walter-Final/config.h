#pragma once

#include <Arduino.h>


// MQTT Configuration
#define SECRET_SSID "RHIT-OPEN"
#define SECRET_PASS ""
// #define SECRET_SSID "0x71A"
// #define SECRET_PASS "343218180x71A"
#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_PORT 1883

#define LIDAR_DATA_TOPIC "walter/lidar_data"
#define SONAR_DATA_TOPIC "walter/sonar_data"
#define MOVE_CONTROL_TOPIC "walter/move_control"
#define LED_DATA_TOPIC "walter/led_data"
#define ENCODER_DATA_TOPIC "walter/encoder_data"
#define MAP_DATA_TOPIC "walter/map_data"

int matrix[4][4] = {{0 99 99 0},{0 0 0 0},{0 99 99 0},{0 99 0 0}};