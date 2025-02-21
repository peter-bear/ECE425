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
#define MAP_DATA_TOPIC "walter/matrix_map"
#define ROBOT_POSITION_TOPIC "walter/robot_position"
#define ROBOT_PATH_PLAN_TOPIC "walter/robot_path_plan"
#define ROBOT_PATH_PLAN_POSITION_TOPIC "walter/robot_path_plan_position"
#define GRID_LOCALIZATION_COMMAND_TOPIC "walter/grid_localization_command"
#define GRID_LOCALIZATION_RESPONSE_TOPIC "walter/grid_localization_response"
#define TOPO_LOCALIZATION_COMMAND_TOPIC "walter/topo_localization_command"