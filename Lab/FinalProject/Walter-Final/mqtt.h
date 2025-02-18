#pragma once
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include "config.h"
#include "encoder.h"
#include "types.h"
#include "led.h"
#include "sensors.h"
#include "advancedBehavior.h"
#include <Arduino.h>

#define publishDataPeriod 500 // publish data every 100 milliseconds

void connectWifi();
void connectMqtt();
void onMqttMessage(int messageSize);
void subscribeTopics();
void publishTopic(const char *topic, const char *Rvalue);
void publishData();

extern WiFiClient wifiClient;
extern MqttClient mqttClient;
extern const char* lidarTopic;
extern const char* sonarTopic;
extern const char* encoderTopic;
extern const char* ledTopic;
extern const char* moveControlTopic;
extern const char* mapDataTopic;
extern const char* robotPositionTopic;
extern const char* robotPathPlanTopic;
extern const char* robotPathPlanPositionTopic;