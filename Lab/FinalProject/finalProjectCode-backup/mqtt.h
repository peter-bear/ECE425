#pragma once
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include "config.h"

void connectWifi();
void connectMqtt();
void onMqttMessage(int messageSize);
void setupTopics();

extern WiFiClient wifiClient;
extern MqttClient mqttClient;