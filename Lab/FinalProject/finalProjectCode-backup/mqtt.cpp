#include "mqtt.h"

// mqtt config
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void connectWifi(){
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(SECRET_SSID);

  while (WiFi.begin(SECRET_SSID, SECRET_PASS) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();
}

void connectMqtt(){
  // attempt to connect to MQTT broker:
  Serial.print("Attempting to connect to MQTT broker: ");
  Serial.println(MQTT_SERVER);

  while (!mqttClient.connect("walter")) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(MQTT_SERVER);

  if (!mqttClient.connect(MQTT_SERVER, MQTT_PORT)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();
  Serial.println();
}

void setupTopics(){
    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(LIDAR_DATA_TOPIC);
    Serial.println();

    // subscribe to a topic
    mqttClient.subscribe(LIDAR_DATA_TOPIC);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(topic);

    Serial.print("Topic: ");
    Serial.println(LIDAR_DATA_TOPIC);

    Serial.println();
}