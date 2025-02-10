#include "mqtt.h"

// mqtt config
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;
const char broker[] = MQTT_SERVER;
const int port = MQTT_PORT;
const char lidarTopic[] = LIDAR_DATA_TOPIC;



void connectWifi() {

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

void connectMqtt() {
  // attempt to connect to MQTT broker:
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1)
      ;
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

void subscribeTopics() {
  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(lidarTopic);
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe(lidarTopic);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  Serial.print("Topic: ");
  Serial.println(lidarTopic);

  Serial.println();
}

void publishTopic(const char *topic, char *Rvalue) {
  //print to serial monitor
  Serial.print("Sending message to topic: ");
  Serial.println(topic);
  Serial.println(Rvalue);
  Serial.println();

  //publish the message to the specific topic
  mqttClient.beginMessage(topic);
  mqttClient.print(Rvalue);
  mqttClient.endMessage();
}