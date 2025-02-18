#include "mqtt.h"

// mqtt config
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char* ssid = SECRET_SSID;
const char* pass = SECRET_PASS;
const char* broker = MQTT_SERVER;
const int port = MQTT_PORT;
const char* lidarTopic = LIDAR_DATA_TOPIC;
const char* sonarTopic = SONAR_DATA_TOPIC;
const char* ledTopic = LED_DATA_TOPIC;
const char* encoderTopic = ENCODER_DATA_TOPIC;
const char* moveControlTopic = MOVE_CONTROL_TOPIC;
const char* mapDataTopic = MAP_DATA_TOPIC;
const char* robotPositionTopic = ROBOT_POSITION_TOPIC;
const char* robotPathPlanTopic = ROBOT_PATH_PLAN_TOPIC;
const char* robotPathPlanPositionTopic = ROBOT_PATH_PLAN_POSITION_TOPIC;

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

void receiveMoveControlMessage(String message) {
  if(message == "MOVE_FORWARD") {
    currentState = MOVE_FORWARD;
  }else if(message == "TURN_LEFT") {
    currentState = TURN_LEFT;
  }else if(message == "TURN_RIGHT") {
    currentState = TURN_RIGHT;
  }else if(message == "MOVE_BACKWARD") {
    currentState = MOVE_BACKWARD;
  }else{
    currentState = STOP;
  }
}

void receiveRobotPathPlanMessage(String message) {
  // parse the message to get the path
  // plannedPath = parsePath(message);
  Serial.println("Received path plan message: " + message);

  // split the message by space
  int firstSpace = message.indexOf(' ');
  int secondSpace = message.indexOf(' ', firstSpace + 1);
  int thirdSpace = message.indexOf(' ', secondSpace + 1);
  
  String x1_str = message.substring(0, firstSpace);
  String y1_str = message.substring(firstSpace + 1, secondSpace);
  String x2_str = message.substring(secondSpace + 1, thirdSpace);
  String y2_str = message.substring(thirdSpace + 1);

  robotStartPosition.x = x1_str.toInt();
  robotStartPosition.y = y1_str.toInt();

  robotGoalPosition.x = x2_str.toInt();
  robotGoalPosition.y = y2_str.toInt();

  // Serial.print("Start position: ");
  // Serial.print(robotStartPosition.x);
  // Serial.print(", ");
  // Serial.println(robotStartPosition.y);

  // Serial.print("Goal position: ");
  // Serial.print(robotGoalPosition.x);
  // Serial.print(", ");
  // Serial.println(robotGoalPosition.y);

  currentState = MATRIX_PATH_PLANNING;
}


String receivedMessage  = "";
String receivedTopic = "";

void onMqttMessage(int messageSize) {
  receivedMessage = "";
  receivedTopic = mqttClient.messageTopic();

  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(receivedTopic);
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    receivedMessage += (char)mqttClient.read();
  }
  Serial.println("Message content: " + receivedMessage);
  Serial.println();

  if (receivedTopic == moveControlTopic) {
    receiveMoveControlMessage(receivedMessage);
  }else if(receivedTopic == robotPathPlanPositionTopic){
    receiveRobotPathPlanMessage(receivedMessage);
  }

}

void subscribeTopics() {
  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(moveControlTopic);
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe(moveControlTopic);
  mqttClient.subscribe(robotPathPlanPositionTopic);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  Serial.print("Topic: ");
  Serial.println(lidarTopic);

  Serial.println();
}

void publishTopic(const char *topic, const char *Rvalue) {
  //publish the message to the specific topic
  mqttClient.beginMessage(topic);
  mqttClient.print(Rvalue);
  mqttClient.endMessage();
}


//set publishDataPeriod for sending messages (milliseconds)
unsigned long mqttPreMillis = 0;
char mqttBuffer[64];

void publishMatrixMapData(){
  int index = 0;
  for(int i = 0; i < MATRIX_SIZE_X; i++){
    for(int j = 0; j < MATRIX_SIZE_Y; j++){
      index += sprintf(mqttBuffer + index, "%d ", pathPlanMatrix[i][j]);
    }
    index += sprintf(mqttBuffer + index, ";");
  }

  publishTopic(mapDataTopic, mqttBuffer);
}

void publishRobotPosition(){
  sprintf(mqttBuffer, "%d %d", currentRobotPosition.x, currentRobotPosition.y);
  publishTopic(robotPositionTopic, mqttBuffer);
}

void publishRobotPathPlan(){
  PositionQueue path = plannedPath;

  if(path.isEmpty()){
    // Serial.println("No path to publish");
    return;
  }

  int index = 0;

  for(int i = 0; i < path.length(); i++){
    Position current = path.getByIndex(i);
    index += sprintf(mqttBuffer + index, "%d %d;", current.x, current.y);
  }

  publishTopic(robotPathPlanTopic, mqttBuffer);
}

void publishRobotSensorData(){
  lidar_data = RPC.call("read_lidars").as<struct lidar>();
  sprintf(mqttBuffer, "%d %d %d %d", lidar_data.front, lidar_data.back, lidar_data.left, lidar_data.right);
  publishTopic(lidarTopic, mqttBuffer);

  sonar_data = RPC.call("read_sonars").as<struct sonar>();
  sprintf(mqttBuffer, "%d %d", lidar_data.left, lidar_data.right);
  publishTopic(sonarTopic, mqttBuffer);

  sprintf(mqttBuffer, "%d %d", encoder[LEFT_ENCODER], encoder[RIGHT_ENCODER]);
  publishTopic(encoderTopic, mqttBuffer);

  sprintf(mqttBuffer, "%d %d %d", LED_Status[0], LED_Status[1], LED_Status[2]);
  publishTopic(ledTopic, mqttBuffer);
}

// publish data
void publishData() {
  // publish data
  unsigned long currentMillis = millis();
  if (currentMillis - mqttPreMillis >= publishDataPeriod) {
    mqttPreMillis = currentMillis;

    publishRobotSensorData();
    publishMatrixMapData();
    publishRobotPosition();
    publishRobotPathPlan();
  }
}