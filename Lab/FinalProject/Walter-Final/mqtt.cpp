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
const char* gridLocalizationCommandTopic = GRID_LOCALIZATION_COMMAND_TOPIC;
const char* gridLocalizationResponseTopic = GRID_LOCALIZATION_RESPONSE_TOPIC;
const char* topoLocalizationCommandTopic = TOPO_LOCALIZATION_COMMAND_TOPIC;


String receivedMessage  = "";
String receivedTopic = "";

/**
 * Connects to the WiFi network specified by the SECRET_SSID and SECRET_PASS
 * variables. This function will retry indefinitely if the connection fails.
 * If the connection is successful, it prints a message to the serial monitor
 * indicating success.
 */
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

/**
 * Attempts to connect to the MQTT broker specified by the broker variable.
 * If the connection is successful, it prints a message to the serial monitor
 * indicating success.
 * If the connection fails, it prints a message to the serial monitor
 * indicating the error code and then enters an infinite loop.
 */
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

/**
 * Handles move control messages received from the MQTT broker.
 * The message is one of "MOVE_FORWARD", "TURN_LEFT", "TURN_RIGHT", "MOVE_BACKWARD", or "STOP".
 * The function sets the currentState accordingly.
 * @param message The message received from the MQTT broker.
 */
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

/**
 * Handles robot path planning messages received from the MQTT broker.
 * The message is in the format "x1 y1 x2 y2" where (x1, y1) is the start position
 * and (x2, y2) is the goal position. The function parses the message to get the
 * start and goal positions and sets the currentState to MATRIX_PATH_PLANNING.
 * @param message The message received from the MQTT broker.
 */
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

  currentState = MATRIX_PATH_PLANNING;
}

/**
 * Handles grid localization messages received from the MQTT broker.
 * If the message is "START", it enables the grid localization state and resets
 * the robot's position to unknown.
 * If the message is "STOP", it disables the grid localization state and stops
 * the robot's movement.
 * @param message The message received from the MQTT broker.
 */
void receiveGridLocalizationMessage(String message) {
  Serial.println("Received grid localization message: " + message);

  if(message=="START"){
    isLocalizing = true;
    findTheLocation = false;
    currentState = GRID_LOCALIZATION;
    initializeBelif();
  }else if(message=="STOP"){
    isLocalizing = false;
    currentState = STOP;
  }
}

/**
 * Handles topology localization messages received from the MQTT broker.
 * This function is triggered when a message is received from the topic
 * specified by the topoLocalizationCommandTopic variable.
 * It sets the current state to TOPO_LOCALIZATION if the message is "START",
 * and to STOP if the message is "STOP". It also sets the findTheLocation flag
 * to false and the isLocalizing flag to true when starting topology localization.
 * @param message The message received from the MQTT broker.
 */
void receiveTopoLocalizationMessage(String message) {
  Serial.println("Received topo localization message: " + message);

  if(message=="START"){
    isLocalizing = true;
    findTheLocation = false;
    currentState = TOPO_LOCALIZATION;
  }else if(message=="STOP"){
    isLocalizing = false;
    currentState = STOP;
  }
}

/**
 * Callback function that is triggered when an MQTT message is received.
 * This function processes the received message by determining its topic
 * and content. It prints the topic, message size, and message content to
 * the serial monitor. Based on the topic, it delegates the message to the
 * appropriate handler function for further processing.
 * 
 * @param messageSize The size of the received message in bytes.
 * 
 * The function handles messages for the following topics:
 * - moveControlTopic: Delegates to receiveMoveControlMessage to handle robot movement commands.
 * - robotPathPlanPositionTopic: Delegates to receiveRobotPathPlanMessage to handle path planning positions.
 * - gridLocalizationCommandTopic: Delegates to receiveGridLocalizationMessage to handle grid localization commands.
 * - topoLocalizationCommandTopic: Delegates to receiveTopoLocalizationMessage to handle topology localization commands.
 */

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
  }else if(receivedTopic == gridLocalizationCommandTopic){
    receiveGridLocalizationMessage(receivedMessage);
  }else if(receivedTopic == topoLocalizationCommandTopic){
    receiveTopoLocalizationMessage(receivedMessage);
  }

}

/**
 * Subscribes to the necessary MQTT topics to receive control messages and 
 * positioning information from the server.
 * The topics subscribed to are:
 * - moveControlTopic: control messages for the robot's movement (forward, backward, left, right, stop)
 * - robotPathPlanPositionTopic: the robot's path planning position
 * - gridLocalizationCommandTopic: the command to start/stop the grid localization process
 * - topoLocalizationCommandTopic: the command to start/stop the topology localization process
 */
void subscribeTopics() {
  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  // subscribe to a topic
  mqttClient.subscribe(moveControlTopic);
  mqttClient.subscribe(robotPathPlanPositionTopic);
  mqttClient.subscribe(gridLocalizationCommandTopic);
  mqttClient.subscribe(topoLocalizationCommandTopic);
  
  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

}

/**
 * Publishes a message to the MQTT broker with the given topic and value.
 * @param topic the topic to publish the message to
 * @param Rvalue the value to publish
 */
void publishTopic(const char *topic, const char *Rvalue) {
  //publish the message to the specific topic
  mqttClient.beginMessage(topic);
  mqttClient.print(Rvalue);
  mqttClient.endMessage();
}


//set publishDataPeriod for sending messages (milliseconds)
unsigned long mqttPreMillis = 0;
char mqttBuffer[64];

/**
 * Publishes the current map data to the MQTT broker as a string.
 * The map data is represented as a matrix of numbers, where each number represents the type of cell in the grid.
 * The matrix is represented as a string of numbers separated by spaces, and each row is separated by a semicolon.
 * The function first determines the current state (grid localization or topology localization) and then
 * loops through the matrix, printing each value to the string buffer.
 * Finally, the function publishes the string buffer to the MQTT broker with the topic specified by the mapDataTopic variable.
 */
void publishMatrixMapData(){
  int index = 0;
  for(int i = 0; i < MATRIX_SIZE_X; i++){
    for(int j = 0; j < MATRIX_SIZE_Y; j++){
      if(currentState == GRID_LOCALIZATION){
        index += sprintf(mqttBuffer + index, "%d ", mapMatrix[i][j]);
      }else if(currentState == TOPO_LOCALIZATION){
        index += sprintf(mqttBuffer + index, "%d ", topoMatrix[i][j]);
      }
    }
    index += sprintf(mqttBuffer + index, ";");
  }

  publishTopic(mapDataTopic, mqttBuffer);
}

/**
 * Publishes the current position of the robot to the MQTT broker.
 * The position is formatted as "x y" where x and y are the coordinates
 * of the robot's current position. The message is published to the topic
 * specified by the robotPositionTopic variable.
 */

void publishRobotPosition(){
  sprintf(mqttBuffer, "%d %d", currentRobotPosition.x, currentRobotPosition.y);
  publishTopic(robotPositionTopic, mqttBuffer);
}

void publishRobotPossiblePosition(){
  if(isCalculatingPosition) return;
  int index = 0;
  if(possiblePositions.isEmpty()){
    // Serial.println("No possible positions to publish");
    return;
  }

  for(int i = 0; i < possiblePositions.length(); i++){
    Position current = possiblePositions.getByIndex(i);
    index += sprintf(mqttBuffer + index, "%d %d;", current.x, current.y);
  }

  publishTopic(gridLocalizationResponseTopic, mqttBuffer);
}

/**
 * Publishes the current path planned by the robot to the MQTT broker.
 * The path is represented as a string of x, y coordinates separated by spaces and semicolons.
 * For example, "0 0; 1 0; 2 0; 3 0" represents a path from (0,0) to (3,0).
 * If the path is empty, the function returns without publishing anything.
 * The function publishes the message to the topic specified by the robotPathPlanTopic variable.
 */
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

/**
 * Publishes the current sensor readings to the MQTT broker as follows:
 * - lidarTopic: front, back, left, right
 * - sonarTopic: left, right
 * - encoderTopic: left, right
 * - ledTopic: status of LED 0, 1, 2
 */
void publishRobotSensorData(){
  // lidar_data = RPC.call("read_lidars").as<struct lidar>();
  sprintf(mqttBuffer, "%d %d %d %d", lidar_data.front, lidar_data.back, lidar_data.left, lidar_data.right);
  publishTopic(lidarTopic, mqttBuffer);

  // sonar_data = RPC.call("read_sonars").as<struct sonar>();
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
    publishRobotPossiblePosition();
  }
}