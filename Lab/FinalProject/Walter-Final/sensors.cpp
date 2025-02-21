#include "sensors.h"

uint16_t wait = 100;
int16_t ft_lidar = 8;
int16_t bk_lidar = 9;
int16_t lt_lidar = 10;
int16_t rt_lidar = 11;

int16_t lidar_pins[4] = { 8, 9, 10, 11 };

int16_t leftSnr = 3;
int16_t rightSnr = 4;


struct lidar lidarData;
struct sonar sonarData;

struct lidar read_lidars() {
  return lidarData;
}

struct sonar read_sonars() {
  return sonarData;
}

// reads a lidar given a pin
int read_lidar(int pin) {
  // Wait for pulse
  int16_t t = pulseIn(pin, HIGH);

  int d = MAX_LIDAR_DISTANCE;  // Default to "no object"
  if (t != 0 && t <= 1850) {
    d = (t - 1000) * 3 / 40;
    if (d < 0)
      d = 0;
  }

  delay(10);  // More stable than delayMicroseconds
  return d;
}

// reads a sonar given a pin
int read_sonar(int pin) {
  float velocity((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
  uint16_t distance, pulseWidthUs;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);            // Set the trig pin High
  delayMicroseconds(10);              // Delay of 10 microseconds
  digitalWrite(pin, LOW);             // Set the trig pin Low
  pinMode(pin, INPUT);                // Set the pin to input mode
  pulseWidthUs = pulseIn(pin, HIGH);  // Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * velocity / 2.0;
  if (distance < 0 || distance > 50) {
    distance = 0;
  }
  return distance;
}

void setupSensors() {
  for (int i = 0; i < LIDAR_NUM; i++) {
    pinMode(lidar_pins[i], OUTPUT);
  }
  RPC.bind("read_lidars", read_lidars);  // bind a method to return the lidar data all at once
  RPC.bind("read_sonars", read_sonars);  // bind a method to return the lidar data all at once
}

void readSensors() {
  // Add delays between readings to allow sensor to stabilize
  lidarData.front = read_lidar(ft_lidar);
  // delay(10);
  lidarData.back = read_lidar(bk_lidar);
  // delay(10);
  lidarData.left = read_lidar(lt_lidar);
  // delay(10);
  lidarData.right = read_lidar(rt_lidar);
  // delay(10);

  sonarData.right = read_sonar(leftSnr);
  // delay(10);
  sonarData.left = read_sonar(rightSnr);
  // delay(10);
}