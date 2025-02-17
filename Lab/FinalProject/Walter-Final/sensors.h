#pragma once
#include <Arduino.h>
#include "RPC.h"

#define MAX_LIDAR_DISTANCE 40
#define LIDAR_NUM 4

// a struct to hold lidar data
struct lidar {
  // this can easily be extended to contain sonar data as well
  int front;
  int back;
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(front, back, left, right);  // https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
};

// a struct to hold sonar data
struct sonar {
  // this can easily be extended to contain sonar data as well
  int left;
  int right;
  // this defines some helper functions that allow RPC to send our struct (I found this on a random forum)
  MSGPACK_DEFINE_ARRAY(left, right);  // https://stackoverflow.com/questions/37322145/msgpack-to-pack-structures https://www.appsloveworld.com/cplus/100/391/msgpack-to-pack-structures
};

struct sonar read_sonars();
struct lidar read_lidars();

int read_lidar(int pin);
int read_sonar(int pin);
void setupSensors();
void readSensors();

extern struct lidar lidar_data;
extern struct sonar sonar_data;
