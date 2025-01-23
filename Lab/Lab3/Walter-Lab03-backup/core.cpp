#include "core.h"
#include <Arduino.h>

void setupM7() {
    init_stepper();
    attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);
    delay(1000);
}

void loopM7() {
    digitalWrite(redLED, LOW);
    digitalWrite(ylwLED, LOW);
    digitalWrite(grnLED, LOW);

    while (true) {
        // Uncomment the behavior you want to use
        // smartWanderBehavior();
        smartFollowBehavior();
        // runawayBehavior();
        // followWallBehavior();
    }
}

void setupM4() {
    for (int i = 0; i < numOfSens; i++) {
        pinMode(lidar_pins[i], OUTPUT);
    }
    RPC.bind("read_lidars", read_lidars);
    RPC.bind("read_sonars", read_sonars);
}

void loopM4() {
    lidar_data_M7_read.front = read_lidar(ft_lidar);
    delay(50);
    lidar_data_M7_read.back = read_lidar(bk_lidar);
    delay(50);
    lidar_data_M7_read.left = read_lidar(lt_lidar);
    delay(50);
    lidar_data_M7_read.right = read_lidar(rt_lidar);
    delay(50);
}