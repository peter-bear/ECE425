#include "core.h"

void setup() {
    int baudrate = 9600;
    randomSeed(analogRead(0));
    Serial.begin(baudrate);
    Serial.println("Robot starting...Put ON TEST STAND");

    RPC.begin();
    if (HAL_GetCurrentCPUID() == CM7_CPUID) {
        // if on M7 CPU, run M7 setup & loop
        setupM7();
        while (1) loopM7();
    } else {
        // if on M4 CPU, run M4 setup & loop
        setupM4();
        while (1) loopM4();
    }
}

void loop() {
    // Empty because we use while(1) loops in setup()
}