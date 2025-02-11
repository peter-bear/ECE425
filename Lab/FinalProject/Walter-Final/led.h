#pragma once
#include <Arduino.h>
#include "types.h"

// state LEDs connections
#define redLED 5            // red LED for displaying states
#define grnLED 6            // green LED for displaying states
#define ylwLED 7            // yellow LED for displaying states
#define enableLED 13        // stepper enabled LED

void initLEDs();
void updateLEDs();
void allOFF();

extern int LED_Status[3];