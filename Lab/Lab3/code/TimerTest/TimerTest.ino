#include "mbed.h"

const int ledPin = 5;
volatile bool ledState = false;
static int previousMillis = 0;

// Function to check if 1 second has passed
bool isOneSecondPassed() {
    int currentMillis = millis();
    if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;
        return true;
    }
    return false;
}

// Function to toggle LED state
void toggleLED() {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    Serial.println("Timer triggered!");
}

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
}

void loop() {
    if (isOneSecondPassed()) {
        toggleLED();
    }
}