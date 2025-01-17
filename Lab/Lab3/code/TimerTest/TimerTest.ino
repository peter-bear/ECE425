#include "mbed.h"

const int ledPin = 5;
volatile bool ledState = false;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  static int previousMillis = 0;
  int currentMillis = millis(); // Get current time in milliseconds
  
  // Check if 1 second has passed
  if(currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    
    // Toggle LED
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    
    Serial.println("Timer triggered!");
  }
}