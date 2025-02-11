#include "led.h"

int leds[3] = { 5, 6, 7 };  // array of LED pin numbers
int pauseTime = 2500;  // time before robot moves

int LED_Status[3] = {0, 0, 0};

void initLEDs(){
    pinMode(enableLED, OUTPUT);                   // set enable LED as output
    digitalWrite(enableLED, LOW);                 // turn off enable LED
    pinMode(redLED, OUTPUT);                      // set red LED as output
    pinMode(grnLED, OUTPUT);                      // set green LED as output
    pinMode(ylwLED, OUTPUT);                      // set yellow LED as output
    digitalWrite(redLED, HIGH);                   // turn on red LED
    digitalWrite(ylwLED, HIGH);                   // turn on yellow LED
    digitalWrite(grnLED, HIGH);                   // turn on green LED
    delay(pauseTime / 5);                         // wait 0.5 seconds
    digitalWrite(redLED, LOW);                    // turn off red LED
    digitalWrite(ylwLED, LOW);                    // turn off yellow LED
    digitalWrite(grnLED, LOW);                    // turn off green LED
}

void allOFF() {
    for (int i = 0; i < 3; i++) {
      digitalWrite(leds[i], LOW);
      LED_Status[i] = 0;
    }
    
  }

void updateLEDs() {
    allOFF();
  
    if (currentState == FOLLOWING_RIGHT) {
      digitalWrite(redLED, HIGH);
      LED_Status[0] = 1;
      digitalWrite(ylwLED, HIGH);
      LED_Status[2] = 1;
    } else if (currentState == FOLLOWING_LEFT) {
      digitalWrite(grnLED, HIGH);
      LED_Status[1] = 1;
      digitalWrite(ylwLED, HIGH);
      LED_Status[2] = 1;
    } else if (currentState == FOLLOWING_CENTER) {
      digitalWrite(redLED, HIGH);
      LED_Status[0] = 1;
      digitalWrite(grnLED, HIGH);
      LED_Status[1] = 1;
      digitalWrite(ylwLED, HIGH);
      LED_Status[2] = 1;
    } else if (currentState == RANDOM_WANDER) {
      digitalWrite(grnLED, HIGH);
      LED_Status[1] = 1;
    } else if (currentState == TOO_CLOSE || currentState == CLOSE_TO_RIGHT) {
      digitalWrite(ylwLED, HIGH);
      LED_Status[2] = 1;
    } else if (currentState == TOO_FAR || currentState == CLOSE_TO_LEFT) {
      digitalWrite(redLED, HIGH);
      LED_Status[0] = 1;
    } else if (currentState == INSIDE_CORNER) {
      digitalWrite(redLED, HIGH);
      LED_Status[0] = 1;
      digitalWrite(grnLED, HIGH);
      LED_Status[1] = 1;
    } else if (currentState == COLLIDE_BEHAVIOR) {
      digitalWrite(redLED, HIGH);
      LED_Status[0] = 1;
    } else if (currentState == RUNAWAY_BEHAVIOR) {
      digitalWrite(ylwLED, HIGH);
      LED_Status[2] = 1;
    } else if (currentState == FOLLOW_BEHAVIOR) {
      digitalWrite(redLED, HIGH);
      LED_Status[0] = 1;
      digitalWrite(grnLED, HIGH);
      LED_Status[1] = 1;
    }
  }

