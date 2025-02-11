#include "encoder.h"


volatile long encoder[2] = { 0, 0 };  // interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = { 0, 0 };          // variable to hold encoder speed (left, right)
int accumTicks[2] = { 0, 0 };         // variable to hold accumulated ticks since last reset

// interrupt function to count left encoder tickes
void LwheelSpeed() {
  encoder[LEFT_ENCODER]++;  // count the left wheel encoder interrupts
}

// interrupt function to count right encoder ticks
void RwheelSpeed() {
  encoder[RIGHT_ENCODER]++;  // count the right wheel encoder interrupts
}

/*this function will reset the encoders*/
void resetEncoder() {
  encoder[LEFT_ENCODER] = 0;   // clear the left encoder data buffer
  encoder[RIGHT_ENCODER] = 0;  // clear the right encoder data buffer
}

/**
 * @brief Sets the position of both the left and right stepper motor encoders.
 * 
 * This function updates the encoder positions for both the left and right stepper motors
 * to the specified position.
 * 
 * @param position The new position to set for both the left and right encoders.
 */
void setStepperCounter(long position) {
    encoder[LEFT_ENCODER] = position;
    encoder[RIGHT_ENCODER] = position;
  }

// function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                                                    // print manager timer
  if (millis() - timer > 100) {                                                      // print encoder data every 100 ms or so
    lastSpeed[LEFT_ENCODER] = encoder[LEFT_ENCODER];                                 // record the latest left speed value
    lastSpeed[RIGHT_ENCODER] = encoder[RIGHT_ENCODER];                               // record the latest right speed value
    accumTicks[LEFT_ENCODER] = accumTicks[LEFT_ENCODER] + encoder[LEFT_ENCODER];     // record accumulated left ticks
    accumTicks[RIGHT_ENCODER] = accumTicks[RIGHT_ENCODER] + encoder[RIGHT_ENCODER];  // record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT_ENCODER]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT_ENCODER]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT_ENCODER]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT_ENCODER]);
    encoder[LEFT_ENCODER] = 0;   // clear the left encoder data buffer
    encoder[RIGHT_ENCODER] = 0;  // clear the right encoder data buffer
    timer = millis();            // record current time since program started
  }
}


void attachEncoders() {
  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);  // init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);  // init the interrupt mode for the right encoder
}