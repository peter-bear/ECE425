#pragma once
#include <Arduino.h>

// define encoder pins
#define LEFT_ENCODER 0   // left encoder
#define RIGHT_ENCODER 1  // right encoder

#define ltEncoder 18  // left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
#define rtEncoder 19  // right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)


#define encoderRatio 20  // ratio between the encoder ticks and steps

void LwheelSpeed();
void RwheelSpeed();
void resetEncoder();
void setStepperCounter(long position);
void print_encoder_data();
void attachEncoders();

extern volatile long encoder[2];
extern int lastSpeed[2];
extern int accumTicks[2];