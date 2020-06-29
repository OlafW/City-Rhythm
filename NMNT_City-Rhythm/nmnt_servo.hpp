#ifndef _NMNT_SERVO_HPP
#define _NMNT_SERVO_HPP

#include "Arduino.h"

// --------Servos-------//
const int numServo = 4;
const int PULSEMIN[] = {215, 200, 250, 230};       
const int PULSEMAX[] = {350, 280, 300, 315};

const float minServoFreq = 0.2; // (Hz)
const float maxServoFreq = 4;   // (Hz)
float freqSmooth = 0.9995;      // smoothing factor to lerp between freqs (0-1)

const int SensorDistance = 2; // (meters)
const float stepsPerSeconds = 5.8 / float(SensorDistance * 2.0); // very crude

struct NMNT_Servo {
  float freq;     //   (Hz)
  float targetFreq;
  float phase;
};

#endif
