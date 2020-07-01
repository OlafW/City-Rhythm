#ifndef _NMNT_SERVO_HPP
#define _NMNT_SERVO_HPP

#include "Arduino.h"

// --------Servos-------//

struct NMNT_Servo {
  float freq;     //   (Hz)
  float phase;
};

const int numServo = 4;
const int PULSEMIN[] = {215, 200, 250, 230};       
const int PULSEMAX[] = {350, 280, 300, 315};

const float minServoFreq = 0.2; // (Hz)
const float maxServoFreq = 4;   // (Hz)
float freqSmooth = 0.9995;      // smoothing factor to lerp between freqs (0-1)

const int SensorDistance = 2; // (meters)
const float stepsPerSeconds = 5.8 / float(SensorDistance * 2.0); // very crude


// different behaviors
const int MODE_DIRECT = 0;
const int MODE_AVG = 1;
const int MODE_ROUND = 2;

int MODE = MODE_DIRECT;

// MODE_AVG
const int numFreq = 5;
float servoFreqs[numFreq];
unsigned int freqIndex = 0;
float freqSum = 0;
float freqAvg = 0;

// MODE_ROUND
unsigned int sensorCounter = 0;


#endif
