#ifndef _NMNT_SERVO_HPP
#define _NMNT_SERVO_HPP

#include "Arduino.h"

// --------Servos-------//

struct NMNT_Servo {
  float freq; // (Hz)
  float phase;
};

const int numServo = 4;
const int PULSEMIN[] = {215, 200, 250, 230};       
const int PULSEMAX[] = {350, 280, 300, 315};

const float minServoFreq = 0.2;   // (Hz)
const float maxServoFreq = 4.0;   // (Hz)
const float randFreqAmnt = 0.05; // (rand freq deviation, 0 for no random)

const int SensorDistance = 2; // (meters)
const float stepsPerSeconds = 5.8 / float(SensorDistance * 2.0); // very crude

// different modes of behavior
enum Modes {MODE_DIRECT = 0, MODE_AVG, MODE_ROUND, NUM_MODES};
int MODE = MODE_DIRECT;

// MODE_AVG
const int numFreq = 5;
float servoFreqs[numFreq];
int freqIndex = 0;
float freqSum = 0;
float freqAvg = 0;

// MODE_ROUND
int sensorCounter = 0;

// not using
float freqSmooth = 0.9995;      // smoothing factor to lerp between freqs (0-1)


#endif
