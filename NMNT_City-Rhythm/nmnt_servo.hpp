#ifndef _NMNT_SERVO_HPP
#define _NMNT_SERVO_HPP

#include "Arduino.h"

// --------Servos-------//

struct NMNT_Servo {
  float freq; // (Hz)
  float phase;
};

const int numServo = 4;
const int PULSEMIN[numServo] = {215, 200, 250, 230};       
const int PULSEMAX[numServo] = {350, 280, 300, 315};

const float minServoFreq = 0.2;     // (Hz)
const float maxServoFreq = 4.0;     // (Hz)
const float randFreq = 0.05;        // (random freq deviation, 0 for no random)

const int sensorDistance = 2; // (meters)
const float stepsPerSeconds = 5.8 / float(sensorDistance * 2.0); // very crude

// Different modes of behavior
enum MODES {MODE_DIRECT=0, MODE_ROUND, MODE_AVG, MODE_SLEEP};
int MODE = MODE_SLEEP; //MODE_DIRECT;                               // Start in sleep mode

// MODE_ROUND
int sensorCounter = 0;


///// not using ////
float freqSmooth = 0.9995;      // smoothing factor to lerp between freqs (0-1)

// MODE_AVG
const int numFreq = 5;
float servoFreqs[numFreq];
int freqIndex = 0;
float freqSum = 0;
float freqAvg = 0;

///// not using ////

#endif
