#ifndef _NMNT_DIST_SENSOR_HPP_
#define _NMNT_DIST_SENSOR_HPP_

#include "Arduino.h"
//#include <NewPing.h>

// defines pins numbers
const int trigPin1 = 9;       // the pin trig should be connected to 9  //right
const int echoPin1 = 10;      // the pin echo should be connected to 10 
const int trigPin2 = 5; // left
const int echoPin2 = 6;

// defines variables for distance measurement
long duration1;
int distance1;
long duration2;
long distance2;

// defines arrays to measure the average distance over time (used for calibration)
const int numAvg = 100;
const int numStep = 10;
unsigned long average1[numAvg];
unsigned long average2[numAvg];
unsigned long lastSteps1[numStep];
unsigned long lastSteps2[numStep];

// Timing between sensors
boolean fromLeft = false;
boolean timerStart = false;
unsigned long sensorInterval = 0;
bool setSensorValue = false;

// Debouncing
unsigned long debounceL = 0;
unsigned long debounceR = 0;
unsigned int bounceTime = 2000;   // (ms)
unsigned long timeOutMax = 10000; // (ms)

unsigned long range (byte trig, byte echo) {          // Calculates the distance in cm

  digitalWrite(trig, LOW);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long duration = pulseIn(echo, HIGH);
  return duration / 58.2;

}

unsigned long calculateAverage(long input[], int arraySize) {   // Calculates the average of the array that is used for calibraton

  long sum = 0;
  long average;

  for (int i = arraySize - 1; i > 0; i--) {
    input[i] = input[i - 1];
  }

  for (int j = 0; j < arraySize; j++) {
    sum += input[j];
  }

  average = sum / arraySize;

  return average;

}

//unsigned long lastSteps(long input[]) {         // Calculates the average of the last three distances (could also have used the previous function, but C does not have an array.length() function and mweh)
//
//  long sum = 0;                                 // The reason for this function is that the sensor is sometimes faulty, but it's always only one number that's faulty.
//  long average;
//
//  for (int i = 2; i > 0; i--) {
//    input[i] = input[i - 1];
//  }
//
//  for (int j = 0; j < 3; j++) {
//    sum += input[j];
//  }
//
//  average = sum / 3;
//
//  return average;
//
//}

unsigned long walkingBy(long stepping[], long input[]) {
  long margin;

  long currentDist = calculateAverage(stepping, numStep);           // Calculates the average of the last three distances
  bool someoneWalksBySensor;
                                                                    // Calculates the average distance in the last 100 frames
  margin = calculateAverage(input, numAvg) * 0.75;                  // Someone is walking by if they're at least 25% closer to the server than the average distance

  if (currentDist < margin) {                                       // Checks if someone is walking by
    someoneWalksBySensor = true;
  }
  else {
    someoneWalksBySensor = false;
  }
  return someoneWalksBySensor;
}

#endif
