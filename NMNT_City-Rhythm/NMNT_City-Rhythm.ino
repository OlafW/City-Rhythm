#include <Adafruit_PWMServoDriver.h>
#include "nmnt_dist_sensor.hpp"

Adafruit_PWMServoDriver servoShield = Adafruit_PWMServoDriver();

// --------Servos-------//
const int numServo = 1;
const int PULSEMIN = 200;     // Minimum pulse: ~125
const int PULSEMAX = 300;     // Maximum pulse: ~550

float servoFreq[numServo];     // Servo speed (Hz)
float targetFreq[numServo];

float minServoFreq = 0.2; // (Hz)
float maxServoFreq = 4;   // (Hz)
float freqSmooth = 0.995; // smoothing factor to lerp between freqs (0-1)

void setup() {
  Serial.begin(9600);

  servoShield.begin();
  servoShield.setPWMFreq(60);

  for (int i = 0; i < numServo; i++) {
    servoFreq[i] = 2; // Initial frequency
    targetFreq[i] = servoFreq[i];
    servoShield.setPWM(i, 0, PULSEMIN);
  }

  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
}


void loop() {
  float cm1, cm2;                         // Used to save the distance in cm in every frame
  delayMicroseconds(5);

  cm1 = range(trigPin1, echoPin1);        // Calculates the current distance in cm (see range function)
  delayMicroseconds(10);
  average1[0] = cm1;                      // The first item of the array is the current distance
  lastSteps1[0] = cm1;

  cm2 = range(trigPin2, echoPin2);
  delayMicroseconds(10);
  average2[0] = cm2;
  lastSteps2[0] = cm2;

  boolean sensL = walkingBy(lastSteps2, average2);
  boolean sensR = walkingBy(lastSteps1, average1);

  // If we haven't detected anything yet
  if (!timerStart) {
    // If the left sensor detects a state from LOW to HIGH first
    if (sensL && !sensR && millis() - debounceL > bounceTime) {
      fromLeft = true;
      timerStart = true;
      debounceL = millis();
      Serial.println("From left");
    }

    // If the right sensor detects a state from LOW to HIGH first
    else if (sensR && !sensL && millis() - debounceR > bounceTime) {
      fromLeft = false;
      timerStart = true;
      debounceR = millis();
      Serial.println("From right");
    }
  }

  // We've detected one sensor and started timing.
  // See when the other sensor detects.
  else {
    // Left sensor was first.
    // Stop timing when the right sensor is HIGH.
    if (fromLeft) {
      if (sensR == HIGH) {
        // left debounce time is interval
        sensorInterval = millis() - debounceL;
        Serial.print("Arrived right at: ");
        Serial.println(sensorInterval);
        timerStart = false;
        debounceR = millis();
      }
    }

    // Right sensor was first
    // Stop timing when the left sensor is HIGH.
    else if (sensL == HIGH) {
      // right debounce time is interval
      sensorInterval = millis() - debounceR;
      Serial.print("Arrived left at: ");
      Serial.println(sensorInterval);
      timerStart = false;
      debounceL = millis();
    }
  }

  //-----Servos------//

  // Set the servo speed corresponding to sonar interval
  float sonarFreq = 1000.0 / (float)sensorInterval;
  
  if (sonarFreq > minServoFreq && sonarFreq < maxServoFreq) {
    for (int i = 0; i < numServo; i++) {
      targetFreq[i] = sonarFreq;
      freqSmooth = floatMap(abs(servoFreq[i] - targetFreq[i]), 0, maxServoFreq-minServoFreq, 0.75, 0.99);
    }
  }

  float t = millis() * 0.001; // time (s)

  for (int i = 0; i < numServo; i++) {
    servoFreq[i] = lerp(servoFreq[i], targetFreq[i], freqSmooth);

    float phi = sin(TWO_PI * t * servoFreq[i]);

    int PWM = (int)floatMap(phi, -1.0, 1.0, PULSEMIN, PULSEMAX);
    servoShield.setPWM(i, 0, PWM);
  }
}

float floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float lerp(float x, float y, float m) {
  return m * x + (1.0 - m) * y;
}
