#include <Adafruit_PWMServoDriver.h>
#include "nmnt_dist_sensor.hpp"
#include "nmnt_servo.hpp"

Adafruit_PWMServoDriver servoShield = Adafruit_PWMServoDriver();

NMNT_Servo servo[numServo];


void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));

  // Sensors
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input

  // initialize Servos
  servoShield.begin();
  servoShield.setPWMFreq(60);

  for (int i = 0; i < numServo; i++) {
    servo[i].freq = minServoFreq;     // + randomFloat(-randFreq, randFreq); // Initial frequency
    servo[i].phase = random(TWO_PI);  //i / float(numServo - 1) * PI;
    servoShield.setPWM(i, 0, PULSEMIN);
  }

  // MODE_AVG
  for (int i = 0; i < numFreq; i++) {
    servoFreqs[i] = servo[0].freq;// + randomFloat(-randFreq, randFreq);
    freqSum += servoFreqs[i];
  }
  Serial.println("System start!");
}


void loop() {
  float cm1 = 0;
  float cm2 = 0;                         // Used to save the distance in cm in every frame
  delayMicroseconds(5);

  // right
  //  cm1 = range(trigPin1, echoPin1);    // Calculates the current distance in cm (see range function)
  //  delayMicroseconds(10);
  //  average1[0] = cm1;                      // The first item of the array is the current distance
  //  lastSteps1[0] = cm1;
  //
  //  // left
  //  cm2 = range(trigPin2, echoPin2);
  //  delayMicroseconds(10);
  //  average2[0] = cm2;
  //  lastSteps2[0] = cm2;
  //
  //  boolean sensL = walkingBy(lastSteps2, average2);
  //  boolean sensR = walkingBy(lastSteps1, average1);

  // debug
  boolean sensL = false;
  boolean sensR = false;
  if (Serial.available() > 0) {
    char a = (char)Serial.read();
    //      Serial.write(a);

    if (a == 'l') {
      sensL = true;
    }
    else if (a == 'r') {
      sensR = true;
    }
  }

  //  Serial.print("l ");
  //  Serial.print(sensL);
  //  Serial.print(" r ");
  //  Serial.println(sensR);
  //  return;

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
      // time-out
      if (millis() - debounceL > sensorTimeOut) {
        Serial.println("Took too long");
        timerStart = false;
      }
      else if (sensR == HIGH) {
        // left debounce time is interval
        sensorInterval = millis() - debounceL;
        Serial.print("Arrived right at: ");
        Serial.println(sensorInterval);
        timerStart = false;
        debounceR = millis();
        setSensorValue = true;
      }
    }

    // Right sensor was first
    // Stop timing when the left sensor is HIGH.
    else {
      // time-out
      if (millis() - debounceR > sensorTimeOut) {
        Serial.println("Took too long");
        timerStart = false;
      }

      else if (sensL == HIGH) {
        // right debounce time is interval
        sensorInterval = millis() - debounceR;
        Serial.print("Arrived left at: ");
        Serial.println(sensorInterval);
        timerStart = false;
        debounceL = millis();
        setSensorValue = true;
      }
    }
  }


  //-----Servos------//


  // if new sensor value was set
  if (setSensorValue) {
    setSensorValue = false;

    // Calculate servo speed from dist sensor interval
    float distFreq = 1.0 / float((sensorInterval * 0.001) / stepsPerSeconds);

    if (distFreq >= minServoFreq && distFreq <= maxServoFreq) {
      sleepTime = millis();

      // if in sleepmode, go to different mode
      if (MODE == MODE_SLEEP) {
//        MODE = MODE_DIRECT;
        MODE = MODE_ROUND;
        Serial.println("Out of sleep mode");
      }

      // Set servo freqs according to mode
      switch (MODE) {

        case MODE_DIRECT:
          for (int i = 0; i < numServo; i++) {
            servo[i].freq = distFreq + randomFloat(-randFreq, randFreq);
          }
          break;

        case MODE_ROUND:
          servo[sensorCounter].freq = distFreq;
          sensorCounter++;

          // go to direct mode after all servos have been set
          if (sensorCounter >= numServo) {
            sensorCounter = 0;
            MODE = MODE_DIRECT;
          }
          break;

          //        case MODE_AVG:
          //          // Get average freq of n sensor readings
          //          freqSum -= servoFreqs[freqIndex];
          //          servoFreqs[freqIndex] = distFreq;
          //          freqSum += servoFreqs[freqIndex];
          //          freqIndex = (freqIndex + 1) % numFreq;
          //          freqAvg = freqSum / (float)numFreq;
          //
          //          for (int i = 0; i < numServo; i++) {
          //            servo[i].freq = freqAvg + randomFloat(-randFreq, randFreq);
          //          }
          //          break;
      }

      // Set phase direction (in every mode);
      for (int i = 0; i < numServo; i++) {
        if (fromLeft) servo[i].phase = i / float(numServo - 1) * PI;
        else servo[i].phase =  (1.0 - i / float(numServo - 1)) * PI;
      }
    }
  }

  // Check if there is inactivity for more then systemSleep time
  // If so, set servos to minfreq
  if (millis() - sleepTime > systemSleep) {
    if (MODE != MODE_SLEEP) {
      Serial.println("No activity for too long, sleeping");
      MODE = MODE_SLEEP;
      for (int i = 0; i < numServo; i++) {
        servo[i].freq = minServoFreq + randomFloat(-0.05, 0.05);
      }
    }
  }

  float time_s = millis() * 0.001;

  for (int i = 0; i < numServo; i++) {
    //   servo[i].freq = lerp(servo[i].freq , servo[i].targetFreq , freqSmooth);

    float phi = cos(TWO_PI * time_s * servo[i].freq + servo[i].phase);

    int PWM = (int)floatMap(phi, -1.0, 1.0, PULSEMIN[i], PULSEMAX[i]);
    servoShield.setPWM(i, 0, PWM);
  }
}

float floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float randomFloat(float minR, float maxR) {
  unsigned int res = 10000;
  int randInt = random(res);

  return floatMap(randInt, 0, res - 1, minR, maxR);
}

float lerp(float x, float y, float m) {
  return m * x + (1.0 - m) * y;
}
