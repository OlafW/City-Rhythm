#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servoShield = Adafruit_PWMServoDriver();

// --------Servos-------//
const int numServo = 1;
const int PULSEMIN = 125; // Minimum pulse: ~125
const int PULSEMAX = 225; // Maximum pulse: ~550

float freq[numServo]; // Servo speed (Hz)
float modFreq = 0.05; // Hz
float modAmp = 0; // amount of modulation (Hz)


// --------PIR Sensors-------//

// A struct to represent a pair of PIR sensors
struct PIR {
  int left;   // left sensor
  int right;  // right sensor

  boolean prevL = false;
  boolean prevR = false;

  boolean fromLeft = false;
  boolean timerStart = false;
  unsigned long startT = 0;
  unsigned long interval = 0;
};

const int numSensor = 1;  // number of pairs
PIR pir[numSensor];

int calibrationTime = 10; // seconds


void setup() {
  Serial.begin(9600);

  servoShield.begin();
  servoShield.setPWMFreq(60);

  for (int i = 0; i < numServo; i++) {
    freq[i] = 0.5; // Initial frequency
    servoShield.setPWM(i, 0, PULSEMIN);
  }

  int pinOffset = 6;
  for (int i = 0; i < numSensor; i++) {
    pir[i].left = i + pinOffset;
    pir[i].right = i + pinOffset + 1;

    pinMode(pir[i].left, INPUT);
    pinMode(pir[i].right, INPUT);
    digitalWrite(pir[i].left, LOW);
    digitalWrite(pir[i].right, LOW);
  }

  // give the sensors some time to calibrate
  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
}


void loop() {

  // Read through all the sensors
  for (int i = 0; i < numSensor; i++) {
    boolean sensL = digitalRead(pir[i].left);
    boolean sensR = digitalRead(pir[i].right);

    // We haven't detected anything yet
    if (!pir[i].timerStart) {
      // If the left sensor detects a state from LOW to HIGH first
      if ((!pir[i].prevL && sensL) && (!pir[i].prevR && !sensR)) {
        pir[i].fromLeft = true;
        pir[i].timerStart = true;
        pir[i].startT = millis();
        Serial.println("From left");
      }
      // If the right sensor detects a state from LOW to HIGH first
      else if ((!pir[i].prevR && sensR) && (!pir[i].prevL && !sensL)) {
        pir[i].fromLeft = false;
        pir[i].timerStart = true;
        pir[i].startT = millis();
        Serial.println("From right");
      }
    }

    // We've detected one sensor and started timing.
    // See when the other sensor detects.
    else {
      // Left sensor was first.
      // Stop timing when the right sensor detects.
      if (pir[i].fromLeft) {
        if (sensR == HIGH) {
          pir[i].interval = millis() - pir[i].startT;
          Serial.print("Arrived right at: ");
          Serial.println(pir[i].interval);
          pir[i].timerStart = false;
        }
      }

      // Right sensor was first
      // Stop timing when the left sensor detects.
      else if (sensL == HIGH) {
        pir[i].interval = millis() - pir[i].startT;
        Serial.print("Arrived left at: ");
        Serial.println(pir[i].interval);
        pir[i].timerStart = false;
      }
    }
    
    pir[i].prevL = sensL;
    pir[i].prevR = sensR;
  }

  //-----Set the servos------//
//  float t = millis() * 0.001;
//  float mod = (sin(TWO_PI * t * modFreq) * 0.5 + 0.5) * modAmp;
//
//  for (int i = 0; i < numServo; i++) {
//    float phi = sin((TWO_PI * t * freq[i]) + mod);
//
//    int PWM = (int)floatMap(phi, -1.0, 1.0, PULSEMIN, PULSEMAX);
//    servoShield.setPWM(i, 0, PWM);
//  }
}

float floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
