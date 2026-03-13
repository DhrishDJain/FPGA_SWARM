#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

class Encoder {
private:
  uint8_t pinC1, pinC2;
  volatile long* encoderCount;
  volatile long lastCount;
  volatile int* lastC1State;
  float distancePerPulseMM;

public:
  float distanceTraveledCM;  // Net distance (can be negative)
  float totalDistanceCM;     // Total distance (always positive, like odometer)

  Encoder(uint8_t c1Pin, uint8_t c2Pin, volatile long* count, volatile int* c1State) {
    pinC1 = c1Pin;
    pinC2 = c2Pin;
    encoderCount = count;
    lastC1State = c1State;
    lastCount = 0;
    distanceTraveledCM = 0.0;
    totalDistanceCM = 0.0;

    float wheelCircumferenceMM = PI * WHEEL_DIAMETER_MM;
    distancePerPulseMM = wheelCircumferenceMM / PULSES_PER_REV;
  }

  void IRAM_ATTR handleInterrupt() {
    int c1State = digitalRead(pinC1);
    int c2State = digitalRead(pinC2);

    if (c1State != *lastC1State) {
      if (c1State == HIGH) {
        *encoderCount += (c2State == HIGH) ? 1 : -1;  // Swapped LOW/HIGH
      } else {
        *encoderCount += (c2State == LOW) ? 1 : -1;  // Swapped LOW/HIGH
      }
    }
    *lastC1State = c1State;
  }


  bool initialize() {
    pinMode(pinC1, INPUT);
    pinMode(pinC2, INPUT);
    delay(10);

    *lastC1State = digitalRead(pinC1);

    Serial.print("Encoder ready - C1: GPIO");
    Serial.print(pinC1);
    Serial.print(", C2: GPIO");
    Serial.println(pinC2);
    return true;
  }

  void update() {
    long pulseDelta = *encoderCount - lastCount;

    if (pulseDelta != 0) {
      float distanceMM = pulseDelta * distancePerPulseMM;

      // Net distance (can be negative) - for position tracking
      distanceTraveledCM += distanceMM / 10.0;  // Removed abs()!

      // Total distance (always positive) - like odometer
      totalDistanceCM += abs(distanceMM) / 10.0;

      lastCount = *encoderCount;
    }
  }

  void reset() {
    *encoderCount = 0;
    lastCount = 0;
    distanceTraveledCM = 0.0;
    totalDistanceCM = 0.0;
  }

  long getCount() {
    return *encoderCount;
  }
  float getNetDistance() {
    return distanceTraveledCM;
  }
  float getTotalDistance() {
    return totalDistanceCM;
  }
  bool isReady() {
    return true;
  }
};

#endif
