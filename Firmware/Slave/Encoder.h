#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

class Encoder {
private:
  uint8_t pinC1, pinC2; 
  volatile long* encoderCount;

  long lastCount;
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
    // Direction = XOR(C1, C2) — same formula for BOTH rising and falling edges.
    // Old code used (c2==c1) on falling edge which is the inverse, causing each
    // rising +1 to be cancelled by a falling -1, netting zero forever.
    *encoderCount += (c1State ^ c2State) ? 1 : -1;
    *lastC1State = c1State;
  }



  bool initialize() {
    pinMode(pinC1, INPUT_PULLUP);
    pinMode(pinC2, INPUT_PULLUP);
    delay(10);

    *lastC1State = digitalRead(pinC1);

    Serial.print("Encoder ready - C1: GPIO");
    Serial.print(pinC1);
    Serial.print(", C2: GPIO");
    Serial.println(pinC2);
    return true;
  }

  void update() {
    long currentCount;

    noInterrupts();
    currentCount = *encoderCount;  // atomic snapshot
    interrupts();

    long pulseDelta = currentCount - lastCount;

    if (pulseDelta == 0) {
      return;
    }

    float distanceMM = pulseDelta * distancePerPulseMM;

    distanceTraveledCM += distanceMM / 10.0;
    totalDistanceCM += abs(distanceMM) / 10.0;

    lastCount = currentCount;

    // DEBUG
    // Serial.printf("[ENC] count=%ld delta=%ld net=%.2fcm total=%.2fcm\n",
    //               currentCount, pulseDelta,
    //               distanceTraveledCM, totalDistanceCM);
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
