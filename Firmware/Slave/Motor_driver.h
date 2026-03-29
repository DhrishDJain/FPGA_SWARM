#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "config.h"  // was ../config.h  (if it had one)
#include <Arduino.h>


class MotorControl {
private:
  uint8_t leftIN1, leftIN2, leftPWM;
  uint8_t rightIN1, rightIN2, rightPWM;

public:
  int currentSpeed;
  String currentState;

  MotorControl(uint8_t lIN1, uint8_t lIN2, uint8_t lPWM, uint8_t rIN1,
               uint8_t rIN2, uint8_t rPWM)
    : leftIN1(lIN1), leftIN2(lIN2), leftPWM(lPWM), rightIN1(rIN1),
      rightIN2(rIN2), rightPWM(rPWM), currentSpeed(MOTOR_DEFAULT_SPEED),
      currentState("STOP") {}
  bool initialize() {
    pinMode(leftIN1, OUTPUT);
    pinMode(leftIN2, OUTPUT);
    pinMode(rightIN1, OUTPUT);
    pinMode(rightIN2, OUTPUT);

    bool leftOK = ledcAttach(leftPWM, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
    bool rightOK = ledcAttach(rightPWM, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);

    Serial.printf("[MOTOR] ledcAttach GPIO%d: %s\n", leftPWM, leftOK ? "OK" : "FAIL");
    Serial.printf("[MOTOR] ledcAttach GPIO%d: %s\n", rightPWM, rightOK ? "OK" : "FAIL");

    stop();
    Serial.println("Motors ready");
    return true;
  }


  void stop() {
    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, LOW);
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, LOW);
    ledcWrite(leftPWM, 0);
    ledcWrite(rightPWM, 0);
    currentState = "STOP";
  }

  void forward(int speed = -1) {
    if (speed == -1)
      speed = currentSpeed;
    driveRaw(speed, speed);
    currentState = "FORWARD";
  }

  void backward(int speed = -1) {
    if (speed == -1)
      speed = currentSpeed;
    driveRaw(-speed, -speed);
    currentState = "BACKWARD";
  }

  void left(int speed = -1) {
    if (speed == -1)
      speed = currentSpeed;
    driveRaw(-speed, speed);
    currentState = "LEFT";
  }

  void right(int speed = -1) {
    if (speed == -1)
      speed = currentSpeed;
    driveRaw(speed, -speed);
    currentState = "RIGHT";
  }

  void setSpeed(int speed) {
    currentSpeed = constrain(speed, 0, 255);
  }

  void driveRaw(int leftSpeed, int rightSpeed) {
    // Left motor
    if (leftSpeed >= 0) {
      digitalWrite(leftIN1, HIGH);
      digitalWrite(leftIN2, LOW);
      ledcWrite(leftPWM, constrain(leftSpeed, 0, 255));
    } else {
      digitalWrite(leftIN1, LOW);
      digitalWrite(leftIN2, HIGH);
      ledcWrite(leftPWM, constrain(-leftSpeed, 0, 255));
    }

    // Right motor (inverted mounting)
    if (rightSpeed >= 0) {
      digitalWrite(rightIN1, HIGH);
      digitalWrite(rightIN2, LOW);
      ledcWrite(rightPWM, constrain(rightSpeed, 0, 255));
    } else {
      digitalWrite(rightIN1, LOW);
      digitalWrite(rightIN2, HIGH);
      ledcWrite(rightPWM, constrain(-rightSpeed, 0, 255));
    }

    // ✅ Debug print every call
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 300) {
      lastPrint = millis();
      // Serial.printf("[MOTOR] L=%d (IN1=%d,IN2=%d)  R=%d (IN1=%d,IN2=%d)\n",
      //               leftSpeed,
      //               leftSpeed >= 0 ? 1 : 0,
      //               leftSpeed >= 0 ? 0 : 1,
      //               rightSpeed,
      //               rightSpeed >= 0 ? 0 : 1,  // right is inverted
      //               rightSpeed >= 0 ? 1 : 0);
    }

    currentState = "NAV";
  }


  bool isReady() {
    return true;
  }
};

#endif
