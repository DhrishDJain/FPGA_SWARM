#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"

class MotorControl {
private:
  uint8_t leftIN1, leftIN2, leftPWM;
  uint8_t rightIN1, rightIN2, rightPWM;

public:
  int currentSpeed;
  String currentState;

  MotorControl(uint8_t lIN1, uint8_t lIN2, uint8_t lPWM,
               uint8_t rIN1, uint8_t rIN2, uint8_t rPWM) {
    leftIN1 = lIN1;
    leftIN2 = lIN2;
    leftPWM = lPWM;
    rightIN1 = rIN1;
    rightIN2 = rIN2;
    rightPWM = rPWM;
    currentSpeed = MOTOR_DEFAULT_SPEED;
    currentState = "STOP";
  }

  bool initialize() {
    pinMode(leftIN1, OUTPUT);
    pinMode(leftIN2, OUTPUT);
    pinMode(rightIN1, OUTPUT);
    pinMode(rightIN2, OUTPUT);

    ledcAttach(leftPWM, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
    ledcAttach(rightPWM, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);

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
    if (speed == -1) speed = currentSpeed;
    speed = constrain(speed, 0, 255);

    digitalWrite(leftIN1, HIGH);
    digitalWrite(leftIN2, LOW);
    digitalWrite(rightIN1, HIGH);
    digitalWrite(rightIN2, LOW);
    ledcWrite(leftPWM, speed);
    ledcWrite(rightPWM, speed);
    currentState = "FORWARD";
  }

  void backward(int speed = -1) {
    if (speed == -1) speed = currentSpeed;
    speed = constrain(speed, 0, 255);

    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, HIGH);
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, HIGH);
    ledcWrite(leftPWM, speed);
    ledcWrite(rightPWM, speed);
    currentState = "BACKWARD";
  }

  void left(int speed = -1) {
    if (speed == -1) speed = currentSpeed;
    speed = constrain(speed, 0, 255);

    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, HIGH);
    digitalWrite(rightIN1, HIGH);
    digitalWrite(rightIN2, LOW);
    ledcWrite(leftPWM, speed);
    ledcWrite(rightPWM, speed);
    currentState = "LEFT";
  }

  void right(int speed = -1) {
    if (speed == -1) speed = currentSpeed;
    speed = constrain(speed, 0, 255);

    digitalWrite(leftIN1, HIGH);
    digitalWrite(leftIN2, LOW);
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, HIGH);
    ledcWrite(leftPWM, speed);
    ledcWrite(rightPWM, speed);
    currentState = "RIGHT";
  }

  void setSpeed(int speed) {
    currentSpeed = constrain(speed, 0, 255);
  }

  // Drive each wheel at an independent speed (used by Navigator)
  void driveRaw(int leftSpeed, int rightSpeed) {
    leftSpeed  = constrain(leftSpeed,  0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    digitalWrite(leftIN1,  HIGH);
    digitalWrite(leftIN2,  LOW);
    digitalWrite(rightIN1, HIGH);
    digitalWrite(rightIN2, LOW);
    ledcWrite(leftPWM,  leftSpeed);
    ledcWrite(rightPWM, rightSpeed);
    currentState = "NAV";
  }

  bool isReady() {
    return true;
  }
};

#endif
