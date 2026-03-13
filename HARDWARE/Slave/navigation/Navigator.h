#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include <math.h>
#include "../config.h"
#include "../motors/Motor_driver.h"
#include "../sensor/Encoder.h"
#include "../sensor/MPU6050_DMP.h"

enum NavState { NAV_IDLE, NAV_NAVIGATING, NAV_ARRIVED };

/*
 * Navigator — proportional heading controller + internal dead-reckoning.
 *
 * Call update() every loop. It handles its own X/Y dead-reckoning using
 * the attached encoder and IMU, and drives the attached motors directly.
 */
class Navigator {
private:
  Encoder*      encoder;
  MPU6050_DMP*  imu;
  MotorControl* motors;

  float prevDist;

public:
  NavState state;
  float targetX, targetY;
  float posX, posY;

  Navigator() : encoder(nullptr), imu(nullptr), motors(nullptr),
                prevDist(0), state(NAV_IDLE),
                targetX(0), targetY(0), posX(0), posY(0) {}

  void init(Encoder* _encoder, MPU6050_DMP* _imu, MotorControl* _motors) {
    encoder = _encoder;
    imu     = _imu;
    motors  = _motors;
  }

  void setTarget(float tx, float ty) {
    targetX = tx;
    targetY = ty;
    state = NAV_NAVIGATING;
    Serial.printf("[NAV] navigating to (%.2f, %.2f)\n", tx, ty);
  }

  void stop() {
    state = NAV_IDLE;
    if (motors) motors->stop();
  }

  // Call every loop tick
  void update() {
    if (!encoder || !imu || !motors) return;

    // 1. Dead-reckoning update
    float curDist = encoder->distanceTraveledCM;
    float delta   = curDist - prevDist;
    prevDist      = curDist;

    float yawRad = imu->yaw * M_PI / 180.0f;
    posX += delta * sinf(yawRad);
    posY += delta * cosf(yawRad);

    // 2. Navigation control
    if (state != NAV_NAVIGATING) return;

    float dx = targetX - posX;
    float dy = targetY - posY;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < ARRIVE_THRESHOLD_CM) {
      motors->stop();
      state = NAV_ARRIVED;
      Serial.printf("[NAV] ARRIVED (dist=%.1f cm)\n", dist);
      return;
    }

    // Bearing to target in degrees: 0 = +Y axis (forward), positive CW
    float bearing = atan2f(dx, dy) * 180.0f / (float)M_PI;
    if (bearing < 0) bearing += 360.0f;

    // Heading error, normalised to [-180, +180]
    float err = bearing - imu->yaw;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    // P gain → turn differential
    int turn = (int)(HEADING_KP * err);
    turn = constrain(turn, -NAV_TURN_MAX, NAV_TURN_MAX);

    int leftOut  = constrain(NAV_BASE_SPEED - turn, 0, 255);
    int rightOut = constrain(NAV_BASE_SPEED + turn, 0, 255);
    
    motors->driveRaw(leftOut, rightOut);
  }

  bool isArrived() const { return state == NAV_ARRIVED; }
  bool isIdle()    const { return state == NAV_IDLE; }
};

#endif

