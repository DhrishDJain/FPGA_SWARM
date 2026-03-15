#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "config.h"
#include "Motor_driver.h"
#include "Encoder.h"
#include "MPU6050_DMP.h"
#include <math.h>
#include <Arduino.h>

enum NavState { NAV_IDLE,
                NAV_NAVIGATING,
                NAV_ARRIVED };
enum NavMode { NAV_MODE_IDLE,
               NAV_MODE_SINGLE_TARGET,
               NAV_MODE_AREA };

struct AreaTask {
  bool active;
  bool goingToStart;
  float originX, originY;
  uint16_t blocksX, blocksY;
  float blockSize;
  uint16_t curBx, curBy;

  AreaTask()
    : active(false), goingToStart(false),
      originX(0), originY(0),
      blocksX(0), blocksY(0), blockSize(0),
      curBx(0), curBy(0) {}

  void configure(float sx, float sy,
                 uint16_t bx, uint16_t by, float size) {
    originX = sx;
    originY = sy;
    blocksX = bx;
    blocksY = by;
    blockSize = size;
    curBx = 0;
    curBy = 0;
    goingToStart = true;
    active = (blocksX > 0 && blocksY > 0 && blockSize > 0.0f);
  }

  bool hasNext() const {
    return active && (curBy < blocksY);
  }

  bool nextWaypoint(float &tx, float &ty) {
    if (!hasNext()) return false;
    bool flip = (curBy % 2 == 1);
    uint16_t col = flip ? (blocksX - 1 - curBx) : curBx;
    tx = originX + (col + 0.5f) * blockSize;
    ty = originY + (curBy + 0.5f) * blockSize;
    if (++curBx >= blocksX) {
      curBx = 0;
      if (++curBy >= blocksY) active = false;
    }
    return true;
  }

  void stop() {
    active = false;
    goingToStart = false;
  }
};

class Navigator {
private:
  Encoder *encoder;
  MPU6050_DMP *imu;
  MotorControl *motors;
  float prevDist;  // last distance sample (for delta)
  AreaTask area;
  NavMode mode;

public:
  NavState state;
  float targetX, targetY;
  float posX, posY;  // these go to dashboard

  Navigator()
    : encoder(nullptr), imu(nullptr), motors(nullptr),
      prevDist(0.0f), mode(NAV_MODE_IDLE), state(NAV_IDLE),
      targetX(0), targetY(0), posX(0), posY(0) {}

  void init(Encoder *e, MPU6050_DMP *m, MotorControl *mc) {
    encoder = e;
    imu = m;
    motors = mc;
    // Use TOTAL distance as baseline for dead-reckoning
    prevDist = encoder ? encoder->totalDistanceCM : 0.0f;
    posX = posY = 0.0f;
    state = NAV_IDLE;
    mode = NAV_MODE_IDLE;
  }

  void setTarget(float tx, float ty) {
    targetX = tx;
    targetY = ty;
    state = NAV_NAVIGATING;
    mode = NAV_MODE_SINGLE_TARGET;
    Serial.printf("[NAV] goto (%.1f, %.1f)\n", tx, ty);
  }

  void startArea(float sx, float sy,
                 uint16_t bx, uint16_t by, float size) {
    area.configure(sx, sy, bx, by, size);
    if (!area.active) {
      Serial.println("[NAV] invalid area config, ignored");
      return;
    }
    mode = NAV_MODE_AREA;
    targetX = sx;
    targetY = sy;
    state = NAV_NAVIGATING;
    Serial.printf("[NAV] PHASE 1 → going to start (%.1f, %.1f)\n", sx, sy);
    Serial.printf("[NAV] area: %ux%u blocks, %.1fcm each\n", bx, by, size);
  }

  void stop() {
    state = NAV_IDLE;
    mode = NAV_MODE_IDLE;
    area.stop();
    if (motors) motors->stop();
    Serial.println("[NAV] stopped");
  }

  bool isIdle() const {
    return state == NAV_IDLE;
  }
  bool isMissionActive() const {
    return mode == NAV_MODE_AREA && (state == NAV_NAVIGATING || state == NAV_ARRIVED);
  }
  bool isSearching() const {
    return mode == NAV_MODE_AREA && !area.goingToStart;
  }

  void update() {
    if (!encoder || !imu || !motors) return;

    // ── Dead-reckoning ──
    // use totalDistanceCM so delta is always forward path length
    float curDist = encoder->totalDistanceCM;
    float delta = curDist - prevDist;
    prevDist = curDist;

    // ignore tiny noise
    // if (fabsf(delta) < 0.01f) delta = 0.0f;

    float yawRad = imu->yaw * (float)M_PI / 180.0f;
    posX += delta * sinf(yawRad);
    posY += delta * cosf(yawRad);
    Serial.print("delta ");
    Serial.print(delta);
    Serial.print("\tprevDist ");
    Serial.print(prevDist);
    Serial.print("\tcurDist ");
    Serial.println(curDist);
    // At this point posX,posY are what the dashboard sees
    if (state != NAV_NAVIGATING) return;

    float dx = targetX - posX;
    float dy = targetY - posY;
    float dist = sqrtf(dx * dx + dy * dy);

    // ── Arrival check ──
    if (dist < ARRIVE_THRESHOLD_CM) {
      motors->stop();
      Serial.printf("[NAV] arrived (%.1f,%.1f) err=%.1fcm\n", targetX, targetY, dist);

      if (mode == NAV_MODE_AREA && area.goingToStart) {
        area.goingToStart = false;
        Serial.printf("[NAV] ARRIVED at start (%.1f,%.1f)\n",
                      targetX, targetY);
        Serial.println("[NAV] PHASE 2 → beginning serpentine search");

        float tx, ty;
        if (area.nextWaypoint(tx, ty)) {
          targetX = tx;
          targetY = ty;
          state = NAV_NAVIGATING;
          Serial.printf("[NAV] waypoint 1 → (%.1f, %.1f)\n", tx, ty);
        } else {
          state = NAV_IDLE;
          mode = NAV_MODE_IDLE;
          Serial.println("[NAV] area empty, done");
        }

      } else if (mode == NAV_MODE_AREA) {
        Serial.printf("[NAV] block done at (%.1f,%.1f)\n",
                      targetX, targetY);

        float tx, ty;
        if (area.hasNext() && area.nextWaypoint(tx, ty)) {
          targetX = tx;
          targetY = ty;
          state = NAV_NAVIGATING;
          Serial.printf("[NAV] next waypoint → (%.1f, %.1f)\n", tx, ty);
        } else {
          state = NAV_IDLE;
          mode = NAV_MODE_IDLE;
          Serial.println("[NAV] ✅ area mission COMPLETE");
        }

      } else {
        state = NAV_ARRIVED;
      }
      return;
    }

    // ── P-controller ──
    float bearing = atan2f(dx, dy) * 180.0f / (float)M_PI;
    if (bearing < 0.0f) bearing += 360.0f;

    float err = bearing - imu->yaw;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    int turn = constrain((int)(HEADING_KP * err),
                         -NAV_TURN_MAX, NAV_TURN_MAX);
    int leftOut = constrain(NAV_BASE_SPEED - turn, 0, 255);
    int rightOut = constrain(NAV_BASE_SPEED + turn, 0, 255);
    motors->driveRaw(leftOut, rightOut);
  }
};

#endif
