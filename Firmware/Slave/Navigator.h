#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "Encoder.h"
#include "MPU6050_DMP.h"
#include "Motor_driver.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

enum NavState { NAV_IDLE, NAV_NAVIGATING, NAV_ARRIVED };

enum NavMode {
  NAV_MODE_IDLE,
  NAV_MODE_SINGLE_TARGET, // web UI goto
  NAV_MODE_FPGA_TARGET,   // FIX: new — FPGA-assigned single waypoint goto
  NAV_MODE_AREA           // serpentine area scan
};

struct AreaTask {
  bool active;
  bool goingToStart;
  float originX, originY;
  uint16_t blocksX, blocksY;
  float blockSize;
  uint16_t curBx, curBy;

  AreaTask()
      : active(false), goingToStart(false), originX(0), originY(0), blocksX(0),
        blocksY(0), blockSize(0), curBx(0), curBy(0) {}

  void configure(float sx, float sy, uint16_t bx, uint16_t by, float size) {
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

  bool hasNext() const { return active && (curBy < blocksY); }

  bool nextWaypoint(float &tx, float &ty) {
    if (!hasNext())
      return false;
    bool flip = (curBy % 2 == 1);
    uint16_t col = flip ? (blocksX - 1 - curBx) : curBx;
    tx = originX + (col + 0.5f) * blockSize;
    ty = originY + (curBy + 0.5f) * blockSize;
    if (++curBx >= blocksX) {
      curBx = 0;
      if (++curBy >= blocksY)
        active = false;
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
  float prevDist;
  AreaTask area;
  NavMode mode;
  uint32_t navStartTime;

public:
  NavState state;
  float targetX, targetY;
  float posX, posY;

  Navigator()
      : encoder(nullptr), imu(nullptr), motors(nullptr), prevDist(0.0f),
        mode(NAV_MODE_IDLE), state(NAV_IDLE), targetX(0), targetY(0), posX(0),
        posY(0), navStartTime(0) {}

  void init(Encoder *e, MPU6050_DMP *m, MotorControl *mc) {
    encoder = e;
    imu = m;
    motors = mc;
    prevDist = encoder ? encoder->distanceTraveledCM : 0.0f;
    posX = posY = 0.0f;
    state = NAV_IDLE;
    mode = NAV_MODE_IDLE;
  }

  // ── Web UI goto ──────────────────────────────────────────────
  void setTarget(float tx, float ty) {
    targetX = tx;
    targetY = ty;
    prevDist = encoder ? encoder->distanceTraveledCM : 0.0f;
    navStartTime = millis();
    // FIX: explicitly clear area mode so a stale NAV_MODE_AREA
    // doesn't interfere with a fresh single-target goto.
    area.stop();
    mode = NAV_MODE_SINGLE_TARGET;
    state = NAV_NAVIGATING;
    Serial.printf("[NAV] GOTO (%.1f, %.1f)\n", tx, ty);
  }

  // ── FPGA-assigned waypoint goto ──────────────────────────────
  // FIX: separate entry point for FPGA gotos so isMissionActive()
  // and telemetry can distinguish them from web UI gotos.
  void setFpgaTarget(float tx, float ty) {
    // FIX: FPGA coordinates are grid indices. Scale them to centimeters
    // using FPGA_CELL_CM and target the center of the cell.
    targetX = (tx + 0.5f) * FPGA_CELL_CM;
    targetY = (ty + 0.5f) * FPGA_CELL_CM;
    prevDist = encoder ? encoder->distanceTraveledCM : 0.0f;
    navStartTime = millis();
    // FIX: clear area mode — FPGA now controls waypoint sequencing,
    // not the local AreaTask. A fresh 'g' always starts clean.
    area.stop();
    mode = NAV_MODE_FPGA_TARGET;
    state = NAV_NAVIGATING; // ← force out of NAV_ARRIVED immediately
    Serial.printf("[NAV] FPGA GOTO (%.1f, %.1f) → NAV_NAVIGATING\n", targetX, targetY);
  }

  // ── Area scan ────────────────────────────────────────────────
  void startArea(float sx, float sy, uint16_t bx, uint16_t by, float size) {
    area.configure(sx, sy, bx, by, size);
    if (!area.active) {
      Serial.println("[NAV] invalid area config, ignored");
      return;
    }
    posX = posY = 0.0f;
    prevDist = encoder ? encoder->distanceTraveledCM : 0.0f;
    navStartTime = millis();
    mode = NAV_MODE_AREA;
    targetX = sx;
    targetY = sy;

    float distToStart = sqrtf(sx * sx + sy * sy);
    if (distToStart < ARRIVE_THRESHOLD_CM) {
      area.goingToStart = false;
      Serial.printf("[NAV] already at start (%.1f,%.1f), skipping phase 1\n",
                    sx, sy);
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
    } else {
      state = NAV_NAVIGATING;
      Serial.printf("[NAV] PHASE 1 → going to starting point(%.1f, %.1f)\n", sx, sy);
    }
    Serial.printf("[NAV] area: %ux%u blocks, %.1fcm each\n", bx, by, size);
  }

  void stop() {
    state = NAV_IDLE;
    mode = NAV_MODE_IDLE;
    area.stop();
    if (motors)
      motors->stop();
    Serial.println("[NAV] stopped");
  }

  bool isIdle() const { return state == NAV_IDLE; }

  // FIX: isMissionActive() now only returns true when ACTIVELY DRIVING
  // in area mode — NOT when arrived at a waypoint mid-scan.
  // This allows "ARRIVED" to propagate correctly in telemetry.
  bool isMissionActive() const {
    return mode == NAV_MODE_AREA && state == NAV_NAVIGATING;
  }

  // True when area scan is running (any state)
  bool isAreaMode() const { return mode == NAV_MODE_AREA; }

  bool isSearching() const {
    return mode == NAV_MODE_AREA && !area.goingToStart;
  }

  // FIX: helper for slave.ino telemetry to detect FPGA waypoint arrival
  bool isFpgaTarget() const { return mode == NAV_MODE_FPGA_TARGET; }

  void update() {
    if (!encoder || !imu || !motors)
      return;

    // ── Dead-reckoning ──────────────────────────────────────────
    float curDist = encoder->distanceTraveledCM;
    float delta = curDist - prevDist;
    prevDist = curDist;

    float yawRad = imu->yaw * (float)M_PI / 180.0f;
    posX += delta * sinf(yawRad);
    posY += delta * cosf(yawRad);

    // ── Debug print (throttled 500ms) ───────────────────────────
    static uint32_t lastNavPrint = 0;
    if (millis() - lastNavPrint >= 500) {
      lastNavPrint = millis();

      float _dx = targetX - posX;
      float _dy = targetY - posY;
      float _dist = sqrtf(_dx * _dx + _dy * _dy);
      float _bear = atan2f(_dx, _dy) * 180.0f / (float)M_PI;
      if (_bear < 0)
        _bear += 360.0f;

      // Error angle between bearing and current yaw
      float _err = _bear - imu->yaw;
      while (_err > 180.0f)
        _err -= 360.0f;
      while (_err < -180.0f)
        _err += 360.0f;

      const char *modeStr = (mode == NAV_MODE_FPGA_TARGET)     ? "FPGA"
                            : (mode == NAV_MODE_SINGLE_TARGET) ? "WEB"
                            : (mode == NAV_MODE_AREA)          ? "AREA"
                                                               : "IDLE";
      const char *stateStr = (state == NAV_NAVIGATING) ? "NAVIGATING"
                             : (state == NAV_ARRIVED)  ? "ARRIVED"
                                                       : "IDLE";

      Serial.printf("[NAV|%s|%s] "
                    "pos=(%.1f,%.1f) "
                    "tgt=(%.1f,%.1f) "
                    "dist=%.1fcm "
                    "bear=%.0f° "
                    "yaw=%.0f° "
                    "err=%.0f° "
                    "enc=%ld "
                    "tot=%.2fcm "
                    "delta=%.3f\n",
                    modeStr, stateStr, posX, posY, targetX, targetY, _dist,
                    _bear, imu->yaw, _err, encoder->getCount(), curDist, delta);
    }

    if (state != NAV_NAVIGATING)
      return;

    // ── Waypoint timeout watchdog ───────────────────────────────
    if (millis() - navStartTime > NAV_WAYPOINT_TIMEOUT_MS) {
      motors->stop();
      Serial.printf(
          "[NAV] ⚠ TIMEOUT → target=(%.1f,%.1f) pos=(%.1f,%.1f) dist=%.1fcm\n",
          targetX, targetY, posX, posY,
          sqrtf((targetX - posX) * (targetX - posX) +
                (targetY - posY) * (targetY - posY)));
      if (mode == NAV_MODE_AREA) {
        float tx, ty;
        if (area.hasNext() && area.nextWaypoint(tx, ty)) {
          targetX = tx;
          targetY = ty;
          navStartTime = millis();
          prevDist = encoder->distanceTraveledCM;
          state = NAV_NAVIGATING;
          Serial.printf("[NAV] skipped → next (%.1f,%.1f)\n", tx, ty);
        } else {
          state = NAV_IDLE;
          mode = NAV_MODE_IDLE;
          Serial.println("[NAV] timeout — area mission aborted");
        }
      } else {
        state = NAV_ARRIVED;
      }
      return;
    }

    float dx = targetX - posX;
    float dy = targetY - posY;
    float dist = sqrtf(dx * dx + dy * dy);

    // ── Arrival check ───────────────────────────────────────────
    if (dist < ARRIVE_THRESHOLD_CM) {
      motors->stop();
      Serial.printf(
          "[NAV] ✅ ARRIVED at (%.1f,%.1f) | final_err=%.1fcm | yaw=%.0f°\n",
          targetX, targetY, dist, imu->yaw);

       if (mode == NAV_MODE_AREA && area.goingToStart) {
        area.goingToStart = false;
        Serial.println("[NAV] PHASE 2 → beginning serpentine search");
        float tx, ty;
        if (area.nextWaypoint(tx, ty)) {
          targetX = tx;
          targetY = ty;
          navStartTime = millis();
          prevDist = encoder->distanceTraveledCM;
          state = NAV_NAVIGATING;
          Serial.printf("[NAV] waypoint 1 → (%.1f, %.1f)\n", tx, ty);
        } else {
          state = NAV_IDLE;
          mode = NAV_MODE_IDLE;
          Serial.println("[NAV] area empty, done");
        }
      } else if (mode == NAV_MODE_AREA) {
        state = NAV_ARRIVED;
        float tx, ty;
        if (area.hasNext() && area.nextWaypoint(tx, ty)) {
          targetX = tx;
          targetY = ty;
          navStartTime = millis();
          prevDist = encoder->distanceTraveledCM;
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

    // ── P-controller ────────────────────────────────────────────
    float bearing = atan2f(dx, dy) * 180.0f / (float)M_PI;
    if (bearing < 0.0f)
      bearing += 360.0f;

    float err = bearing - imu->yaw;
    while (err > 180.0f)
      err -= 360.0f;
    while (err < -180.0f)
      err += 360.0f;

    int turn = constrain((int)(HEADING_KP * err), -NAV_TURN_MAX, NAV_TURN_MAX);
    int leftOut = constrain(NAV_BASE_SPEED - turn, 0, 255);
    int rightOut = constrain(NAV_BASE_SPEED + turn, 0, 255);
    motors->driveRaw(leftOut, rightOut);

    // Motor output debug (throttled separately at 1s to avoid spam)
    static uint32_t lastMotorPrint = 0;
    if (millis() - lastMotorPrint >= 1000) {
      lastMotorPrint = millis();
      Serial.printf("[MOT] L=%d R=%d | turn=%d | err=%.0f°\n", leftOut,
                    rightOut, turn, err);
    }
  }
};

#endif
