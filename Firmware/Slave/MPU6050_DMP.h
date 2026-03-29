#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "config.h"

class MPU6050_DMP {
private:
  MPU6050 mpu;
  volatile bool* mpuInterrupt;
  bool dmpReady;
  uint8_t fifoBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  uint8_t interruptPin;

public:
  float yaw, pitch, roll;
  bool dataReady;

  MPU6050_DMP(uint8_t intPin, volatile bool* interruptFlag) {
    interruptPin = intPin;
    mpuInterrupt = interruptFlag;
    dmpReady = false;
    dataReady = false;
    yaw = pitch = roll = 0.0;
  }

  void IRAM_ATTR handleInterrupt() {
    *mpuInterrupt = true;
  }

  bool initialize() {
    mpu.initialize();

    if (!mpu.testConnection()) {
      Serial.println("MPU6050 not found");
      return false;
    }

    uint8_t devStatus = mpu.dmpInitialize();

    // Set calibration offsets
    mpu.setXAccelOffset(-1018);
    mpu.setYAccelOffset(594);
    mpu.setZAccelOffset(2514);
    mpu.setXGyroOffset(34);
    mpu.setYGyroOffset(-41);
    mpu.setZGyroOffset(8);

    if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      pinMode(interruptPin, INPUT);
      dmpReady = true;
      Serial.println("MPU6050 ready");
      return true;
    }

    Serial.println("MPU6050 init failed");
    return false;
  }

  void update() {
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw = ypr[0] * 180.0 / PI;
      pitch = ypr[1] * 180.0 / PI;
      roll = ypr[2] * 180.0 / PI;

      while (yaw < 0) yaw += 360.0;
      while (yaw >= 360.0) yaw -= 360.0;

      dataReady = true;
      *mpuInterrupt = false;
    }
  }

  bool isReady() {
    return dmpReady;
  }
};

#endif
