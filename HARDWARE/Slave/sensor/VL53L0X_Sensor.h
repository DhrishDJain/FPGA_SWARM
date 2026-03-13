#ifndef VL53L0X_SENSOR_H
#define VL53L0X_SENSOR_H

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

class VL53L0X_Sensor {
private:
  Adafruit_VL53L0X lox;
  bool initialized;

public:
  uint16_t distance_mm;
  bool valid;

  VL53L0X_Sensor() {
    initialized = false;
    distance_mm = 0;
    valid = false;
  }

  bool initialize() {
    if (lox.begin(VL53L0X_I2C_ADDRESS, false, &Wire)) {
      initialized = true;
      Serial.println("VL53L0X ready");
      return true;
    }
    Serial.println("VL53L0X not found");
    return false;
  }

  void update() {
    if (!initialized) return;

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      distance_mm = measure.RangeMilliMeter;
      valid = true;
    } else {
      distance_mm = 0;
      valid = false;
    }
  }

  bool isReady() {
    return initialized;
  }
};

#endif
