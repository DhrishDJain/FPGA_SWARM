#include <Wire.h>
#include "config.h"
#include "VL53L0X_Sensor.h"

VL53L0X_Sensor lidar;

// Calibration data structure
struct CalibrationPoint {
  uint16_t measured;
  uint16_t actual;
};

CalibrationPoint calPoints[5];
int numCalPoints = 0;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("\n=== VL53L0X LiDAR Calibration Tool ===");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQUENCY);
  
  if (!lidar.initialize()) {
    Serial.println("✗ VL53L0X initialization failed!");
    while(1) delay(100);
  }
  
  Serial.println("\nCalibration Modes:");
  Serial.println("1. Multi-point Calibration (Recommended)");
  Serial.println("2. Two-point Linear Calibration");
  Serial.println("3. Continuous Accuracy Test");
  Serial.println("\nSend '1', '2', or '3' to start");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case '1':
        multiPointCalibration();
        break;
      case '2':
        twoPointCalibration();
        break;
      case '3':
        continuousAccuracyTest();
        break;
    }
  }
}

void multiPointCalibration() {
  Serial.println("\n=== Multi-Point Calibration ===");
  Serial.println("This will measure 5 reference distances");
  Serial.println("You will need: ruler/tape measure");
  
  uint16_t distances[] = {50, 100, 200, 400, 800};  // mm
  numCalPoints = 0;
  
  for (int i = 0; i < 5; i++) {
    Serial.print("\nPlace object at EXACTLY ");
    Serial.print(distances[i]);
    Serial.println(" mm from sensor");
    Serial.println("Send 'm' when ready to measure");
    
    while(Serial.read() != 'm') delay(10);
    
    // Take 20 measurements and average
    uint32_t sum = 0;
    int validCount = 0;
    
    Serial.println("Measuring...");
    for (int j = 0; j < 20; j++) {
      lidar.update();
      if (lidar.valid) {
        sum += lidar.distance_mm;
        validCount++;
      }
      delay(50);
    }
    
    if (validCount > 10) {
      calPoints[numCalPoints].actual = distances[i];
      calPoints[numCalPoints].measured = sum / validCount;
      
      Serial.print("✓ Actual: ");
      Serial.print(distances[i]);
      Serial.print(" mm | Measured: ");
      Serial.print(calPoints[numCalPoints].measured);
      Serial.print(" mm | Error: ");
      Serial.print(((float)calPoints[numCalPoints].measured - distances[i]) / distances[i] * 100);
      Serial.println("%");
      
      numCalPoints++;
    } else {
      Serial.println("✗ Too many invalid readings, skipping point");
    }
  }
  
  // Calculate linear regression for correction
  calculateCorrection();
}

void twoPointCalibration() {
  Serial.println("\n=== Two-Point Linear Calibration ===");
  Serial.println("Recommended distances: 50mm and 400mm");
  
  uint16_t distances[] = {50, 400};
  numCalPoints = 0;
  
  for (int i = 0; i < 2; i++) {
    Serial.print("\nPlace object at EXACTLY ");
    Serial.print(distances[i]);
    Serial.println(" mm from sensor");
    Serial.println("Send 'm' when ready");
    
    while(Serial.read() != 'm') delay(10);
    
    uint32_t sum = 0;
    int validCount = 0;
    
    for (int j = 0; j < 20; j++) {
      lidar.update();
      if (lidar.valid) {
        sum += lidar.distance_mm;
        validCount++;
      }
      delay(50);
    }
    
    if (validCount > 10) {
      calPoints[numCalPoints].actual = distances[i];
      calPoints[numCalPoints].measured = sum / validCount;
      numCalPoints++;
      
      Serial.print("✓ Measured: ");
      Serial.println(sum / validCount);
    }
  }
  
  // Calculate linear correction
  if (numCalPoints == 2) {
    float slope = (float)(calPoints[1].actual - calPoints[0].actual) / 
                  (calPoints[1].measured - calPoints[0].measured);
    float offset = calPoints[0].actual - (slope * calPoints[0].measured);
    
    Serial.println("\n=== Calibration Results ===");
    Serial.print("Correction formula: actual_mm = ");
    Serial.print(slope, 4);
    Serial.print(" * measured_mm + ");
    Serial.println(offset, 2);
    Serial.println("\nAdd to VL53L0X_Sensor.h update() method:");
    Serial.println("distance_mm = (uint16_t)(distance_mm * " + String(slope, 4) + " + " + String(offset, 2) + ");");
  }
}

void calculateCorrection() {
  if (numCalPoints < 2) {
    Serial.println("✗ Not enough calibration points");
    return;
  }
  
  // Simple linear regression
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  
  for (int i = 0; i < numCalPoints; i++) {
    sumX += calPoints[i].measured;
    sumY += calPoints[i].actual;
    sumXY += calPoints[i].measured * calPoints[i].actual;
    sumX2 += calPoints[i].measured * calPoints[i].measured;
  }
  
  float n = numCalPoints;
  float slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
  float offset = (sumY - slope * sumX) / n;
  
  Serial.println("\n=== Calibration Results ===");
  Serial.print("Correction formula: actual_mm = ");
  Serial.print(slope, 4);
  Serial.print(" * measured_mm + ");
  Serial.println(offset, 2);
  
  Serial.println("\nAdd to VL53L0X_Sensor.h update() method:");
  Serial.println("if (measure.RangeStatus != 4) {");
  Serial.println("  distance_mm = (uint16_t)(measure.RangeMilliMeter * " + String(slope, 4) + " + " + String(offset, 2) + ");");
  Serial.println("  valid = true;");
  Serial.println("}");
  
  // Show accuracy
  Serial.println("\nCalibration accuracy:");
  for (int i = 0; i < numCalPoints; i++) {
    float corrected = slope * calPoints[i].measured + offset;
    float error = ((corrected - calPoints[i].actual) / calPoints[i].actual) * 100;
    Serial.print("At ");
    Serial.print(calPoints[i].actual);
    Serial.print("mm: error = ");
    Serial.print(error, 2);
    Serial.println("%");
  }
}

void continuousAccuracyTest() {
  Serial.println("\n=== Continuous Accuracy Test ===");
  Serial.println("Enter actual distance in mm, then monitor readings");
  Serial.println("Example: Type '150' for 150mm distance");
  
  while(!Serial.available()) delay(10);
  int actualDistance = Serial.parseInt();
  
  Serial.print("\nMonitoring at ");
  Serial.print(actualDistance);
  Serial.println(" mm actual distance");
  Serial.println("Reading (mm) | Error (%) | Status");
  Serial.println("----------------------------------------");
  
  unsigned long lastPrint = 0;
  uint32_t sum = 0;
  int count = 0;
  
  while(true) {
    lidar.update();
    
    if (lidar.valid) {
      sum += lidar.distance_mm;
      count++;
    }
    
    if (millis() - lastPrint >= 500 && count > 0) {
      float avgReading = (float)sum / count;
      float error = ((avgReading - actualDistance) / actualDistance) * 100;
      
      Serial.print(avgReading, 1);
      Serial.print(" | ");
      Serial.print(error, 2);
      Serial.print(" | ");
      Serial.println(lidar.valid ? "Valid" : "Invalid");
      
      sum = 0;
      count = 0;
      lastPrint = millis();
    }
    
    if (Serial.available()) {
      Serial.read();
      Serial.println("\n✓ Test stopped");
      return;
    }
    
    delay(10);
  }
}
