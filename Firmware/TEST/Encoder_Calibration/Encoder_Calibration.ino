/*
 * ESP32 Encoder Calibration Tool - Standalone Version
 * Accurately measures pulses per revolution and distance calibration
 * Compatible with your robot project configuration
 */

#include <Arduino.h>

// ==================== CONFIGURATION ====================
#define ENCODER_C1 35
#define ENCODER_C2 33
#define SERIAL_BAUD_RATE 115200

// Current settings (will be tested and adjusted)
#define CURRENT_PULSES_PER_REV 2199
#define CURRENT_WHEEL_DIAMETER_MM 44.0

// ==================== GLOBAL VARIABLES ====================
volatile long encoderCount = 0;
volatile int lastC1State = 0;
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long interruptCount = 0;

// Calibration results storage
float distancePerPulseMM = 0;
unsigned long sessionStartTime = 0;

// ==================== ENCODER ISR ====================
void IRAM_ATTR encoderISR() {
  interruptCount++;
  lastInterruptTime = micros();

  int c1State = digitalRead(ENCODER_C1);
  int c2State = digitalRead(ENCODER_C2);

  if (c1State != lastC1State) {
    if (c1State == HIGH) {
      encoderCount += (c2State == HIGH) ? 1 : -1;
    } else {
      encoderCount += (c2State == LOW) ? 1 : -1;
    }
  }
  lastC1State = c1State;
}

// ==================== UTILITY FUNCTIONS ====================
void resetEncoder() {
  noInterrupts();
  encoderCount = 0;
  interruptCount = 0;
  interrupts();
}

float calculateDistance(long pulses, float pulsesPerRev, float wheelDiameter) {
  float wheelCircumference = PI * wheelDiameter;
  float revolutions = (float)pulses / pulsesPerRev;
  return revolutions * wheelCircumference / 10.0;  // Convert to cm
}

void printDivider() {
  Serial.println("\n========================================================");
}

void waitForCommand(char expected) {
  Serial.print("Send '");
  Serial.print(expected);
  Serial.println("' to continue...");

  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      while (Serial.available()) Serial.read();  // Clear buffer
      if (c == expected) return;
    }
    delay(10);
  }
}

// ==================== CALIBRATION TESTS ====================

// Test 1: Hardware connectivity and signal quality
void hardwareTest() {
  printDivider();
  Serial.println("TEST 1: HARDWARE CONNECTIVITY & SIGNAL QUALITY");
  printDivider();

  Serial.println("\nChecking encoder connections...");
  delay(100);

  int c1 = digitalRead(ENCODER_C1);
  int c2 = digitalRead(ENCODER_C2);

  Serial.print("C1 (GPIO35) initial state: ");
  Serial.println(c1 ? "HIGH" : "LOW");
  Serial.print("C2 (GPIO33) initial state: ");
  Serial.println(c2 ? "HIGH" : "LOW");

  Serial.println("\n>>> MANUAL TEST <<<");
  Serial.println("Slowly rotate wheel by hand and watch for pulses");
  Serial.println("Good encoder shows: alternating states, no missed counts");
  Serial.println("\nPress any key when done...");

  resetEncoder();
  unsigned long startTime = millis();
  long lastCount = 0;

  while (!Serial.available()) {
    if (millis() - startTime > 100) {
      if (encoderCount != lastCount) {
        Serial.print("Count: ");
        Serial.print(encoderCount);
        Serial.print(" | C1: ");
        Serial.print(digitalRead(ENCODER_C1));
        Serial.print(" | C2: ");
        Serial.println(digitalRead(ENCODER_C2));
        lastCount = encoderCount;
      }
      startTime = millis();
    }
    delay(1);
  }

  while (Serial.available()) Serial.read();

  Serial.print("\nTotal pulses detected: ");
  Serial.println(abs(encoderCount));

  if (abs(encoderCount) > 10) {
    Serial.println("✓ Encoder is responding correctly");
  } else {
    Serial.println("✗ WARNING: Very few pulses detected. Check wiring!");
  }
}

// Test 2: Pulses per revolution measurement
void pulsesPerRevTest() {
  printDivider();
  Serial.println("TEST 2: PULSES PER REVOLUTION (CRITICAL)");
  printDivider();

  Serial.println("\nThis determines encoder resolution accuracy");
  Serial.println("\nInstructions:");
  Serial.println("1. Mark a reference point on your wheel (use tape/marker)");
  Serial.println("2. Align mark with a fixed reference point");
  Serial.println("3. We'll count pulses for EXACTLY 1 revolution");
  Serial.println("4. Repeat 3 times for accuracy");

  long measurements[3];

  for (int trial = 0; trial < 3; trial++) {
    Serial.print("\n--- Trial ");
    Serial.print(trial + 1);
    Serial.println(" of 3 ---");

    waitForCommand('s');

    resetEncoder();
    Serial.println("✓ Counting started!");
    Serial.println("Rotate wheel SLOWLY and PRECISELY one full revolution");
    Serial.println("Stop when mark returns to starting position");

    waitForCommand('e');

    measurements[trial] = abs(encoderCount);
    Serial.print("✓ Trial ");
    Serial.print(trial + 1);
    Serial.print(" result: ");
    Serial.print(measurements[trial]);
    Serial.println(" pulses");
  }

  // Calculate statistics
  float avg = (measurements[0] + measurements[1] + measurements[2]) / 3.0;
  float maxDev = 0;

  for (int i = 0; i < 3; i++) {
    float dev = abs(measurements[i] - avg);
    if (dev > maxDev) maxDev = dev;
  }

  float consistency = (1 - (maxDev / avg)) * 100;

  Serial.println("\n=== RESULTS ===");
  Serial.print("Trial 1: ");
  Serial.print(measurements[0]);
  Serial.println(" pulses");
  Serial.print("Trial 2: ");
  Serial.print(measurements[1]);
  Serial.println(" pulses");
  Serial.print("Trial 3: ");
  Serial.print(measurements[2]);
  Serial.println(" pulses");
  Serial.print("\nAverage: ");
  Serial.print(avg, 1);
  Serial.println(" pulses");
  Serial.print("Max deviation: ");
  Serial.print(maxDev, 1);
  Serial.println(" pulses");
  Serial.print("Consistency: ");
  Serial.print(consistency, 1);
  Serial.println("%");

  if (consistency > 95) {
    Serial.println("\n✓ EXCELLENT consistency!");
  } else if (consistency > 90) {
    Serial.println("\n✓ Good consistency");
  } else {
    Serial.println("\n⚠ WARNING: Low consistency. Rotate more carefully!");
  }

  int recommendedPPR = round(avg);
  Serial.print("\n>>> RECOMMENDED CONFIG.H VALUE <<<");
  Serial.print("\n#define PULSES_PER_REV ");
  Serial.println(recommendedPPR);

  Serial.print("\nCurrent setting: ");
  Serial.println(CURRENT_PULSES_PER_REV);

  float errorPercent = ((float)recommendedPPR - CURRENT_PULSES_PER_REV) / CURRENT_PULSES_PER_REV * 100;
  Serial.print("Difference: ");
  Serial.print(abs(errorPercent), 2);
  Serial.println("%");

  if (abs(errorPercent) > 5) {
    Serial.println("\n⚠ SIGNIFICANT DIFFERENCE! Update config.h");
  } else {
    Serial.println("\n✓ Current setting is accurate");
  }
}

// Test 3: Distance accuracy calibration
void distanceCalibrationTest() {
  printDivider();
  Serial.println("TEST 3: DISTANCE ACCURACY CALIBRATION");
  printDivider();

  Serial.println("\nThis tests real-world distance measurement");
  Serial.println("\nYou will need:");
  Serial.println("- Measuring tape or ruler");
  Serial.println("- Flat surface");
  Serial.println("- Your robot on wheels");

  Serial.println("\nRecommended test distances:");
  Serial.println("Option 1: 50 cm (easier, less accurate)");
  Serial.println("Option 2: 100 cm (recommended)");
  Serial.println("Option 3: 200 cm (most accurate)");

  Serial.println("\nEnter test distance in cm (e.g., 100):");
  while (!Serial.available()) delay(10);
  float targetDistance = Serial.parseFloat();
  while (Serial.available()) Serial.read();

  Serial.print("Test distance set to: ");
  Serial.print(targetDistance);
  Serial.println(" cm");

  Serial.println("\nWe'll perform 3 test runs");

  float measurements[3];

  for (int trial = 0; trial < 3; trial++) {
    Serial.print("\n--- Trial ");
    Serial.print(trial + 1);
    Serial.println(" of 3 ---");
    Serial.println("Instructions:");
    Serial.println("1. Place robot at starting line");
    Serial.print("2. Measure exactly ");
    Serial.print(targetDistance);
    Serial.println(" cm ahead (mark end point)");
    Serial.println("3. Send 's' to start measurement");
    Serial.print("4. Move robot STRAIGHT to the ");
    Serial.print(targetDistance);
    Serial.println(" cm mark");
    Serial.println("5. Send 'e' when robot reaches mark");

    waitForCommand('s');

    resetEncoder();
    Serial.println("✓ Measurement started - Move robot now!");

    waitForCommand('e');

    long pulses = abs(encoderCount);
    measurements[trial] = calculateDistance(pulses, CURRENT_PULSES_PER_REV, CURRENT_WHEEL_DIAMETER_MM);

    Serial.print("✓ Trial ");
    Serial.print(trial + 1);
    Serial.print(" complete");
    Serial.print("\n  Pulses: ");
    Serial.println(pulses);
    Serial.print("  Reported distance: ");
    Serial.print(measurements[trial], 2);
    Serial.println(" cm");
    Serial.print("  Error: ");
    Serial.print(((measurements[trial] - targetDistance) / targetDistance * 100), 2);
    Serial.println("%");
  }

  // Calculate results
  float avgMeasured = (measurements[0] + measurements[1] + measurements[2]) / 3.0;
  float errorPercent = ((avgMeasured - targetDistance) / targetDistance) * 100;
  float correctionFactor = targetDistance / avgMeasured;

  Serial.println("\n=== DISTANCE CALIBRATION RESULTS ===");
  Serial.print("Target distance:  ");
  Serial.print(targetDistance);
  Serial.println(" cm");
  Serial.print("Average measured: ");
  Serial.print(avgMeasured, 2);
  Serial.println(" cm");
  Serial.print("Error: ");
  Serial.print(errorPercent, 2);
  Serial.println("%");

  if (abs(errorPercent) < 2) {
    Serial.println("\n✓ EXCELLENT accuracy! No changes needed");
  } else if (abs(errorPercent) < 5) {
    Serial.println("\n✓ Good accuracy. Changes optional");
  } else {
    Serial.println("\n⚠ Significant error. Correction recommended!");

    int correctedPPR = round(CURRENT_PULSES_PER_REV * correctionFactor);

    Serial.println("\n>>> CORRECTION OPTIONS <<<");
    Serial.println("\nOption A: Adjust PULSES_PER_REV");
    Serial.print("#define PULSES_PER_REV ");
    Serial.println(correctedPPR);

    float correctedDiameter = CURRENT_WHEEL_DIAMETER_MM / correctionFactor;
    Serial.println("\nOption B: Adjust WHEEL_DIAMETER_MM");
    Serial.print("#define WHEEL_DIAMETER_MM ");
    Serial.println(correctedDiameter, 2);

    Serial.println("\nRecommendation: Use Option A (adjust PULSES_PER_REV)");
  }
}

// Test 4: Direction verification
void directionTest() {
  printDivider();
  Serial.println("TEST 4: DIRECTION VERIFICATION");
  printDivider();

  Serial.println("\nThis ensures forward motion = positive distance");
  Serial.println("\nRotate wheel in FORWARD direction (robot moving forward)");

  waitForCommand('s');

  resetEncoder();
  Serial.println("✓ Monitoring started");
  Serial.println("Rotate wheel forward 5-10 times");

  waitForCommand('e');

  Serial.print("\nEncoder count: ");
  Serial.println(encoderCount);

  if (encoderCount > 0) {
    Serial.println("✓ CORRECT: Forward = positive count");
  } else if (encoderCount < 0) {
    Serial.println("✗ ERROR: Forward = negative count");
    Serial.println("\nFIX: In Encoder.h handleInterrupt(), swap HIGH/LOW:");
    Serial.println("*encoderCount += (c2State == LOW) ? 1 : -1;  // Change HIGH to LOW");
  } else {
    Serial.println("⚠ No movement detected!");
  }
}

// Test 5: Speed and resolution test
void speedResolutionTest() {
  printDivider();
  Serial.println("TEST 5: SPEED & RESOLUTION TEST");
  printDivider();

  Serial.println("\nThis tests encoder performance at different speeds");
  Serial.println("\nWe'll measure interrupt rate and pulse detection");

  Serial.println("\nRotate wheel at NORMAL robot speed");
  waitForCommand('s');

  resetEncoder();
  unsigned long testStart = millis();
  unsigned long lastCheck = testStart;
  long lastCount = 0;
  long peakRate = 0;

  Serial.println("✓ Monitoring for 5 seconds...");

  while (millis() - testStart < 5000) {
    if (millis() - lastCheck >= 100) {
      long currentPulses = abs(encoderCount - lastCount);
      long pulsesPerSec = currentPulses * 10;  // Scale to per second

      if (pulsesPerSec > peakRate) peakRate = pulsesPerSec;

      lastCount = encoderCount;
      lastCheck = millis();
    }
    delay(1);
  }

  long totalPulses = abs(encoderCount);
  float avgPulsesPerSec = totalPulses / 5.0;
  float rpm = (avgPulsesPerSec / CURRENT_PULSES_PER_REV) * 60;
  float speedCmPerSec = calculateDistance(avgPulsesPerSec, CURRENT_PULSES_PER_REV, CURRENT_WHEEL_DIAMETER_MM);

  Serial.println("\n=== PERFORMANCE RESULTS ===");
  Serial.print("Total pulses: ");
  Serial.println(totalPulses);
  Serial.print("Average rate: ");
  Serial.print(avgPulsesPerSec, 0);
  Serial.println(" pulses/sec");
  Serial.print("Peak rate: ");
  Serial.print(peakRate);
  Serial.println(" pulses/sec");
  Serial.print("Wheel RPM: ");
  Serial.println(rpm, 1);
  Serial.print("Linear speed: ");
  Serial.print(speedCmPerSec, 1);
  Serial.println(" cm/sec");

  if (totalPulses < 100) {
    Serial.println("\n⚠ WARNING: Very low pulse count. Rotate faster or check encoder!");
  } else {
    Serial.println("\n✓ Encoder performing well at operational speed");
  }
}

// ==================== MAIN PROGRAM ====================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════════╗");
  Serial.println("║   ESP32 ENCODER CALIBRATION TOOL v2.0             ║");
  Serial.println("║   Comprehensive Testing & Calibration Suite       ║");
  Serial.println("╚════════════════════════════════════════════════════╝");

  // Initialize encoder pins
  pinMode(ENCODER_C1, INPUT);
  pinMode(ENCODER_C2, INPUT);
  delay(10);

  lastC1State = digitalRead(ENCODER_C1);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1), encoderISR, CHANGE);

  Serial.println("\n✓ Encoder initialized");
  Serial.print("C1 Pin: GPIO");
  Serial.println(ENCODER_C1);
  Serial.print("C2 Pin: GPIO");
  Serial.println(ENCODER_C2);

  Serial.println("\nCurrent Configuration:");
  Serial.print("PULSES_PER_REV: ");
  Serial.println(CURRENT_PULSES_PER_REV);
  Serial.print("WHEEL_DIAMETER_MM: ");
  Serial.println(CURRENT_WHEEL_DIAMETER_MM);

  distancePerPulseMM = (PI * CURRENT_WHEEL_DIAMETER_MM) / CURRENT_PULSES_PER_REV;
  Serial.print("Distance per pulse: ");
  Serial.print(distancePerPulseMM, 4);
  Serial.println(" mm");
}

void loop() {
  printDivider();
  Serial.println("CALIBRATION MENU");
  printDivider();
  Serial.println("\n1 - Hardware Connectivity Test");
  Serial.println("2 - Pulses Per Revolution Calibration ⭐ CRITICAL");
  Serial.println("3 - Distance Accuracy Calibration ⭐ CRITICAL");
  Serial.println("4 - Direction Verification");
  Serial.println("5 - Speed & Resolution Test");
  Serial.println("6 - Run All Tests (Recommended for new setup)");
  Serial.println("\nSelect test (1-6):");

  while (!Serial.available()) delay(10);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();  // Clear buffer

  switch (choice) {
    case '1':
      hardwareTest();
      break;
    case '2':
      pulsesPerRevTest();
      break;
    case '3':
      distanceCalibrationTest();
      break;
    case '4':
      directionTest();
      break;
    case '5':
      speedResolutionTest();
      break;
    case '6':
      Serial.println("\n>>> RUNNING FULL CALIBRATION SUITE <<<");
      hardwareTest();
      delay(2000);
      directionTest();
      delay(2000);
      pulsesPerRevTest();
      delay(2000);
      distanceCalibrationTest();
      delay(2000);
      speedResolutionTest();
      Serial.println("\n✓ Full calibration complete!");
      break;
    default:
      Serial.println("Invalid choice!");
  }

  delay(2000);
}
