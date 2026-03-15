#include <Wire.h>
#include "espnow_slave.h"  // was comms/espnow_slave.h
#include "config.h"
#include "Motor_driver.h"  // was motors/Motor_driver.h
#include "Navigator.h"     // was navigation/Navigator.h
#include "packet.h"
#include "Encoder.h"      // was sensor/Encoder.h
#include "MPU6050_DMP.h"  // was sensor/MPU6050_DMP.h


// ── Packet storage (extern'd by espnow_slave.h) ───────────────
CommandPacket incomingCmd;
TaskPacket incomingTask;
SensorPacket outgoingSensor;

// ── ISR state ─────────────────────────────────────────────────
volatile bool mpuInterrupt = false;
volatile long encoderCount = 0;
volatile int lastC1State = 0;

// ── Objects ───────────────────────────────────────────────────
MPU6050_DMP imu(MPU_INTERRUPT_PIN, &mpuInterrupt);
Encoder encoder(ENCODER_C1, ENCODER_C2,
                &encoderCount, &lastC1State);
MotorControl motors(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM,
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM);
Navigator navigator;

void IRAM_ATTR dmpDataReady() {
  imu.handleInterrupt();
}
void IRAM_ATTR encoderISR() {
  encoder.handleInterrupt();
}

// ── Command handler (manual drive, blocked during mission) ─────
void handleCommand(CommandPacket &cmd) {
  if (cmd.sync != PACKET_SYNC) return;

  if (navigator.state == NAV_NAVIGATING) {
    Serial.println("[CMD] ignored — mission active");
    return;
  }

  navigator.stop();
  switch (cmd.command) {
    case 'f': motors.forward(cmd.speed); break;
    case 'b': motors.backward(cmd.speed); break;
    case 'l': motors.left(cmd.speed); break;
    case 'r': motors.right(cmd.speed); break;
    case 's': motors.stop(); break;
    default: break;
  }
  Serial.printf("[CMD] '%c' speed=%d\n", cmd.command, cmd.speed);
}

// ── Task handler (FPGA area mission or web goto) ──────────────
void handleTask(TaskPacket &task) {
  if (task.sync != PACKET_SYNC) return;

  switch (task.command) {
    case 'a':
      Serial.printf("[TASK] AREA (%.1f,%.1f) %ux%u blk=%.1fcm\n",
                    task.startX, task.startY,
                    task.blocksX, task.blocksY, task.blockSize);
      navigator.startArea(task.startX, task.startY,
                          task.blocksX, task.blocksY, task.blockSize);
      strncpy(outgoingSensor.state, "FPGA_AREA",
              sizeof(outgoingSensor.state) - 1);
      break;

    case 'g':
      Serial.printf("[TASK] GOTO (%.1f,%.1f)\n",
                    task.startX, task.startY);
      navigator.setTarget(task.startX, task.startY);
      strncpy(outgoingSensor.state, "WEB_GOTO",
              sizeof(outgoingSensor.state) - 1);
      break;

    case 's':
      navigator.stop();
      strncpy(outgoingSensor.state, "STOPPED",
              sizeof(outgoingSensor.state) - 1);
      Serial.println("[TASK] mission stopped");
      break;

    default:
      Serial.printf("[TASK] unknown cmd '%c'\n", task.command);
      break;
  }
}

// ── Telemetry ─────────────────────────────────────────────────
void sendSensorData() {
  outgoingSensor.sync = PACKET_SYNC;
  outgoingSensor.slave_id = SLAVE_ID;
  outgoingSensor.yaw = imu.yaw;
  outgoingSensor.distance_cm = encoder.totalDistanceCM;
  
  outgoingSensor.x = navigator.posX;
  outgoingSensor.y = navigator.posY;
  outgoingSensor.lidar_mm = 0;
  outgoingSensor.timestamp = millis();

  if (navigator.isMissionActive()) {
    strncpy(outgoingSensor.state, "FPGA_AREA",
            sizeof(outgoingSensor.state) - 1);
  } else if (navigator.state == NAV_ARRIVED) {
    strncpy(outgoingSensor.state, "ARRIVED",
            sizeof(outgoingSensor.state) - 1);
  } else if (navigator.isIdle()) {
    strncpy(outgoingSensor.state, motors.currentState.c_str(),
            sizeof(outgoingSensor.state) - 1);
  }
  outgoingSensor.state[sizeof(outgoingSensor.state) - 1] = '\0';

  espnow_sendSensor(outgoingSensor);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);
  Serial.printf("\n=== Swarm Slave %d Starting ===\n", SLAVE_ID);
  Serial.println("Stopped. Continue to normal setup...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQUENCY);

  Serial.println("- IMU...");
  imu.initialize();
  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN),
                  dmpDataReady, RISING);

  Serial.println("- Encoder...");
  encoder.initialize();
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1),
                  encoderISR, CHANGE);

  Serial.println("- ESP-NOW...");
  espnow_init(handleCommand, handleTask);

  Serial.println("- Motors...");
  motors.initialize();
  // Serial.println("=== Test 1: forward() ===");
  // motors.forward(180);
  // delay(2000);
  // motors.stop();
  // delay(1000);

  // Serial.println("=== Test 2: driveRaw(80, 240) ===");
  // motors.driveRaw(80, 240);  // exact values from nav debug
  // delay(2000);
  // motors.stop();
  // delay(1000);

  // Serial.println("=== Test 3: driveRaw(0, 255) ===");
  // motors.driveRaw(0, 255);  // right motor only, max speed
  // delay(2000);
  // motors.stop();
  // delay(1000);

  // Serial.println("=== Test 4: driveRaw(180, 180) ===");
  // motors.driveRaw(180, 180);  // same as forward(180)
  // delay(2000);
  // motors.stop();


  Serial.println("- Navigator...");
  navigator.init(&encoder, &imu, &motors);

  Serial.printf("\n✅ Slave %d READY\n", SLAVE_ID);
  Serial.println("Waiting for area assignment from companion...");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  static uint32_t lastPrint = 0;
  // Sensors updated once — Navigator.update() uses these values
  if (imu.isReady()) imu.update();
  encoder.update();

  navigator.update();

  static uint32_t lastSend = 0;
  if (millis() - lastSend >= DATA_SEND_INTERVAL_MS) {
    lastSend = millis();
    sendSensorData();
  }
}
