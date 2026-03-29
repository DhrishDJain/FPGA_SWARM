#include "Encoder.h"
#include "MPU6050_DMP.h"
#include "Motor_driver.h"
#include "Navigator.h"
#include "config.h"
#include "espnow_slave.h"
#include "packet.h"
#include <Wire.h>

// ── Packet storage ─────────────────────────────────────────────
CommandPacket incomingCmd;
TaskPacket incomingTask;
SensorPacket outgoingSensor;

// ── ISR state ──────────────────────────────────────────────────
volatile bool mpuInterrupt = false;
volatile long encoderCount = 0;
volatile int lastC1State = 0;

// ── Objects ────────────────────────────────────────────────────
MPU6050_DMP imu(MPU_INTERRUPT_PIN, &mpuInterrupt);
Encoder encoder(ENCODER_C1, ENCODER_C2, &encoderCount, &lastC1State);
MotorControl motors(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM,
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM);
Navigator navigator;

void IRAM_ATTR dmpDataReady() { imu.handleInterrupt(); }
void IRAM_ATTR encoderISR() { encoder.handleInterrupt(); }

// ── Command handler ────────────────────────────────────────────
void handleCommand(CommandPacket &cmd) {
  if (cmd.sync != PACKET_SYNC)
    return;

  // Block manual drive during NAVIGATING and ARRIVED — navigator owns motors
  if (navigator.state == NAV_NAVIGATING || navigator.state == NAV_ARRIVED) {
    Serial.println("[CMD] ignored — navigator active");
    return;
  }

  navigator.stop();
  switch (cmd.command) {
  case 'f':
    motors.forward(cmd.speed);
    break;
  case 'b':
    motors.backward(cmd.speed);
    break;
  case 'l':
    motors.left(cmd.speed);
    break;
  case 'r':
    motors.right(cmd.speed);
    break;
  case 's':
    motors.stop();
    break;
  default:
    break;
  }
  Serial.printf("[CMD] '%c' speed=%d\n", cmd.command, cmd.speed);
}

// ── Task handler ───────────────────────────────────────────────
void handleTask(TaskPacket &task) {
  if (task.sync != PACKET_SYNC)
    return;

  switch (task.command) {

  case 'a':
    Serial.printf("[TASK] AREA (%.1f,%.1f) %ux%u blk=%.1fcm\n", task.startX,
                  task.startY, task.blocksX, task.blocksY, task.blockSize);
    navigator.startArea(task.startX, task.startY, task.blocksX, task.blocksY,
                        task.blockSize);
    break;

  case 'g':
    Serial.printf("[TASK] FPGA GOTO (%.1f,%.1f)\n", task.startX, task.startY);
    // setFpgaTarget() clears stale area mode and forces NAV_NAVIGATING
    navigator.setFpgaTarget(task.startX, task.startY);
    break;

  case 's':
    navigator.stop();
    Serial.println("[TASK] mission stopped");
    break;

  default:
    Serial.printf("[TASK] unknown cmd '%c'\n", task.command);
    break;
  }
}

// ── Telemetry ──────────────────────────────────────────────────
void sendSensorData() {
  outgoingSensor.sync = PACKET_SYNC;
  outgoingSensor.slave_id = SLAVE_ID;
  outgoingSensor.yaw = imu.yaw;
  outgoingSensor.pitch = imu.pitch;
  outgoingSensor.roll = imu.roll;
  outgoingSensor.distance_cm = encoder.totalDistanceCM;
  outgoingSensor.x = navigator.posX;
  outgoingSensor.y = navigator.posY;
  outgoingSensor.ultrasonic_mm = 0;
  outgoingSensor.timestamp = millis();

  // Priority order: NAVIGATING must be checked before ARRIVED
  // so a fresh GOTO immediately broadcasts movement state.
  if (navigator.isMissionActive()) {
    strncpy(outgoingSensor.state, "FPGA_AREA",
            sizeof(outgoingSensor.state) - 1);
  } else if (navigator.state == NAV_NAVIGATING) {
    strncpy(outgoingSensor.state, "NAVIGATING",
            sizeof(outgoingSensor.state) - 1);
  } else if (navigator.state == NAV_ARRIVED) {
    strncpy(outgoingSensor.state, "ARRIVED", sizeof(outgoingSensor.state) - 1);
  } else {
    strncpy(outgoingSensor.state, motors.currentState.c_str(),
            sizeof(outgoingSensor.state) - 1);
  }
  outgoingSensor.state[sizeof(outgoingSensor.state) - 1] = '\0';

  espnow_sendSensor(outgoingSensor);
}

// ── Setup ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);
  Serial.printf("\n=== Swarm Slave %d Starting ===\n", SLAVE_ID);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQUENCY);

  Serial.println("- IMU...");
  imu.initialize();
  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady,
                  RISING);

  Serial.println("- Encoder...");
  encoder.initialize();
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1), encoderISR, CHANGE);

  Serial.println("- ESP-NOW...");
  espnow_init(handleCommand, handleTask);

  Serial.println("- Motors...");
  motors.initialize();

  Serial.println("- Navigator...");
  navigator.init(&encoder, &imu, &motors);

  Serial.printf("\n✅ Slave %d READY\n", SLAVE_ID);
  Serial.println("Waiting for area assignment from companion...");
}

// ── Loop ───────────────────────────────────────────────────────
void loop() {
  if (imu.isReady())
    imu.update();
  encoder.update();
  navigator.update();

  // ── Telemetry to companion ──────────────────────────────────
  static uint32_t lastSend = 0;
  if (millis() - lastSend >= DATA_SEND_INTERVAL_MS) {
    lastSend = millis();
    sendSensorData();
  }

  // ── Local rich debug (slave Serial Monitor, 2s interval) ────
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug >= 2000) {
    lastDebug = millis();

    // Resolve mode and state strings
    const char *modeStr = (navigator.isFpgaTarget())      ? "FPGA_GOTO"
                          : (navigator.isAreaMode())      ? "AREA"
                          : (navigator.state == NAV_IDLE) ? "IDLE"
                                                          : "WEB_GOTO";

    const char *stateStr = (navigator.state == NAV_NAVIGATING) ? "NAVIGATING"
                           : (navigator.state == NAV_ARRIVED)  ? "ARRIVED"
                                                               : "IDLE";

    float dx = navigator.targetX - navigator.posX;
    float dy = navigator.targetY - navigator.posY;
    float dTgt = sqrtf(dx * dx + dy * dy);

    // Serial.printf(
    //   "─────────────────────────────────────────\n"
    //   "[DBG|%s|%s]\n"
    //   "  pos    : x=%.2f cm  y=%.2f cm\n"
    //   "  target : x=%.2f cm  y=%.2f cm  dist=%.2f cm\n"
    //   "  yaw    : %.1f°\n"
    //   "  encoder: count=%ld  traveled=%.2f cm\n"
    //   "  tx_state: %s\n"
    //   "─────────────────────────────────────────\n",
    //   modeStr, stateStr,
    //   navigator.posX, navigator.posY,
    //   navigator.targetX, navigator.targetY, dTgt,
    //   imu.yaw,
    //   encoder.getCount(), encoder.totalDistanceCM,
    //   outgoingSensor.state
    // );
  }
}
