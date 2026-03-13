#include <Wire.h>

#include "comms/espnow_slave.h"
#include "config.h"
#include "motors/Motor_driver.h"
#include "navigation/Navigator.h"
#include "sensor/Encoder.h"
#include "sensor/MPU6050_DMP.h"

// Global ISR state
volatile bool mpuInterrupt = false;
volatile long encoderCount = 0;
volatile int lastC1State = 0;

// Objects
MPU6050_DMP imu(MPU_INTERRUPT_PIN, &mpuInterrupt);
Encoder encoder(ENCODER_C1, ENCODER_C2, &encoderCount, &lastC1State);
MotorControl motors(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM,
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM);
Navigator navigator;

// ISRs
void IRAM_ATTR dmpDataReady() { imu.handleInterrupt(); }
void IRAM_ATTR encoderISR()   { encoder.handleInterrupt(); }

// ── Manual command handler (called by espnow_slave when CommandPacket arrives) ──
void handleCommand(CommandPacket &cmd) {
  // Manual commands cancel any active navigation
  navigator.stop();

  Serial.print("[CMD] ");
  Serial.println(cmd.command);

  switch (cmd.command) {
  case 'f': motors.forward(cmd.speed);  break;
  case 'b': motors.backward(cmd.speed); break;
  case 'l': motors.left(cmd.speed);     break;
  case 'r': motors.right(cmd.speed);    break;
  case 's': motors.stop();              break;
  }
}

// ── Task handler (called by espnow_slave when TaskPacket arrives) ──
void handleTask(TaskPacket &task) {
  switch (task.command) {
  case 'g':
    navigator.setTarget(task.tx, task.ty);
    break;
  case 's':
    navigator.stop();
    Serial.println("[TASK] navigation stopped");
    break;
  default:
    Serial.printf("[TASK] unknown cmd '%c'\n", task.command);
    break;
  }
}

// ── Send sensor data to companion ──
void sendSensorData() {
  outgoingSensor.slave_id    = SLAVE_ID;
  outgoingSensor.yaw         = imu.yaw;
  outgoingSensor.distance_cm = encoder.distanceTraveledCM;
  outgoingSensor.x           = navigator.posX;
  outgoingSensor.y           = navigator.posY;
  outgoingSensor.lidar_mm    = 0;  // unused
  strcpy(outgoingSensor.state, motors.currentState.c_str());
  outgoingSensor.timestamp   = millis();
  espnow_sendSensor(outgoingSensor);
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("\n=== Slave Starting ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQUENCY);

  imu.initialize();
  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);

  encoder.initialize();
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1), encoderISR, CHANGE);

  motors.initialize();
  
  navigator.init(&encoder, &imu, &motors);

  espnow_init(handleCommand, handleTask);

  Serial.printf("[SLAVE] ID=%d  start pos (0,0)\n", SLAVE_ID);
}

void loop() {
  if (imu.isReady())
    imu.update();
  encoder.update();

  // Navigator computes dead reckoning from sensors, handles PID,   and drives motors
  navigator.update();

  sendSensorData();
  delay(DATA_SEND_INTERVAL_MS);
}
