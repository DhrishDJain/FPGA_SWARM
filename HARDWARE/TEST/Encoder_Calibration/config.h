#ifndef CONFIG_H
#define CONFIG_H

// ==================== ESP-NOW MAC ADDRESSES ====================
#define ESP1_MAC \
  { 0x00, 0x4B, 0x12, 0x99, 0xF2, 0x1C }  // Controller
#define ESP3_MAC \
  { 0xA0, 0xDD, 0x6C, 0x0F, 0xB7, 0xA4 }  // Robot

// ==================== I2C CONFIGURATION ====================
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQUENCY 400000

// ==================== MPU6050 CONFIGURATION ====================
#define MPU_INTERRUPT_PIN 32
#define MPU_I2C_ADDRESS 0x68

// ==================== VL53L0X CONFIGURATION ====================
#define VL53L0X_I2C_ADDRESS 0x29

// ==================== ENCODER CONFIGURATION ====================
#define ENCODER_C1 35        // Changed from 34 to 35
#define ENCODER_C2 33        // Changed from 35 to 33
#define PULSES_PER_REV 2199  // Keep your measured value
#define WHEEL_DIAMETER_MM 44.0
#define WHEEL_BASE_MM 150.0

// ==================== MOTOR PINS (TB6612FNG) ====================
#define LEFT_MOTOR_IN1 14
#define LEFT_MOTOR_IN2 12
#define LEFT_MOTOR_PWM 13
#define RIGHT_MOTOR_IN1 27
#define RIGHT_MOTOR_IN2 26
#define RIGHT_MOTOR_PWM 25

// ==================== MOTOR CONFIGURATION ====================
#define MOTOR_PWM_FREQUENCY 5000
#define MOTOR_PWM_RESOLUTION 8
#define MOTOR_DEFAULT_SPEED 180
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED 0

// ==================== TIMING ====================
#define DATA_SEND_INTERVAL_MS 200

// ==================== SERIAL ====================
#define SERIAL_BAUD_RATE 115200

#endif
// http://192.168.0.117/