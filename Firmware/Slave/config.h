#ifndef CONFIG_H
#define CONFIG_H

// ==================== MAC ADDRESSES ====================
#define ESP1_MAC \
  { 0xA0, 0xDD, 0x6C, 0x0F, 0xB7, 0xA4 }  // companion
#define ESP2_MAC \
  { 0xA0, 0xDD, 0x6C, 0x0F, 0xB7, 0xA5 }  // slave 0
#define ESP3_MAC \
  { 0x00, 0x4B, 0x12, 0x9A, 0xCB, 0x80 }  // slave 1

// ==================== I2C PINS ====================
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQUENCY 400000

// ==================== MPU6050 ====================
#define MPU_INTERRUPT_PIN 23
#define MPU_I2C_ADDRESS 0x68

// ==================== ENCODER PINS ====================
#define ENCODER_C1 15
#define ENCODER_C2 4
#define PULSES_PER_REV 1813
#define WHEEL_DIAMETER_MM 34.0
#define WHEEL_BASE_MM 150.0

// ==================== MOTOR PINS (TB6612FNG) ====================
#define LEFT_MOTOR_IN1 14
#define LEFT_MOTOR_IN2 12
#define LEFT_MOTOR_PWM 13
#define RIGHT_MOTOR_IN1 26
#define RIGHT_MOTOR_IN2 25
#define RIGHT_MOTOR_PWM 33

// ==================== MOTOR CONFIG ====================
#define MOTOR_PWM_FREQUENCY 5000
#define MOTOR_PWM_RESOLUTION 8
#define MOTOR_DEFAULT_SPEED 180
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_SPEED 0

// ==================== TIMING ====================
#define DATA_SEND_INTERVAL_MS 200

// ==================== SERIAL ====================
#define SERIAL_BAUD_RATE 115200

// ==================== SLAVE ID ====================
// 0 → FPGA sends 0xC1
// 1 → FPGA sends 0xC2
// 2 → FPGA sends 0xC3
#define SLAVE_ID 0

// ==================== DEBUG ====================
#define DEBUG_ESPNOW

// ==================== NAVIGATION ====================
#define ARRIVE_THRESHOLD_CM 0.5f
#define HEADING_KP 2.5f
#define NAV_BASE_SPEED 160
#define NAV_TURN_MAX 80
#define NAV_WAYPOINT_TIMEOUT_MS 15000  // 15s max per waypoint

// ==================== FPGA UART (Companion only) ====================
#define FPGA_RX_PIN 16         // GPIO16 ← FPGA TX
#define FPGA_TX_PIN 17         // GPIO17 → FPGA RX
#define FPGA_BAUD_RATE 115200  // must match FPGA clock divider

// ==================== ARENA GRID ====================
#define ARENA_SIZE_CM 300.0f  // 3m × 3m
#define FPGA_GRID_SIZE 10     // 10×10 cells
#define FPGA_CELL_CM 30.0f    // 300/10 = 30cm per cell

#endif
