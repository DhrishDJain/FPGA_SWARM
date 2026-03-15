#ifndef CONFIG_H
#define CONFIG_H
// A0:DD:6C:0F:B7:A5
// ==================== MAC ADDRESSES ====================
#define ESP1_MAC {0xA0, 0xDD, 0x6C, 0x0F, 0xB7, 0xA4} // companion
#define ESP2_MAC {0xA0, 0xDD, 0x6C, 0x0F, 0xB7, 0xA5} // slave 0
#define ESP3_MAC {0x00, 0x4B, 0x12, 0x9A, 0xCB, 0x80} // Slave 1

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
#define WHEEL_DIAMETER_MM 34.0   // was 44.0
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
#define SLAVE_ID 0  // change per device

// ==================== DEBUG ====================
#define DEBUG_ESPNOW // comment out to disable ESP-NOW debug prints

// ==================== NAVIGATION ====================
#define ARRIVE_THRESHOLD_CM  3.0f   // stop when within this distance of target
#define HEADING_KP           2.5f   // proportional gain for heading correction
#define NAV_BASE_SPEED       160    // forward speed during navigation (0-255)
#define NAV_TURN_MAX         80     // max speed differential for turning

#endif
