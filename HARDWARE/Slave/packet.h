#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

// Command packet: Companion → Slave (manual drive)
typedef struct {
  char command; // 'f'=forward, 'b'=backward, 'l'=left, 'r'=right, 's'=stop
  int speed;    // 0-255
  unsigned long timestamp;
} CommandPacket;

// Task packet: Companion → Slave (autonomous nav)
typedef struct {
  char command;        // 'g'=goto target, 's'=stop navigation
  float tx;            // target X (cm)
  float ty;            // target Y (cm)
  unsigned long timestamp;
} TaskPacket;

// Sensor packet: Slave → Companion
typedef struct {
  uint8_t slave_id;  // unique slave index (0-based)
  float yaw;         // degrees from MPU6050 DMP
  float distance_cm; // net distance from encoder
  float x;           // dead-reckoned X position (cm)
  float y;           // dead-reckoned Y position (cm)
  uint16_t lidar_mm; // reserved (unused for now)
  char state[10];    // motor/nav state string
  unsigned long timestamp;
} SensorPacket;

// Survivor packet: Slave → Companion (detection deferred, stub)
typedef struct {
  uint8_t slave_id;
  float x;
  float y;
  unsigned long timestamp;
} SurvivorPacket;

#endif
