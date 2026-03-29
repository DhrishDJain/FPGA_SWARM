#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define PACKET_SYNC 0xAA

typedef struct __attribute__((packed)) {
  uint8_t sync;
  char command; // 'f','b','l','r','s'
  int16_t speed;
  uint32_t timestamp;
} CommandPacket;

typedef struct __attribute__((packed)) {
  uint8_t sync;
  char command; // 'a'=area, 'g'=goto, 's'=stop
  float startX;
  float startY;
  uint16_t blocksX;
  uint16_t blocksY;
  float blockSize;
  uint32_t timestamp;
} TaskPacket;
typedef struct __attribute__((packed)) {
  uint8_t sync;
  uint8_t slave_id;
  float yaw;
  float pitch;
  float roll;
  float distance_cm;
  float x;
  float y;
  uint16_t ultrasonic_mm;
  char state[12];
  uint32_t timestamp;
} SensorPacket;

typedef struct __attribute__((packed)) {
  uint8_t sync;
  uint8_t slave_id;
  float x;
  float y;
  uint32_t timestamp;
} SurvivorPacket;

#endif
