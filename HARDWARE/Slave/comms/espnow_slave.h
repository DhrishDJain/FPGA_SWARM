#ifndef ESPNOW_SLAVE_H
#define ESPNOW_SLAVE_H

#include "../config.h"
#include "../packet.h"
#include <WiFi.h>
#include <esp_now.h>

// ==================== DEBUG ====================
#ifdef DEBUG_ESPNOW
#define ESPNOW_DBG(fmt, ...) Serial.printf("[ESPNOW] " fmt "\n", ##__VA_ARGS__)
#else
#define ESPNOW_DBG(fmt, ...) do {} while (0)
#endif

// Incoming packets from companion
CommandPacket incomingCmd;
TaskPacket    incomingTask;

// Outgoing sensor data to companion
SensorPacket outgoingSensor;

// Companion MAC address
uint8_t companionAddress[] = ESP1_MAC;

// Callbacks registered by Slave.ino
static void (*_onCmdReceived)(CommandPacket &)  = nullptr;
static void (*_onTaskReceived)(TaskPacket &)    = nullptr;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(TaskPacket)) {
    memcpy(&incomingTask, data, sizeof(incomingTask));
    ESPNOW_DBG("TASK recv | cmd='%c' tx=%.2f ty=%.2f ts=%lu",
               incomingTask.command, incomingTask.tx, incomingTask.ty,
               incomingTask.timestamp);
    if (_onTaskReceived)
      _onTaskReceived(incomingTask);

  } else if (len >= (int)sizeof(CommandPacket)) {
    memcpy(&incomingCmd, data, sizeof(incomingCmd));
    ESPNOW_DBG("CMD recv | cmd='%c' speed=%d ts=%lu",
               incomingCmd.command, incomingCmd.speed, incomingCmd.timestamp);
    if (_onCmdReceived)
      _onCmdReceived(incomingCmd);

  } else {
    ESPNOW_DBG("unknown packet len=%d, dropped", len);
  }
}

bool espnow_init(void (*cmdCallback)(CommandPacket &),
                 void (*taskCallback)(TaskPacket &)) {
  _onCmdReceived  = cmdCallback;
  _onTaskReceived = taskCallback;

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, companionAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add companion as peer");
    return false;
  }

  Serial.println("ESP-NOW ready, companion peer added");
  return true;
}

void espnow_sendSensor(SensorPacket &pkt) {
  ESPNOW_DBG("SENSOR send | yaw=%.2f dist=%.2f x=%.2f y=%.2f state=%s ts=%lu",
             pkt.yaw, pkt.distance_cm, pkt.x, pkt.y, pkt.state, pkt.timestamp);
  esp_err_t res = esp_now_send(companionAddress, (uint8_t *)&pkt, sizeof(pkt));
  ESPNOW_DBG("send result: %s", res == ESP_OK ? "OK" : "FAIL");
}

// Stub: survivor packet send (detection deferred)
void espnow_sendSurvivor(SurvivorPacket &pkt) {
  esp_now_send(companionAddress, (uint8_t *)&pkt, sizeof(pkt));
}

#endif
