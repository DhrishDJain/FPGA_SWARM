#ifndef ESPNOW_SLAVE_H
#define ESPNOW_SLAVE_H

#include "config.h"
#include "packet.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifdef DEBUG_ESPNOW
#define ESPNOW_DBG(fmt, ...) Serial.printf("[ESPNOW] " fmt "\n", ##__VA_ARGS__)
#else
#define ESPNOW_DBG(fmt, ...) \
  do { \
  } while (0)
#endif

extern CommandPacket incomingCmd;
extern TaskPacket incomingTask;
extern SensorPacket outgoingSensor;

static void (*_onCmdReceived)(CommandPacket &) = nullptr;
static void (*_onTaskReceived)(TaskPacket &) = nullptr;

// Companion STA MAC — ESP-NOW uses STA interface
static uint8_t _companionMAC[] = { 0xA0, 0xDD, 0x6C, 0x0F, 0xB7, 0xA4 };

void _espnow_onRecv(const esp_now_recv_info_t *info,
                    const uint8_t *data, int len) {
  if (len == (int)sizeof(TaskPacket)) {
    memcpy(&incomingTask, data, sizeof(TaskPacket));
    ESPNOW_DBG("TASK | cmd='%c' sx=%.1f sy=%.1f bx=%u by=%u size=%.1f",
               incomingTask.command,
               incomingTask.startX, incomingTask.startY,
               incomingTask.blocksX, incomingTask.blocksY,
               incomingTask.blockSize);
    if (_onTaskReceived) _onTaskReceived(incomingTask);

  } else if (len == (int)sizeof(CommandPacket)) {
    memcpy(&incomingCmd, data, sizeof(CommandPacket));
    ESPNOW_DBG("CMD | cmd='%c' speed=%d",
               incomingCmd.command, incomingCmd.speed);
    if (_onCmdReceived) _onCmdReceived(incomingCmd);

  } else {
    ESPNOW_DBG("unknown len=%d dropped", len);
  }  // ✅ closes else
}  // ✅ closes _espnow_onRecv

bool espnow_init(void (*cmdCB)(CommandPacket &),
                 void (*taskCB)(TaskPacket &)) {
  _onCmdReceived = cmdCB;
  _onTaskReceived = taskCB;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Force channel 1 to match companion softAP
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  Serial.printf("[SLAVE] MAC: %s\n", WiFi.macAddress().c_str());
  Serial.printf("[SLAVE] Channel: %d\n", WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return false;
  }  // ✅ closes if

  esp_now_register_recv_cb(_espnow_onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, _companionMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] add companion peer failed");
    return false;
  }  // ✅ closes if

  Serial.println("[ESPNOW] ready");
  return true;
}  // ✅ closes espnow_init

void espnow_sendSensor(SensorPacket &pkt) {
  // ESPNOW_DBG("TX | x=%.1f y=%.1f yaw=%.1f state=%s",
  //            pkt.x, pkt.y, pkt.yaw, pkt.state);
  esp_now_send(_companionMAC, (uint8_t *)&pkt, sizeof(pkt));
}

void espnow_sendSurvivor(SurvivorPacket &pkt) {
  esp_now_send(_companionMAC, (uint8_t *)&pkt, sizeof(pkt));
}

#endif
