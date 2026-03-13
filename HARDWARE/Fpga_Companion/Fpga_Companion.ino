/*
 * Companion (ESP1 / FPGA companion board)
 * WiFi AP + LittleFS web dashboard + ESP-NOW slave comms
 *
 * AP:   SwarmCompanion  /  swarm1234
 * URL:  http://192.168.4.1
 * Test: http://192.168.4.1/sim  (sends fake slave data to browser)
 *
 * LittleFS upload: Tools → ESP32 LittleFS Data Upload
 *   (place index.html in Fpga_Companion/data/)
 *
 * Library needed: "WebSockets by Markus Sattler" (Library Manager)
 */
// 00:4B:12:99:F2:1C
#include "../packet.h"
#include "web_server.h" // brings in WiFi AP, WebServer, WebSocket, LittleFS
#include <WiFi.h>
#include <esp_now.h>

// ── Slave MAC addresses (index = slave_id the slave firmware reports) ──
static const uint8_t SLAVE_MACS[][6] = {
    {0x00, 0x4B, 0x12, 0x99, 0xF2, 0x1C}, // Slave 0
                                          // Add more slaves here
};
static const int NUM_SLAVES = sizeof(SLAVE_MACS) / 6;

CommandPacket outgoingCmd;
TaskPacket    outgoingTask;
SensorPacket  incomingSensor;

// ── Command helpers (also called from web_server.h via extern) ──
void sendCommandTo(uint8_t idx, char cmd) {
  if (idx >= NUM_SLAVES) return;
  outgoingCmd.command   = cmd;
  outgoingCmd.timestamp = millis();
  esp_now_send(SLAVE_MACS[idx], (uint8_t *)&outgoingCmd, sizeof(outgoingCmd));
}

void broadcastCommand(char cmd) {
  for (int i = 0; i < NUM_SLAVES; i++)
    sendCommandTo(i, cmd);
}

// ── Task helper: send autonomous nav task to a slave ──
void sendTaskTo(uint8_t idx, char cmd, float tx, float ty) {
  if (idx >= NUM_SLAVES) return;
  outgoingTask.command   = cmd;
  outgoingTask.tx        = tx;
  outgoingTask.ty        = ty;
  outgoingTask.timestamp = millis();
  Serial.printf("[TASK] slave=%u cmd=%c tx=%.2f ty=%.2f\n", idx, cmd, tx, ty);
  esp_now_send(SLAVE_MACS[idx], (uint8_t *)&outgoingTask, sizeof(outgoingTask));
}

// ── ESP-NOW callbacks ──
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == (int)sizeof(SurvivorPacket)) {
    // Survivor packet — detection deferred, just log for now
    SurvivorPacket svr;
    memcpy(&svr, data, sizeof(svr));
    Serial.printf("[RECV] SURVIVOR id=%u x=%.2f y=%.2f ts=%lu\n",
                  svr.slave_id, svr.x, svr.y, svr.timestamp);
    return;
  }

  if (len < SENSOR_PKT_MIN) {
    Serial.printf("[RECV] packet too short (min %d), dropped\n", SENSOR_PKT_MIN);
    return;
  }

  memset(&incomingSensor, 0, sizeof(incomingSensor));
  memcpy(&incomingSensor, data, min((int)sizeof(incomingSensor), len));

  Serial.printf("[RECV] id=%u yaw=%.2f dist=%.2f x=%.2f y=%.2f state=%s ts=%lu\n",
                incomingSensor.slave_id, incomingSensor.yaw,
                incomingSensor.distance_cm, incomingSensor.x, incomingSensor.y,
                incomingSensor.state, incomingSensor.timestamp);

  ws_broadcastSensor(incomingSensor);
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

// ── Setup ──
void setup() {
  Serial.begin(115200);

  // 1. WiFi AP + WebServer + WebSocket (must happen before esp_now_init)
  webserver_init();

  // 2. ESP-NOW (uses AP channel set in webserver_init)
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // 3. Register slave peers on the same channel as the AP
  for (int i = 0; i < NUM_SLAVES; i++) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, SLAVE_MACS[i], 6);
    peer.channel = AP_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK)
      Serial.printf("[ESP-NOW] peer %d add failed\n", i);
  }

  outgoingCmd.speed = 180;
  Serial.println("[READY] http://192.168.4.1  |  test: http://192.168.4.1/sim");
}

// ── Loop ──
void handleSerial() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("AREA:")) {
      int commaIdx = line.indexOf(',');
      if (commaIdx > 5) {
        int size = line.substring(5, commaIdx).toInt();
        int blockSize = line.substring(commaIdx + 1).toInt();
        if (size > 0 && blockSize > 0) {
          Serial.printf("[WEB] Received Area Config: Size=%d, Block=%d\n", size, blockSize);
          ws_broadcastArena(size, blockSize);
        }
      }
    }
  }
}

void loop() {
  handleSerial();
  webserver_loop(); // handles HTTP requests + WebSocket frames
}
