#include "packet.h"
#include "web_server.h"
#include <WiFi.h>
#include <esp_now.h>
// 00:4B:12:99:F2:1C
static const uint8_t SLAVE_MACS[][6] = {
  { 0x00, 0x4B, 0x12, 0x9A, 0xCB, 0x80 },  // Slave 1
  { 0x00, 0x4B, 0x12, 0x99, 0xF2, 0x1C },  // Slave 0
};
static const int NUM_SLAVES = sizeof(SLAVE_MACS) / sizeof(SLAVE_MACS[0]);

CommandPacket outgoingCmd;
TaskPacket outgoingTask;
SensorPacket incomingSensor;

// ── ESP-NOW send helpers ───────────────────────────────────────

void sendCommandTo(uint8_t idx, char cmd, int speed = 180) {
  if (idx >= NUM_SLAVES) return;
  outgoingCmd.sync = PACKET_SYNC;
  outgoingCmd.command = cmd;
  outgoingCmd.speed = (int16_t)speed;
  outgoingCmd.timestamp = millis();
  esp_err_t r = esp_now_send(SLAVE_MACS[idx],
                             (uint8_t *)&outgoingCmd, sizeof(outgoingCmd));
  Serial.printf("[CMD→%u] '%c' speed=%d %s\n",
                idx, cmd, speed, r == ESP_OK ? "OK" : "FAIL");
}

void broadcastCommand(char cmd, int speed = 180) {
  for (int i = 0; i < NUM_SLAVES; i++) sendCommandTo(i, cmd, speed);
}

void sendAreaTaskTo(uint8_t idx,
                    float sx, float sy,
                    uint16_t bx, uint16_t by,
                    float bsize) {
  if (idx >= NUM_SLAVES) return;
  outgoingTask.sync = PACKET_SYNC;
  outgoingTask.command = 'a';
  outgoingTask.startX = sx;
  outgoingTask.startY = sy;
  outgoingTask.blocksX = bx;
  outgoingTask.blocksY = by;
  outgoingTask.blockSize = bsize;
  outgoingTask.timestamp = millis();
  esp_err_t r = esp_now_send(SLAVE_MACS[idx],
                             (uint8_t *)&outgoingTask, sizeof(outgoingTask));
  Serial.printf("[AREA→%u] start=(%.1f,%.1f) %ux%u blk=%.1fcm %s\n",
                idx, sx, sy, bx, by, bsize, r == ESP_OK ? "OK" : "FAIL");
}

// Backward compat for web UI goto command
void sendTaskTo(uint8_t idx, char cmd, float tx, float ty) {
  if (idx >= NUM_SLAVES) return;
  outgoingTask.sync = PACKET_SYNC;
  outgoingTask.command = cmd;  // preserve 'g'
  outgoingTask.startX = tx;
  outgoingTask.startY = ty;
  outgoingTask.blocksX = 0;
  outgoingTask.blocksY = 0;
  outgoingTask.blockSize = 0;
  outgoingTask.timestamp = millis();
  esp_now_send(SLAVE_MACS[idx], (uint8_t *)&outgoingTask, sizeof(outgoingTask));
}

// ── Serial Monitor FPGA simulator ─────────────────────────────
//
//  Commands (Serial Monitor → line ending = Newline, 115200 baud):
//
//  A <id> <startX> <startY> <blocksX> <blocksY> <blockSize>
//    e.g.  A 0 0.0 0.0 3 3 30.0
//    e.g.  A 1 90.0 0.0 3 3 30.0
//
//  S <id|all>     → stop slave(s)
//  ? or H         → help
// ──────────────────────────────────────────────────────────────

void printHelp() {
  Serial.println("\n─────────── Serial FPGA Simulator ───────────");
  Serial.println("  A <id> <sx> <sy> <bx> <by> <bsize>");
  Serial.println("    e.g. A 0 0.0 0.0 3 3 30.0");
  Serial.println("         A 1 90.0 0.0 3 3 30.0");
  Serial.println("  S <id|all>  → stop slave(s)");
  Serial.println("  ?           → show this help");
  Serial.println("──────────────────────────────────────────────\n");
}

void parseSerialCommand() {
  static char buf[96];
  static uint8_t pos = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf[pos] = '\0';
      pos = 0;

      char *line = buf;
      while (*line == ' ') line++;
      if (*line == '\0') return;

      Serial.printf("[INPUT] %s\n", line);
      char cmd = (char)toupper((unsigned char)line[0]);

      if (cmd == 'A') {
        uint8_t id = 0;
        float sx = 0, sy = 0, bsize = 0;
        uint16_t bx = 0, by = 0;
        int n = sscanf(line + 1, " %hhu %f %f %hu %hu %f",
                       &id, &sx, &sy, &bx, &by, &bsize);
        if (n == 6) {
          if (id >= NUM_SLAVES) {
            Serial.printf("[ERR] Slave id %u out of range (0-%d)\n",
                          id, NUM_SLAVES - 1);
          } else {
            Serial.printf("[SIM] Slave %u → area (%.1f,%.1f) "
                          "%ux%u blk=%.1fcm\n",
                          id, sx, sy, bx, by, bsize);
            sendAreaTaskTo(id, sx, sy, bx, by, bsize);
          }
        } else {
          Serial.printf("[ERR] Got %d/6 args. "
                        "Format: A <id> <sx> <sy> <bx> <by> <bsize>\n",
                        n);
        }

      } else if (cmd == 'S') {
        char *arg = line + 1;
        while (*arg == ' ') arg++;
        if (strncasecmp(arg, "all", 3) == 0) {
          broadcastCommand('s', 0);
          Serial.println("[SIM] STOP all slaves");
        } else {
          uint8_t id = (uint8_t)atoi(arg);
          if (id < NUM_SLAVES) {
            sendCommandTo(id, 's', 0);
            Serial.printf("[SIM] STOP slave %u\n", id);
          } else {
            Serial.printf("[ERR] Slave id %u out of range\n", id);
          }
        }

      } else if (cmd == '?' || cmd == 'H') {
        printHelp();
      } else {
        Serial.printf("[ERR] Unknown cmd '%c'. Type ? for help\n", cmd);
      }

    } else {
      if (pos < sizeof(buf) - 1) buf[pos++] = c;
      else {
        pos = 0;
        Serial.println("[ERR] Input too long, cleared");
      }
    }
  }
}

// ── Setup & Loop ───────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== SwarmCompanion Starting ===");

  Serial.println("- Web server...");
  if (!webserver_init()) {
    while (1) {
      delay(1000);
      Serial.println("[HALTED] webserver failed");
    }
  }

  Serial.println("- ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    while (1) {
      delay(1000);
      Serial.println("[HALTED] espnow failed");
    }
  }
  esp_now_register_send_cb(
    [](const wifi_tx_info_t *info, esp_now_send_status_t status) {
      Serial.printf("[ESP-NOW TX] to %02X:%02X:%02X:%02X:%02X:%02X → %s\n",
                    info->des_addr[0], info->des_addr[1], info->des_addr[2],
                    info->des_addr[3], info->des_addr[4], info->des_addr[5],
                    status == ESP_NOW_SEND_SUCCESS ? "DELIVERED" : "FAILED");
    });



  esp_now_register_recv_cb(
    [](const esp_now_recv_info_t *info, const uint8_t *data, int len) {
      if (len == (int)sizeof(SensorPacket)) {
        memcpy(&incomingSensor, data, sizeof(incomingSensor));
        if (incomingSensor.sync == PACKET_SYNC) {
          ws_broadcastSensor(incomingSensor);
          // Optional: print to serial for debugging
          Serial.printf("[RX←%u] x=%.1f y=%.1f yaw=%.1f state=%s\n",
                        incomingSensor.slave_id,
                        incomingSensor.x, incomingSensor.y,
                        incomingSensor.yaw, incomingSensor.state);
        }
      }
    });

  for (int i = 0; i < NUM_SLAVES; i++) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, SLAVE_MACS[i], 6);
    peer.channel = 1;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }

  Serial.println("\n✅ COMPANION READY");
  Serial.printf("• WiFi  : %s / %s\n", AP_SSID, AP_PASSWORD);
  Serial.printf("• Web UI: http://%s\n",
                WiFi.softAPIP().toString().c_str());
  printHelp();
  Serial.printf("• Companion MAC: %s\n", WiFi.macAddress().c_str());
  Serial.println("• Registered slave MACs:");
  for (int i = 0; i < NUM_SLAVES; i++) {
    Serial.printf("  Slave %d: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
                  SLAVE_MACS[i][0], SLAVE_MACS[i][1], SLAVE_MACS[i][2],
                  SLAVE_MACS[i][3], SLAVE_MACS[i][4], SLAVE_MACS[i][5]);
  }
}

void loop() {
  webserver_loop();
  parseSerialCommand();
}
