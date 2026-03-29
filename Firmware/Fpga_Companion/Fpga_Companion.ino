#include "packet.h"
#include "web_server.h"
#include <WiFi.h>
#include <esp_now.h>

// ── FPGA UART (Serial2) ────────────────────────────────────────
#define FPGA_RX 16
#define FPGA_TX 17
#define FPGA_BAUD 115200

#define FPGA_REQ_BASE 0xC1
#define FPGA_RESP_SYNC 0xBB
#define FPGA_MAP_SYNC 0xAA
#define FPGA_PP_FREE 0x01
#define FPGA_PP_OBSTACLE 0x10
#define FPGA_OBSTACLE_THRESH_MM 300

static const uint8_t SLAVE_MACS[][6] = {
  { 0x00, 0x4B, 0x12, 0x9A, 0xCB, 0x80 },  // Slave 0
  { 0x00, 0x4B, 0x12, 0x99, 0xF2, 0x1C },  // Slave 1
};
static const int NUM_SLAVES = sizeof(SLAVE_MACS) / sizeof(SLAVE_MACS[0]);

CommandPacket outgoingCmd;
TaskPacket outgoingTask;
SensorPacket incomingSensor;

static uint8_t lastRequestedSlave = 0;
static bool arrivedSlave[8] = {};
static uint8_t hardcodedPP[8] = {
  FPGA_PP_FREE, FPGA_PP_FREE, FPGA_PP_FREE, FPGA_PP_FREE,
  FPGA_PP_FREE, FPGA_PP_FREE, FPGA_PP_FREE, FPGA_PP_FREE
};

static float simulatedX[8] = {0};
static float simulatedY[8] = {0};

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
  ws_broadcastTask(idx, sx, sy);
  Serial.printf("[AREA→%u] start=(%.1f,%.1f) %ux%u blk=%.1fcm %s\n",
                idx, sx, sy, bx, by, bsize, r == ESP_OK ? "OK" : "FAIL");
}

void sendTaskTo(uint8_t idx, char cmd, float tx, float ty) {
  if (idx >= NUM_SLAVES) return;
  outgoingTask.sync = PACKET_SYNC;
  outgoingTask.command = cmd;
  outgoingTask.startX = tx;
  outgoingTask.startY = ty;
  outgoingTask.blocksX = 0;
  outgoingTask.blocksY = 0;
  outgoingTask.blockSize = 0;
  outgoingTask.timestamp = millis();
  esp_now_send(SLAVE_MACS[idx], (uint8_t *)&outgoingTask, sizeof(outgoingTask));
  if (cmd == 'g' || cmd == 'a') {
    ws_broadcastTask(idx, tx, ty);
  }
  Serial.printf("[GOTO→%u] x=%.1f y=%.1f\n", idx, tx, ty);
}

// ── FPGA UART helpers ──────────────────────────────────────────

void requestFpgaCoordinate(uint8_t slaveId) {
  uint8_t req = FPGA_REQ_BASE | (slaveId & 0x0F);
  // Serial2.write(req); // Disabled for testing
  Serial.printf("[FPGA→] (SIMULATED) Requested coord for slave %u (0x%02X)\n", slaveId, req);

  // Hardcoded FPGA response for testing: increment of 1 in x or y randomly
  // Starting from 0,0 and incrementing each time.
  if (random(2) == 0) {
    simulatedX[slaveId] += 1.0;
  } else {
    simulatedY[slaveId] += 1.0;
  }

  uint8_t fx = (uint8_t)simulatedX[slaveId];
  uint8_t fy = (uint8_t)simulatedY[slaveId];

  Serial.printf("[FPGA←] (SIMULATED) Coord X=%u Y=%u → sending goto to slave %u\n", fx, fy, slaveId);
  sendTaskTo(slaveId, 'g', (float)fx, (float)fy);
}

// FIX: pp is now derived from hardcodedPP OR lidar reading if provided
void notifyFpgaArrived(uint8_t slaveId, float x, float y, uint16_t lidarMm = 0xFFFF) {
  uint8_t pp;
  if (lidarMm != 0xFFFF) {
    // Auto-detect from live lidar reading
    pp = (lidarMm < FPGA_OBSTACLE_THRESH_MM) ? FPGA_PP_OBSTACLE : FPGA_PP_FREE;
  } else {
    // Fall back to manually preset value via 'F <id> [0|1]'
    pp = (slaveId < 8) ? hardcodedPP[slaveId] : FPGA_PP_FREE;
  }

  uint8_t pkt[4] = {
    FPGA_MAP_SYNC,
    (uint8_t)constrain((int)x, 0, 255),
    (uint8_t)constrain((int)y, 0, 255),
    pp
  };
  Serial2.write(pkt, 4);
  Serial.printf("[FPGA→] MAP UPDATE: X=%u Y=%u pp=0x%02X (%s)\n",
                pkt[1], pkt[2], pp,
                pp == FPGA_PP_OBSTACLE ? "OBSTACLE" : "FREE");
}

// ── FIX: State-machine FPGA response parser ────────────────────
// Replaces the broken ">= 3 bytes" approach that caused framing loss.
// Reads one byte at a time — robust against partial UART delivery.
void handleFpgaResponse() {
  static enum { WAIT_SYNC,
                WAIT_X,
                WAIT_Y } state = WAIT_SYNC;
  static uint8_t fx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    switch (state) {
      case WAIT_SYNC:
        if (b == FPGA_RESP_SYNC) {
          state = WAIT_X;
        } else {
          Serial.printf("[FPGA←] Unexpected byte 0x%02X, discarding\n", b);
        }
        break;

      case WAIT_X:
        fx = b;
        state = WAIT_Y;
        break;

      case WAIT_Y:
        {
          uint8_t fy = b;
          state = WAIT_SYNC;
          Serial.printf("[FPGA←] Coord X=%u Y=%u → sending goto to slave %u\n",
                        fx, fy, lastRequestedSlave);
          sendTaskTo(lastRequestedSlave, 'g', (float)fx, (float)fy);
          // ✅ DO NOT clear arrivedSlave here — slave is still physically at
          //    old position and will keep sending ARRIVED until it moves.
          //    Flag is cleared in the recv callback when state != ARRIVED.
          break;
        }
    }
  }
}


// ── Serial Monitor commands ────────────────────────────────────

void printHelp() {
  Serial.println("\n─────────── Serial FPGA Simulator ───────────");
  Serial.println("  A <id> <sx> <sy> <bx> <by> <bsize>");
  Serial.println("    e.g. A 0 0.0 0.0 3 3 30.0");
  Serial.println("         A 1 90.0 0.0 3 3 30.0");
  Serial.println("  F <id> [0|1]  → manually trigger map notify + coord request");
  Serial.println("                  0=free (default)  1=obstacle");
  Serial.println("  S <id|all>    → stop slave(s)");
  Serial.println("  ?             → show this help");
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
            Serial.printf("[ERR] Slave id %u out of range (0-%d)\n", id, NUM_SLAVES - 1);
          } else {
            Serial.printf("[SIM] Slave %u → area (%.1f,%.1f) %ux%u blk=%.1fcm\n",
                          id, sx, sy, bx, by, bsize);
            sendAreaTaskTo(id, sx, sy, bx, by, bsize);
          }
        } else {
          Serial.printf("[ERR] Got %d/6 args. Format: A <id> <sx> <sy> <bx> <by> <bsize>\n", n);
        }

      } else if (cmd == 'F') {
        // Manual debug override:
        // Sends AA map notify with preset pp, then immediately sends 0xC1 coord request.
        // Useful for testing FPGA response without needing a robot to physically arrive.
        char *arg = line + 1;
        while (*arg == ' ') arg++;
        uint8_t id = (uint8_t)strtoul(arg, &arg, 10);
        while (*arg == ' ') arg++;
        uint8_t pp = (*arg == '1') ? FPGA_PP_OBSTACLE : FPGA_PP_FREE;

        if (id < NUM_SLAVES) {
          hardcodedPP[id] = pp;
          lastRequestedSlave = id;

          // Send dummy map update at (0,0) so FPGA knows the pp, then request coord
          notifyFpgaArrived(id, 0, 0);  // STEP 1: AA 00 00 pp
          requestFpgaCoordinate(id);    // STEP 2: 0xC1 | id

          Serial.printf("[SIM] F: slave=%u pp=0x%02X (%s) — waiting for FPGA coord\n",
                        id, pp, pp == FPGA_PP_OBSTACLE ? "OBSTACLE" : "FREE");
        } else {
          Serial.printf("[ERR] Slave id %u out of range\n", id);
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
  Serial2.begin(FPGA_BAUD, SERIAL_8N1, FPGA_RX, FPGA_TX);
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

  esp_now_register_send_cb([](const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.printf("[ESP-NOW TX] to %02X:%02X:%02X:%02X:%02X:%02X → %s\n", info->des_addr[0], info->des_addr[1], info->des_addr[2], info->des_addr[3], info->des_addr[4], info->des_addr[5],
                  status == ESP_NOW_SEND_SUCCESS ? "DELIVERED" : "FAILED");
  });

  esp_now_register_recv_cb([](const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len == (int)sizeof(SensorPacket)) {
      memcpy(&incomingSensor, data, sizeof(incomingSensor));
      if (incomingSensor.sync == PACKET_SYNC) {
        ws_broadcastSensor(incomingSensor);
        Serial.printf("[Slave %u] x=%.1f y=%.1f yaw=%.1f pitch=%.1f roll=%.1f state=%s\n",
                      incomingSensor.slave_id,
                      incomingSensor.x, incomingSensor.y,
                      incomingSensor.yaw, incomingSensor.pitch, incomingSensor.roll, incomingSensor.state);

        uint8_t sid = incomingSensor.slave_id;
        if (sid < 8) {
          if (strcmp(incomingSensor.state, "ARRIVED") == 0) {
            if (!arrivedSlave[sid]) {
              arrivedSlave[sid] = true;

              // ── STEP 1: Notify FPGA of current cell status ──
              // Pass lidar reading if your SensorPacket carries it,
              // otherwise falls back to hardcodedPP set by 'F' command.
              // e.g. notifyFpgaArrived(sid, incomingSensor.x, incomingSensor.y,
              //                        incomingSensor.lidar_mm);
              notifyFpgaArrived(sid, incomingSensor.x, incomingSensor.y);

              // ── STEP 2: Immediately request next coordinate ──
              lastRequestedSlave = sid;
              requestFpgaCoordinate(sid);
            }
          } else {
            arrivedSlave[sid] = false;
          }
        }
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
  Serial.printf("• Web UI: http://%s\n", WiFi.softAPIP().toString().c_str());
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
  handleFpgaResponse();  // State-machine: reads BB X Y from FPGA, routes goto to slave
}
