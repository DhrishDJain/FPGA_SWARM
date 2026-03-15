#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "packet.h"
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <LittleFS.h>

#define AP_SSID "SwarmCompanion"
#define AP_PASSWORD "swarm1234"
#define AP_CHANNEL 1

// ✅ Pointers — constructed inside webserver_init() after scheduler starts
static WebServer *webServer = nullptr;
static WebSocketsServer *wsServer = nullptr;
static DNSServer *dnsServer = nullptr;

// Forward declarations (defined in .ino)
extern void sendCommandTo(uint8_t idx, char cmd, int speed);
extern void broadcastCommand(char cmd, int speed);
extern void sendTaskTo(uint8_t idx, char cmd, float tx, float ty);

// ── Broadcast helpers ──────────────────────────────────────────

void ws_broadcastSensor(const SensorPacket &pkt) {
  if (!wsServer) return;
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"sensor\",\"id\":%u,\"x\":%.1f,\"y\":%.1f,"
           "\"yaw\":%.1f,\"dist\":%.1f,\"lidar\":%u,\"state\":\"%s\"}",
           pkt.slave_id, pkt.x, pkt.y, pkt.yaw,
           pkt.distance_cm, pkt.lidar_mm, pkt.state);
  wsServer->broadcastTXT(buf);
}

void ws_broadcastArena(int size, int blockSize) {
  if (!wsServer) return;
  char buf[80];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"arena\",\"size\":%d,\"grid\":%d}", size, blockSize);
  wsServer->broadcastTXT(buf);
}

// ── WebSocket event handler ────────────────────────────────────

void _wsEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Client %u connected\n", client);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Client %u disconnected\n", client);
    return;
  }
  if (type != WStype_TEXT) return;

  // Parse: {"type":"cmd","cmd":"f","target":"0"}
  char *msg = (char *)payload;

  char *cmdPos = strstr(msg, "\"cmd\":");
  char *targetPos = strstr(msg, "\"target\":");
  if (!cmdPos || !targetPos) return;

  // Extract cmd char
  char *p = strchr(cmdPos, ':') + 1;
  while (*p == ' ' || *p == '"') p++;
  char cmd = *p;

  // Extract target string
  char targetStr[8] = { 0 };
  p = strchr(targetPos, ':') + 1;
  while (*p == ' ' || *p == '"') p++;
  int i = 0;
  while (*p && *p != '"' && *p != ',' && *p != '}' && i < 7)
    targetStr[i++] = *p++;
  targetStr[i] = '\0';

  Serial.printf("[WS] target=%s cmd='%c'\n", targetStr, cmd);

  if (strncmp(targetStr, "all", 3) == 0) {
    broadcastCommand(cmd, 180);
  } else {
    sendCommandTo((uint8_t)atoi(targetStr), cmd, 180);
  }
}

// ── File server ───────────────────────────────────────────────

void _serveFile(const char *path) {
  if (!LittleFS.exists(path)) {
    webServer->send(404, "text/plain",
                    "File not found. Upload data/index.html via LittleFS.");
    return;
  }
  File f = LittleFS.open(path, "r");
  String ct = String(path).endsWith(".html") ? "text/html" : "text/plain";
  webServer->streamFile(f, ct);
  f.close();
}

// ── Init ──────────────────────────────────────────────────────

bool webserver_init() {
  // Allocate after RTOS scheduler is running
  webServer = new WebServer(80);
  wsServer = new WebSocketsServer(81);
  dnsServer = new DNSServer();

  if (!LittleFS.begin(true)) {
    Serial.println("[WEB] LittleFS mount failed");
    return false;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.printf("[WEB] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  webServer->on("/", HTTP_GET, []() {
    _serveFile("/index.html");
  });

  // Web UI task endpoint: {"slave":0,"cmd":"g","tx":100,"ty":200}
  webServer->on("/task", HTTP_POST, []() {
    if (!webServer->hasArg("plain")) {
      webServer->send(400, "text/plain", "missing body");
      return;
    }
    const char *body = webServer->arg("plain").c_str();

    auto extract = [&](const char *key) -> float {
      char search[24];
      snprintf(search, sizeof(search), "\"%s\":", key);
      const char *p = strstr(body, search);
      if (!p) return 0.0f;
      p = strchr(p, ':') + 1;
      while (*p == ' ' || *p == '"') p++;
      return atof(p);
    };

    auto extractCmd = [&]() -> char {
      const char *p = strstr(body, "\"cmd\":");
      if (!p) return 's';
      p = strchr(p, ':') + 1;
      while (*p == ' ' || *p == '"') p++;
      return *p;
    };

    uint8_t slaveIdx = (uint8_t)extract("slave");
    char cmd = extractCmd();
    float tx = extract("tx");
    float ty = extract("ty");

    sendTaskTo(slaveIdx, cmd, tx, ty);
    webServer->send(200, "application/json", "{\"ok\":true}");
  });

  webServer->onNotFound([]() {
    _serveFile("/index.html");
  });
  webServer->begin();

  wsServer->begin();
  wsServer->onEvent(_wsEvent);

  dnsServer->start(53, "*", WiFi.softAPIP());

  Serial.println("[WEB] Ready");
  return true;
}

void webserver_loop() {
  dnsServer->processNextRequest();
  webServer->handleClient();
  wsServer->loop();
}

#endif
