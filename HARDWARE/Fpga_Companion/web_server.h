#ifndef WEB_SERVER_H
#define WEB_SERVER_H

// Minimum bytes the old SensorPacket occupied (yaw+dist_cm+state+ts, packed
// loosely ~24)
#define SENSOR_PKT_MIN 20

#include "../packet.h"
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <math.h>
#include <stdio.h>

#define AP_SSID "SwarmCompanion"
#define AP_PASSWORD "swarm1234"
#define AP_CHANNEL 1

WebServer webServer(80);
WebSocketsServer wsServer(81);
DNSServer dnsServer;

// Declared in main sketch
extern void sendCommandTo(uint8_t idx, char cmd);
extern void broadcastCommand(char cmd);
extern void sendTaskTo(uint8_t idx, char cmd, float tx, float ty);

// ---- Forward sensor packet as JSON to all WS clients ----
void ws_broadcastSensor(const SensorPacket &pkt) {
  char buf[220];
  snprintf(buf, sizeof(buf),
           "{\"id\":%u,\"yaw\":%.2f,\"dist\":%.2f,\"x\":%.2f,\"y\":%.2f,"
           "\"lidar\":%u,\"state\":\"%s\",\"ts\":%lu}",
           pkt.slave_id, pkt.yaw, pkt.distance_cm, pkt.x, pkt.y, pkt.lidar_mm,
           pkt.state, pkt.timestamp);
  wsServer.broadcastTXT(buf);
  // Serial.println(buf); // echo for Serial Monitor debugging
}

// ---- Forward new map configuration (size, block) to all WS clients ----
void ws_broadcastArena(int size, int blockSize) {
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"type\":\"arena\",\"size\":%d,\"grid\":%d}", size, blockSize);
  wsServer.broadcastTXT(buf);
}

// ---- WebSocket event ----
void _wsEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Client %u connected\n", client);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Client %u disconnected\n", client);
    return;
  }
  if (type != WStype_TEXT)
    return;
  String msg = String((char *)payload);

  int cmdIdx = msg.indexOf("\"cmd\":\"");
  if (cmdIdx < 0)
    return;
  char cmd = msg.charAt(cmdIdx + 7);

  int tgtIdx = msg.indexOf("\"target\":\"");
  if (tgtIdx < 0) {
    broadcastCommand(cmd);
    return;
  }

  String tgt = "";
  int i = tgtIdx + 10;
  while (i < (int)msg.length() && msg.charAt(i) != '"')
    tgt += msg.charAt(i++);

  if (tgt == "all")
    broadcastCommand(cmd);
  else
    sendCommandTo((uint8_t)tgt.toInt(), cmd);
}

// ---- Serve any file from LittleFS ----
void _serveFile(const String &path) {
  if (!LittleFS.exists(path)) {
    webServer.send(404, "text/plain",
                   "File not found. Upload data/ via LittleFS.");
    return;
  }
  File f = LittleFS.open(path, "r");
  String ct = "text/plain";
  if (path.endsWith(".html"))
    ct = "text/html";
  else if (path.endsWith(".css"))
    ct = "text/css";
  else if (path.endsWith(".js"))
    ct = "application/javascript";
  webServer.streamFile(f, ct);
  f.close();
}

// ---- Init (call before esp_now_init) ----
bool webserver_init() {
  if (!LittleFS.begin(true)) {
    Serial.println("[WEB] LittleFS mount failed");
    return false;
  }

  WiFi.mode(WIFI_AP_STA); // AP for browser + STA for ESP-NOW receive
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL);
  Serial.print("[WEB] AP  MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("[WEB] STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("[WEB] AP  IP : ");
  Serial.println(WiFi.softAPIP());

  webServer.on("/", HTTP_GET, []() { _serveFile("/index.html"); });

  // /task – send autonomous navigation task to a slave
  // Body JSON: {"slave":0,"cmd":"g","tx":100.0,"ty":50.0}
  webServer.on("/task", HTTP_POST, []() {
    if (!webServer.hasArg("plain")) {
      webServer.send(400, "text/plain", "missing body");
      return;
    }
    String body = webServer.arg("plain");
    // simple JSON parse (no library needed for flat object)
    auto extract = [&](const char *key, String &out) {
      int i = body.indexOf(String('"') + key + '"');
      if (i < 0) return false;
      i = body.indexOf(':', i);
      if (i < 0) return false;
      i++;
      while (i < (int)body.length() && (body[i] == ' ' || body[i] == '"')) i++;
      int j = i;
      while (j < (int)body.length() && body[j] != ',' && body[j] != '}' && body[j] != '"') j++;
      out = body.substring(i, j);
      return true;
    };
    String sSlave, sCmd, sTx, sTy;
    extract("slave", sSlave);
    extract("cmd",   sCmd);
    extract("tx",    sTx);
    extract("ty",    sTy);
    uint8_t slaveIdx = (uint8_t)sSlave.toInt();
    char    cmd      = sCmd.length() > 0 ? sCmd[0] : 'g';
    float   tx       = sTx.toFloat();
    float   ty       = sTy.toFloat();
    sendTaskTo(slaveIdx, cmd, tx, ty);
    webServer.send(200, "application/json",
                   "{\"ok\":true,\"slave\":" + String(slaveIdx) +
                   ",\"cmd\":\"" + cmd + "\",\"tx\":" + String(tx) +
                   ",\"ty\":" + String(ty) + "}");
  });
  // /sim – sends one fake sensor packet to all WS clients (for UI testing)
  webServer.on("/sim", HTTP_GET, []() {
    static float t = 0;
    t += 0.1f;
    SensorPacket fake = {};
    fake.slave_id = 0;
    fake.yaw = fmod(t * 20.0f, 360.0f);
    fake.distance_cm = t * 5.0f;
    fake.x = 80.0f * cos(t * 0.3f);
    fake.y = 80.0f * sin(t * 0.3f);
    fake.lidar_mm = 1500;
    strncpy(fake.state, "FWD", 9);
    fake.timestamp = millis();
    ws_broadcastSensor(fake);
    webServer.send(200, "text/plain", "Simulated packet sent");
  });

  // Redirect every 404 / unknown URI to index so captive-portal works too
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "http://swarm.local/", true);
    _serveFile("/index.html");
  });
  webServer.begin();

  wsServer.begin();
  wsServer.onEvent(_wsEvent);

  // Captive-portal DNS: answer every query with our AP IP
  dnsServer.start(53, "*", WiFi.softAPIP());

  // mDNS: accessible as http://swarm.local
  if (MDNS.begin("swarm")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[mDNS] swarm.local ready");
  }

  Serial.println("[WEB] HTTP:80  WS:81  DNS:53  ready");
  Serial.println("[WEB] Open: http://swarm.local  or  http://192.168.4.1");
  return true;
}

// ---- Call every loop() ----
void webserver_loop() {
  dnsServer.processNextRequest();
  webServer.handleClient();
  wsServer.loop();
}

#endif
