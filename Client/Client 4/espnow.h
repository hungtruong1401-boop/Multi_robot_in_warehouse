#ifndef ESPNOW_H
#define ESPNOW_H

#include <WiFi.h>
#include <esp_now.h>
#include "khaibao.h"
#include <string.h>
#include <stdio.h>

typedef struct {
  char msg[32];
} EspNowMsg;

static EspNowMsg espRx;// thu nhận
static EspNowMsg espTx;// thư gửi

static uint8_t espMasterMac[6] = {0};// MAC MASTER
static volatile bool espHasCmd   = false;//có tin
static volatile bool espNeedSend = false;// gửi tinuuuuuuuuuuu

static volatile bool espHasPose = false;
static int16_t startX = 0;
static int16_t startY = 0;

static inline bool checkmac(const uint8_t mac[6]) {
  for (int i = 0; i < 6; i++) if (mac[i] != 0) return false;
  return true;
}

static inline void addMac(const uint8_t *mac) {
  if (!mac || checkmac(mac)) return;
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }
}

static void EspNow(const esp_now_recv_info *info,
                         const uint8_t *data,
                         int len) {
  if (!info || !data || len <= 0) return;

  if (len > (int)sizeof(EspNowMsg)) len = sizeof(EspNowMsg);
  memcpy(&espRx, data, len);
  espRx.msg[sizeof(espRx.msg) - 1] = '\0';

  memcpy(espMasterMac, info->src_addr, 6);// Lưu MAC người gửi làm Maste
  espHasCmd = true;// Bật cờ báo có lệnh

  // -------- HANDSHAKE: chỉ cần "HELLO" là trả về (để nạp chung 5 con) --------
  if (strncmp(espRx.msg, "HELLO", 5) == 0) {
    // trả kèm MAC để PC biết ai là ai (gói <=32 bytes)
    // Chuẩn bị câu trả lời kèm MAC
    uint8_t m[6]; WiFi.macAddress(m);
    snprintf(espTx.msg, sizeof(espTx.msg),
             "HELLO_OK,%02X%02X%02X", m[3], m[4], m[5]);
    espNeedSend = true;// Dặn gửi
    return;
  }

  // -------- POSE: nhận cả POS,... và SETPOSE,... --------
  // POS,x,y,H
  // SETPOSE,x,y,H
  int x, y; char h;
  if (sscanf(espRx.msg, "POS,%d,%d,%c", &x, &y, &h) == 3 ||
      sscanf(espRx.msg, "SETPOSE,%d,%d,%c", &x, &y, &h) == 3) {

    h = (char)toupper((unsigned char)h);
    if (h == 'N' || h == 'E' || h == 'S' || h == 'W') {
      startX = (int16_t)x;
      startY = (int16_t)y;
      heading = h;         // ✅ set hướng ban đầu
      espHasPose = true;

      snprintf(espTx.msg, sizeof(espTx.msg), "POSACK,%d,%d,%c", x, y, h);
      espNeedSend = true;
    }
    return;
  }
}

static inline void espNowInit() {
  WiFi.mode(WIFI_STA);
  delay(80);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAIL");
    while (1) delay(100);
  }
  esp_now_register_recv_cb(EspNow);

  Serial.print("ESP32 MAC: ");
  Serial.println(WiFi.macAddress());
}

static inline void espSendRaw(const char *msg) {
  if (!msg) return;
  if (checkmac(espMasterMac)) return;

  strncpy(espTx.msg, msg, sizeof(espTx.msg));// 2. Copy tin nhắn vào hộp Tx
  espTx.msg[sizeof(espTx.msg) - 1] = '\0';

  addMac(espMasterMac);
  esp_now_send(espMasterMac, (uint8_t*)&espTx, sizeof(espTx));
}

static inline void espSendMirror(const char *msg) {
  espSendRaw(msg);
}

static inline bool espPoseReady() { return espHasPose; }
static inline int16_t espPoseX() { return startX; }
static inline int16_t espPoseY() { return startY; }

#endif // ESPNOW_H
