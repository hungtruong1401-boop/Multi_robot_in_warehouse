#include <WiFi.h>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>

/* ====================================================
   1. CẤU HÌNH & DANH BẠ (ID <-> MAC)
   ==================================================== */
#define MAX_ROBOTS 5

static const uint32_t STEP_FLUSH_MS = 25;       
static const int      STEP_MAX_BEFORE_FLUSH = 6; 

typedef struct {
  uint8_t id;      
  uint8_t mac[6];  
} RobotPeer;

// --- ĐIỀN MAC CỦA BẠN VÀO ĐÂY ---
RobotPeer peers[] = {
  {1, {0x84, 0x1F, 0xE8, 0x69, 0xEF, 0x6C}}, // ESP1
  {2, {0xB0, 0xCB, 0xD8, 0xE9, 0x5A, 0x58}}, // ESP2
  {3, {0x28, 0x05, 0xA5, 0x2F, 0x24, 0x44}}, // ESP3
  {4, {0xD4, 0xE9, 0xF4, 0xA3, 0xCC, 0xC8}}, // ESP4
  {5, {0x70, 0x4B, 0xCA, 0x27, 0x3D, 0xEC}}, // ESP5
};

static const int PEER_N = sizeof(peers) / sizeof(peers[0]);

/* ====================================================
   2. BIẾN TOÀN CỤC & QUEUE (ĐÃ NÂNG CẤP BUFFER)
   ==================================================== */
// [SỬA 1] Tăng kích thước tin nhắn lên 100 bytes để chứa chuỗi lệnh dài
#define MSG_SIZE 100 

// Gói tin gửi đi (TX)
typedef struct { char msg[MSG_SIZE]; } Message;
Message txMsg;

// Gói tin nhận về (RX)
typedef struct {
  uint8_t id;      
  char msg[MSG_SIZE];    
} RxItem;

QueueHandle_t rxQueue = NULL;

// Quản lý gộp bước
typedef struct {
  int  pend;    
  char head;    
  int  lastIdx; 
} StepAgg;

static StepAgg agg[MAX_ROBOTS + 1]; 

/* ====================================================
   3. CÁC HÀM HELPER
   ==================================================== */

static int get_id_from_mac(const uint8_t *mac) {
  for (int i = 0; i < PEER_N; i++) {
    if (memcmp(mac, peers[i].mac, 6) == 0) return peers[i].id;
  }
  return 0; 
}

static const uint8_t* get_mac_from_id(uint8_t id) {
  for (int i = 0; i < PEER_N; i++) {
    if (peers[i].id == id) return peers[i].mac;
  }
  return NULL;
}

static inline void print_frame(uint8_t id, const char* payload) {
  char out[128]; 
  // Master chỉ việc chuyển tiếp, PC sẽ xử lý BATCH_DONE
  snprintf(out, sizeof(out), "@%u,%s\n", (unsigned)id, payload);
  Serial.write(out);
}

static void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

/* ====================================================
   4. CALLBACK NHẬN TIN
   ==================================================== */
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len <= 0) return;
  int senderId = get_id_from_mac(info->src_addr);
  if (senderId == 0) return; 

  RxItem item;
  item.id = senderId;
  
  // [SỬA 2] Bảo vệ tràn bộ nhớ đệm mới
  int n = (len > (MSG_SIZE - 1)) ? (MSG_SIZE - 1) : len;
  memcpy(item.msg, data, n);
  item.msg[n] = '\0';

  if (rxQueue) xQueueSend(rxQueue, &item, 0);
}

/* ====================================================
   5. XỬ LÝ GỘP BƯỚC
   ==================================================== */
static void flushSteps(uint8_t id) {
  if (id < 1 || id > MAX_ROBOTS) return;
  if (agg[id].pend <= 0) return;

  char payload[48];
  snprintf(payload, sizeof(payload), "STEP,%c,%d", agg[id].head, agg[id].pend);
  print_frame(id, payload); 

  agg[id].pend = 0;
}

static uint32_t lastFlushMs = 0;
static void flushStepsIfDue() {
  uint32_t now = millis();
  if (now - lastFlushMs >= STEP_FLUSH_MS) {
    for (uint8_t id = 1; id <= MAX_ROBOTS; id++) {
      flushSteps(id);
    }
    lastFlushMs = now;
  }
}

static void resetIdx(uint8_t id) {
  if (id >= 1 && id <= MAX_ROBOTS) agg[id].lastIdx = 0;
}

static bool parseStepMsg(uint8_t id, const char* s, char &h, int &delta) {
  if (strncmp(s, "STEP,", 5) != 0) return false;
  
  // STEP,H (len=6) hoặc STEP,H,idx
  if (strlen(s) < 6) return false;

  h = s[5]; 
  delta = 1;

  const char* p = strchr(s + 5, ',');
  if (!p) return true; 

  int k = atoi(p + 1);
  if (k <= 0) { delta = 1; return true; }

  // Logic chống trùng lặp (giữ nguyên)
  if (k <= 6) {
    int last = agg[id].lastIdx;
    if (k < last) last = 0; 
    delta = k - last;
    agg[id].lastIdx = k;
    return true;
  }
  delta = k; 
  return true;
}

// [SỬA 3] Cập nhật hàm này để nhận biết BATCH_DONE
static bool isCriticalStatus(const char* s) {
  if (strncmp(s, "BATCH_DONE", 10) == 0) return true; // Quan trọng nhất
  if (strncmp(s, "DONE", 4) == 0) return true;
  if (strncmp(s, "TURN", 4) == 0) return true;
  return false;
}

/* ====================================================
   6. GỬI LỆNH
   ==================================================== */
static void send_to_robot(int id, const char* payload) {
  const uint8_t* targetMac = get_mac_from_id(id);
  if (targetMac == NULL) {
    print_frame(0, "ERR:ID_NOT_FOUND");
    return;
  }
  
  // Buffer txMsg giờ đã lớn (100 byte), thoải mái gửi chuỗi dài
  strncpy(txMsg.msg, payload, sizeof(txMsg.msg));
  txMsg.msg[sizeof(txMsg.msg) - 1] = '\0';

  esp_now_send(targetMac, (uint8_t*)&txMsg, sizeof(txMsg));
  
  // Log ngắn gọn để PC biết đã gửi
  // char log[32]; snprintf(log, sizeof(log), "TX_OK_ESP%d", id);
  // print_frame(0, log);
}

/* ====================================================
   7. SETUP & LOOP
   ==================================================== */
void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  Serial.setTimeout(10);

  WiFi.mode(WIFI_STA);

  // Queue tạo với kích thước RxItem mới
  rxQueue = xQueueCreate(20, sizeof(RxItem));

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAIL");
    while (1) delay(100);
  }
  esp_now_register_recv_cb(onReceive);

  for (int i = 0; i < PEER_N; i++) {
    addPeer(peers[i].mac);
  }
  Serial.println("=== MASTER V2 (BATCH SUPPORT) ===");
}

void loop() {
  // --- 1. NHẬN TIN TỪ ROBOT ---
  if (rxQueue) {
    RxItem item;
    while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
      flushStepsIfDue();

      char h; int delta;
      
      // Nếu là STEP -> Gom
      if (parseStepMsg(item.id, item.msg, h, delta)) {
        if (delta <= 0) continue; 
        if (agg[item.id].pend == 0) agg[item.id].head = h;
        
        // Đổi hướng -> In hướng cũ ngay
        if (agg[item.id].head != h) {
          flushSteps(item.id);
          agg[item.id].head = h;
        }
        agg[item.id].pend += delta;
        if (agg[item.id].pend >= STEP_MAX_BEFORE_FLUSH) {
          flushSteps(item.id);
        }
        continue;
      }

      // Nếu là tin khác (BATCH_DONE, TURN, DONE...) -> In ngay
      flushSteps(item.id); // Xả các bước tồn đọng trước
      print_frame(item.id, item.msg);

      // [SỬA 4] Reset bộ đếm nếu gặp tin quan trọng
      if (isCriticalStatus(item.msg)) {
        resetIdx(item.id);
      }
    }
  }
  flushStepsIfDue();

  // --- 2. GỬI LỆNH TỪ PC XUỐNG ROBOT ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); // VD: "ESP3 FFLFR"
    cmd.trim();
    if (cmd.length() == 0) return;

    int p = cmd.indexOf(' ');
    if (p < 0) return;

    String targetStr = cmd.substring(0, p); 
    String payload   = cmd.substring(p + 1);
    targetStr.trim();
    payload.trim();

    if (targetStr.startsWith("ESP")) { 
      int id = targetStr.substring(3).toInt(); 
      if (id > 0 && id <= MAX_ROBOTS) {
        // payload lúc này chứa cả chuỗi lệnh dài (VD: "FFLFR")
        // Hàm send_to_robot đã hỗ trợ buffer lớn
        send_to_robot(id, payload.c_str());
      }
    }
  }
}