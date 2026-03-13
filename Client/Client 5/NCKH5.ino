#include <EEPROM.h>
#include "motor.h"
#include "sensors.h"
#include "run.h"
#include "khaibao.h"
#include "RFID.h"
#include "espnow.h"
bool isStreamingLine = false;
unsigned long lastStreamTime = 0;
char control = 'S';


// Khai báo thực tế cho các biến extern
String cmdQueue = "";
String urgentBuffer = "";
volatile bool forceAbort = false;

int32_t savedPulse = 0;
int32_t originalPulse = 0;
 static inline void handleEspNow() {
  if (espNeedSend) {
    espNeedSend = false;
    addMac(espMasterMac);
    esp_now_send(espMasterMac, (uint8_t*)&espTx, sizeof(espTx));
  }

  if (!espHasCmd) return;
  espHasCmd = false;

  // Lệnh hệ thống ưu tiên
  if (strcmp(espRx.msg, "CALIB") == 0) { motoresStop(); calibrate(); return; }
  if (strcmp(espRx.msg, "LINEON") == 0) { isStreamingLine = true; return; }
  if (strcmp(espRx.msg, "LINEOFF") == 0) { isStreamingLine = false; return; }

  String incoming = String(espRx.msg);

  // --- XỬ LÝ LỆNH KHẨN CẤP (Bắt đầu bằng '!') ---
  if (incoming.charAt(0) == '!') {
    Serial.println(">>> URGENT SIGNAL RECEIVED");
    forceAbort = true;                    // 1. Báo hàm di chuyển dừng ngay
    urgentBuffer = incoming.substring(1); // 2. Lưu lệnh khẩn vào bộ đệm
  } 
  else {
    // Lệnh thường -> Xếp vào cuối hàng đợi
    cmdQueue += incoming;
  }
}
void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);
  Serial.begin(115200);
  // Load thông số cũ nếu có
  if (!EEPROM.begin(512)) {
    Serial.println("EEPROM Init Failed!"); 
    // Nếu lỗi EEPROM, có thể nháy đèn báo hiệu ở đây
  } else {
    Serial.println("EEPROM Init OK");
    loadCalibEEPROM(); // Load dữ liệu calib cũ ra dùng
  }
   SPI.begin();           // Bắt buộc phải có để giao tiếp với RFID 
  rfid.PCD_Init();       // Khởi tạo chip MFRC522 
  Serial.println("RFID Ready!");
  espNowInit();
  encoderInit();
  Serial.println("STEP MODE READY");
}
void loop() {
  handleEspNow(); 
  
  if (isStreamingLine && millis() - lastStreamTime > 100) {
     sendRawToMaster();
     lastStreamTime = millis();
  }

  if (cmdQueue.length() > 0 || urgentBuffer.length() > 0) {
    if (urgentBuffer.length() > 0) {
        cmdQueue = urgentBuffer + cmdQueue;
        urgentBuffer = "";
        forceAbort = false;
        Serial.println(">> Urgent command injected!");
    }

    if (cmdQueue.length() == 0) return;

    // 1. Lấy lệnh hiện tại
    char currentCmd = cmdQueue.charAt(0);
    cmdQueue.remove(0, 1);

    // 2. CHECK LỆNH TIẾP THEO (LOOK AHEAD)
    // Nếu lệnh tiếp theo giống lệnh hiện tại -> không dừng (stopEnd = false)
    char nextCmd = (cmdQueue.length() > 0) ? cmdQueue.charAt(0) : '\0'; 
    bool stopEnd = (nextCmd != currentCmd); // Nếu khác nhau thì mới dừng

    int32_t backupSaved = 0;
    int32_t backupOriginal = 0;
    if (savedPulse > 0) {
       backupSaved = savedPulse;
       backupOriginal = originalPulse;
       savedPulse = 0;
       originalPulse = 0;
    }

    forceAbort = false; 
    Serial.print("Run: "); Serial.println(currentCmd);
    
    // Debug chế độ chạy
    if (!stopEnd) Serial.println(">> Continuous Mode (No Stop)");

    bool success = true;
    switch (currentCmd) {
      case 'F': 
        // Truyền tham số stopEnd vào hàm go
        success = go(200, 4410, stopEnd);
        break;
      case 'L': 
        // Truyền tham số stopEnd vào hàm turn
        turnLeft(200, 1150, stopEnd);
        success = true;
        break;
      case 'R': 
        turnRight(200, 1150, stopEnd);
        success = true; 
        break;
      case 'B': 
        motoresStop(); delay(500); 
        success = true; 
        break;
    }

    if (success == false) {
      Serial.println(">> GO Interrupted!");
      cmdQueue = urgentBuffer + String(currentCmd) + cmdQueue;
      urgentBuffer = "";
      forceAbort = false;
    } 
    else {
      if (backupSaved > 0) {
        savedPulse = backupSaved;
        originalPulse = backupOriginal;
      }
      
      // Nếu hết sạch hàng đợi thì báo xong cả lô
      if (cmdQueue.length() == 0) {
        Serial.println(">> ALL DONE");
        espSendMirror("BATCH_DONE");
        motoresStop(); // Đảm bảo dừng hẳn khi hết lệnh
      }
      
      // BỎ đoạn motoresStop() và delay(100) ở đây
      // Việc dừng đã được xử lý bên trong hàm go/turn dựa vào biến stopEnd
    }
  }
}

