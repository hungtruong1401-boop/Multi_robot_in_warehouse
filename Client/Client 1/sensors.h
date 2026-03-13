#ifndef sensors
#define sensors
#include "espnow.h"
#include "khaibao.h"

struct CalibData {
  uint8_t  magic;
  uint16_t minVal[num];
  uint16_t maxVal[num];
  uint16_t threshold[num];
};

CalibData calib;
static inline void saveCalibEEPROM() {

  calib.magic = EEPROM_MAGIC;

  for (uint8_t i = 0; i < num; i++) {
    calib.minVal[i]    = ValueMin[i];
    calib.maxVal[i]    = ValueMax[i];
    calib.threshold[i] = threshold[i];
  }
  EEPROM.put(0, calib);
  EEPROM.commit();
  SerialBT.println("EEPROM SAVED");

}
static inline bool loadCalibEEPROM() {
  EEPROM.get(0, calib);
  if (calib.magic != EEPROM_MAGIC) {
    SerialBT.println("EEPROM INVALID");
    return false;
  }
  for (uint8_t i = 0; i < num; i++) {
    ValueMin[i]  = calib.minVal[i];
    ValueMax[i]  = calib.maxVal[i];
    threshold[i] = calib.threshold[i];
  }
  SerialBT.println("EEPROM LOADED");
  return true;
}

// ------------------- HÀM ĐỌC CẢM BIẾN -------------------
static inline void readSensor() {
  for (uint8_t i = 0; i < num; i++) {
    sensorValue[i] = analogRead(sensorPin[i]); // 0–4095 (12-bit)
  }
}
static inline void Test() {
  readSensor();   // đọc sensorValue[]

  for (uint8_t i = 0; i < num; i++) {
  
    SerialBT.print(sensorValue[i]);
    SerialBT.print("  ");
  }

  SerialBT.println();
}
static inline void calibrate() {
  espSendMirror("CALIB START(5s)...");
  for (uint8_t i = 0; i < num; i++) {
    ValueMin[i] = 4096;
    ValueMax[i] = 0;
  }
  unsigned long start = millis();
  while (millis() - start < 5000) {
    readSensor();
    for (uint8_t i = 0; i < num; i++) {
      uint16_t val = sensorValue[i];
      if (val < ValueMin[i]) ValueMin[i] = val;
      if (val > ValueMax[i]) ValueMax[i] = val;
    }
    delay(3);
  }
  char buf[64];
  for (uint8_t i = 0; i < num; i++) {
    threshold[i] =ValueMax[i] +300;
    snprintf(buf, sizeof(buf), "S%d:%d-%d|TH:%d", i, ValueMin[i], ValueMax[i], threshold[i]);
    espSendMirror(buf);
    delay(50); // Delay xíu để Master kịp nhận, không bị nghẽn
  }
  saveCalibEEPROM();
  // Debug
  espSendMirror("CALIB DONE");
}
static inline void sendLineToMaster() {
  readSensor();
  char binaryLine[6]; // Chuỗi "10101"
  
  for (uint8_t i = 0; i < num; i++) {
    // So sánh với ngưỡng đã calib
    if (sensorValue[i] > 100) 
      binaryLine[i] = '1';
    else 
      binaryLine[i] = '0';
  }
  binaryLine[num] = '\0'; // Kết thúc chuỗi

  // Gửi gói tin: "LINE,10101" (Gọn nhẹ để không làm tắc mạng ESP-NOW)
  char buf[32];
  snprintf(buf, sizeof(buf), "LINE,%s", binaryLine);
  espSendMirror(buf);
}
static inline void sendRawToMaster() {
  readSensor(); // Đọc giá trị mới nhất vào mảng sensorValue[]

  char buf[128]; // Buffer đủ lớn để chứa chuỗi số
  int offset = 0;

  // 1. Gắn nhãn gói tin là "RAW" để bên Master dễ phân biệt
  offset += snprintf(buf + offset, sizeof(buf) - offset, "RAW");

  // 2. Vòng lặp ghép các giá trị sensor vào chuỗi
  for (uint8_t i = 0; i < num; i++) {
    // Kết quả sẽ dạng: "RAW,1024,35,4095,200..."
    offset += snprintf(buf + offset, sizeof(buf) - offset, ",%d", sensorValue[i]);
  }

  // 3. Gửi sang Master
  espSendMirror(buf);
}
static inline void readLine() {
  sumSensor = 0;
  sumWeight = 0;

  readSensor();

  for (uint8_t i = 0; i < num; i++) {
    if (sensorValue[i] >  100)
      S[i] = 1;
    else
      S[i] = 0;

    sumSensor += S[i];
    sumWeight += S[i] * weightValue[i];

    Serial.print(S[i]);
    Serial.print(' ');
    SerialBT.print(S[i]);
    SerialBT.print(' ');
  }

  Serial.print("|||| Sensor: ");
  Serial.print(sumSensor);
  Serial.print(" Weight: ");
  Serial.println(sumWeight);

  SerialBT.print("|||| Sensor: ");
  SerialBT.print(sumSensor);
  SerialBT.print(" Weight: ");
  SerialBT.println(sumWeight);
}

// static inline void updateLine() {
//   sumSensor = 0;
//   sumWeight = 0;

//   readSensor();   // đọc sensorValue[]
//   for (uint8_t i = 0; i < num; i++) {
//     if (sensorValue[i] > 200)
//       S[i] = 1;
//     else
//       S[i] = 0;
//   sumSensor += S[i];
//   sumWeight += S[i] * weightValue[i];
//   }
// }
static inline void updateLine() {
  // 1. Reset biến tổng
  sumSensor = 0;
  sumWeight = 0;

  // 2. Đọc cảm biến
  readSensor();

  // 3. Chuyển sang nhị phân (S[i])
  for (uint8_t i = 0; i < num; i++) {
    // Giữ nguyên số 200 như bạn muốn
    if (sensorValue[i] > 200) S[i] = 1;
    else S[i] = 0;
  }

  // ==========================================================
  // MAGIC LOGIC: "BỊT MẮT" KHI GẶP NGÃ 3 ĐỂ ĐI THẲNG
  // ==========================================================
  
  // Chỉ lọc khi mắt giữa (S[2]) còn thấy đường (tức là đang đi thẳng qua ngã)
  if (S[2] == 1) {
    
    // --- TRƯỜNG HỢP 1: NGÃ 3 BÊN TRÁI (S0 đen, nhưng S4 trắng) ---
    // Robot sẽ bị lệch trái -> Cần lờ đi bên trái
    if (S[0] == 1 && S[4] == 0) {
      S[0] = 0; // Coi như mắt trái ngoài cùng không nhìn thấy gì
      S[1] = 0; // Xóa luôn mắt trái áp út cho chắc (để chỉ bám mắt giữa và phải)
    }

    // --- TRƯỜNG HỢP 2: NGÃ 3 BÊN PHẢI (S4 đen, nhưng S0 trắng) ---
    // Robot sẽ bị lệch phải -> Cần lờ đi bên phải
    else if (S[4] == 1 && S[0] == 0) {
      S[4] = 0; // Coi như mắt phải ngoài cùng không nhìn thấy gì
      S[3] = 0; // Xóa luôn mắt phải áp út
    }
    
    // Lưu ý: Nếu cả S[0] và S[4] đều đen (Ngã 4), ta không làm gì cả.
    // Vì -10 cộng với 10 tự bằng 0 -> Robot tự đi thẳng.
  }

  // 4. Tính toán tổng sau khi đã "xào nấu" S[]
  for (uint8_t i = 0; i < num; i++) {
    sumSensor += S[i];
    sumWeight += S[i] * weightValue[i];
  }
}
static inline void printMINMAXTH() {
  Serial.println("=== EEPROM MIN / MAX / TH ===");
  SerialBT.println("=== EEPROM MIN / MAX / TH ===");

  for (uint8_t i = 0; i < num; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print("  min="); Serial.print(ValueMin[i]);
    Serial.print("  max="); Serial.print(ValueMax[i]);
    Serial.print("  th=");  Serial.println(threshold[i]);

    SerialBT.print("S"); SerialBT.print(i);
    SerialBT.print("  min="); SerialBT.print(ValueMin[i]);
    SerialBT.print("  max="); SerialBT.print(ValueMax[i]);
    SerialBT.print("  th=");  SerialBT.println(threshold[i]);
  }

  Serial.println("============================");
  SerialBT.println("============================");
}




#endif