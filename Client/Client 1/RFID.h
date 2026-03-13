#ifndef RFID
#define RFID
#include "khaibao.h"
#include "espnow.h"

static String lastUID = ""; 
static unsigned long lastReadTime = 0; // Thêm biến này để theo dõi thời gian

static inline void RF() {
  // 1. Nếu quét được thẻ
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    lastReadTime = millis(); // Cập nhật thời điểm vừa thấy thẻ

    String currentUID = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
      currentUID += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
      currentUID += String(rfid.uid.uidByte[i], HEX);
    }
    currentUID.toUpperCase();

    if (currentUID != lastUID) {
      lastUID = currentUID; 
      char buf[48];
      snprintf(buf, sizeof(buf), "HT,%s", currentUID.c_str());
      espSendMirror(buf);
    }
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  } 
  // 2. TỰ ĐỘNG RESET: Nếu quá 2 giây không thấy thẻ nào thì cho phép quét lại
  else if (millis() - lastReadTime > 2000) {
    lastUID = ""; 
  }
}
#endif