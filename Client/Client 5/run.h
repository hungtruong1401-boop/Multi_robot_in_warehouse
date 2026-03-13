#ifndef run
#define run
#include"khaibao.h"
#include "espnow.h"
#include "RFID.h"
uint8_t State() {
  if (sumSensor == 0) return MATLINE;
  if ((S[2]) && sumSensor >= 1 && sumSensor <= 3)
    return BAMLINE;
  if (sumSensor >= 5) return BAMLINE;
  return BAMLINE;
}

static inline void PID() {
  uint8_t state = State();
  
  // 1. Xử lý khi mất line hoàn toàn
  if (state == MATLINE) {
    motores(lastTurn < 0 ? -200 : 200, lastTurn < 0 ? 200 : -200);
    return;
  }

  lostTime = millis();
  int speed = baseSpeed;

    currPos = (float)sumWeight / sumSensor;

    // Chỉ cập nhật hướng rẽ khi thực sự mất line giữa (đến khúc cua gấp)
    if (state == LEFT) {
      lastTurn = -1;
    }
    if (state == RIGHT) {
      lastTurn = 1;
    }
  

  // -------- TÍNH TOÁN PID (Giữ nguyên) --------
  eCurr = cenPos - currPos;
  dE = eCurr - ePrev;
  eSum += eCurr;
  // eSum = constrain(eSum, -80, 80); // Bỏ comment nếu cần thiết
  
  u = Kp * eCurr + Kd * dE + Ki * eSum;
  
  int leftSpeed  = speed - u;
  int rightSpeed = speed + u;
  
  leftSpeed  = constrain(leftSpeed, -speedmax, speedmax);
  rightSpeed = constrain(rightSpeed, -speedmax, speedmax);
  
  motores(leftSpeed, rightSpeed);
  ePrev = eCurr;
}

bool go(int speed, int pulse, bool stopEnd) {
  int32_t targetPulse;
  int32_t totalScope;

  if (savedPulse > 0) {
    targetPulse = savedPulse;
    totalScope = originalPulse;
  } else {
    targetPulse = pulse;
    totalScope = pulse;
    originalPulse = pulse;
  }

  float stepPulse = totalScope / 6.0;
  int offset = totalScope - targetPulse; 
  int sentStep = 0;
  int lastSentStep = (int)(offset / stepPulse); 

  // BỎ motores(0, 0) và delay(10) ở đầu để xe không bị khựng nếu đang chạy F liên tiếp
  eSum = 0; ePrev = 0;
  lastUID = ""; // << THÊM: Reset vết thẻ mỗi khi bắt đầu đoạn đường mới
  resetEncoder(); // Reset ngay lập tức để đếm đoạn đường mới

  // PID Loop giữ nguyên
  while ((abs(countA) + abs(countB)) / 2 < targetPulse) {
    if (forceAbort) {
      motoresStop();
      int currentDist = (abs(countA) + abs(countB)) / 2;
      savedPulse = targetPulse - currentDist;
      if(savedPulse < 0) savedPulse = 0;
      return false;
    }

    updateLine();
    PID();

    // << THÊM: Quét RFID liên tục khi xe đang chạy
    RF();

    // Logic gửi STEP giữ nguyên
    float totalDistCovered = offset + (abs(countA) + abs(countB)) / 2;
    int curStep = (int)(totalDistCovered / stepPulse);
    if (curStep > 6) curStep = 6;

    if (curStep != lastSentStep && curStep > sentStep && curStep <= 6) {
      sentStep = curStep;
      lastSentStep = curStep;
      char buf[24];
      snprintf(buf, sizeof(buf), "STEP,%c,%d", heading, sentStep);
      espSendMirror(buf);
    }
  }

  // --- XỬ LÝ KẾT THÚC ---
  espSendMirror("DONE"); // Vẫn gửi DONE để báo hoàn thành 1 lệnh F
  
  if (stopEnd) {
    motoresStop(); // Chỉ phanh nếu lệnh tiếp theo KHÔNG phải là F
    delay(30);
  }
  
  resetEncoder();
  savedPulse = 0;
  originalPulse = 0;
  return true;
}

#endif