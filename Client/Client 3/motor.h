#ifndef motor
#define motor
#include "khaibao.h"
#include "espnow.h"
static inline void motores(int pwmA, int pwmB) {
  pwmA = constrain(pwmA, -255, 255);
  pwmB = constrain(pwmB, -255, 255);
  // Motor trái
  if (pwmA > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (pwmA < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  // Motor phải
  if (pwmB > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (pwmB < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMA, abs(pwmA)*0.45);
  analogWrite(PWMB, abs(pwmB)*0.45);
}
/* =====================================================
   ENCODER – X1 (2 INTERRUPT, ỔN ĐỊNH)
   ===================================================== */

void motoresStop(){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);
}

volatile int32_t countA = 0;   // bánh phải
volatile int32_t countB = 0;   // bánh trái

void IRAM_ATTR readEncA() {
  if (digitalRead(ENCA_B)) countA++;
  else countA--;
}

void IRAM_ATTR readEncB() {
  if (digitalRead(ENCB_B)) countB++;
  else countB--;
}

static inline void encoderInit() {
  pinMode(ENCA_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCB_A, INPUT_PULLUP);
  pinMode(ENCB_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), readEncA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), readEncB, RISING);

  countA = 0;
  countB = 0;
}
void updateHeading(char cmd) {
  if (cmd == 'L')
    heading = (heading=='N')?'W':(heading=='W')?'S':(heading=='S')?'E':'N';
  else if (cmd == 'R')
    heading = (heading=='N')?'E':(heading=='E')?'S':(heading=='S')?'W':'N';
}
static inline void resetEncoder() {
  noInterrupts();
  countA = 0;
  countB = 0;
  interrupts();
}
// Hàm hỗ trợ gửi tin ngay trong lúc quay
void checkAndReportTurn(int32_t currentCount, int counts_90, int &sentCount, int multiplier, char dir) {
  // Tính xem đã quay được bao nhiêu lần 90 độ
  int progress = currentCount / counts_90;
  
  // Nếu vượt qua mốc mới và chưa báo cáo mốc đó (trừ mốc cuối cùng để dành báo sau)
  if (progress > sentCount && sentCount < multiplier - 1) {
    sentCount++;
    if (dir == 'L') updateHeading('L');
    else updateHeading('R');
    
    char buf[16];
    snprintf(buf, sizeof(buf), "TURN,%c", dir);
    espSendMirror(buf);
  }
}

void turnLeft(int speed, int counts_90, bool stopEnd) {
  // BỎ dòng motores(0, 0); và delay ở đầu để nối lệnh mượt hơn
  // Chỉ reset encoder để đếm góc mới
  resetEncoder(); 
  
  // quay nhanh
  motores(-speed, speed);
  while (abs(countB) < counts_90 / 2) {}

  motores(-95, 95); // Giảm tốc
  while (abs(countB) < counts_90) {}

  updateHeading('L');
  espSendMirror("TURN,L"); // Vẫn gửi báo cáo như cũ

  if (stopEnd) {
    motoresStop();
    delay(30); // Chỉ delay nếu đây là lệnh cuối cùng của chuỗi rẽ
  }
  
  resetEncoder(); // Reset chuẩn bị cho lệnh sau
}

// --- SỬA HÀM TURN RIGHT ---
void turnRight(int speed, int counts_90, bool stopEnd) {
  // BỎ dòng motores(0, 0); ở đầu
  resetEncoder();

  motores(speed, -speed);
  while (abs(countA) < counts_90 / 2) {}

  motores(95, -95);
  while (abs(countA) < counts_90) {}

  updateHeading('R');
  espSendMirror("TURN,R");

  if (stopEnd) {
    motoresStop();
    delay(30);
  }
  resetEncoder();
}

#endif