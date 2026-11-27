#include "DisplayPower.h"
#include <Arduino.h>

void DisplayPower_begin() { 
  // Khởi tạo các chân LED RGB onboard của XIAO nRF52840
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // Ép tất cả về HIGH để TẮT đèn (Vì LED onboard là Active LOW)
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void DisplayPower_showOn() {
  // Bật màu Xanh lá
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW); 
  digitalWrite(LED_BLUE, HIGH);
}

void DisplayPower_showRed() {
  // Bật màu Đỏ
  digitalWrite(LED_RED, LOW);   
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void DisplayPower_showOff() { 
  // Tắt toàn bộ
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}