#include <Arduino.h>

// 初めに一回実行される setup 関数
void setup() {
  pinMode(2, OUTPUT); // D2 ピンを output に設定
}
// 毎周期実行される loop 関数
void loop() {
  digitalWrite(2, HIGH); // D2 ピンを HIGH に切り替え
  delay(1000);
  // 1000ms sleep
  digitalWrite(2, LOW); // D2 ピンを LOW に切り替え
  delay(1000);
}
