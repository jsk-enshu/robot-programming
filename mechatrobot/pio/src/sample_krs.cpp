#include "Arduino.h"
#include <IcsSoftSerialClass.h>

const byte S_RX_PIN = A6;
const byte S_TX_PIN = 2;
const byte EN_PIN = A7;
const long BAUDRATE = 115200;
const int TIMEOUT = 200;
IcsSoftSerialClass krs(S_RX_PIN, S_TX_PIN, EN_PIN, BAUDRATE, TIMEOUT);

void setup()
{
  krs.begin();
  krs.setPos(0, 7500); // 位置指令　ID:0サーボ中央:7500 指定範囲:3500～11500(10ステップ指定）
  delay(3000);
}
void loop()
{
  krs.setPos(0, 3500); // 位置指令　ID:0サーボ反時計回り最大:3500
  delay(2000); // 2秒待つ
  krs.setPos(0, 7500); // 位置指令　ID:0サーボ中央:7500
  delay(2000); // 2秒待つ
  krs.setPos(0, 11500); // 位置指令　ID:0サーボ時計回り最大:11500
  delay(2000); // 2秒待つ
  krs.setPos(0, 7500); // 位置指令　ID:0サーボ中央:7500
  delay(2000); // 2秒待つ
}
