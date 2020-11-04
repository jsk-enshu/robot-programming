void setup() {
  pinMode(2, OUTPUT); // digital pinのモードを設定
}

void loop() {

  digitalWrite(2, HIGH); // digital pinをHIGH/LOWに切り替え
  delay(1000);           // sleep [msec]
  digitalWrite(2, LOW);
  delay(1000);

}
