# ArduinoのLED_BUILTINとは

## 概要

`LED_BUILTIN`は，Arduinoボード上に組み込まれているLED（基板上のLED）を制御するための定義済み定数である．この定数を使用することで，外部にLEDを接続しなくても，基板上のLEDでプログラムの動作確認ができる．

## LED_BUILTINのピン配置

`LED_BUILTIN`は，ボードの種類によって異なるデジタルピンに接続されている：

| ボード名 | LED_BUILTINのピン番号 | 備考 |
|:---------|:---------------------:|:-----|
| Arduino Nano | 13 | D13に接続 |
| Seeeduino Nano | 13 | D13に接続（Arduino Nano完全互換） |
| Arduino Uno | 13 | D13に接続 |
| Arduino Mega | 13 | D13に接続 |
| Arduino Leonardo | 13 | D13に接続 |

ほとんどのArduinoボードでは，`LED_BUILTIN`はデジタルピン13（D13）に接続されている．

## LED_BUILTINの定義場所

`LED_BUILTIN`は，Arduinoのボード定義ファイルで定義されている．具体的には，各ボードの`pins_arduino.h`ファイルで定義されている．

### Arduino公式のボード定義

Arduino Nano（ATmega328P）の場合，以下のGitHubリポジトリで定義されている：

**リポジトリ**: [arduino/ArduinoCore-avr](https://github.com/arduino/ArduinoCore-avr)

[ArduinoCore-avr/variants/standard/pins_arduino.h](https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h)

[LED_BUILTIN定義の行](https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h#L54)

**該当コード**:
```c
#define LED_BUILTIN 13
```

### Seeeduino AVRのボード定義

Seeeduino Nanoの場合，Seeed Studio社が提供するボード定義ファイルで定義されている：

**リポジトリ**: [Seeed-Studio/Seeed_Platform](https://github.com/Seeed-Studio/Seeed_Platform)

**ファイルパス**:
```
avr/variants/standard/pins_arduino.h
```

Seeeduino NanoはArduino Nanoと完全互換であるため，同様に`LED_BUILTIN`は13に定義されている．

## LED_BUILTINの使用例

### 基本的なLED点滅

```c
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // LEDを点灯
  delay(1000);                      // 1秒待機
  digitalWrite(LED_BUILTIN, LOW);   // LEDを消灯
  delay(1000);                      // 1秒待機
}
```

### 数値の代わりにLED_BUILTINを使う理由

以下の2つのコードは，Arduino Nanoでは同じ動作をするが，`LED_BUILTIN`を使う方が推奨される：

**非推奨（ハードコード）:**
```c
pinMode(13, OUTPUT);
digitalWrite(13, HIGH);
```

**推奨（LED_BUILTIN使用）:**
```c
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, HIGH);
```

**理由:**
1. **可搬性**: ボードを変更しても，コードを書き換える必要がない
2. **可読性**: コードの意図が明確になる
3. **保守性**: ピン番号が変更されても，ボード定義ファイルの更新だけで対応できる

## LED_BUILTINの利点

1. **外部配線不要**
   - 外部にLEDや抵抗を接続する必要がない
   - ブレッドボードなしでプログラムの動作確認ができる

2. **デバッグに便利**
   - プログラムの動作状態を視覚的に確認できる
   - センサ値の閾値判定結果を簡単に表示できる

3. **初学者に優しい**
   - 配線ミスのリスクがない
   - すぐにプログラミングの学習を始められる

4. **段階的な開発を可能にする**
   - `LED_BUILTIN`を使うことでまずプログラムを書き込んでちゃんと動作するかというデバッグの一歩が達成できる
   - プログラムが正しく動作することを確認できればここから少しずつ差分（diff）を追加していけば自分の目標が達成できるだろう
   - これはソフトウェア開発の基本的なアプローチである：
     1. まず最小限の動作するプログラムを作成する
     2. 動作を確認する
     3. 少しずつ機能を追加する
     4. 各段階で動作を確認する
   - このアプローチにより問題が発生した際にどの変更が原因かを特定しやすくなる

## LED_BUILTINを使ったデバッグ例

### センサ値の閾値判定

```c
#define SENSOR_PIN A0
#define THRESHOLD 512

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  int sensorValue = analogRead(SENSOR_PIN);

  if (sensorValue > THRESHOLD) {
    digitalWrite(LED_BUILTIN, HIGH);  // 閾値を超えたら点灯
  } else {
    digitalWrite(LED_BUILTIN, LOW);   // 閾値以下なら消灯
  }

  delay(100);
}
```

### 通信状態の表示

```c
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    // データを受信したらLEDを点滅
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);

    char data = Serial.read();
    // データ処理．..
  }
}
```

## 注意事項

1. **D13ピンとの競合**
   - `LED_BUILTIN`はD13に接続されているため，D13を他の目的で使用する場合は注意が必要
   - D13に外部回路を接続すると，基板上のLEDも影響を受ける

2. **電流制限**
   - 基板上のLEDには既に適切な抵抗が実装されているため，追加の抵抗は不要
   - 外部LEDを接続する場合は，必ず電流制限抵抗を接続すること

3. **ボードによる違い**
   - 一部のボードでは，`LED_BUILTIN`が異なるピンに接続されている場合がある
   - 使用するボードのドキュメントを確認すること

## 参考資料

- [Arduino公式リファレンス - LED_BUILTIN](https://www.arduino.cc/reference/en/language/variables/constants/ledbuiltin/)
- [Arduino Core AVR - pins_arduino.h](https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h)
- [Seeed Studio - Seeeduino Nano](https://wiki.seeedstudio.com/Seeeduino-Nano/)
