# 機械系二学科　夏学期演習 マイコンプログラミング演習

```{figure} fig/micon-programming/arduino-uno-r4-wifi.jpg
---
width: 500px
name: fig:arduino-r4-wifi
---
Arduino UNO R4 WiFi
```

## 1. 本演習について

### 1.1 本演習の進め方

1. 資料を UTOL または Slack からダウンロードする
2. 資料に従って課題を進める．班ごとに分かれ，理解を深めるために積極的に議論すること．課題は個人で取り組むが，ソースコードの直接のやりとり以外，例えばソースコードを画面で表示する，エラーメッセージを表示する，情報源の URL をチャットで共有する等，積極的な情報交換を行うとよい．質問がある場合はTAを呼ぶこと
3. 演習課題1, 2, 3, 4が完了したらそれぞれTAが巡回してチェックするので，作成したプログラムを実行している様子を見せること
4. 発展課題5以降（モータ制御，シリアル通信，Wi-Fi）が出来た者はTAを呼んでチェックを受けること
5. 取り組んだ課題の画面キャプチャ，実機の写真，ソースコード，並びに考察を記したpdfファイルをUTOLに提出すること

### 1.2 講義や他の演習と本演習の関連

既にソフトウェア第1や第2の講義で基本的なプログラミングについて学んでいるが，ソフトウェアの授業で主に扱っているのは計算機（所謂コンピュータ）で動作するソフトウェアである．それに対して本演習で取り組むのは**マイコンで動作するソフトウェア**であり，そのようなソフトウェアを**組み込みソフトウェア**と呼ぶ．

マイコンは計算機と比較して計算資源が少ないため，組み込みプログラミングを行う際にはこれまで計算機プログラミングでは意識してこなかったようなハードウェアの振る舞いや制約を考慮する必要がある．しかし，組み込みプログラミングによって様々なモータやセンサなどを扱えるようになり実際のものを動かせるようになる．

実際，組み込みソフトウェアは家電などの電気製品だけでなく自動車などの機械製品にも不可欠な要素であり，モータの制御やセンサとの通信などの重要な役割を担っている．

先日の電子回路演習においてステッピングモータやDCモータの原理や駆動方法も既に学んできているが，本演習においてマイコンからそれらのモータを制御することで，電子回路演習で取り組んだアナログ・デジタル回路よりも汎用性と拡張性の高いシステムを設計することができるようになる．

## 2. マイコン

### 2.1 マイコンボードArduino UNO R4 WiFi

本演習で用いる**Arduino UNO R4 WiFi**は，32ビットマイコンとESP32-S3 Wi-Fiモジュール（ESP32-S3-MINI-1-N8）を搭載した最新のUNOボードである．Renesas社のRA4M1シリーズマイコン（R7FA4M1AB3CFM#AA0）を搭載し，48MHz Arm Cortex-M4マイクロプロセッサをベースとしている．

```{figure} fig/micon-programming/arduino-uno-r4-wifi-block-diagram.jpg
---
width: 600px
name: fig:arduino-r4-wifi-block
---
Arduino UNO R4 WiFi ブロック図
```

#### Arduino UNO R4 WiFiの主な特徴

| 項目 | 仕様 |
|------|------|
| マイコン | Renesas RA4M1 (R7FA4M1AB3CFM#AA0) |
| プロセッサ | 48MHz Arm Cortex-M4（浮動小数点演算ユニット付き） |
| 動作電圧 | 5V |
| フラッシュメモリ | 256kB |
| SRAM | 32kB |
| EEPROM | 8kB |
| デジタルI/Oピン | 14本 |
| アナログ入力 | 6チャンネル（14ビットADC） |
| DAC | 最大12ビット（A0ピン） |
| PWM出力 | D3, D5, D6, D9, D10, D11 |
| 通信 | UART×1, SPI×1, I2C×2, CAN×1 |
| Wi-Fi | ESP32-S3モジュール（802.11 b/g/n） |
| Bluetooth | Bluetooth 5 LE |
| LEDマトリクス | 12×8 赤色LED |
| USB | USB-C |
| 推奨入力電圧 | 6-24V（VINピン） |

```{figure} fig/micon-programming/arduino-uno-r4-wifi-pinout.jpg
---
width: 600px
name: fig:arduino-r4-wifi-pinout
---
Arduino UNO R4 WiFi ピン配置図
```

Arduinoにはマイコン初心者にも容易に開発ができるよう各ボード専用のソフトウェア開発環境とそれに付属するライブラリなどが用意されている．そのため本来ならばタイマーやA/D変換などの機能を用いるために行わなければならないレジスタの操作の大部分を隠蔽した容易なプログラミングが可能となる．

```{note}
**Arduino UNO R4 WiFiの特徴的な機能**

- **12×8 LEDマトリクス**: アニメーション，テキストスクロール，ミニゲームなど様々な表示が可能
- **Wi-Fi/Bluetooth接続**: ESP32-S3モジュールによるIoTアプリケーション開発が可能
- **DAC出力**: A0ピンで最大12ビットのデジタル-アナログ変換が可能
- **Qwiic I2Cコネクタ**: I2Cデバイスの簡単な接続が可能
```

### 2.2 組み込みプログラミングと計算機プログラミングの主な違い

#### 2.2.1 クロス開発とその開発環境

**クロス開発**とはアプリケーションの実行環境とは異なる環境でシステムを開発することである．マイコンにおける組み込みソフトウェアの開発もクロス開発に該当する．

コンピュータ上でコンパイルしたマイコン用の実行ファイルをマイコンのプログラムメモリに転送して書き込む処理は煩雑になりがちだが，Arduinoは初心者向けの開発環境が用意されているので，マイコンとコンピュータをケーブルで接続しボードを選択してボタンをクリックするだけでクロス開発の難しさを意識することなく組み込みプログラミングを行うことができる．

Arduinoでクロス開発を行う際には**Arduino IDE**というマイコンボード用の初心者向けプログラム開発環境ソフトウェアを用いることができる．一般に計算機プログラミングでC言語のプログラム開発を行う際は`.c`ファイルにプログラムを書くが，Arduino IDEで開発を行う際には**Sketch**の`.ino`ファイルにプログラムを書く．

プログラムの文法は実質的にC言語と同様であるが，main関数の代わりに**setup関数**と**loop関数**を書くという点が大きく異なる：

- **setup関数**: Arduinoが起動したときに最初の1回だけ実行される関数で，各ピンの状態を設定したりシリアル通信を開始したりする
- **loop関数**: setup関数が実行された後に電源が切れるまで繰り返し実行される関数

```cpp
void setup() {
    // 最初に一回だけ実行される
}

void loop() {
    // 毎周期実行される
}
```

```{tip}
Arduinoの`setup()`と`loop()`の仕組みは，実際には内部の`main()`関数から呼び出されている．
Arduino UNO R4 WiFi（Renesas RA4M1）の場合，その実装は[main.cpp](https://github.com/arduino/ArduinoCore-renesas/blob/main/cores/arduino/main.cpp)で確認できる．
```

#### 2.2.2 動作確認・デバッグ方法

マイコンや電子回路におけるデバッグ方法には以下のようなものが挙げられる：

**A) シリアル通信を介したプリントデバッグ**

計算機プログラミングにおいてコード内にprintf()関数を挿入することで処理の実行状況や変数の値を確認する方法に相当する．

```cpp
void setup() {
    Serial.begin(9600);  // シリアル通信の初期化
}

void loop() {
    Serial.println("This is Serial Print");
    delay(1000);  // 1000ms待機
}
```

`Serial`クラスの実装は[Serial.h](https://github.com/arduino/ArduinoCore-renesas/blob/main/cores/arduino/Serial.h)で定義されている．

**B) LEDにより状態を表示する方法**

マイコンの出力とはoutputに設定したピンからHIGHとLOWの値を信号として書き出すことである．そのHIGH/LOWに合わせてLEDを点滅させることでマイコンのピンの出力状態を確認する．`pinMode`や`digitalWrite`などのピン操作関数は[digital.cpp](https://github.com/arduino/ArduinoCore-renesas/blob/main/cores/arduino/digital.cpp)で実装されている．

外部LEDを使用する例:
```cpp
void setup() {
    pinMode(2, OUTPUT);  // D2ピンをoutputに設定
}

void loop() {
    digitalWrite(2, HIGH);  // D2ピンをHIGHに出力
    delay(1000);
}
```

内蔵LED（`LED_BUILTIN`）を使用する例:
```cpp
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);  // 内蔵LEDをoutputに設定
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);  // 内蔵LEDを点灯
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);   // 内蔵LEDを消灯
    delay(1000);
}
```

```{tip}
**LED_BUILTINを使ったデバッグのすすめ**

`LED_BUILTIN`は，Arduinoボード上に内蔵されているLEDを制御するための定義済み定数です（[pins_arduino.h](https://github.com/arduino/ArduinoCore-renesas/blob/main/variants/UNOWIFIR4/pins_arduino.h)で定義）．
外部にLEDを接続しなくても，基板上のLEDでプログラムの動作確認ができます．

- **配線不要**: 外部LEDや抵抗を接続する必要がない
- **デバッグに便利**: プログラムの動作状態を視覚的に確認できる
- **可搬性**: ボードを変更してもコードを書き換える必要がない

詳しくは{doc}`tips/arduino-builtin-led`を参照．
```

```{note}
**Arduino UNO R4 WiFiの内蔵LEDマトリクス**

Arduino UNO R4 WiFiには`LED_BUILTIN`（D13）に加えて，12×8のLEDマトリクスも内蔵されています．
より高度な表示が必要な場合は，[Arduino_LED_Matrixライブラリ](https://github.com/arduino/ArduinoCore-renesas/blob/main/libraries/Arduino_LED_Matrix/src/Arduino_LED_Matrix.h)を使用してLEDマトリクスを制御することもできます．
```

**C) 外部計測機器を用いる方法**

電圧計や電流計，オシロスコープによる外部計測機器を用いて，LEDの点灯だけでは確認できないより詳細な情報を確認する方法である．

#### 2.2.3 割り込み処理

**割り込み**とはマイコンがメインルーティンの一連の処理を行っている最中に，緊急な処理を要求された場合に緊急な処理を先に実行し，緊急な処理の終了後に作業途中の処理を再開することである．

多くの場合，マイコンはCPUコアを一つしか持たないため，様々な処理を並行して行うためには割り込みという概念が必要になる．割り込みを使わずに緊急な処理が必要かどうかを常に確認する方式も考えられ，その監視処理を**ポーリング**と呼ぶが，ポーリングは煩雑かつ非効率であるため，計算資源が潤沢ではないマイコンには不向きである．以下のような場合に割り込み処理が有効・必須となる：

- 一定時間毎に処理を行いたい
- 外部からの入力を受け取るタイミングが分からない

```{note}
従来のArduino UNO R3ではD2とD3のみが外部割り込みに対応していたが，Arduino UNO R4 WiFi（Renesas RA4M1, ARM Cortex-M4）ではより多くのピンで`attachInterrupt()`が使用できる．ただし，すべてのピンが対応しているわけではなく，またIRQチャンネルを共有するピン同士は同時に使用できない制約がある．詳細は[Arduino公式ドキュメントのattachInterrupt()](https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/)を参照すること．
```

```{warning}
割り込み処理の中でグローバル変数を使う場合は`volatile`修飾子をつけて宣言する必要がある．`volatile`をつけないと，コンパイラの最適化により変数の変更が反映されない場合がある．
例: `volatile int count = 0;`
```

**外部割り込みの例: ボタンでLEDをトグル**

以下は，D2ピンに接続したボタンを押すたびに`LED_BUILTIN`の点灯/消灯を切り替える例である．`attachInterrupt`により，ボタンが押された瞬間（FALLINGエッジ）に割り込み関数`toggleLED`が呼ばれる．`loop()`内でポーリングする必要がなく，他の処理を行いながらボタン入力を検出できる．

```cpp
#define BUTTON_PIN 2

volatile bool led_state = false;

void toggleLED() {
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // 内蔵プルアップ抵抗を使用
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggleLED, FALLING);
}

void loop() {
    // loop内では他の処理を自由に行える
    // ボタン押下は割り込みで処理される
}
```

```{tip}
`INPUT_PULLUP`を使うと外部にプルアップ抵抗を接続する必要がない．ボタンの片方をD2ピンに，もう片方をGNDに接続するだけで動作する．
ボタンを押すとD2ピンがHIGHからLOWに変化し（FALLING），割り込みが発生する．
```

### 2.3 マイコンによるモータ制御

**DCモータ**は一般にPWM制御によってモータに送る電力を制御する．**PWM (Pulse Width Modulation)** とは日本語ではパルス幅変調という．電圧のパルスの大きさと周期は一定で，オンのときのパルス幅（時間）を調整することで平均電圧を制御する．周期に対するオンの時間の割合を**デューティー比**と呼ぶ．`analogWrite`によるPWM出力の実装は[analog.cpp](https://github.com/arduino/ArduinoCore-renesas/blob/main/cores/arduino/analog.cpp)で確認できる．

```{figure} fig/micon-programming/pwm-duty-cycle.jpg
---
width: 400px
name: fig:pwm
---
PWM信号の概念図
```

**サーボモータ**もPWM信号により制御することができるが，PWM信号によりモータの角度を制御するという点でPWMの大きさにより電力を制御するDCモータとは異なる．

## 3. 演習課題1: Arduinoを用いたLEDの点灯

Arduinoには，デジタル/アナログのIOが複数ポート用意されており，各種モータやセンサなどを接続し，入出力を制御することができる．また，ライブラリも充実し世界中にユーザも多いため，メカトロ設計制作やIoTデバイスのプラットフォームとしてよく用いられている．

### LED_BUILTINを使った動作確認

外部にLEDを接続する前に，まずArduinoボード上に内蔵されているLED（`LED_BUILTIN`）を使ってプログラムの動作確認を行うことを推奨する．これにより，配線ミスのリスクなしにプログラムが正しく書き込めているかを確認できる．

```{tip}
**段階的な開発アプローチ**

1. まず`LED_BUILTIN`を使って最小限の動作するプログラムを作成する
2. 動作を確認する
3. 少しずつ機能を追加する（外部LED，センサなど）
4. 各段階で動作を確認する

このアプローチにより，問題が発生した際にどの変更が原因かを特定しやすくなる．
詳しくは{doc}`tips/arduino-builtin-led`を参照．
```

**Arduino IDEの起動と使い方:**

```{tip}
初回利用時にはArduino IDEのインストール，および「ボードマネージャでArduino UNO R4のサポートをインストール」する必要がある．本ページ末尾の{ref}`appendix-arduino-ide`，特に{ref}`appendix-board-manager-uno-r4`の節，および{doc}`tips/arduino-ide`，{doc}`tips/arduino-upload`を参照すること．
```

Windowsキーを押して「arduino」と入力し，Arduino IDEを起動する．起動すると以下のような画面が表示される．

```{figure} fig/micon-programming/arduino-ide-led-builtin-upload.png
---
width: 80%
---
Arduino IDEの画面．`setup()`関数と`loop()`関数にプログラムを記述する．
```

Arduino IDEの基本操作:

- 画面上部左側の**チェックマーク（Verify）ボタン**でプログラムをコンパイルできる
- その隣の**矢印（Upload）ボタン**でプログラムをArduinoに書き込むことができる
- Arduino UNO R4 WiFiをUSBケーブルでPCに接続した状態で，画面上部のボード選択欄から**Arduino UNO R4 WiFi**を選択する

**LED_BUILTINを使った基本的なLチカ（LED点滅）:**

```cpp
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);  // 内蔵LEDをoutputに設定
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);  // LEDを点灯
    delay(1000);                      // 1秒待機
    digitalWrite(LED_BUILTIN, LOW);   // LEDを消灯
    delay(1000);                      // 1秒待機
}
```

```{note}
`LED_BUILTIN`はArduino UNO R4 WiFiではD13ピンに接続されている．
ピン番号を直接書く（`pinMode(13, OUTPUT)`）よりも`LED_BUILTIN`を使う方が，コードの意図が明確になり，他のボードへの移植性も高くなる．
```

### 演習課題1: LED点滅回路

Arduinoのデジタル信号HIGH/LOWを周期的に切り替えてLEDを点滅させる．実物のブレッドボードとArduino UNO R4 WiFiに実装すること．

```cpp
void setup() {
    pinMode(2, OUTPUT);  // D2をoutputに設定
}

void loop() {
    digitalWrite(2, HIGH);  // D2をHIGHに切り替え
    delay(1000);            // 1000ms待機
    digitalWrite(2, LOW);   // D2をLOWに切り替え
    delay(1000);            // 1000ms待機
}
```

```{figure} fig/micon-programming/tinkercad-led-blink-simulation.jpg
---
width: 400px
name: fig:led-blink
---
LED点滅の確認
```

```{figure} fig/micon-programming/led-blink.png
---
width: 400px
name: fig:led-breadboard
---
実物ブレッドボードでのLED点滅回路の実装
```

### <span style="color:green">チェックポイント: LED点滅の確認</span>

```{exercise} LED点滅の確認
:label: ex_led_blink

1. `LED_BUILTIN`を使った内蔵LEDの点滅を確認せよ
2. 外部LEDをブレッドボード上に配線し，D2ピンからのHIGH/LOW出力でLEDが点滅することを確認せよ
3. TAに動作を見せてチェックを受けること
```

## 4. 演習課題2: LEDマトリクスを使った表示演習

Arduino UNO R4 WiFiに内蔵されている12×8 LEDマトリクスを活用し，パターン表示やテキストスクロールを行う．

```{figure} fig/micon-programming/led-matrix-schematic.jpg
---
width: 500px
name: fig:led-matrix-schematic
---
LEDマトリクス回路図
```

### LEDマトリクスの仕組み: チャーリプレクシング

Arduino UNO R4 WiFiの12×8 LEDマトリクスは合計96個のLEDで構成されているが，96本のピンを使っているわけではない．**チャーリプレクシング（Charlieplexing）** という手法により，わずか**11本のピン**で96個すべてのLEDを個別に制御している．

#### なぜ11本のピンで96個のLEDを制御できるのか

通常，1個のLEDを点灯するにはHIGH（電流の供給元）とLOW（電流の吸い込み先）の2本のピンが必要である．チャーリプレクシングでは，マイコンのピンが取りうる3つの状態を活用する：

| ピンの状態 | 意味 |
|-----------|------|
| **HIGH（出力）** | 電流を供給する |
| **LOW（出力）** | 電流を吸い込む |
| **ハイインピーダンス（入力）** | 回路から切り離された状態 |

N本のピンがあるとき，HIGHにするピンとLOWにするピンの組み合わせは **N × (N - 1)** 通りある．つまり：

- **11本のピン** → 11 × 10 = **110通り** の組み合わせ
- 96個のLEDを制御するには十分

#### 動作原理

2本のピン間に，**互いに逆向き**に2個のLEDを並列接続する．電流の向きによってどちらのLEDが点灯するかが決まる．

```text
        +--->|---+        (LED1: A -> B 方向)
        |        |
PinA ---+        +--- PinB
        |        |
        +---|<---+        (LED2: B -> A 方向)
```

- PinA = HIGH, PinB = LOW → LED1 が点灯（PinAからPinBに電流が流れる）
- PinA = LOW, PinB = HIGH → LED2 が点灯（PinBからPinAに電流が流れる）
- 他のピンはすべてハイインピーダンス（入力モード）にして回路から切り離す

実際には人間の目に見えない速さで1個ずつ順番にLEDを点灯させる（ダイナミック駆動）ことで，すべてのLEDが同時に光っているように見せている．

```{note}
チャーリプレクシングは，少ないピン数で多数のLEDを制御できる一方，同時に点灯できるLEDは1個だけである．
そのため，多くのLEDを同時に光らせると1個あたりの点灯時間が短くなり，輝度が下がるというトレードオフがある．
```

### 2-1: LEDマトリクスの基本表示

まずはLEDマトリクスの基本的な使い方を確認する．Arduino UNO R4 WiFiには12×8のLEDマトリクスが内蔵されており（{numref}`fig:arduino-r4-wifi` 参照），[`Arduino_LED_Matrix`ライブラリ](https://github.com/arduino/ArduinoCore-renesas/blob/main/libraries/Arduino_LED_Matrix/src/Arduino_LED_Matrix.h)を使用して制御できる．

```cpp
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

void setup() {
    matrix.begin();

    uint8_t frame[8][12] = {
        {1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0},
        {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0},
        {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0},
        {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0},
        {1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0},
        {0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0},
    };

    matrix.renderBitmap(frame, 8, 12);
}

void loop() {
    delay(1000);
}
```

```{tip}
LEDマトリクスのフレームデータは8行×12列の2次元配列で表現する．1がLED点灯，0が消灯に対応する．
配列の内容を変更して，自分だけのパターンを作ってみよう．
```

### <span style="color:green">チェックポイント: LEDマトリクスの基本表示</span>

```{exercise} LEDマトリクスの基本表示
:label: ex_led_matrix_basic

1. 上記のサンプルコードを書き込み，LEDマトリクスにパターンが表示されることを確認せよ
2. フレームデータを変更して，自分だけのオリジナルパターンを表示せよ
3. TAに動作を見せてチェックを受けること
```

### 2-2: テキストのスクロール表示

[`ArduinoGraphics`ライブラリ](https://github.com/arduino-libraries/ArduinoGraphics/blob/master/src/ArduinoGraphics.h)を使用すると，LEDマトリクスにテキストをスクロール表示できる．

```{warning}
**インクルード順序に注意**: `ArduinoGraphics.h`は必ず`Arduino_LED_Matrix.h`より**先に**インクルードすること．
順序が逆だと`beginDraw()`や`textFont()`等のメソッドが使えずコンパイルエラーになる．
これは[Arduino_LED_Matrix.h内の条件付きインクルード](https://github.com/arduino/ArduinoCore-renesas/blob/main/libraries/Arduino_LED_Matrix/src/Arduino_LED_Matrix.h)で`ArduinoGraphics.h`が先に読み込まれているかどうかを判定しているためである．
```

```cpp
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

void setup() {
    matrix.begin();
}

const char text[] = "Hello!";

void loop() {
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(50);
    matrix.textFont(Font_5x7);
    matrix.beginText(matrix.width() - 1, 1, 0xFFFFFF);
    matrix.println(text);
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();
}
```

```{note}
`ArduinoGraphics`ライブラリは，Arduino IDEのライブラリマネージャから別途インストールする必要がある．
[スケッチ] -> [ライブラリをインクルード] -> [ライブラリを管理] から「ArduinoGraphics」を検索してインストールすること．
```

```{figure} fig/micon-programming/install-arduino-graphics.png
---
width: 400px
name: fig:install-arduino-graphics
---
Arduino IDEのライブラリマネージャからArduinoGraphicsをインストールする
```

### <span style="color:green">チェックポイント: テキストのスクロール表示</span>

```{exercise} テキストのスクロール表示
:label: ex_led_matrix_scroll

1. `ArduinoGraphics`ライブラリをインストールし，LEDマトリクスにテキストがスクロール表示されることを確認せよ
2. 表示するテキストを自分の好きな文字列に変更して動作を確認せよ
3. TAに動作を見せてチェックを受けること
```

## 5. 演習課題3: 超音波距離センサとLEDマトリクスによる距離の可視化

Arduinoで超音波距離センサ(HC-SR04)の値を読み，前方物体との距離を計測する．ここで使用する`pulseIn`関数は[pulse.cpp](https://github.com/arduino/ArduinoCore-renesas/blob/main/cores/arduino/pulse.cpp)で実装されている．

```{figure} fig/micon-programming/hc-sr04-front.jpg
---
width: 400px
name: fig:ultrasonic-sensor
---
超音波距離センサ HC-SR04
```

**HC-SR04のピンアサイン:**

| Pin | Symbol | Description |
|-----|--------|-------------|
| 1 | VCC | 5V電源 |
| 2 | Trig | トリガー入力ピン |
| 3 | Echo | レシーバー出力ピン |
| 4 | GND | グランド |

```{figure} fig/micon-programming/hc-sr04.png
---
width: 500px
name: fig:ultrasonic-timing
---
超音波距離センサの動作タイミング
```

```{figure} fig/micon-programming/hc-sr04-connection.jpg
---
width: 500px
name: fig:ultrasonic-schematic
---
超音波距離センサ HC-SR04 の接続図
```

### 3-1: シリアル通信による距離の確認

```{tip}
シリアルモニタを初めて使う場合は，{doc}`tips/arduino-serial-monitor` と本ページ末尾の付録{ref}`appendix-serial-monitor`を参照すること．ボーレートをArduino側（`Serial.begin(9600)`）と一致させる必要がある．
```

まずはシリアル通信で距離の値を確認する．

```cpp
#define TRIG_PIN 2
#define ECHO_PIN 3

const float time_gain = 1.0 / 58;  // 時間[us]から距離[cm]への変換係数

float read_sensor() {
    // トリガー送信
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // エコーがHIGHの時間を計測
    return time_gain * pulseIn(ECHO_PIN, HIGH);
}

void setup() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.begin(9600);
}

float distance;

void loop() {
    distance = read_sensor();
    Serial.println(distance);
    delay(500);
}
```

### 演習課題3: LEDマトリクスによる距離の時系列表示

超音波距離センサで計測した距離を，LEDマトリクスに時系列のバーグラフとして表示する．12列のLEDマトリクスを左にスクロールしながら，最新の計測値を右端の列に縦方向のバーとして描画する．距離が近いほどバーが高くなる．

```{figure} fig/micon-programming/led-matrix-distance.png
---
width: 500px
name: fig:led-matrix-distance
---
LEDマトリクスによる距離の時系列表示
```

```cpp
#include "Arduino_LED_Matrix.h"

#define TRIG_PIN 2
#define ECHO_PIN 3

ArduinoLEDMatrix matrix;
uint8_t frame[8][12] = {0};

const float time_gain = 1.0 / 58;  // 時間[us]から距離[cm]への変換係数
const float max_distance = 100.0;   // 表示上の最大距離 [cm]

float read_sensor() {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return time_gain * pulseIn(ECHO_PIN, HIGH);
}

void setup() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.begin(9600);
    matrix.begin();
}

void loop() {
    float distance = read_sensor();
    Serial.println(distance);

    // 全列を左に1列シフト（時系列スクロール）
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 11; col++) {
            frame[row][col] = frame[row][col + 1];
        }
    }

    // 距離をバーの高さ(0〜8)に変換（近いほど高い）
    float clamped = constrain(distance, 0, max_distance);
    int height = 8 - (int)(clamped / max_distance * 8);

    // 右端の列にバーを描画（下から点灯）
    for (int row = 0; row < 8; row++) {
        frame[row][11] = ((7 - row) < height) ? 1 : 0;
    }

    matrix.renderBitmap(frame, 8, 12);
    delay(200);
}
```

```{tip}
LEDマトリクスの12列が時間軸，8行が距離軸に対応している．
手をセンサに近づけたり遠ざけたりすると，波形のようなパターンがリアルタイムにスクロール表示される．
`max_distance`の値を変更すると，表示する距離の範囲を調整できる．
```

```{admonition} 超音波距離センサの計測誤差について
:class: tip

シリアル通信で計測した距離を表示すると，値に微小な変化や実際の距離との差があることに気づくだろう．HC-SR04のデータシートによると距離の計測精度は3mmであり，シリアルコンソールで確認される誤差はこの範囲内であれば正常動作である．また，`pulseIn`関数の時間計測の精度も影響している．

本演習ではセンサの計測値に応じてLEDマトリクスに表示するだけなので，これらの誤差がシステムに大きな影響を及ぼすことはない．しかし，センサの計測値に応じて自動車や機械，ロボット等を**フィードバック制御**する際は，これらの誤差により振動したり暴走したりする場合がある．従ってセンサの計測値を扱う際には，その**精度**，**分解能**，**ヒステリシス**，**応答性**等の特性により誤差が生じることを意識して回路・プログラム・機械を組み上げる必要がある．
```

### <span style="color:green">チェックポイント: 超音波距離センサとLEDマトリクスによる距離の可視化</span>

```{exercise} 超音波距離センサとLEDマトリクスによる距離の可視化
:label: ex_ultrasonic_led_matrix

1. 超音波距離センサ（HC-SR04）を配線し，シリアルモニタで距離の値が正しく表示されることを確認せよ
2. LEDマトリクスに距離の時系列バーグラフがスクロール表示されることを確認せよ
3. 手をセンサに近づけたり遠ざけたりして，LEDマトリクスの表示がリアルタイムに変化することを確認せよ
4. TAに動作を見せてチェックを受けること
```

## 6. 演習課題4: Arduinoを用いたステッピングモータ駆動

Arduinoで複数のステップ波形を生成しステッピングモータを駆動する．

```{figure} fig/micon-programming/stepper-motor-waveform.jpg
---
width: 400px
name: fig:stepper-waveform
---
ステッピングモータを駆動する波形
```

```cpp
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6
#define MOTOR_PIN3 7
#define MOTOR_PIN4 8

const float step_period = 10;  // step入力の時間幅 [msec]

void setup() {
    Serial.begin(9600);
    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);
    pinMode(MOTOR_PIN3, OUTPUT);
    pinMode(MOTOR_PIN4, OUTPUT);
}

void loop() {
    // 1001
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
    digitalWrite(MOTOR_PIN3, LOW);
    digitalWrite(MOTOR_PIN4, HIGH);
    Serial.println("1001");
    delay(step_period);

    // 1100
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, HIGH);
    digitalWrite(MOTOR_PIN3, LOW);
    digitalWrite(MOTOR_PIN4, LOW);
    Serial.println("1100");
    delay(step_period);

    // 0110
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, HIGH);
    digitalWrite(MOTOR_PIN3, HIGH);
    digitalWrite(MOTOR_PIN4, LOW);
    Serial.println("0110");
    delay(step_period);

    // 0011
    digitalWrite(MOTOR_PIN1, LOW);
    digitalWrite(MOTOR_PIN2, LOW);
    digitalWrite(MOTOR_PIN3, HIGH);
    digitalWrite(MOTOR_PIN4, HIGH);
    Serial.println("0011");
    delay(step_period);
}
```

Arduinoには**[Stepper](https://github.com/arduino-libraries/Stepper/blob/master/src/Stepper.h)**ライブラリが用意されているので，これを用いることもできる：

```cpp
#include <Stepper.h>

#define MOTOR_PIN1 5
#define MOTOR_PIN2 6
#define MOTOR_PIN3 7
#define MOTOR_PIN4 8
#define STEPS_PER_ROTATE_28BYJ48 2048  // 1回転に必要なステップ数

const int StepsPerRotate = STEPS_PER_ROTATE_28BYJ48;
int rpm = 15;    // 毎分の回転数 (1-15rpmでないと動かない)
int Steps = 512; // モータに与えるステップ数 (90度回転)

Stepper myStepper(StepsPerRotate, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);

void setup() {
    Serial.begin(9600);
    myStepper.setSpeed(rpm);
}

void loop() {
    Serial.println("Rotate");
    myStepper.step(Steps);
    delay(500);
}
```

### 演習課題4: ステッピングモータの駆動

上記のプログラムを参考に，実物のブレッドボードとArduino UNO R4 WiFiでステッピングモータを駆動する．

モータドライバには大電流ダーリントン・トランジスタ・アレイ**ULN2003**（TEXAS INSTRUMENTS）を使用する．ULN2003は入力信号を増幅し，ステッピングモータを駆動するのに十分な電力を供給する．ArduinoのD5〜D8ピンからの信号をULN2003のIN1〜IN4に接続し，出力ポートにステッピングモータのコネクタを接続する．

```{warning}
ArduinoのGNDとULN2003ボードの5V・GNDを接続する必要があることに注意する．
```

```{figure} fig/micon-programming/stepper-motor-uln2003-wiring.jpg
---
width: 500px
name: fig:stepper-circuit
---
ステッピングモータの駆動回路
```

```{figure} fig/micon-programming/stepping-motor-port.png
---
width: 500px
name: fig:stepper-port
---
ステッピングモータの5V・GND・Arduino配線
```

```{figure} fig/micon-programming/stepping-motor-with-arduino.png
---
width: 400px
name: fig:stepper-wiring
---
Arduinoの配線例
```

```{admonition} デジタル回路演習で作成した回路との比較
:class: tip

デジタル回路演習の課題4で作成したステッピングモータ駆動回路と，本演習の課題4でマイコンを用いて作成した機能はほぼ等価である．しかし，両者には以下のような違いがある：

- **マイコンによる実装**: ソースコードを書き換えるだけで機能を拡張できる．例えば演習課題3のように超音波距離センサと連携させるなど，ソフトウェアの変更のみで新しい機能を追加できる
- **ICによる実装**: マイコンのように拡張は容易ではないが，マイコンのハードウェア制約に影響されない回路を実現できる．例えばマイコンの動作速度よりも高速な信号処理を行う場合や，マイコンのデジタル入出力ポートが不足する場合には，ICとマイコンを組み合わせて実装する場合がある

このように，マイコンは汎用性と拡張性に優れるが，ICによる回路はハードウェアレベルでの高速処理に適しており，用途に応じて使い分けることが重要である．
```

### <span style="color:green">チェックポイント: ステッピングモータの駆動</span>

```{exercise} ステッピングモータの駆動
:label: ex_stepper_motor

1. ULN2003モータドライバを介してステッピングモータ（28BYJ-48）をArduinoのD5〜D8ピンに配線せよ
2. ArduinoのGNDとULN2003ボードの5V・GNDが正しく接続されていることを確認せよ
3. **直接波形を生成する方法**（`digitalWrite`で1001 / 1100 / 0110 / 0011のパターンを出力するプログラム）でステッピングモータが回転することを確認せよ
4. **Stepperライブラリを用いる方法**でも同じくステッピングモータが回転することを確認せよ
5. シリアルモニタで現在のステップ値（`Serial.println("1001")`などの出力）が確認できることを確認せよ
6. `step_period` や `rpm` の値を変えて回転速度が変化することを確認せよ
7. TAに動作を見せてチェックを受けること
```

## 7. 発展課題5: Arduinoを用いた様々なモータ制御

### 発展課題5-1: サーボモータの駆動

Arduinoで[Servoライブラリ](https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h)を使ってサーボモータ([SG90](https://akizukidenshi.com/catalog/g/g108761/))を駆動する．詳細な仕様は[データシート](https://akizukidenshi.com/goodsaffix/SG90_a.pdf)を参照すること．

```{figure} fig/micon-programming/servo-motor-with-arduino.jpg
---
width: 400px
name: fig:servo-motor
---
サーボモータのための接続
```

```{warning}
**SG90の配線に注意**

SG90のケーブルは3本あり，それぞれ以下の通りである：

- **オレンジ**: PWM信号線（Arduinoのデジタルピンに接続）
- **赤（真ん中）**: VCC（5Vに接続）
- **茶色**: GND（グランドに接続）

配線を間違えるとサーボモータが破損する可能性があるため，接続前に必ず確認すること．
```

```{tip}
**3線サーボの配線規則**

SG90に限らず，3線式サーボモータでは一般的に真ん中の線がVCCになっている．
これは，コネクタを逆向きに挿してしまった場合でもVCCとGNDが直結せず，ショートによる破損を防ぐための設計上の配慮である．
```

```cpp
#define SERVO_PIN 2
#include <Servo.h>

Servo servo;

void setup() {
    servo.attach(SERVO_PIN);
}

void loop() {
    servo.write(45);
    delay(1000);
    servo.write(90);
    delay(1000);
    servo.write(135);
    delay(1000);
    servo.write(90);
    delay(1000);
}
```

### 発展課題5-2: 超音波センサとサーボモータの連携

超音波距離センサ（HC-SR04）で計測した距離に応じてサーボモータ（SG90）の角度を変化させるシステムを作成する．距離が近いほどサーボモータの角度が大きくなるようにする．

```{figure} fig/micon-programming/servo-motor-with-distance.jpg
---
width: 500px
name: fig:servo-motor-with-distance
---
超音波センサとサーボモータの接続
```

```cpp
#define SERVO_PIN 2
#define TRIG_PIN 3
#define ECHO_PIN 4

#include <Servo.h>

Servo servo;

const float max_distance = 50.0;  // 最大計測距離 [cm]

void setup() {
    Serial.begin(9600);
    servo.attach(SERVO_PIN);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop() {
    // 超音波センサで距離を計測
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.0343 / 2.0;  // 距離 [cm]

    // 距離をサーボの角度（0〜180度）にマッピング
    // 距離が近いほど角度が大きくなる
    int angle = 0;
    if (distance < max_distance) {
        angle = (int)((1.0 - distance / max_distance) * 180.0);
    }
    angle = constrain(angle, 0, 180);

    servo.write(angle);

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Angle: ");
    Serial.println(angle);

    delay(200);
}
```

超音波距離センサ（HC-SR04）で計測した距離に応じて，サーボモータ（SG90）の角度を変化させると同時に，LEDマトリクスにバーメーターとして距離を表示する．距離が近いほどサーボモータの角度が大きくなり，LEDマトリクスの点灯行数も増える．

```cpp
#define SERVO_PIN 2
#define TRIG_PIN 3
#define ECHO_PIN 4

#include <Servo.h>
#include "Arduino_LED_Matrix.h"

Servo servo;
ArduinoLEDMatrix matrix;

const float max_distance = 50.0;  // 最大計測距離 [cm]
uint8_t frame[8][12] = {0};       // LEDマトリクスのフレームバッファ

void setup() {
    Serial.begin(9600);
    servo.attach(SERVO_PIN);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    matrix.begin();
}

void loop() {
    // 超音波センサで距離を計測
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.0343 / 2.0;  // 距離 [cm]

    // 距離をサーボの角度（0〜180度）にマッピング
    int angle = 0;
    if (distance < max_distance) {
        angle = (int)((1.0 - distance / max_distance) * 180.0);
    }
    angle = constrain(angle, 0, 180);
    servo.write(angle);

    // 距離をLEDマトリクスの点灯行数（0〜8）にマッピング
    int bars = 0;
    if (distance < max_distance) {
        bars = (int)((1.0 - distance / max_distance) * 8.0) + 1;
    }
    bars = constrain(bars, 0, 8);

    // LEDマトリクスを左にスクロール
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 11; col++) {
            frame[row][col] = frame[row][col + 1];
        }
    }
    // 最新の計測値を右端の列に描画
    for (int row = 0; row < 8; row++) {
        frame[row][11] = (row >= 8 - bars) ? 1 : 0;
    }
    matrix.renderBitmap(frame, 8, 12);

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Angle: ");
    Serial.print(angle);
    Serial.print(", Bars: ");
    Serial.println(bars);

    delay(200);
}
```

```{tip}
`max_distance`の値を変更すると，反応する距離の範囲を調整できる．
LEDマトリクスには距離の時系列変化がスクロール表示され，同時にサーボモータも距離に応じて動く．
```

### 発展課題5-3: DCモータの駆動

DCモータをマイコンで駆動するには，Arduinoのデジタル出力ピン（数十mA）では電流が足りないため，**モータドライバIC**を介して大電流をスイッチングする必要がある．本演習では東芝のフルブリッジドライバ **TB6643KQ**（最大4.5 A）を実機で用いる．

ここでは新しいデバイスを扱うときの基本パターン **「(1) データシートを読む → (2) 回路を作る → (3) コードで動かす」** の順に進める．この流れは組み込みエンジニアとして身につけるべき手順である．

```{figure} fig/micon-programming/tb6643kq-package.jpg
---
width: 200px
name: fig:tb6643kq-package
---
TB6643KQ（HSIP7-P-2.54A パッケージ）の外観．7本のリード端子を持つ
```

#### (1) データシートを読む

データシートは「**そのICを正しく安全に使うための一次資料**」である．インターネット上の作例や記事だけを頼りに配線すると，ピンアサインを間違えたり定格を超えてICを破壊することがある．[TB6643KQのデータシート（秋月電子）](https://akizukidenshi.com/goodsaffix/TB6643KQ_datasheet_ja_20110621.pdf)を例に，**最低限ここだけは読むべき**という項目を順番に見ていこう．

**① 特長 (Features)**

データシートの先頭にはICの主要な仕様が箇条書きで書かれている．**まずここを読んで「自分の用途で使えるか」を判断する**．TB6643KQの場合：

- 電源電圧 (VM): 最大 **50 V**
- 出力電流: 最大 **4.5 A**
- 出力ON抵抗: 0.55 Ω (標準)
- PWM制御可能（IN1, IN2でスイッチング）
- **正転 / 逆転 / ショートブレーキ / ストップ** の4モード
- 過電流検出 (ISD) / 過電圧検出 (VSD) / 熱遮断 (TSD) / 低下電圧検出 (UVLO) の保護回路内蔵

この欄を見るだけで「**5 Vロジックで制御でき，最大4.5 A流せて，保護回路もある**」という用途への適合性が判断できる．

**② ブロック図 / 応用回路例 (Block Diagram)**

ICの内部構成と「**メーカが推奨する典型的な配線例**」が描かれている．データシートを参考に回路を組むときの出発点になる図．

```{figure} fig/micon-programming/tb6643kq-block-diagram.jpg
---
width: 600px
name: fig:tb6643kq-block
---
TB6643KQ のブロック図（応用回路例）．IN1, IN2の入力からControl/Predriverを経て4つのMOSFETによるフルブリッジでモータを駆動する．VM端子の近くに**3個のコンデンサ**が描かれていることに注目
```

このブロック図から次のことが読み取れる：

- IN1, IN2 はICの内部でプルダウンされており，**未接続だとLOW扱い**になる
- 出力段は**4つのMOSFETと逆並列ダイオード**で構成されたフルブリッジ
- 各MOSFETに過電流検出 (ISD detection) が付いている
- VM端子の近くにバイパスコンデンサ（パスコン）を配置するのがメーカ推奨

**③ 端子説明・外形図 (Pin Description / Outline)**

ピン番号と機能の対応はここで確認する．**配線前に必ず読むべき**最重要項目．TB6643KQ (HSIP7-P-2.54A) は7本ピン：

| ピン番号 | 名称 | 機能 |
|---------|------|------|
| 1 | IN1 | 制御信号入力 1（Arduinoから） |
| 2 | IN2 | 制御信号入力 2（Arduinoから） |
| 3 | OUT1 | モータ出力 1 |
| 4 | GND | グランド |
| 5 | OUT2 | モータ出力 2 |
| 6 | N.C. | 接続なし (No Connection) |
| 7 | VM | モータ電源 (10〜45 V) |

```{figure} fig/micon-programming/tb6643kq-outline.jpg
---
width: 350px
name: fig:tb6643kq-outline
---
TB6643KQ の外形図とピン配置．**型番が印字された面から見て**左端が1番ピン (IN1)，右端が7番ピン (VM) になる
```

```{warning}
**ピン番号の数え方を取り違えないこと**

HSIP7-P-2.54Aは，**型番のシール（印字）面から見て**左から1, 2, 3, ..., 7番である．裏返すと左右が逆になるので注意．ブレッドボードに挿す前に必ずどちらが1番ピンかを確認すること．また**6番ピンは N.C.（接続なし）**なので，どこにも繋がない（誤って繋いでも害はないが，誤配線の原因になる）．
```

**④ 絶対最大定格 (Absolute Maximum Ratings)**

「**瞬時たりとも超えてはならない**」値が書かれている．これを超えるとIC破壊・発煙・発火の原因となる．

| 項目 | 記号 | 定格 |
|------|------|------|
| 電源電圧 | VM | 50 V |
| 出力電圧 | $V_O$ | 50 V |
| 出力電流 (ピーク) | $I_O(peak)$ | 4.5 A |
| 入力電圧 | $V_{IN}$ | **-0.3 〜 5.5 V** |
| 許容損失（放熱板なし）| $P_D$ | 1.25 W |
| 動作温度 | $T_{opr}$ | -40 〜 85 °C |

ここから「**IN1, IN2 には 5.5 V を超える電圧を絶対に印加してはならない**」ことが読み取れる．Arduino UNO R4 WiFi のロジック電圧は 5 V なので問題ないが，もし他の3.3V系マイコン（ESP32など）と混在させる場合は，HIGH時に5Vが出ないことを確認すること．

**⑤ 動作範囲 (Operating Range)**

「絶対最大定格」が**壊れる境界線**であるのに対し，「動作範囲」は**設計時に守るべき推奨範囲**である．通常はこちらの範囲内で使う．

| 項目 | 記号 | 定格 |
|------|------|------|
| 電源電圧 | $VM_{opr}$ | **10 〜 45 V** |
| PWM 周波数 | $f_{PWM}$ | 〜 100 kHz |
| 出力電流 (平均) | $I_O$ (平均) | 〜 1.5 A (放熱板なし) |

```{important}
**TB6643KQのVMは最低でも 10 V** が必要である．Arduino UNO R4 WiFiの5V電源では駆動できない．モータ用に **12 VのACアダプタ** や **乾電池ボックス（単3×8本=12 V等）** などの外部電源を別途用意すること．`analogWrite`のPWM周波数（Arduino UNO R4 WiFi標準で約490 Hz）はTB6643KQの100 kHz上限に十分収まっている．
```

**⑥ 入出力ファンクション表 (Truth Table)**

ICを「**どう操作すればどう動くか**」を示す表．データシートの中で**最もよく参照する項目**．

| IN1 | IN2 | OUT1 | OUT2 | モード |
|-----|-----|------|------|--------|
| H | H | L | L | **ショートブレーキ**（急停止） |
| L | H | L | H | 正転（または逆転） |
| H | L | H | L | 逆転（または正転） |
| L | L | Hi-Z | Hi-Z | **ストップ**（出力オフ・惰性回転） |

ArduinoのD9, D10ピンをIN1, IN2に繋ぎ，HIGH/LOWを切り替えるだけで4モードを使い分けられる．**PWM制御**は，例えば IN2をLOW固定にして IN1にPWMを入れる方式で実現する：

- PWM ON（IN1=H, IN2=L）: 正転
- PWM OFF（IN1=L, IN2=L）: ストップ（Hi-Z）

```{figure} fig/micon-programming/tb6643kq-pwm-on-off.jpg
---
width: 600px
name: fig:tb6643kq-pwm
---
TB6643KQ の PWM 制御時の電流経路．PWM ON期間（左）→OFFへの遷移（中）→PWM OFF期間（右）と推移する．上下のMOSFETが同時にONにならないよう**200 ns / 500 nsのデッドタイムがIC内部で生成**されるため，外部から OFFタイムを挿入する必要はない
```

**⑦ 保護機能 (UVLO / VSD / TSD / ISD)**

TB6643KQには各種保護回路が内蔵されている：

| 保護機能 | 動作条件 | 動作内容 |
|---------|---------|---------|
| **UVLO**（低下電圧検出）| VM < 8 V | 出力を Hi-Z にして停止 |
| **VSD**（過電圧検出）| VM > 53 V | 出力を Hi-Z にして停止 |
| **TSD**（熱遮断）| Tj > 170°C | 出力を Hi-Z にして停止 |
| **ISD**（過電流検出）| 4.5〜8 A を 5.1 µs 以上 | 全出力を Hi-Z にして停止 |

```{warning}
これらの保護機能は**補助的なもの**であり，「絶対最大定格を超えても保護してくれる」ものではない．データシートにも「**いかなる場合でも IC を保護するというものではありません**」と明記されている．設計段階で動作範囲内に収まるようにすることが大前提．
```

**⑧ 許容損失と熱設計 (Power Dissipation)**

ICが連続的に消費できる電力の上限．TB6643KQは**放熱板なしで Ta = 25°C のとき $P_D$ = 1.25 W しか許容されない**．消費電力 $P = R_{ON} \times I^2$ で，例えば 1.5 A 連続で流すと $P = 0.55 \times 1.5^2 \approx 1.24$ W となり，放熱板なしではほぼ上限に達する．

```{figure} fig/micon-programming/tb6643kq-pd-ta.jpg
---
width: 400px
name: fig:tb6643kq-pd-ta
---
TB6643KQ の許容損失 - 周囲温度特性．(1) 10°C/Wの放熱板を使用した場合 7.8 W，(2) 放熱板なしの場合 1.25 W．温度が高くなると許容損失は直線的に減少する
```

```{tip}
データシート末尾の「使用上の注意事項」「使用上の留意点」も必ず読むこと．「適切な電源ヒューズを使用」「逆起電力対策」など，**現場で起きる事故を防ぐ知見**が詰まっている．
```

#### (2) 回路を作る

データシートで読み取った情報を元に，Arduino UNO R4 WiFi と TB6643KQ の配線を決める．

**配線表:**

| TB6643KQ ピン | 接続先 | 備考 |
|---------------|--------|------|
| 1 (IN1) | Arduino **D9**（PWM対応） | 方向 or PWM入力 |
| 2 (IN2) | Arduino **D10**（PWM対応） | 方向 or PWM入力 |
| 3 (OUT1) | DCモータ端子A | — |
| 4 (GND) | Arduino GND **かつ** モータ電源 (−) | **共通GND** |
| 5 (OUT2) | DCモータ端子B | — |
| 6 (N.C.) | 未接続 | — |
| 7 (VM) | モータ用外部電源 (+) | 10〜45 V．**パスコン要** |

**結線図（テキスト版）:**

```text
                         +12V (モータ用外部電源)
                          |
                          +--+ 100μF 電解 + 0.1μF セラミック (パスコン)
                          |     |        |
                          |     GND      GND
                          |
        Arduino           |
        D9  ───────►  IN1(1)        VM(7)
        D10 ───────►  IN2(2)
                        OUT1(3) ──[DCモータ]── OUT2(5)
                        GND(4) ───────┐
                                      |
        Arduino GND ──────────────────+── モータ電源 GND
                            （共通GND）
```

```{warning}
**GNDの共通化を忘れない**

Arduino の GND，TB6643KQ の 4番ピン GND，モータ外部電源の (−) を**必ず同じ電位**にすること（共通GND）．これがないと IN1/IN2 のHIGH/LOWの基準が定まらず，誤動作やIC破壊の原因になる．モータ電源とArduino電源を別系統で取る場合は，GND同士をジャンパで必ず繋ぐ．
```

```{important}
**バイパスコンデンサ（パスコン）を必ず入れる**

TB6643KQの**7番ピン (VM) のすぐ近くに**，以下を**並列で**GNDに接続する：

- **セラミックコンデンサ 0.1 µF**（高周波ノイズの除去）
- **電解コンデンサ 100 µF 以上**（VM電圧の2倍以上の耐圧．12V電源なら25V耐圧以上を推奨）

データシートのブロック図（{numref}`fig:tb6643kq-block`）でも VM端子近傍に**3個のコンデンサ**が描かれている．これがないとモータの起動・反転時のサージで TB6643KQ や Arduino が破損する恐れがある．電解コンデンサは**極性を間違えない**こと（長いリードが+，缶の白帯側が−）．
```

```{tip}
**放熱について**

平均1 A を超えるような連続駆動を行う場合は，TB6643KQの背面金属タブにヒートシンクをネジ留めする．データシートに「**IC裏面の金属部分は ICチップ裏面と電気的に接続されている**」とあるため，ヒートシンクは**絶縁シートを挟むか，GNDに落とす**こと（他の電位に接触させない）．
```

#### (3) コードで動かす

回路ができたらArduinoから制御する．まずは `analogWrite` を使った最も基本的な例から始めて，より細かい制御が必要な場合は `FspTimer` を使った自作PWMに進む．

**a. `analogWrite` を使った基本制御**

IN2をLOW固定，IN1にPWMを入れて正転方向にデューティー比を変えながら回す例．上の入出力ファンクション表（⑥）に従ってショートブレーキ・ストップも切り替える：

```cpp
#define IN1_PIN 9   // TB6643KQ IN1 (PWM対応ピン)
#define IN2_PIN 10  // TB6643KQ IN2 (PWM対応ピン)

void setup() {
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    // 正転 (IN1=PWM, IN2=LOW) : デューティー比 50%
    Serial.println("Forward (duty 50%)");
    analogWrite(IN1_PIN, 128);
    digitalWrite(IN2_PIN, LOW);
    delay(2000);

    // ショートブレーキ (IN1=HIGH, IN2=HIGH) : 急停止
    Serial.println("Short brake");
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, HIGH);
    delay(500);

    // 逆転 (IN1=LOW, IN2=PWM) : デューティー比 50%
    Serial.println("Reverse (duty 50%)");
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, 128);
    delay(2000);

    // ストップ (IN1=LOW, IN2=LOW) : Hi-Z で惰性回転
    Serial.println("Stop (Hi-Z)");
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    delay(500);
}
```

```{tip}
**「ショートブレーキ」と「ストップ (Hi-Z)」の違い**

- **ショートブレーキ (IN1=H, IN2=H)**: 両出力をLOWに短絡し，モータの慣性で生じる起電力を回生して急速に減衰させる → **急停止**
- **ストップ (IN1=L, IN2=L)**: 両出力をハイインピーダンスにする → モータは**惰性で回り続けて**やがて摩擦で止まる

用途によって使い分ける（例: ロボットの位置制御では急停止が必要なのでショートブレーキ）．
```

**b. `FspTimer` (タイマー割り込み) によるPWM自作**

Arduino UNO R4 WiFiに内蔵されている `FspTimer` を使ってタイマー割り込みでPWM波形を自作することもできる．`analogWrite` では実現しにくい細かなPWM制御（周波数を変える，特殊な波形を出すなど）が必要なときに使う．

```{note}
`FspTimer` はArduino UNO R4 WiFiのコアライブラリに含まれているため，追加インストールは不要．
従来のArduino UNO R3で使われていた `MsTimer2` はAVR専用であり，Arduino UNO R4 WiFi（Renesas RA4M1）では使用できない．
割り込み処理の中でグローバル変数を使う場合は `volatile` 修飾子をつけて宣言すること．
```

```cpp
#define IN1_PIN 9   // TB6643KQ IN1
#define IN2_PIN 10  // TB6643KQ IN2

#include "FspTimer.h"

FspTimer pwm_timer;

volatile int duty;
volatile int pwm_count;

// タイマー割り込みで呼ばれるコールバック関数
// IN1にPWM, IN2はLOW固定 (正転方向)
void pwm_cycle(timer_callback_args_t __attribute((unused)) *args) {
    if (pwm_count < 100) {
        if (pwm_count == 0) {
            digitalWrite(IN1_PIN, HIGH);
        } else if (pwm_count == duty) {
            digitalWrite(IN1_PIN, LOW);
        }
        ++pwm_count;
    } else {
        pwm_count = 0;
    }
    digitalWrite(IN2_PIN, LOW);
}

void setup() {
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    duty = 0;
    pwm_count = 0;

    // 1msec (1000Hz) 毎に pwm_cycle を呼ぶタイマーを設定
    uint8_t timer_type = GPT_TIMER;
    int8_t timer_ch = FspTimer::get_available_timer(timer_type);
    pwm_timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_ch, 1000.0, 0.0, pwm_cycle);
    pwm_timer.setup_overflow_irq();
    pwm_timer.open();
    pwm_timer.start();
}

void loop() {
    duty = 20;   // デューティー比 20%
    delay(2000);
    duty = 50;   // デューティー比 50%
    delay(2000);
    duty = 80;   // デューティー比 80%
    delay(2000);
}
```

```{note}
**Arduino UNO R4 WiFiのPWM出力ポートについて**

Arduino UNO R4 WiFiでは，以下のピンでPWM出力が可能です：
- **D3, D5, D6, D9, D10, D11** （PWM対応ピン）

`analogWrite` 関数を用いてPWM出力ができるピンはこれらのPWM対応ピンのみです．
```

```{admonition} マイコンのピンの役割
:class: tip

Arduinoのピンをよく見ると，D3, D5, D6, D9, D10, D11ピンには「~」の表記がある．これはPWM波形を出力できるピンであることを意味している．`analogWrite`関数でPWM出力ができるのはこれらのPWM対応ピンのみであり，上のコード例でも `IN1_PIN` に **D9**，`IN2_PIN` に **D10** を指定していることに注意したい．

また，一般にマイコンのピンはそれぞれ機能が決まっているが，ArduinoのPWM出力ピンがデジタル入出力の機能も兼ねているように，一つのポートを複数の機能で兼用していることがある．回路を設計する際には，利用したい機能同士が兼用ピンとなって重複していないかの確認が必要である．
```

````{admonition} 参考: シミュレーション用のドライバIC (L293D)
:class: note

Tinkercad等のシミュレータで実機が無い環境で試す場合は，Texas Instruments の **L293D**（最大600 mA）も使える．L293DはTB6643KQと異なり **ENABLE端子 (EN1, EN2)** を持ち，これをHIGHにした上で IN1, IN2 を制御する2チャンネルH-ブリッジである．ピンアサインや動作モードは[L293Dのデータシート](https://www.ti.com/lit/ds/symlink/l293.pdf)で確認すること．**「データシートを読んでから配線する」という流れは，どのドライバICでも変わらない**．

```{figure} fig/micon-programming/dc-motor-l293d-circuit.jpg
---
width: 500px
name: fig:dc-motor-circuit
---
L293Dを用いたDCモータ駆動回路の例（参考）
```
````

### <span style="color:green">チェックポイント: DCモータの駆動</span>

```{exercise} DCモータの駆動 (TB6643KQ)
:label: ex_dc_motor_tb6643kq

1. TB6643KQのデータシート（[PDF](https://akizukidenshi.com/goodsaffix/TB6643KQ_datasheet_ja_20110621.pdf)）の **特長・端子説明・絶対最大定格・動作範囲・入出力ファンクション表** に目を通せ
2. ArduinoのD9 → IN1，D10 → IN2，OUT1/OUT2 → DCモータ，VM → 外部電源 (+)，GND → Arduino GND および 外部電源 (−)，として配線せよ．**VM-GND間にパスコン（0.1 µF + 100 µF）を入れる**ことを忘れないこと
3. `analogWrite` 版のサンプルスケッチを書き込み，正転 → 急停止 → 逆転 → 惰性停止のサイクルが動作することを確認せよ
4. `analogWrite` の値を変えてデューティー比と回転速度の関係を観察せよ
5. 余裕があれば `FspTimer` 版も試し，タイマー割り込みによるPWM自作の動作を確認せよ
6. TAに動作を見せてチェックを受けること
```

## 8. 発展課題6: シリアル通信を介した計算機とマイコン間の通信

```{figure} fig/micon-programming/serial-communication.png
---
width: 500px
name: fig:serial-communication
---
マイコン・計算機間の文字列の送受信の概念図
```

```{admonition} ボーレートを合わせる理由
:class: important

シリアル通信では，送信側と受信側でボーレート（1秒間に送受信するビット数）を一致させる必要がある．ボーレートが一致していないと，受信側がビットの境界を正しく認識できず，データが化けたり，フレーミングエラーが発生する．{numref}`fig:baudrate` に示すように，送信側が速すぎる場合はビットの読み取りタイミングがずれてフレーミングエラーとなり，遅すぎる場合は同じビットを二重に読み取ってデータの重複が起こる．Arduino側の`Serial.begin(9600)`と計算機側の設定（例：`baudrate=9600`）を必ず同じ値に揃えること．
```

```{figure} fig/micon-programming/baudrate.png
---
width: 600px
name: fig:baudrate
---
ボーレートの不一致によるエラーの例．送信側と受信側のボーレートが一致している場合（Panel 1）は正常に通信できるが，不一致の場合（Panel 2, 3）はフレーミングエラーやデータの重複が発生する．
```

### 発展課題6-1: シリアル通信コマンドtioを用いた文字列の表示

計算機でUSBにより接続したArduinoが計算機で`/dev/ttyACM0`として認識されていてボーレート9600 bpsの場合：

```bash
tio -b 9600 /dev/ttyACM0
```

`tio`は軽量で使いやすいシリアル端末ツールである．`-b`オプションでボーレートを指定する（指定しない場合のデフォルトは115200 bps）．

**tioの終了方法**: `Ctrl-T` を押した直後に `q` を押す．

```{note}
Arduino UNO R4 WiFiはUSB-Cで接続し，OSによって次のように認識される：

- **Linux**: `/dev/ttyACM0`
- **macOS**: `/dev/cu.usbmodem*`（`*`にはシリアル番号が入る）
- **Windows**: `COM*`

以降の例では`/dev/ttyACM0`を使うが，macOS/Windowsでは自分の環境のデバイス名に読み替えること．デバイスが見つからない場合は{doc}`tips/arduino-device-detection`を参照．
```

```{admonition} tioのインストール
:class: tip

- **Ubuntu/Debian**: `sudo apt install tio`
- **macOS (Homebrew)**: `brew install tio`
- **Windows**: WSL を使うか，[GitHubのリリースページ](https://github.com/tio/tio/releases)からバイナリを取得

`tio --help` で利用可能なオプションを確認できる．
```

````{admonition} tioがうまくいかない場合
:class: warning

以下を順に確認すること：

1. **Arduino IDEのシリアルモニタが開いていないか確認**：一つのシリアルポートに同時アクセスできるのは1プログラムだけなので，IDEのシリアルモニタは閉じる
2. **権限エラー（`Permission denied`）の場合**：`sudo usermod -a -G dialout $USER` を実行後にログアウト・再ログイン（macOSでは不要）
3. **`tio`が利用できないときの代替コマンド**：以下のいずれかを使う
   ```bash
   # cu を使う（終了は ~ → Ctrl-D．Ubuntuでは sudo apt install cu）
   cu -l /dev/ttyACM0 -s 9600
   # screen を使う（終了は Ctrl-A → K → y）
   screen /dev/ttyACM0 9600
   # picocom を使う（終了は Ctrl-A → Ctrl-X）
   sudo apt install picocom
   picocom -b 9600 /dev/ttyACM0
   # minicom を使う
   sudo apt install minicom
   minicom -D /dev/ttyACM0 -b 9600
   ```
4. **PythonのpyserialやArduino IDEのシリアルモニタで代替**：後述のPythonによる方法（pyserial）でも文字列を確認できる
````

tioコマンドの代わりに，C言語で直接シリアルポートを読み取ることもできる：

**C言語によるシリアル読み取りプログラム:**

```c
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>

int main(int argc, char *argv[]) {
    const char *port_name = "/dev/ttyACM0";
    if (argc > 1) {
        port_name = argv[1];
    }

    int serial_port = open(port_name, O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB;   // パリティなし
    tty.c_cflag &= ~CSTOPB;   // ストップビット1
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8ビットデータ
    tty.c_cflag &= ~CRTSCTS;  // ハードウェアフロー制御なし
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;  // 1秒タイムアウト
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    char buf[256];
    while (1) {
        memset(buf, '\0', sizeof(buf));
        int n = read(serial_port, buf, sizeof(buf));
        if (n > 0) {
            printf("%s", buf);
            fflush(stdout);
        }
    }

    close(serial_port);
    return 0;
}
```

コンパイルと実行：

```{note}
Ubuntuで`gcc`が入っていない場合は事前に`sudo apt install build-essential`でCコンパイラ一式を導入する．
```

```bash
gcc -o read_serial read_serial.c
./read_serial /dev/ttyACM0
```

**C++（boost::asio）によるシリアル読み取りプログラム:**

````{admonition} Boostライブラリのインストール
:class: important

`<boost/asio.hpp>` は標準C++ライブラリには含まれていないため，事前にBoostライブラリのインストールが必要：

- **Ubuntu/Debian**:
  ```bash
  sudo apt install libboost-all-dev
  ```
- **macOS (Homebrew)**:
  ```bash
  brew install boost
  ```
  コンパイル時は `-I/opt/homebrew/include -L/opt/homebrew/lib`（Apple Silicon）または `-I/usr/local/include -L/usr/local/lib`（Intel Mac）を追加する
- **Windows**: [Boost公式サイト](https://www.boost.org/releases/latest/) からダウンロード，もしくは WSL/MSYS2 を利用するのが簡単

Boostを使わずに済ませたい場合は，前掲のC言語版（`termios`を使う）またはPython版（`pyserial`）を用いる．
````

```cpp
#include <boost/asio.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
    const char *port_name = "/dev/ttyACM0";
    if (argc > 1) {
        port_name = argv[1];
    }

    try {
        boost::asio::io_context io;
        boost::asio::serial_port port(io, port_name);
        port.set_option(boost::asio::serial_port_base::baud_rate(9600));

        while (1) {
            boost::asio::streambuf buf;
            boost::asio::read_until(port, buf, "\n");
            std::istream is(&buf);
            std::string line;
            std::getline(is, line);
            std::cout << ">>" << line << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```

コンパイルと実行：

```bash
g++ -std=c++17 -o read_serial read_serial.cpp -lboost_system
./read_serial /dev/ttyACM0
```

### 発展課題6-2: 計算機側からの文字列の入力

```{figure} fig/micon-programming/pyserial.png
---
width: 500px
name: fig:pyserial
---
Pythonによるシリアル通信の実行例
```

**Arduino側のプログラム:**

```cpp
void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(100);  // シリアルポートが接続されるのを待つ
    }
}

int delay_ms = 500;
int count = 0;

void loop() {
    Serial.print("[");
    Serial.print(count);
    Serial.print("] Hello world!");
    Serial.println();
    count++;
    delay(delay_ms);

    if (Serial.available() > 0) {
        Serial.print("Change delay to ");
        delay_ms = Serial.readString().toInt();
        Serial.println(delay_ms, DEC);
    }
}
```

**Python環境のセットアップ:**

Pythonでシリアル通信を行うには`pyserial`ライブラリが必要である．以下の手順でvenv（仮想環境）を作成し，インストールする：

```{note}
Ubuntuで`python3 -m venv`が「The virtual environment was not created successfully because ensurepip is not available」と失敗する場合は，事前に`sudo apt install python3-venv`を実行する．
```

```bash
python3 -m venv ~/arduino_venv
source ~/arduino_venv/bin/activate
pip install pyserial
```

以降，Pythonのシリアル通信プログラムを実行する際は，事前に仮想環境を有効化しておくこと：

```bash
source ~/arduino_venv/bin/activate
```

**Python側のプログラム:**

```python
import serial

ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
loop = 0

while ser.is_open:
    print(">>" + ser.readline().decode().strip())
    if loop == 10:
        ser.write(b"100\n")
    elif loop == 50:
        ser.write(b"500\n")
    elif loop == 70:
        break
    loop = loop + 1
```

### 発展課題6-3: シリアル通信を介したLEDマトリクスへのテキスト表示

計算機からシリアル通信で送信したテキストを，LEDマトリクスにスクロール表示する．

**Arduino側のプログラム:**

```cpp
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;
String message = "";

void setup() {
    Serial.begin(9600);
    matrix.begin();
    while (!Serial) {
        delay(100);
    }
    Serial.println("Ready. Send text to display on LED matrix.");
}

void loop() {
    if (Serial.available() > 0) {
        message = Serial.readStringUntil('\n');
        Serial.print("Displaying: ");
        Serial.println(message);

        matrix.beginDraw();
        matrix.stroke(0xFFFFFFFF);
        matrix.textScrollSpeed(50);
        matrix.textFont(Font_5x7);
        matrix.beginText(matrix.width() - 1, 1, 0xFFFFFF);
        matrix.println(message);
        matrix.endText(SCROLL_LEFT);
        matrix.endDraw();
    }
}
```

**Python側のプログラム:**

```python
import serial

ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)

# Arduinoの起動を待つ
print(ser.readline().decode().strip())

while True:
    text = input("表示するテキストを入力 (Ctrl+Cで終了): ")
    ser.write((text + "\n").encode())
    print(">>" + ser.readline().decode().strip())
```

```{tip}
Pythonプログラムを実行したら，Arduino UNO R4 WiFiの基板上にある白いリセットボタンを押してArduinoを再起動すると，シリアル通信の接続が確実に確立される．
```

```{note}
`ArduinoGraphics`ライブラリが必要です．インストールしていない場合は，Arduino IDEのライブラリマネージャからインストールすること（{numref}`fig:install-arduino-graphics` 参照）．
```

### 発展課題6-4: シリアル通信を介した計算機側からのサーボモータの制御

**Arduino側のプログラム:**

```cpp
#define SERVO_PIN 2
#include <Servo.h>

Servo servo;

void setup() {
    Serial.begin(9600);
    servo.attach(SERVO_PIN);
    while (!Serial) {
        delay(100);  // シリアルポートが接続されるのを待つ
    }
    Serial.println("Ready. Send angle (0-180).");
}

int angle = 0;

void loop() {
    if (Serial.available() > 0) {
        angle = Serial.readStringUntil('\n').toInt();
        servo.write(angle);
        Serial.print("Change angle to ");
        Serial.println(angle, DEC);
    }
    delay(10);  // CPU負荷軽減のための短い待機
}
```

**Python側のプログラム:**

```python
import serial

ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)

# Arduinoの起動を待つ
print(ser.readline().decode().strip())

while True:
    angle = input("角度を入力 (0-180, Ctrl+Cで終了): ")
    ser.write((angle + "\n").encode())
    print(">>" + ser.readline().decode().strip())
```

**C言語による計算機側のプログラム:**

```c
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>

int main(int argc, char *argv[]) {
    const char *port_name = "/dev/ttyACM0";
    if (argc > 1) {
        port_name = argv[1];
    }

    int serial_port = open(port_name, O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    printf("begin loop\n");
    char buf[256];
    for (int i = 0; ; i++) {
        memset(buf, '\0', sizeof(buf));
        int n = read(serial_port, buf, sizeof(buf));
        if (n > 0) {
            printf("%s", buf);
            fflush(stdout);
        }
        if (i % 2 == 0) {
            write(serial_port, "10\n", strlen("10\n"));
            printf("changed angle to 10\n");
        } else {
            write(serial_port, "100\n", strlen("100\n"));
            printf("changed angle to 100\n");
        }
        sleep(1);
    }

    close(serial_port);
    return 0;
}
```

**C++（boost::asio）による計算機側のプログラム:**

```{note}
Boostライブラリのインストールが必要．インストール手順は前述の「Boostライブラリのインストール」を参照．
```


```cpp
#include <boost/asio.hpp>
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[]) {
    const char *port_name = "/dev/ttyACM0";
    if (argc > 1) {
        port_name = argv[1];
    }

    try {
        boost::asio::io_context io;
        boost::asio::serial_port port(io, port_name);
        port.set_option(boost::asio::serial_port_base::baud_rate(9600));

        while (true) {
            sleep(1);
            boost::asio::write(port, boost::asio::buffer("10\n", 3));
            std::cout << "changed angle to 10" << std::endl;
            sleep(1);
            boost::asio::write(port, boost::asio::buffer("100\n", 4));
            std::cout << "changed angle to 100" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```

コンパイルと実行：

```bash
g++ -std=c++17 -o serial_servo serial_servo.cpp -lboost_system
./serial_servo /dev/ttyACM0
```

## 9. 発展課題7: Wi-Fiを使ったスマートフォンからLEDマトリクスへのメッセージ送信

Arduino UNO R4 WiFiをWebサーバとして動作させ，同じWi-Fiネットワークに接続したスマートフォンのブラウザからメッセージを送信し，LEDマトリクスにスクロール表示するシステムを作成する．Wi-Fi機能は[WiFiS3ライブラリ](https://github.com/arduino/ArduinoCore-renesas/blob/main/libraries/WiFiS3/src/WiFiS3.h)を使用する．

```{figure} fig/micon-programming/arduino-led-matrix-web.png
---
width: 500px
name: fig:led-matrix-web
---
スマートフォンからLEDマトリクスへのメッセージ送信
```

**手順:**

1. Arduino UNO R4 WiFiをWi-Fiアクセスポイントとして設定する
2. Webサーバを起動し，メッセージ入力用のHTMLページを配信する
3. スマートフォンからArduinoのアクセスポイントに接続し，ブラウザでメッセージを送信する
4. 受信したメッセージをLEDマトリクスにスクロール表示する

```cpp
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

// アクセスポイントのSSIDとパスワード
char ap_ssid[] = "Arduino-LED";
char ap_pass[] = "12345678";

WiFiServer server(80);
ArduinoLEDMatrix matrix;
String currentMessage = "Hello!";
bool messageUpdated = true;

void setup() {
    Serial.begin(9600);
    matrix.begin();

    // アクセスポイントモードで起動
    Serial.print("Creating access point: ");
    Serial.println(ap_ssid);
    int status = WiFi.beginAP(ap_ssid, ap_pass);
    if (status != WL_AP_LISTENING) {
        Serial.println("Failed to create AP");
        while (true);
    }

    server.begin();
    IPAddress ip = WiFi.localIP();
    Serial.print("AP IP address: ");
    Serial.println(ip);
    Serial.println("Connect to the AP and open http://192.168.4.1");
}

void scrollText(const char* text) {
    matrix.beginDraw();
    matrix.stroke(0xFFFFFFFF);
    matrix.textScrollSpeed(50);
    matrix.textFont(Font_5x7);
    matrix.beginText(matrix.width() - 1, 1, 0xFFFFFF);
    matrix.println(text);
    matrix.endText(SCROLL_LEFT);
    matrix.endDraw();
}

void loop() {
    if (messageUpdated) {
        scrollText(currentMessage.c_str());
        messageUpdated = false;
    }

    WiFiClient client = server.available();
    if (client) {
        String request = "";
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                request += c;
                if (c == '\n' && request.endsWith("\r\n\r\n")) {
                    break;
                }
            }
        }

        // メッセージの取得（GETパラメータから）
        if (request.indexOf("GET /send?msg=") >= 0) {
            int start = request.indexOf("msg=") + 4;
            int end = request.indexOf(" ", start);
            String msg = request.substring(start, end);
            // URLデコード（スペースの+変換）
            msg.replace("+", " ");
            currentMessage = msg;
            messageUpdated = true;
            Serial.print("New message: ");
            Serial.println(currentMessage);
        }

        // HTMLページの送信
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html; charset=UTF-8");
        client.println();
        client.println("<!DOCTYPE html><html><head>");
        client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
        client.println("<style>");
        client.println("body{font-family:sans-serif;text-align:center;padding:20px;}");
        client.println("input[type=text]{font-size:24px;padding:10px;width:80%;}");
        client.println("input[type=submit]{font-size:24px;padding:10px 30px;margin:10px;}");
        client.println("</style></head><body>");
        client.println("<h1>Arduino LED Matrix</h1>");
        client.println("<form action='/send'>");
        client.println("<input type='text' name='msg' placeholder='Message'><br>");
        client.println("<input type='submit' value='Send'>");
        client.println("</form>");
        client.print("<p>Current: ");
        client.print(currentMessage);
        client.println("</p></body></html>");
        client.stop();
    }
}
```

```{note}
**接続方法**

1. スケッチをArduino UNO R4 WiFiに書き込む
2. スマートフォンのWi-Fi設定から「Arduino-LED」に接続する（パスワード: `12345678`）
3. ブラウザで `http://192.168.4.1` を開く
4. テキストボックスにメッセージを入力して「Send」を押す
5. LEDマトリクスにメッセージがスクロール表示される
```

```{warning}
アクセスポイントモードを使用するため，演習室のWi-Fiとは別のネットワークになる．
スマートフォンがArduinoのアクセスポイントに接続している間は，インターネットには接続できない．
また，`ap_ssid`は他の人と重複しないように，自分の名前や班番号などを含めた固有の名前に変更すること（例: `"Arduino-LED-GroupA"`）．
```

(appendix-arduino-ide)=
## 付録: Arduino IDEの使い方

Arduino IDEはArduinoの統合開発環境である．プログラムエディタ，プログラムのコンパイル，マイコンへの焼きこみ，シリアル通信の表示機能などが統合されている．

### 1. ArduinoとPCの接続準備

Arduino UNO R4 WiFiとPCをUSB-Cケーブルで接続する．

**Ubuntuの場合の注意事項:**

`"can't open device "/dev/ttyACM0": Permission denied"`のようなエラーがでる場合は以下を実行する：

```bash
sudo usermod -a -G dialout <user-name>
```

場合によっては再起動する必要がある．

(appendix-board-manager-uno-r4)=
### 2. ボードマネージャでArduino UNO R4のサポートをインストール

Arduino IDEを起動し，以下の手順でArduino UNO R4 WiFiのサポートをインストールする：

1. [ツール] -> [ボード] -> [ボードマネージャ]を開く
2. 「Arduino UNO R4」を検索
3. 「Arduino UNO R4 Boards」をインストール

### 3. 焼きこみ設定

Arduinoマイコンへプログラムを焼きこむための設定：

1. [ツール] -> [ボード] -> [Arduino UNO R4 Boards] -> [Arduino UNO R4 WiFi]
2. [ツール] -> [シリアルポート] -> [/dev/ttyACM0]（または認識されているポート）

### 4. プログラムの検証及び書き込み

1. IDEの[検証]（チェックマークボタン）をクリックするとプログラムのコンパイルが始まる
2. エラーがない場合は「コンパイルが完了しました．」と表示される
3. [マイコンボードへ書き込む]（右矢印ボタン）を押すとマイコンへの書き込みが行われる

```{figure} fig/micon-programming/arduino-uno-r4-wifi-uploading.png
---
width: 400px
name: fig:upload
---
プログラムの書き込み
```

(appendix-serial-monitor)=
### 5. シリアルモニタでのプリント文の確認

[ツール] -> [シリアルモニタ]でシリアルモニタを起動できる．

```{warning}
一つのArduinoと同時にシリアル通信できるPC上のプログラムは一つに限られる．
シリアルモニタを起動していると発展課題6（シリアル通信）で他のプログラム（cuやC言語・Python）からArduinoへ接続できないので，発展課題6に取り組むときはIDEのシリアルモニタを終了する必要がある．
```

### 6. 書き込みや通信ができなくなった場合

書き込みやシリアル通信ができなくなった場合，まずUSBケーブルを抜き差しする．それでも解決しない場合は，まずデバイスファイルの有無を確認する：

```bash
ls /dev/ttyACM*
```

`/dev/ttyACM0` が表示されていれば，OS側はArduinoを認識している．表示されない場合は，ターミナルで以下のコマンドを実行してカーネルログから認識状況を確認する：

```bash
sudo dmesg --follow
```

```{note}
Ubuntu 24.04 以降は `kernel.dmesg_restrict=1` がデフォルトのため，`dmesg` の閲覧には `sudo` が必要である（`adm` グループに所属していれば不要）．`sudo` が使えない場合は `journalctl -kf` でも同じカーネルログをリアルタイムに確認できる．
```

この状態でUSBケーブルを抜き差しし，Arduinoが正しく認識されたログ（`usb 1-1: new full-speed USB device`，`idVendor=2341, idProduct=1002`，`cdc_acm ... ttyACM0`など）が表示されることを確認してから，再度書き込みや通信を試みる．ログが一切出ない場合はUSBケーブル（充電専用ケーブルになっていないか）やUSBハブを疑う．

Ubuntu・macOS・Windowsでの確認方法を含めた詳細は{doc}`tips/arduino-device-detection`を参照．

スケッチがプロセッサをロックしてUSB経由でボードにアクセスできなくなった場合は，電源投入直後にリセットボタンを2回素早く押すことでブートローダーモードに入ることができる．
