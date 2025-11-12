# 3日目 メカトロボットの製作とプログラミング

## 本日の演習内容

本日の演習ではアクチュエータ，センサ，ROSによるデバイス制御を統合したメカトロボット[^1]を学習する．ブレッドボード上にSeeeduino Nanoを用いた電子回路を作成しマイコンプログラミングを行い動作させる．
また作成した電子回路とPC間でのROSを用いた通信方法を学びPCから制御指令を送ることや回路からのセンサ信号処理を行いメカトロボットに対する基礎を身につける．

本演習で用いるサンプルプログラムは<https://github.com/jsk-enshu/robot-programming/tree/master/mechatrobot>においてあり演習環境をセットアップした際に手元にダウンロードされていると思うので参照するとよい．


本日の目標は以下である．

<div class="screen">

- Seeeduino Nanoのマイコンプログラミングを習得する

- rosserialによるSeeeduino NanoとROSの通信が実装できるようになる

- 回路とプログラムの組み合わせのデバッグに慣れる

  ハードウェアとソフトウェアの統合開発ではどちらか一方のときと比較してデバッグが格段に難しくなる．
  プログラムにバグがあると思ってデバッグしていたらハードウェア側にバグが見つかった（その逆も）ということがよくある．

  <span style="color:red">**「見つからないバグは探していないところにある」**</span>

- 電子回路・電子部品の扱いに慣れきらない

  電子部品はPCなどの製品と比較して保護機能が少ないので簡単に壊れる．
  過電圧，過電流，逆電圧，ショート，静電気など一瞬で壊れてしまうため，回路に電源を入れる前に配線を確認することを怠らない．
  （例えば，回路に電源が投入されたままジャンパー線を差し替えない．）
  一瞬の怠惰で回路全体が壊れる可能性がある．

  <span style="color:red">**電源を入れる瞬間は常に緊張することを忘れない**</span>

</div>


## 本演習で作成するメカトロボットの概要

本演習では，ステッピングモータ，超音波センサ，Seeeduino Nano，ROSを組み合わせたメカトロボットを作成する．{numref}`Fig. %s <fig:mechatrobot-overview>` に本日作成するメカトロボットを示す．

:::{figure} fig/mechatrobot.jpg
:align: center
:name: fig:mechatrobot-overview

本演習で取り組むメカトロボットの配線例
:::

## 環境構築とソフトウェア更新

演習を開始する前に，必ず[環境構築](environment-setup.md)のページを参照して，最新バージョンのソフトウェアを取得すること．

# Arduinoを用いた電子回路

## Seeeduino Nano

Arduinoにはデジタル/アナログのIOが複数ポート用意されており各種モータやセンサなどを接続し入出力を制御することができる．またライブラリも充実し世界中にユーザも多いためメカトロ設計制作やIoTデバイスのプラットフォームとしてよく用いられている．

本演習ではArduino Nanoの完全互換品であるSeeeduino Nano[^2]を用いる．Seeeduino NanoはArduino Nanoと同じ機能を提供しながらUSB接続がType-Cに改良されており対称的でリバーシブルな設計となっている．またオンボードのGrove I2Cコネクタを装備しているためGroveエコシステムのセンサやアクチュエータを簡単に接続できる．サイズは43mm×18mmと非常にコンパクトである．

下図にSeeeduino Nanoの外観をとピン配置[^3]を示す．マイコンにはMicrochip社のAVRマイコン`ATmega328P`[^4]が搭載されておりプログラマブルに様々な機能を実装できる．PCとはUSB Type-Cケーブルで接続する．

:::{figure} fig/seeeduino-Nano.jpg
:align: center
:name: fig:seeeduino-nano

Seeeduino Nano
:::

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/seeeduino-Nano-overview.jpg
:align: center
:name: fig:seeeduino-nano-overview

Seeeduino Nanoの外観
:::

:::{grid-item}
:::{figure} fig/arduinonano-pinout.jpg
:align: center
:name: fig:seeeduino-nano-pinout

Seeeduino Nanoのピン配置（Arduino Nano互換）
:::

::::

## 実機を扱う上での注意事項

電子回路の作成と動作確認を行う際は以下の注意事項を必ず守ること．これらを怠ると一瞬で回路全体が壊れる可能性がある．

<div class="warning">

### <span style="color:red">**1. 結線時は必ずUSBを取り外し通電させない！**</span>

配線作業を行う際は<span style="color:red">**必ずSeeeduino NanoのUSBケーブルをPCから抜いて電源を切った状態で行うこと**</span>.通電したまま配線を変更すると一瞬の接触で短絡や過電流が発生しマイコンや周辺回路が破損する．

- **配線前**：USBケーブルを抜く
- **配線作業**：電源が切れた状態で配線を確認しながら作業
- **配線完了後**：配線を再度確認してからUSBケーブルを接続
- **動作確認後**：次の配線変更時は再びUSBケーブルを抜く

### <span style="color:red">**2. 短絡（ショート）させない！**</span>

短絡は電子回路における最も一般的な故障原因である．以下のような接続は絶対に避けること：

**例1: 電源とGNDの短絡**
- 5VとGNDを直接接続してはいけない
- ジャンパー線の誤配線により発生しやすい
- 瞬時に大電流が流れ，マイコンや電源回路が破損する

**例2: I/OピンとGNDの短絡**
- I/Oピンをoutput HIGHに設定した状態でGNDに接続してはいけない
- デジタルピンから大電流が流れ，ピンが破損する
- プログラムと配線の両方を確認すること

**短絡を防ぐための対策：**
- **配線はきれいにわかりやすくまとめる**
- ジャンパー線が絡まないように整理する
- 色分けを活用する（例：赤=5V，黒=GND，その他=信号線）
- ブレッドボードの穴の位置を慎重に確認する
- 配線完了後，別の人にも確認してもらう

### <span style="color:red">**3. 過電圧を加えない！（外部電源使用時）**</span>

外部電源を使用する場合は，**必ず定格電圧を確認すること**.

- Seeeduino Nano：5V動作（USB経由）
- モータドライバ：仕様書で確認（通常5V）
- センサ：仕様書で確認（3.3Vまたは5V）

**外部電源使用時の注意：**
- 電圧を測定してから接続する
- 極性（+/-）を間違えない
- 定格電圧を超える電源を接続しない

</div>

<div class="note">

**配線作業のチェックリスト：**

1. [ ] USBケーブルを抜いた
2. [ ] 5VとGNDが短絡していないか確認した
3. [ ] I/Oピンの接続が正しいか確認した
4. [ ] ジャンパー線が整理されている
5. [ ] プログラムのピン設定と配線が一致している
6. [ ] 電源電圧が正しいか確認した
7. [ ] 別の人に配線を確認してもらった
8. [ ] USB接続前に深呼吸して最終確認した

</div>

**電源投入時の心構え：**

電源を入れる瞬間は常に緊張すること．「大丈夫だろう」という油断が回路破壊につながる．配線ミスがあった場合，<span style="color:red">**回路に電源を入れた瞬間に部品が破損し嫌な臭いがし，二度と使えなくなる[^short]**</span>.

## Arduinoを用いたマイコンプログラミング

次にSeeeduino Nanoにマイコンプログラミングを行いLEDを点滅させる[^5].
Arduinoでは一般的に起動時に一度`setup()`関数が呼ばれその後`loop()`関数が周期的に実行される．
今回のサンプルプログラムでは`setup()`で使用するpinの設定を行い`loop()`でデジタル信号のHigh/Lowを周期的に切り替えている．
これらのプログラムをArduinoマイコンへと書き込み回路を適切に修正することで所望の挙動を実現する．
書き込みには統合開発環境のArduino IDEを用いる．

### ステップ1: 基板上のLEDで書き込み確認

まずプログラムの書き込みが正しく行えることを確認するため基板上のLED（`LED_BUILTIN`）を点滅させる．
`LED_BUILTIN`はArduinoボード上に組み込まれているLEDを指す定義でありSeeeduino NanoやArduino Nanoではデジタルピン13（D13）に接続されている．
`LED_BUILTIN`を使用することで外部にLEDを接続しなくても基板上のLEDで動作確認ができる．

**デバッグの第一歩：**

`LED_BUILTIN`を使うことでまずプログラムを書き込んでちゃんと動作するかというデバッグの一歩が達成できる．プログラムが正しく動作することを確認できればここから少しずつ差分（diff）を追加していけば自分の目標が達成できるだろう．

段階的な開発プロセスとデバッグ方法の詳細については[付録のデバッグ方法](tips/debugging.md)を参照のこと．
`LED_BUILTIN`の詳細については[ArduinoのLED_BUILTINとは](tips/arduino-builtin-led.md)を参照のこと．

<div class="screen">

``` c
/// led_builtin_sample.ino ///

// 初めに一回実行されるsetup関数
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // 基板上のLEDピンをoutputに設定
}

// 毎周期実行されるloop関数
void loop() {
  digitalWrite(LED_BUILTIN, HIGH); // LEDを点灯
  delay(1000); // 1000ms待機
  digitalWrite(LED_BUILTIN, LOW); // LEDを消灯
  delay(1000); // 1000ms待機
}
```

</div>

### <span style="color:green">チェックポイント: 基板上のLEDでの書き込み確認</span>

```{exercise} 基板上のLEDでの書き込み確認
:label: ex_builtin_led

上記のプログラムをSeeeduino Nanoに書き込み基板上のLEDが点滅することを確認しよう．

Arduino IDE上でソースコードが文字化けする場合は[Arduino IDEでのエラー等の対処法](tips/arduino-errors.md)を参照すること．
Arduino IDEを用いたプログラム書き込みの手順は[Arduinoへのプログラム書き込み方法](tips/arduino-upload.md)に記載しているので適宜参照すること．
```

### ステップ2: ブレッドボードでの電子回路作成

書き込みが正しく行えることを確認できたら次はブレッドボード上に外部LEDを点滅させるための回路を製作する．

LED回路を例としてブレッドボード上にSeeeduino Nanoを含む電子回路を作成する．下図はLED点滅回路の回路図でありそれをブレッドボード上に製作したものである．回路図の電源電圧5VとGNDはSeeeduino Nanoの5VとGNDにそれぞれジャンプワイヤを用いて接続する．Seeeduino Nanoのピン配置は[公式ドキュメント](https://wiki.seeedstudio.com/ja/Seeeduino-Nano/)や実物のシルクから確認すること．Seeeduino Nanoへの電源供給はUSB Type-C端子をPCと接続することでなされる．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/led-circuit-diagram.jpg
:align: center
:name: fig:led-circuit-diagram

LED点灯回路図
:::

:::{grid-item}
:::{figure} fig/led-circuit-with-arduino.jpg
:align: center
:name: fig:led-circuit-with-arduino

ブレッドボード上でのSeeeduino Nanoを用いたLED点灯回路の実物実装例
:::

::::

### <span style="color:green">チェックポイント: ブレッドボード上でのLED点滅回路の製作</span>

```{exercise} ブレッドボード上でのLED点滅回路の製作
:label: ex_breadboard_led

LED点滅回路をブレッドボード上に製作しLEDの点滅を確認してみよう．
配線を間違えて短絡（ショート）させないように注意すること．
```

### ステップ3: 外部LEDを使った回路製作のプログラム

**実際の電子回路製作では外部部品を制御することが重要である**.
基板上のLEDだけでなく外部のLED，モータ，センサなどを制御できるようになることがメカトロニクスの基礎となる．

以下のサンプルプログラムではデジタルピン2（D2）を使用して外部LEDを制御する．

<div class="screen">

``` c
/// led_sample.ino ///

// 初めに一回実行されるsetup関数
void setup() {
  pinMode(2, OUTPUT); // D2ピンをoutputに設定
}

// 毎周期実行されるloop関数
void loop() {
  digitalWrite(2, HIGH); // D2ピンをHIGHに切り替え（LEDを点灯）
  delay(1000); // 1000ms待機
  digitalWrite(2, LOW); // D2ピンをLOWに切り替え（LEDを消灯）
  delay(1000); // 1000ms待機
}
```

</div>

### <span style="color:green">チェックポイント: 外部LED点滅回路の作成</span>

```{exercise} 外部LED点滅回路の作成
:label: ex_external_led

Seeeduino Nanoを用いた外部LED点滅回路を作成しLEDの点滅を確認してみよう．

LED点滅のサンプルプログラムは，演習ワークスペース[robot-programming/mechatrobot/sketchbook/led_sample/led_sample.ino](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/sketchbook/led_sample/led_sample.ino)にあるので，書き込みに利用しても良い．

プログラムでは，デジタル信号出力にD2ピンを使用しているため，
実物回路も前のチェックポイント（LED点灯回路）のものから適切に修正する必要がある．
具体的にはLEDのアノード（長い方）側に接続されている抵抗をSeeeduino NanoのD2ピンに接続する．

<span style="color:red">**外部LEDを制御できることは今後のモータやセンサ制御の基礎となるため必ず実施すること**</span>.
```

# Arduinoを用いたメカトロ制御

## アクチュエータ制御

メカトロデバイスのアクチュエータ制御としてSeeeduino Nanoからモータを制御するための回路およびプログラム作成を行う．
配布教材でステッピングモータ（28BYJ48）とモータドライバ基板を配布しているのでこのモータドライバへ制御信号を送るための電子回路およびArduinoプログラムを作成する．

### ステッピングモータとモータドライバ基板の接続

ステッピングモータを制御するには以下のような配線を行う：

**電源の接続：**
1. モータドライバ基板の5V端子をブレッドボードの電源レール（赤線）に接続
2. モータドライバ基板のGND端子をブレッドボードの電源レール（青線）に接続
3. Seeeduino Nanoの5V端子を電源レール（赤線）に接続
4. Seeeduino NanoのGND端子を電源レール（青線）に接続

**制御信号の接続：**
1. モータドライバ基板のIN1をSeeeduino NanoのD5に接続
2. モータドライバ基板のIN2をSeeeduino NanoのD6に接続
3. モータドライバ基板のIN3をSeeeduino NanoのD7に接続
4. モータドライバ基板のIN4をSeeeduino NanoのD8に接続

**モータの接続：**
- ステッピングモータのコネクタをモータドライバ基板のモータ端子に接続

プログラムで設定した信号出力番号（D5-D8）とSeeeduino Nanoのデジタルポートを一致させることが重要である．

### ステッピングモータ制御プログラム

ステッピングモータを駆動するサンプルプログラムを以下に示す．
Arduinoにはステッピングモータを駆動するライブラリとして`Stepper`クラスが用意されているのでこれを用いている．
ステッピングモータを駆動するためには位相のずれたステップ波形を生成する必要があるがこれを`Stepper`クラスが行っている．
興味がある人はこれがどのように実装されているかを知るために[Stepperクラスのソースコード](https://github.com/arduino-libraries/Stepper/blob/master/src/Stepper.h)を見てみると良い．

<div class="screen">

``` c
/// Stepping_motor_sample_28BYJ48.ino ///

#include <Stepper.h>

#define BAUD 9600 // シリアル通信のボーレート
#define MOTOR_PIN1 5 // 使用するモータのpin
#define MOTOR_PIN2 6
#define MOTOR_PIN3 7
#define MOTOR_PIN4 8

// 1回転に必要なステップ数． 360[deg] / 5.625[deg/step] / 2(相励磁) * 64(gear比)
#define STEPS_PER_ROTATE_28BYJ48 2048

const int StepsPerRotate = STEPS_PER_ROTATE_28BYJ48;

// 毎分の回転数(rpm)
int rpm = 5; // 1-15rpmでないと動かない

// モータに与えるステップ数
int Steps = 512; // 90度回転． 360deg : 90deg = 2048 : 512

// ライブラリとモータ配線の整合性を取り， C1, C2を入れ替える
// ref https://github.com/arduino-libraries/Stepper/blob/master/src/Stepper.cpp
Stepper myStepper(StepsPerRotate, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);

void setup() {
  Serial.begin(BAUD);

  // シリアル通信の初期化
  myStepper.setSpeed(rpm); // rpmを設定
}

void loop() {
  // ステッピングモータを正転
  Serial.println("Forward");
  myStepper.step(Steps);
  delay(500);
  Serial.println();
}
```

</div>

同じプログラムが[robot-programming/mechatrobot/sketchbook/stepping_motor_sample_28BYJ48/stepping_motor_sample_28BYJ48.ino](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/sketchbook/stepping_motor_sample_28BYJ48/stepping_motor_sample_28BYJ48.ino)にあるので書き込みに利用しても良い．

実物回路の作成にあたってはプログラムで設定した信号出力番号とSeeeduino Nanoのデジタルポートが一致するように配線する．また電源としてドライバ基板とSeeeduino Nanoの5VとGNDをそれぞれ接続する．

<span style="color:red">**ドライバ基板の5VとGNDを間違えないように**</span>下図を見てよく確認すること！

:::{figure} fig/stepping-motor-driver-connection.jpg
:align: center
:name: fig:stepping-motor-driver-connection

ステッピングモータの配線図
:::

モータドライバ基板の実物回路の例を下図に示す．

:::{figure} fig/stepping-motor-circuit-with-arduino.jpg
:align: center
:name: fig:stepping-motor-circuit-with-arduino

ステッピングモータ駆動回路の配線例
:::

### <span style="color:green">チェックポイント: ステッピングモータの動作確認</span>

```{exercise} ステッピングモータの動作確認
:label: ex_stepper_motor

実際にプログラムを書き込みステッピングモータの動作を確認してみよう．
```

## センサデータ処理

メカトロデバイスのセンサデータ処理としてSeeeduino Nanoでセンサデータを処理するための回路およびプログラム作成を行う．
配布教材で超音波センサ（HC-SR04[^6]）を配布しているので超音波を使って距離の測定[^7]をする電子回路およびArduinoプログラムを作成する．

### 超音波センサの接続

超音波センサ（HC-SR04）をSeeeduino Nanoに接続するには以下のような配線を行う：

**電源の接続：**
1. 超音波センサのVCC端子をブレッドボードの電源レール（赤線）に接続
2. 超音波センサのGND端子をブレッドボードの電源レール（青線）に接続
3. Seeeduino Nanoの5V端子を電源レール（赤線）に接続
4. Seeeduino NanoのGND端子を電源レール（青線）に接続

**信号線の接続：**
1. 超音波センサのTrig端子をSeeeduino NanoのD9に接続
2. 超音波センサのEcho端子をSeeeduino NanoのD10に接続

プログラムで設定した信号番号（D9，D10）とSeeeduino Nanoのデジタルポートを一致させることが重要である．

### 超音波センサ制御プログラム

超音波センサを用いて距離を計測するサンプルプログラムを以下に示す．

1.  トリガ端子を10us以上HIGHにする．

2.  センサが， 40kHzの8つの超音波パルスを送信する．

3.  戻ってきた超音波パルスを受信すると，エコー端子がHIGHとなり，このHIGHとなっていた時間が超音波パルスを送信してから受信するまでの時間となる．

4.  この時間に音速をかけて半分にした数値が距離となる．

センサの使い方は下図のように距離を測定するものとなっておりこの手順に沿ってプログラムを組んでいる．

:::{figure} fig/ultrasonic-sensor-signal.jpg
:align: center
:name: fig:ultrasonic-sensor-signal

超音波センサとの信号の送受信の仕方
:::

プログラム中の`Serial.print()`関数はプログラムの処理結果をprintする関数でありArduino IDEの **「ツール」→「シリアルモニタ」** で確認できる．ボーレートを正しく設定すること．

また，Arduino IDEには **「ツール」→「シリアルプロッタ」** という機能もあり，センサの値をリアルタイムでグラフ表示できる．超音波センサの距離データを可視化する際に便利である．

<div class="screen">

``` c
/// ultrasonic_sensor_sample.ino ///

// HC-SR04 Ultrasonic sensor
// https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380

#define TRIG_PIN 9
#define ECHO_PIN 10

float duration, distance;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  // calculate distance
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  if(duration>0) {
    distance = (duration*.0343)/2; // ultrasonic speed is 340m/s = 0.034cm/us
    Serial.print(duration);
    Serial.print(" us ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  delay(200);

  if(distance > 6) {
    Serial.println("outside");
    delay(100);
  } else if (distance <= 6) {
    Serial.println("inside");
    delay(100);
  }

  Serial.println("");
}
```

</div>

同じプログラムが[robot-programming/mechatrobot/sketchbook/ultrasonic_sensor_sample/ultrasonic_sensor_sample.ino](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/sketchbook/ultrasonic_sensor_sample/ultrasonic_sensor_sample.ino)にあるので書き込みに利用しても良い．

実物回路の作成にあたってはこれまで同様プログラムで設定した信号出力番号とSeeeduino Nanoのデジタルポートを一致させ電源をSeeeduino Nanoから取るように配線する．
次に超音波センサをに作成した実物回路の例を示す．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/ultrasonic-sensor.jpg
:align: center
:name: fig:ultrasonic-sensor

超音波センサ
:::

:::{grid-item}
:::{figure} fig/ultrasonic-sensor-circuit-with-arduino.jpg
:align: center
:name: fig:ultrasonic-sensor-circuit

超音波センサ回路の配線例
:::

::::

プログラムを書き込んだ後，Arduino IDEのシリアルモニタやシリアルプロッタで超音波センサの動作を確認できる．以下にそれぞれの表示例を示す．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/arduino-range-sensor-serial-print.png
:align: center
:name: fig:arduino-range-sensor-serial-print

シリアルモニタでの超音波センサ出力例．距離データと判定結果がテキストで表示される．
:::

:::{grid-item}
:::{figure} fig/arduino-range-sensor-serial-plotter.png
:align: center
:name: fig:arduino-range-sensor-serial-plotter

シリアルプロッタでの超音波センサ出力例．距離データがリアルタイムでグラフ表示される．
:::

::::

# ROSを用いたメカトロボットのプログラミング

この章では前章までのSeeeduino Nanoを用いたアクチュエータ制御，センサ処理といったメカトロプログラミングにROSによるロボットシステムを統合したメカトロボットのプログラミングに取り組む．
これまでの演習課題ではプログラム処理が電子回路のマイコン内で閉じていたのに対しこの章ではマイコンとPCとで通信し連携させることを学ぶ．
これによってPCからデバイスに制御指令を送ることやデバイスのセンサ信号をPCで受け取り処理するといったプログラミングが可能となり高いレイヤでの比較的複雑で知的な処理を行えるようになる．

## ArduinoでのROSによるプログラミングを行う環境構築

以下の手順で`rosserial_arduino`をArduinoで使うための環境構築を行う．

```{code-block} console
$ source /opt/ros/one/setup.bash
$ source ~/ros_ws/devel/setup.bash
$ mkdir -p ~/Arduino/libraries/
$ cd ~/Arduino/libraries/
$ rm -rf ros_lib
$ rosrun rosserial_arduino make_libraries.py ./
# 最後の．/(ドットスラッシュ)はカレントディレクトリを意味する．ライブラリのコピー先として現在のディレクトリを指定するために．(ドット)も忘れないように入力する必要がある．
```

**micro-ROSについて[^rosserial_microros]**

## 本演習で作成するメカトロボット

本演習ではステッピングモータ，超音波センサ，Seeeduino Nano，ROSを組み合わせたメカトロボットに触れる．
{numref}`図． %s <fig:mechatrobot-middle>` は本演習で作成するメカトロボットの配線例である．

:::{figure} fig/mechatrobot.jpg
:align: center
:name: fig:mechatrobot-middle

本演習で取り組むメカトロボットの配線例
:::

まずは動作確認のため簡単なLチカプログラムのrosserial版をSeeeduino Nanoに書き込んで動作を確認する．

### rosserialを使ったLEDの点滅プログラム

rosserialの動作確認として簡単なLチカプログラムを試してみよう．
このプログラムはLEDを1秒ごとに点滅させながらLEDの状態をROSのtopicとして配信する．

以下のスケッチはGitHubリポジトリのサンプルプログラムとして提供されている．

[robot-programming/mechatrobot/sketchbook/led_sample_rosserial/led_sample_rosserial.ino](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/sketchbook/led_sample_rosserial/led_sample_rosserial.ino)

<div class="screen">

```cpp
#include <ros.h>
#include <std_msgs/Bool.h>

// ROS node handle
ros::NodeHandle nh;

// LED state message
std_msgs::Bool led_state_msg;

// Publisher for LED state
ros::Publisher led_state_pub("led/state", &led_state_msg);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize ROS
  nh.initNode();
  nh.advertise(led_state_pub);

  // Wait for connection
  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
}

void loop() {
  // Turn LED ON
  digitalWrite(LED_BUILTIN, HIGH);
  led_state_msg.data = true;
  led_state_pub.publish(&led_state_msg);
  nh.spinOnce();
  delay(1000);

  // Turn LED OFF
  digitalWrite(LED_BUILTIN, LOW);
  led_state_msg.data = false;
  led_state_pub.publish(&led_state_msg);
  nh.spinOnce();
  delay(1000);
}
```

</div>

このプログラムを動かすには以下の手順で実行する．

```{code-block} console
# ターミナル1: roscoreを起動（既に起動している場合は不要）
$ source /opt/ros/one/setup.bash
$ roscore
```

```{code-block} console
# ターミナル2: rosserial_pythonを起動してSeeeduino Nanoと通信
$ source /opt/ros/one/setup.bash
$ rosrun rosserial_python serial_node.py /dev/ttyUSB0
# /dev/ttyUSB0はSeeeduino Nanoのデバイスファイル名．
# 環境によって異なる場合があるのでls /dev/ttyUSB*で確認する．
# デバイスが見つからないなどのエラーが発生した場合は
# 付録のrosserialトラブルシューティングを参照．
```

**注意：** `could not open port /dev/ttyUSB0`などのエラーが表示される場合は[付録のrosserialトラブルシューティング](tips/rosserial-troubleshooting.md)を参照してデバイスファイル名やボーレートを確認すること．

`rosserial_python`の接続が成功すると以下のようなメッセージが表示される．

<div class="screen">

```
[INFO] [1762030748.557237]: ROS Serial Python Node
[INFO] [1762030748.563965]: Connecting to /dev/ttyUSB0 at 57600 baud
[INFO] [1762030750.668005]: Requesting topics...
[INFO] [1762030750.759299]: Note: publish buffer size is 280 bytes
[INFO] [1762030750.761196]: Setup publisher on led/state [std_msgs/Bool]
```

</div>

`Setup publisher on led/state`と表示されればArduinoとROSの通信が確立されている．

```{code-block} console
# ターミナル3: LEDの状態を確認
$ source /opt/ros/one/setup.bash
$ rostopic echo /led/state
```

`rostopic echo /led/state`を実行するとLEDの点灯状態が1秒ごとに切り替わるのに合わせて以下のように`true`と`false`が交互に表示される．

```
data: True
---
data: False
---
data: True
---
data: False
---
```

実際にSeeeduino Nanoの基板上のLED（`LED_BUILTIN`）が点滅しているのを確認しながらROSのtopicでその状態が配信されていることを確認しよう．

### <span style="color:green">チェックポイント: rosserialを使ったLED点滅プログラム</span>

```{exercise} rosserialを使ったLED点滅プログラム
:label: ex_rosserial_led

rosserialを使ったLEDの点滅プログラムを実行してみよう．

1. Arduino IDEで[robot-programming/mechatrobot/sketchbook/led_sample_rosserial/led_sample_rosserial.ino](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/sketchbook/led_sample_rosserial/led_sample_rosserial.ino)を開きSeeeduino Nanoに書き込む．

2. `roscore`，`rosserial_python`，`rostopic echo`を順に起動する．

3. 基板上のLED（`LED_BUILTIN`）が点滅していることとtopicで状態が配信されていることを確認する．
```

### メカトロボットのプログラムの書き込み

rosserialの動作確認ができたら次はメカトロボットを制御するプログラムをSeeeduino Nanoに書き込む．プログラムは[robot-programming/mechatrobot/sketchbook/MechatrobotDriver/MechatrobotDriver.ino](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/sketchbook/MechatrobotDriver/MechatrobotDriver.ino)にある．

次にスケッチのピン配置を参考にブレッドボードを配線する．ステッピングモータと超音波センサへ電源供給の配線をすることに加えSeeeduino Nanoのデジタルピンの内ステップ波形を生成する信号線4本とセンサ処理を行う2本をそれぞれ接続する．

### ROS 1とROS 2の統合システム

本演習のメカトロボットでは**rosserial（ROS 1）**でArduinoと通信し**ros2_control（ROS 2）**でモータ制御を行う．ROS 1とROS 2を連携させるために**ros1_bridge**を使用してtopicをブリッジする．

システム構成は以下のようになる：

- **ROS 1側**：rosserial経由でSeeeduino Nanoと通信
- **ROS 2側**：ros2_controlでモータ制御とrvizで可視化
- **ros1_bridge**：ROS 1とROS 2のtopicを相互に変換

:::{figure} fig/mechatrobot-system.png
:align: center
:name: fig:mechatrobot-system

メカトロボットのシステム構成図．ROS 1側でrosserialを介してSeeeduino Nanoと通信しROS 2側でros2_controlによるモータ制御とrviz2による可視化を行う．ros1_bridgeがROS 1とROS 2のtopicを相互に変換する．
:::

実機の用意ができたら以下の5つのプログラムを順に起動する．

**注意：** ros1_bridgeを使用するには事前にインストールが必要である．[付録のROS 1 Bridge](tips/ros1-bridge.md)を参照してインストールすること．

**ターミナル1: roscore（ROS 1マスターノード）**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ roscore
```

**ターミナル2: ros1_bridge（ROS 1とROS 2のブリッジ）**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2/bridge/install/setup.bash
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

**ターミナル3: rosserial（Seeeduino Nanoとの通信）**

```{code-block} console
$ source ~/ros_ws/devel/setup.bash
$ roslaunch mechatrobot mechatrobot_driver.launch
# could not open port /dev/ttyUSB0などと表示される場合は
# Seeeduino NanoのUSBケーブルを抜いたり差したりした状態の/dev/ttyUSB???のデバイスファイル名の存在を比較し
# Seeeduino Nanoのデバイスファイル名を調べてport:=/dev/ttyUSB???
# のようにroslaunchのオプション引数で適切なデバイスファイル名を指定する．
```

**ターミナル4: ros2_control（モータ制御）**

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch mechatrobot_ros2 mechatrobot_controller.launch.py
```

**ターミナル5: rvizとGUI（可視化）**

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch mechatrobot_ros2 mechatrobot_display.launch.py
```

それぞれのプログラムの概要を以下に示す．より詳しく知りたい場合はファイルの中身を追っていくとよい．
プログラムを正しく起動するとrvizにのような表示がされる．

- **roscore**
  ROS 1のマスターノード．ROS 1のノード間通信を管理する．

- **ros1_bridge**
  ROS 1とROS 2の間でtopicをブリッジする．`--bridge-all-topics`オプションにより全てのtopicが自動的に変換される．
  詳細は[付録のROS 1 Bridge](tips/ros1-bridge.md)を参照．

- **mechatrobot_driver.launch**
  rosserial[^8]を起動してメカトロボット（Seeeduino Nano）とPCとの通信を行う（ROS 1）．
  通信はrostopicを介して行いArduino内で走っている`loop()`関数かあるいはPC側でのプログラムでpub/subされたtopicをやりとりする．

- **mechatrobot_controller.launch.py**
  ros2_control[^9]を用いてモータ制御指令を生成する（ROS 2）．controllerとして位置軌道を生成する`position_trajectory_controller`を使用している．

- **mechatrobot_display.launch.py**
  ROS 2可視化ツールのrviz2[^10]，制御GUIを起動する．
  モータ角度は`/motor1/command`，超音波センサデータは`/range`のtopicでpublishしておりrvizで可視化すると確認しやすい．


::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/mechatrobot-rviz.jpg
:align: center
:name: fig:mechatrobot-rviz

メカトロボットのrvizでの表示
:::

:::{grid-item}
:::{figure} fig/mechatrobot-rviz-with-tf.jpg
:align: center
:name: fig:mechatrobot-rviz-with-tf

tfを表示すると，ロボットモデルのlink, joint等の位置関係が可視化されわかりやすい場合もある．
:::

::::

## ロボットモデルの構成

ロボットモデルは[mechatrobot/urdf/robot.urdf](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/urdf/robot.urdf)に定義されている．
ロボットモデルは`link`と`joint`の連結で表されており以下に記述の一部を紹介する．

`link`部では`link`名や幾何情報が記述されている．
幾何情報は既に用意されたプリミティブ形状の他自身で作成した3Dデータを読み込ませることも可能である．
`origin`には`link`の3dモデル原点からの変位(`xyz`)，回転(`rpy`)の情報を記入しここがrvizにおける`link`原点となる．

<div class="screen">

``` xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.053 0.082 0.01"/>
    </geometry>
  </visual>
</link>
```

</div>

`joint`部では`joint`名，`joint`の種類，親と子に相当する`link`名や`joint`の基準位置を記述する．
`origin`には親`link`の原点からの変位(`xyz`)，回転(`rpy`)の情報を記入しここがrvizにおける`joint`原点となる．

<div class="screen">

``` xml
<joint name="base_to_motor" type="fixed">
  <parent link="base_link" />
  <child link="motor_link" />
  <origin xyz="0 0.08 0" rpy="0 0 0" />
</joint>
```

</div>

## アクチュエータの制御: GUIによる操作インターフェース

ROSにはGUIでロボットを操作するインターフェースが用意されており本演習では`rqt_joint_trajectory_controller`を取り上げる．

下図は`rqt_joint_trajectory_controller`のGUIで起動した後はoffの状態である．
GUI中で`/controller_manager`と`positioin_trajectory_controller`を選択し電源ボタン(赤)をクリックするとcontrollerは電源ボタン(緑)のonとなりGUIからロボットを操作出来るようになる．
`joint1`のバーと数値は角度\[rad\]を示しており左右に動かすことでrvizおよび実物のモータが動くので試してみてほしい．
正しく起動できていればrviz上に表示されているモータ回転軸に相当する座標系が回転するのが確認できる．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/mechatrobot-rqt-off.jpg
:align: center
:name: fig:rqt_joint_trajectory_controller-off

rqt_joint_trajectory_controller (OFFの状態)
:::

:::{grid-item}
:::{figure} fig/mechatrobot-rqt-on.jpg
:align: center
:name: fig:rqt_joint_trajectory_controller-on

rqt_joint_trajectory_controller (ONの状態)
:::

::::

## センサデータの可視化

超音波センサの認識領域は`/range`の円錐形で可視化している．
前方物体の距離に応じて円錐が変形するので超音波センサの前に手をかざしてみたり実際に定規で距離を確認してみるとよい．

:::{figure} fig/range-sensor-rviz.png
:align: center
:name: fig:range-sensor-visualized

rvizで測距センサを可視化している様子
:::

### <span style="color:green">チェックポイント: メカトロボットの動作確認</span>

```{exercise} メカトロボットの動作確認
:label: ex_mechatrobot

メカトロボットの動作確認を行ってみよう．

1.  `rviz`にメカトロボットが表示されているか．

2.  `rqt_joint_trajectory_controller`でモータ角度を制御すると実機および`rviz`は動作するか．

3.  超音波センサの`/range`が`rviz`で可視化されており前方物体の距離認識はできているか．
```

## 画像処理との連携

PCと通信することでPCでのプログラム処理結果をロボット制御に反映させることが出来る．
ここではPC側で顔認識の画像処理を行い認識結果に基づいてモータ制御を行う方法を説明する．

以下を実行する[^motor_command_batching].

```{code-block} console
$ roslaunch mechatrobot mechatrobot_driver.launch
$ roslaunch mechatrobot sample_face_detect.launch

# euslisp sample
$ rosrun mechatrobot motor-command-by-face.l

# あるいは python sample
$ rosrun mechatrobot motor-command-by-face.py
```

- [sample_face_detect.launch](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/launch/sample_face_detect.launch)
  PCのインカメラの画像に対して顔認識を行うサンプルプログラム．
  {numref}`Fig. %s <fig:face-detection>` は認識結果の一例である．

- [motor-command-by-face.l](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/euslisp/motor-command-by-face.l), [motor-command-by-face.py](https://github.com/jsk-enshu/robot-programming/tree/master/mechatrobot/scripts/motor-command-by-face.py)
  顔認識結果を受け， 顔の位置が画面の左であればモータに左回転指令，
  右であれば右回転指令を送る．

:::{figure} fig/face-detection.jpg
:align: center
:name: fig:face-detection

顔認識結果
:::

### <span style="color:green">チェックポイント: 顔認識を利用したモータ制御</span>

```{exercise} 顔認識を利用したモータ制御
:label: ex_face_motor

上記プログラムを実行して顔認識の画像処理結果を利用してモータ制御してみよう．
```

# 本日の演習課題

## 課題0(発展)

さきほどの顔認識のプログラムはROS 1からのコントロールする方法であった．ROS 2から直接コントロールするプログラムを書いてみよ．

## 課題1(発展)

本演習ではICS変換基板を介してSeeeduino NanoからKRS（近藤サーボ）に指令を送る．[付録の近藤サーボ制御](tips/kondo-servo.md)を参考にしてSeeeduino Nanoのスケッチで実装したプログラムから近藤サーボモータを動かしたり角度を取得したりしてみよう．

**実装内容の例：**
- サーボを指定した位置に移動させる
- サーボの現在位置を読み取る
- サーボをフリー状態にする

<div class="screen">

1.  [付録の近藤サーボ制御](tips/kondo-servo.md)を参考にICS Library for Arduinoをインストールする．

2.  サーボのID番号を確認しプログラム内で正しいID番号を指定する．

3.  `ics.setPos(id, position)`でサーボの位置を制御する（位置範囲：3500〜11500）．

4.  `ics.getPos(id)`でサーボの現在位置を取得する．

</div>

## 課題2(発展)

（課題1を含むので課題1ができてから挑戦することをお勧めする．）課題1ではSeeeduino Nano基板から近藤サーボモータへの制御指令を送った．PC・Seeeduino Nano基板間でROSシリアル通信を行うことでPCから`topic`をpub/subしてSeeeduino Nanoに接続した近藤サーボモータを制御してみよう．

PC上での`topic`のpub/subによりSeeeduino Nanoを介して接続された近藤サーボモータの制御・状態確認を行うプログラムを`rosserial`を使って実装してみよう．

**実装内容の例：**
- PCで`topic`を`publish`してサーボの目標回転角度を送る
- PCで`topic`を`subscribe`してサーボの回転角度を取得する

<div class="screen">

1.  [付録の近藤サーボ制御](tips/kondo-servo.md)を参考にして課題1のスケッチに`topic`をpub/subするROSノードを追加すればよい．

2.  `rosserial_arduino`ライブラリを使用して`std_msgs::Int16`型のメッセージでサーボ位置を送受信する．

3.  `Serial`は`rosserial`通信に，`Serial1`はICS通信に使用する点に注意する．

4.  `rosrun rosserial_python serial_node.py`でROSシリアルノードを起動し`/servo_command`や`/servo_position`の`topic`を送受信する．

</div>

## 課題3(発展)

近藤サーボモータとSeeeduino Nano間のICS通信を行うプログラムではICS Library for Arduinoを使用している．[付録の近藤サーボ制御](tips/kondo-servo.md)を参考にしてサーボに新しい機能を追加しPCからコマンドを送ってみよう．

**実装内容の例：**
- 複数のサーボを同時に制御する機能を追加する
- サーボのトルクON/OFF機能を実装する
- サーボの速度を制御する機能を追加する

<div class="screen">

まずはSeeeduino Nano上のプログラムから新しい機能を実装して動作を試してから`rosserial`を通じてPCから動かすとよい．

</div>

## 課題4(発展)

メカトロボットにおいて顔認識と超音波センサの両方の結果を用いてステッピングモータを制御するプログラムおよび電子回路を作成し動作確認してみよう．

例．
超音波センサからの距離が3cm以内になればモータが左回転し顔認識すればモータが右回転する．

## 課題5(発展)

今回のメカトロボットの末端リンクを自分で作成した3Dモデルに変更し，自分オリジナルのメカトロボットを作成してみよう．
モデルを変更した後， モータへ角度指令を送ると，
rviz上で3Dモデルが回転することが確認できる．
rvizをスクリーンキャプチャして提出すること．

<div class="screen">


1.  [mechatrobot/urdf/robot.urdf](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/urdf/robot.urdf)を見ると`link1`の表示モデルのところで`sample-3d-model.stl`をコメントアウトしている．
    boxモデルの代わりにこちらをコメントインすると3dモデルを変更することが出来る．
    urdfを変更したら`mechatrobot_controller.launch.py`と`mechatrobot_display.launch.py`を再起動することに注意．

</div>

## 課題6(発展)

今回の演習ではステッピングモータと超音波センサ，顔認識を例としたメカトロボットのプログラミングを紹介したがそれ以外の組み合わせでの処理をプログラミングし動作確認してみよう．例えばステッピングモータを配布教材で配っているサーボモータやDCモータに置きかえることや自分でセンサを持っていればそれをSeeeduino Nanoから処理できるようにプログラミングしてみること，画像処理として顔認識以外の処理を試してみることなど．

## 課題7(発展)

冬学期演習「ロボット制御とシミュレーション」で説明した通りアームロボットの制御基板（DXHUB）を介してPCからDynamixelサーボモータに指令を送っているがDynamixelモータはシリアル通信で制御されているためDXHUBの代わりにSeeeduino Nano基板から制御指令を送ることもできる．本演習では近藤サーボを使用しているがこの課題ではDynamixelサーボモータを扱う．[付録のDynamixel制御](tips/dynamixel.md)を参考にして課題1〜3で近藤サーボに対して行った課題をDynamixelサーボモータでも実装してみよう．

**実装内容：**

1. **課題1のDynamixel版**：Seeeduino Nanoのスケッチで実装したプログラムからDynamixelサーボモータを動かしたり角度を取得したりする

2. **課題2のDynamixel版**：PC・Seeeduino Nano基板間でROSシリアル通信を行いPCから`topic`をpub/subしてSeeeduino Nanoに接続したDynamixelモータを制御する

3. **課題3のDynamixel版**：Dynamixelサーボに未実装のコマンドを送る関数を追加してPCからコマンドを送る（例：DynamixelについているLEDを点灯させる）

<div class="screen">

1.  [付録のDynamixel制御](tips/dynamixel.md)を参考にDynamixel通信ライブラリの使い方を確認する．

2.  [robot-programming/mechatrobot/sketchbook/dynamixel_motor_sample/](https://github.com/jsk-enshu/robot-programming/tree/master/mechatrobot/sketchbook/dynamixel_motor_sample/)のサンプルコードを参考にする．

3.  [robot-programming/mechatrobot/sketchbook/ros_dynamixel_motor_sample/ros_dynamixel_motor_sample.ino](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot/sketchbook/ros_dynamixel_motor_sample/ros_dynamixel_motor_sample.ino)でROSシリアル通信の実装例を確認する．

4.  近藤サーボとDynamixelの通信プロトコルの違いを理解しそれぞれの特徴を比較してみる．

</div>

## 課題8(発展)

1,2日目のGazebo環境に今回のメカトロボットを表示させ動作シミュレーションしてみよう．
3Dモデルやロボット構成は自由に作ってみるとよい．

## 課題9(発展)

ESP32やSTM32, raspberry pi picoなどのマイコンを持っている場合は今回の課題をそれらのマイコンを使って`rosserial`で通信してみよう．

## 課題10(発展)

課題9では`rosserial`で行ったが，micro-rosを使ってROS 2のみで全体の通信が完結するように行ってみよ．

[^1]: メカトロニクスとロボティクスを組み合わせた造語で，機械・電子・制御・情報などの要素技術を統合したロボットシステムを指す．

[^2]: Seeeduino Nanoは，Seeed Studio社が開発したArduino Nano完全互換のマイクロコントローラーボードである．詳細は[公式ページ](https://wiki.seeedstudio.com/ja/Seeeduino-Nano/)を参照のこと．

[^3]: ピン配置の詳細は，実物のシルク印刷や[公式ドキュメント](https://wiki.seeedstudio.com/ja/Seeeduino-Nano/)で確認できる．

[^4]: `ATmega328P`は，Microchip社（旧Atmel社）の8ビットAVRマイクロコントローラーである．32KBのフラッシュメモリ，2KBのSRAM，16MHzのクロック周波数を持つ．

[^5]: Arduinoのプログラミングについては，[Arduino公式チュートリアル](https://www.arduino.cc/en/Tutorial/HomePage)を参照すると良い．

[^6]: HC-SR04は，超音波を用いた距離測定センサである．測定範囲は約2cm〜400cmで，精度は約3mmである．

[^7]: 超音波センサの原理は，超音波の発射から反射波の受信までの時間を測定し，音速（約340m/s）から距離を計算する．

[^8]: rosserialは，ArduinoなどのマイクロコントローラーとROSを通信させるためのプロトコルおよびライブラリである．詳細は[rosserial Wiki](http://wiki.ros.org/rosserial)を参照のこと．

[^9]: ros_controlは，ROSでロボットのコントローラーを実装するためのフレームワークである．詳細は[ros_control Wiki](http://wiki.ros.org/ros_control)を参照のこと．またros2_controlは[ros2_control](https://control.ros.org/rolling/index.html)を参照すること．

[^10]: rvizは，ROSの3次元可視化ツールである．センサデータやロボットの状態を可視化できる．

[^11]: rqt_joint_trajectory_controllerは，関節軌道制御のためのGUIツールである．

[^short]: この嫌なにおいは回路基板や部品に使われている「プラスチック」や「樹脂」，「化学物質」が，ショートによる高熱で燃えたり溶けたりしたものである．当然体には悪い．

[^rosserial_microros]: 本演習ではROS 1のrosserialを使用しているが，ROS 2ネイティブの通信を行うmicro-ROSという選択肢もある．micro-ROSはリソース制約のあるマイクロコントローラー向けに最適化されたROS 2の実装であり，ROS 2のDDS通信を直接使用できるため，rosserialのようなブリッジが不要でありより効率的な通信が可能である．しかしながら，本演習で配布しているSeeeduino Nano（ATmega328P搭載）は32KBのフラッシュメモリと2KBのSRAMしか持たないため，micro-ROSを動作させるには非力である．micro-ROSを快適に動作させるにはESP32（520KB SRAM）やSTM32（数百KBのRAM）などのより高性能なマイクロコントローラーが望ましい．もし本演習でmicro-ROSを使用したメカトロボットのプログラミングがしたかったなどがあれば，学期末のアンケートでその旨を記載してほしい．皆さんからのフィードバックが多ければ，来年度以降の演習ではESP32などのより高性能なマイコンを使用したmicro-ROSベースの演習内容に更新される可能性があり，それは後輩たちのためになるだろう．

[^motor_command_batching]: 改めて起動し直すのは，mechatrobot_controller.launchが既に起動しているとモータ角度指令topicである`/motor1/command`がバッティングしてしまうためである．
