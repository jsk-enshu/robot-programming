# 近藤サーボ（KRS）の制御

## 概要

近藤科学社の**KRS（KONDO ROBOT SERVO）シリーズ**は，ICS（Interactive Communication System）通信プロトコルを使用したシリアル通信制御可能なサーボモータである．本演習ではICS変換基板とArduino（Seeeduino Nano）を使用して近藤サーボを制御する．

### ICS（Interactive Communication System）について

ICSは，モジュール・コントロールボード間の双方向データ通信規格である．コントロールボードでのサーボの制御時の通信や，PCなどを使用してのサーボモータの設定変更などが可能である．コマンドリファレンスが公開されているため，自作ボードでの制御にも対応している．

**ICS3.5/3.6の主な特徴：**

- **最大1.25Mbpsの高速通信**が可能
- **「スピード」や「ストレッチ」のほか，「温度制限」や「電流制限」などサーボモータの様々な特性を動作中に任意に変更可能**
- **マルチドロップ接続では最大32個接続可能**（モジュール，コントロールボードの能力によって最大数が変わる）
- **ICS3.5では「シリアル」⇔「PWM」の切り替えが可能**
- **ICS3.6はICS3.5の上位互換**で，現在値を取得するコマンドが追加されている

**ICS3.5/3.6の主な機能一覧：**

| 機能名 | 説明 |
|:-------|:-----|
| スピード | サーボの最大出力（デューティー比）を設定．軸の回転速度が変化する |
| ストレッチ | 軸の保持特性を設定 |
| パンチ | サーボの初動を調整 |
| デッドバンド | サーボのニュートラル帯域（不感帯）を調整 |
| プロテクション | サーボがロックしたときに脱力するまでの時間を設定 |
| リミッタ | 正転・逆転の移動範囲を制限 |
| 温度制限 | 設定した温度を超えるとサーボが脱力状態になる |
| 電流制限 | 設定した電流を超えるとサーボが脱力状態になる |
| リバース | サーボが指定した方向とは逆方向に移動する |
| スレーブ | サーボが返事を返さなくなる |
| 回転モード | サーボの軸が指定の方向に回転する |
| レスポンス | 出力軸の動作開始時の立ち上がり特性を設定 |
| ダンピング | 出力軸の動作停止時のブレーキ特性を設定 |
| ユーザーオフセット | 出力軸の初期位置をユーザーが任意に設定可能 |
| シリアル専用 | シリアル信号での制御専用のモジュールとして設定 |
| 現在値（ICS3.6のみ） | サーボの現在値を取得 |

## 必要な機器

### 基本構成

| 機器名 | 説明 | 備考 |
|:-------|:-----|:-----|
| ICS変換基板 | UARTをICS通信に変換する基板 | 近藤科学製 |
| Arduino（Seeeduino Nano） | マイコンボード | Arduino Nano完全互換 |
| KRSサーボモータ | ICS対応サーボモータ | KRS-2552HV等 |
| バッテリー | サーボ用の電源 |  |
| Dual USBアダプター（近藤科学製） | ICS通信をPCから行うためのもの||

## ハードウェアのセットアップ

### 1. サーボモータのID設定

近藤サーボは出荷時にID番号が全て**0**に設定されている．複数のサーボを使用する場合は，事前に各サーボに異なるID番号を設定する必要がある．
注意として演習で使用するサーボはすでにIDが振られている可能性があるためIDを確認すること．

(ubuntu-ics-manager)=
#### 方法1: Ubuntuでics-managerを使用する場合

**事前準備：**

1. udevルールの設定とユーザー権限の追加：

```bash
curl -fsSL https://raw.githubusercontent.com/iory/rcb4/main/rcb4/assets/system/99-rcb4-udev.rules | sudo tee /etc/udev/rules.d/99-rcb4-udev.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

2. Dual USBアダプターを接続する前に，スライドスイッチが**ICS側**を向くようにする

3. USBを接続したときに赤色のLEDが点灯することを確認する

```{figure} ../fig/krs-dual-usb-connection.jpg
:name: krs-dual-usb-connection
:align: center

Dual USBアダプターとサーボの接続例（スライドスイッチをICS側に設定）
```

**Python仮想環境の準備：**

```bash
sudo apt install python3-venv
mkdir ~/rcb4-venv
python3 -m venv ~/rcb4-venv/
source ~/rcb4-venv/bin/activate
```

**rcb4パッケージのインストール：**

```bash
pip install rcb4 pyyaml
```

**ID設定手順：**

1. Dual USBアダプターとサーボをPCに接続した状態で以下のコマンドを実行(末尾の数字でサーボの通信速度設定が上書きされるため、arduinoから動かすサーボの場合は115200, jedyに組み込むサーボの場合は1250000とする)：

```bash
ics-manager --baudrate 1250000
```

2. 矢印キーで操作して適宜，値を設定する．ここでは他のモジュールと重複しないようにIDを書き換える

3. **Current Servo ID**が選択されている状態で，矢印キー（←→）で設定したいID番号（緑色でハイライト表示）を選択する

4. **Enterキー**を押すとIDの書き込みが行われる

**ics-managerの操作画面例：**

```
--- Servo Status ---
>>  Current Servo ID: 0 -> Next Servo ID: [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 ]
    Angle: -120
    Speed: 127
    Stretch: 127
    Baud Rate: 1.25Mbps
    Rotation Mode: Disabled
    Slave Mode: Disabled
    Reverse Mode: Disabled
    Serial Mode: Disabled
    Free: Enabled
----------------------

Use ↑↓ to navigate, ←→ to adjust Current Servo ID or Servo Angles
Press 'Enter' when Current Servo ID is selected to save the currently highlighted ID in green.
Press 'z' to reset servo position
Press 'r' to toggle rotation mode (enables continuous wheel-like rotation)
Press 'f' to set free mode
Press 'd' to set default EEPROM parameters (WARNING: This action will overwrite the servo's EEPROM).

'q' to quit.
```

#### 方法2: シリアルマネージャーソフトを使用する場合（Windows）

**設定手順：**

1. シリアルマネージャーソフトをPCにインストールする[^1]

2. Dual USBアダプターにサーボを1つだけ接続する

3. Dual USBアダプターをPCのUSBポートに接続する

4. シリアルマネージャーソフトを起動し，サーボのIDを検索する

5. 検出されたサーボ（ID:0）を選択し，新しいID番号（1，2，3...）に変更する

6. 設定を保存する

7. 全てのサーボについて，上記の手順を繰り返す

### 2. サーボモータの接続と配線

サーボモータをICS変換基板に接続する際は以下の点に注意する．

- **コネクタの向き**：3本線のコネクタの爪（ツメ）がVH 2pin（白いコネクタ）側を向くように接続する
- **複数接続**：ICS変換基板には複数のサーボ接続端子がありデイジーチェーン接続が可能である

```{figure} ../fig/ics-board-servo-connection.jpg
:name: ics-board-connection
:align: center

ICS変換基板とサーボの接続例（コネクタの爪をVH 2pin（白いコネクタ）側に向ける）
```

## ソフトウェアのセットアップ

### ICS Library for Arduino のインストール

近藤科学が提供する**ICS Library for Arduino**を使用してサーボを制御する．

**インストール手順：**

1. 近藤科学のWebサイトからICS Library for Arduinoをダウンロードする[^2]
   - 例：[ICS_Library_for_Arduino_V2_12.zip](https://kondo-robot.com/w/wp-content/uploads/ICS_Library_for_Arduino_V2_12.zip)

2. ダウンロードしたZIPファイルを解凍する

3. 解凍したフォルダを`~/Arduino/libraries`ディレクトリに配置する

4. 正しくインストールされると，以下のようなディレクトリ構成になる：

```bash
~/Arduino/libraries$ ls
IcsClass_V210  IcsSoftSerialClass_V210  KoCustomSoftSerial  ros_lib
```

**別の方法：Arduino IDEからインストール**

1. Arduino IDEを起動する

2. **「スケッチ」→「ライブラリをインクルード」→「．ZIP形式のライブラリをインストール」**を選択する

3. ダウンロードしたZIPファイルを選択してインストールする

4. Arduino IDEを再起動する

## ArduinoとICS変換基板の配線

ArduinoとICS変換基板を接続する際は以下の配線を行う[^3]：

| ICS変換基板 | Arduino |
|:------------|:--------|
| VOUT | VIN |
| IOREF | 5V |
| GND | GND |
| TX | 指定のピン（例：8番ピン） |
| RX | 指定のピン（例：2番ピン） |
| EN | 指定のピン（例：9番ピン） |

**注意事項：**
- ICS変換基板のVOUTをArduinoのVINに接続する
- ICS変換基板のIOREFをArduinoの5Vに接続する
- GNDをICS変換基板とArduinoで共通接続する
- TX，RX，ENピンは任意のデジタルピンに接続可能（プログラムで指定）

## プログラミング

### IcsSoftSerialClassを使用した基本的なサーボ制御プログラム

以下は，IcsSoftSerialClassを使用してサーボを制御するサンプルプログラムである．

```cpp
#include "Arduino.h"
#include <IcsSoftSerialClass.h>

// ピン設定
const byte S_RX_PIN = 8;   // ICS変換基板のTX → ArduinoのRX
const byte S_TX_PIN = 2;   // ICS変換基板のRX → ArduinoのTX
const byte EN_PIN = 9;     // Enable信号ピン
const long BAUDRATE = 115200;  // 通信速度
const int TIMEOUT = 200;   // タイムアウト時間（ms）

// IcsSoftSerialClassのインスタンス作成
IcsSoftSerialClass krs(S_RX_PIN, S_TX_PIN, EN_PIN, BAUDRATE, TIMEOUT);

void setup() {
  krs.begin();
  krs.setPos(0, 7500);  // 位置指令　ID:0サーボ中央:7500 指定範囲:3500～11500
  delay(3000);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  krs.setPos(0, 3500);  // 位置指令　ID:0サーボ反時計回り最大:3500
  delay(2000);          // 2秒待つ

  digitalWrite(LED_BUILTIN, LOW);
  krs.setPos(0, 7500);  // 位置指令　ID:0サーボ中央:7500
  delay(2000);          // 2秒待つ

  digitalWrite(LED_BUILTIN, HIGH);
  krs.setPos(0, 11500); // 位置指令　ID:0サーボ時計回り最大:11500
  delay(2000);          // 2秒待つ

  digitalWrite(LED_BUILTIN, LOW);
  krs.setPos(0, 7500);  // 位置指令　ID:0サーボ中央:7500
  delay(2000);          // 2秒待つ
}
```

**プログラムの説明：**

1. サーボは最初に中央位置（7500）に移動し，3秒待機する
2. ループ処理で以下を繰り返す：
   - 反時計回り最大位置（3500）に移動
   - 中央位置（7500）に移動
   - 時計回り最大位置（11500）に移動
   - 中央位置（7500）に戻る
3. LED_BUILTINを使って動作状態を確認できる

### 主要な関数（IcsSoftSerialClass）

| 関数名 | 説明 | 引数 | 戻り値 |
|:-------|:-----|:-----|:-------|
| `begin()` | ICS通信の初期化 | なし | なし |
| `setPos(id, pos)` | サーボの位置を設定 | `id`: サーボID，`pos`: 目標位置（3500-11500） | 設定後の位置 |
| `getPos(id)` | サーボの現在位置を取得 | `id`: サーボID | 現在位置（3500-11500） |
| `setFree(id)` | サーボをフリー（脱力）状態にする | `id`: サーボID | なし |
| `setSpeed(id, speed)` | サーボのスピードを設定 | `id`: サーボID，`speed`: スピード値（0-127） | 設定後のスピード値 |
| `setStretch(id, stretch)` | サーボのストレッチを設定 | `id`: サーボID，`stretch`: ストレッチ値（0-127） | 設定後のストレッチ値 |

### 位置指定の範囲

近藤サーボの位置指定は**3500〜11500**の範囲で指定する：

- **3500**: 最小角度（約-135度）
- **7500**: 中央位置（0度）
- **11500**: 最大角度（約+135度）

位置と角度の関係は以下の式で表される：

$$
\text{角度[度]} = \frac{(\text{位置} - 7500) \times 270}{8000}
$$

### サーボの零点（0度位置）の確認

KRSサーボの軸には小さいドット（マーク）が打ってある．サーボが0度（中央位置：7500）のとき，このドットがサーボケースの真ん中に来る．これによりサーボの零点位置を視覚的に確認できる．

```{figure} ../fig/krs-zero-point.jpg
:name: krs-zero-point
:align: center

KRSサーボの零点マーク（軸のドットがサーボの真ん中に来ると0度）
```

**注意事項：**
- Seeeduino Nanoには`Serial`（USB通信用）と`Serial1`（ハードウェアシリアル）の2つのシリアルポートがある
- `Serial`はrosserial通信に使用するためICS通信には`Serial1`を使用する

## トラブルシューティング

### サーボが動かない場合

1. **電源の確認**
   - ACアダプターがコンセントに接続されているか
   - 電源スイッチがONになっているか
   - ICS変換基板のLEDが点灯しているか

2. **配線の確認**
   - サーボのコネクタが正しい向きで接続されているか（3本線のコネクタの爪がVH 2pin（白いコネクタ）側を向いているか）
   - コネクタがしっかり差し込まれているか
   - ArduinoとICS変換基板の配線が正しいか（VOUT→VIN，IOREF→5V，GND→GND，TX/RX/ENが正しく接続されているか）

3. **ID番号の確認**
   - プログラムで指定したID番号とサーボのID番号が一致しているか
   - 複数のサーボに同じID番号が設定されていないか

4. **通信速度（ボーレート）の確認**
   - ICS通信は**115200bps**で初期化されているか
   - プログラム内のBAUDRATEが115200に設定されているか確認する

5. **ピン番号の確認**
   - プログラムで指定したピン番号（S_RX_PIN，S_TX_PIN，EN_PIN）が実際の配線と一致しているか

### プログラムの書き込みエラー

- **解決方法**：USB接続とICS通信で同じ`Serial`を使用している場合は競合が発生する．ICS通信には`Serial1`またはIcsSoftSerialClassを使用すること．残念ながらArduino NanoにはSerialが1つしか存在しないためソフトウェアシリアルを使わないとrosserialなどの通信ができない．

### Dual USBアダプターが認識されない（Ubuntu）

- **解決方法**：udevルールが正しく設定されているか確認し，ユーザーが`dialout`および`plugdev`グループに追加されているか確認する．設定後は**再ログイン**が必要である．

## 参考資料

[^1]: 近藤科学 - ICS3.5/3.6 Manager software: <https://kondo-robot.com/faq/ics35mag>

[^2]: 近藤科学 - ICS Library for Arduino: <https://kondo-robot.com/faq/ics-library-a2>

[^3]: 近藤科学 - ICS変換基板チュートリアル: <https://kondo-robot.com/faq/ics_board_-tutorial1>

- [近藤科学 - ICS3.5/3.6の説明](https://kondo-robot.com/faq/ics3-5-explain)
- [近藤科学 - ICS変換基板チュートリアル（続編）](https://kondo-robot.com/faq/ics_board_-tutorial1-2)
- [近藤科学 - KRSサーボシリーズ](https://kondo-robot.com/product-category/servomotor/krs)
- [rcb4パッケージ（GitHubリポジトリ）](https://github.com/iory/rcb4)
