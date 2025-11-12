# 近藤サーボ（KRS）の制御

## 概要

近藤科学社の**KRS（KONDO ROBOT SERVO）シリーズ**は，ICS（Intelligent Control System）通信プロトコルを使用したシリアル通信制御可能なサーボモータである．本演習ではICS変換基板とArduino（Seeeduino Nano）を使用して近藤サーボを制御する．

## 必要な機器

### 基本構成

| 機器名 | 説明 | 備考 |
|:-------|:-----|:-----|
| ICS変換基板 | UARTをICS通信に変換する基板 | 近藤科学製 |
| KSBシールド2 | Arduinoに接続するシールド | 近藤科学製 |
| Arduino（Seeeduino Nano） | マイコンボード | Arduino Nano完全互換 |
| KRSサーボモータ | ICS対応サーボモータ | KRS-2552HV等 |
| USBケーブル | PCとArduinoを接続 | Type-B（Nano用） |

### 電源システム

サーボモータの対応電圧に応じて以下のいずれかを選択する．

**HV（High Voltage）対応サーボ（9-12V）の場合：**
- 12V 5A ACアダプター
- HV電源スイッチハーネス

**LV（Low Voltage）対応サーボ（6-7.4V）の場合：**
- 5.9V 2A ACアダプター
- LV電源スイッチハーネス

## ハードウェアのセットアップ

### 1. 基板の組み立て

1. **KSBシールド2へのピンヘッダのはんだ付け**
   - KSBシールド2にピンヘッダをはんだ付けする
   - Arduinoのピン配置に合わせて取り付ける

2. **ICS変換基板へのピンヘッダのはんだ付け**
   - ICS変換基板にピンヘッダをはんだ付けする

3. **基板の接続**
   - ArduinoにKSBシールド2を装着する
   - KSBシールド2にICS変換基板を装着する

### 2. 電源の接続

1. **ACアダプターと電源スイッチハーネスの接続**
   - ACアダプターのDCプラグを電源スイッチハーネスに接続する

2. **ICS変換基板への電源接続**
   - 電源スイッチハーネスの出力端子をICS変換基板の電源端子に接続する
   - **極性に注意**：赤線がプラス（+），黒線がマイナス（-）

### 3. サーボモータのID設定

近藤サーボは出荷時にID番号が全て**0**に設定されている．複数のサーボを使用する場合は，事前に各サーボに異なるID番号を設定する必要がある．

**必要な機器：**
- Dual USBアダプター（近藤科学製）
- シリアルマネージャーソフト（近藤科学提供の無料ソフト）

**設定手順：**

1. シリアルマネージャーソフトをPCにインストールする[^1]

2. Dual USBアダプターにサーボを1つだけ接続する

3. Dual USBアダプターをPCのUSBポートに接続する

4. シリアルマネージャーソフトを起動し，サーボのIDを検索する

5. 検出されたサーボ（ID:0）を選択し，新しいID番号（1，2，3...）に変更する

6. 設定を保存する

7. 全てのサーボについて，上記の手順を繰り返す

### 4. サーボモータの接続

サーボモータをICS変換基板に接続する際は以下の点に注意する．

- **コネクタの向き**：コネクタの爪がある方をボードの内側（基板側）に向けて接続する
- **複数接続**：ICS変換基板には複数のサーボ接続端子がありデイジーチェーン接続が可能である

```{figure} ../fig/ics-board-servo-connection.jpg
:name: ics-board-connection
:align: center

ICS変換基板とサーボの接続例（コネクタの爪を基板側に向ける）
```

## ソフトウェアのセットアップ

### ICS Library for Arduino のインストール

近藤科学が提供する**ICS Library for Arduino Ver.3**を使用してサーボを制御する．

**インストール手順：**

1. 近藤科学のWebサイトから**ICS Library for Arduino Ver.3**をダウンロードする[^2]

2. ダウンロードしたZIPファイルを解凍する

3. Arduino IDEを起動する

4. **「スケッチ」→「ライブラリをインクルード」→「．ZIP形式のライブラリをインストール」**を選択する

5. 解凍したライブラリフォルダを選択してインストールする

6. Arduino IDEを再起動する

## プログラミング

### 基本的なサーボ制御プログラム

ICS Library for Arduino Ver.3に付属するサンプルプログラムを使用してサーボの基本動作を確認できる．

**サンプルプログラムの場所：**
- **「ファイル」→「スケッチ例」→「ICS」→「SerialServo」**

### サンプルプログラムの実行

```cpp
#include <ICS.h>

// ICSクラスのインスタンス作成
// 引数: シリアルポート番号
ICS ics(&Serial);

void setup() {
  // ICS通信の初期化（通信速度: 115200bps）
  ics.begin();
}

void loop() {
  // サーボID: 1のサーボを7500の位置に移動
  ics.setPos(1, 7500);
  delay(1000);

  // サーボID: 1のサーボを9000の位置に移動
  ics.setPos(1, 9000);
  delay(1000);
}
```

### 主要な関数

| 関数名 | 説明 | 引数 | 戻り値 |
|:-------|:-----|:-----|:-------|
| `begin()` | ICS通信の初期化 | なし | なし |
| `setPos(id, pos)` | サーボの位置を設定 | `id`: サーボID，`pos`: 目標位置（3500-11500） | 設定後の位置 |
| `getPos(id)` | サーボの現在位置を取得 | `id`: サーボID | 現在位置（3500-11500） |
| `setFree(id)` | サーボをフリー（脱力）状態にする | `id`: サーボID | なし |

### 位置指定の範囲

近藤サーボの位置指定は**3500〜11500**の範囲で指定する：

- **3500**: 最小角度（約-135度）
- **7500**: 中央位置（0度）
- **11500**: 最大角度（約+135度）

位置と角度の関係は以下の式で表される：

$$
\text{角度[度]} = \frac{(\text{位置} - 7500) \times 270}{8000}
$$

## ROSとの連携

Seeeduino Nanoで近藤サーボを制御しROSと連携させることで，PCから`topic`を通じてサーボを制御できる．

**必要なライブラリ：**
- rosserial_arduino

**実装例：**

```cpp
#include <ros.h>
#include <std_msgs/Int16.h>
#include <ICS.h>

ros::NodeHandle nh;
ICS ics(&Serial1);  // Serial1をICS通信に使用（Serialはrosserialに使用）

// サーボ位置指令のコールバック関数
void servo_callback(const std_msgs::Int16& cmd_msg) {
  int position = cmd_msg.data;
  ics.setPos(1, position);  // サーボID:1に位置指令を送る
}

// サーボ位置指令のSubscriber
ros::Subscriber<std_msgs::Int16> sub("servo_command", &servo_callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  Serial1.begin(115200);  // ICS通信用（Serial1）
  ics.begin();
}

void loop() {
  nh.spinOnce();
  delay(10);
}
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
   - サーボのコネクタが正しい向きで接続されているか
   - コネクタがしっかり差し込まれているか

3. **ID番号の確認**
   - プログラムで指定したID番号とサーボのID番号が一致しているか
   - 複数のサーボに同じID番号が設定されていないか

4. **通信速度の確認**
   - ICS通信は115200bpsで初期化されているか

### プログラムの書き込みエラー

- **解決方法**：USB接続とICS通信で同じ`Serial`を使用している場合は競合が発生する．ICS通信には`Serial1`を使用すること．

## 参考資料

[^1]: 近藤科学 - シリアルマネージャーソフト: <https://kondo-robot.com/product-category/soft>

[^2]: 近藤科学 - ICS Library for Arduino: <https://kondo-robot.com/product/ics-library-for-arduino>

- [近藤科学 - ICS変換基板チュートリアル](https://kondo-robot.com/faq/ics_board_-tutorial1-2)
- [近藤科学 - KRSサーボシリーズ](https://kondo-robot.com/product-category/servomotor/krs)
- [近藤科学 - ICS通信仕様](https://kondo-robot.com/faq/ics-command)
