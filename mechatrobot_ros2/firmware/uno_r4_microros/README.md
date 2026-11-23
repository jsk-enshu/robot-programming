# micro-ROS on Arduino UNO R4 WiFi（USB 有線のみ）

同じトピックをシリアル＋PC 側ブリッジで出す版（`../uno_r4_serial`）との比較は [../README.md](../README.md)。

Arduino UNO R4 WiFi を、WiFi を使わずに USB シリアル経由で ROS 2 Jazzy につなぐ PlatformIO プロジェクト。
DDS の通信は PC の外に出さない（`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`）。

| env | 中身 | publisher / subscriber |
| --- | --- | --- |
| `pubsub_sample` | 最小の送受信サンプル。`/uno_r4/counter` を 10 Hz で出し、`/uno_r4/led` を受ける | 1 / 1 |
| `mechatrobot` | `mechatrobot/sketchbook/MechatrobotDriver`（rosserial 版）の移植。トピック名・型・周期は同じ | 3 / 1 |

## なぜ 32 KB の SRAM に収まるか

micro-ROS は publisher や subscriber の数だけメモリを静的に確保する。
micro_ros_arduino のビルド済みライブラリは publisher 10 / subscriber 5 分を確保しているので、
ここでは env ごとに、スケッチが使う数だけを確保したライブラリをビルドする（`metas/*.meta`）。

2026-11-18 に PlatformIO（renesas-ra、Arduino コア）でビルドしたときの静的 RAM:

| ビルド | 静的 RAM（32,768 B 中） |
| --- | --- |
| 空のスケッチ（Arduino コアだけ） | 2,480 B（7.6%） |
| micro_ros_arduino のビルド済みライブラリ（publisher 10 / subscriber 5）＋ publisher 1 / subscriber 1 のスケッチ | 16,892 B（51.6%） |
| `pubsub_sample`（publisher 1 / subscriber 1） | 13,324 B（40.7%） |
| `mechatrobot`（publisher 3 / subscriber 1、sensor_msgs/Range を含む） | 19,868 B（60.6%） |

## 実機で確かめたこと（2026-11-18、UNO R4 WiFi 単体、センサ・モータは未接続）

この節はサーボ版に変える前の、ステッピングモータ（28BYJ-48）版のファームウェアで確かめたことである。

- `pubsub_sample`: `/uno_r4/counter` が 9.98 Hz で届く。`/uno_r4/led` に送った値がカウンタに反映される（PC → ボードも動く）
- `mechatrobot`: `range` 20 Hz、`led/state` 2 Hz、`motor1/position` 20 Hz。`range` の `frame_id` と範囲、タイムスタンプ（agent と時刻同期、PC 時刻との差 約 -0.06 s）
- `motor1/command` に 30 / 0 を送ると `motor1/position` が 1 ステップずつ追従して 30 / 0 で止まる（ピン出力のロジックまで。モータを実際に回したわけではない）
- agent を止めて起動し直すと、ボードをリセットせずに再接続する

**HC-SR04 と 28BYJ-48 をつないだ状態では確かめていない。** センサがないと `pulseIn` が毎回タイムアウト（約 27 ms）するため、
上の確認ではステッピングモータが公称（5 rpm = 30°/s）より遅く、約 7°/s で進んだ。センサをつなげば echo は距離に応じて早く返る。

## rosserial 版からの変更点（`mechatrobot`）

- モータはステッピングモータ（28BYJ-48）からサーボ（SG90、信号線 D8）に変えた。サーボは `loop()` の中で `millis()` を見ながら 1° ずつ（200°/s）目標へ進める。rosserial 版はコールバックの中で `stepper.step()` を一度に呼ぶので、5 rpm で 360° 回すと約 12 秒間通信が止まっていた
- `motor1/position` はサーボに書いている角度（サーボは角度を返さないので実測ではない）。−90〜90° で、0° がサーボの真ん中
- `pulseIn` のタイムアウトを最大距離 4.5 m の往復時間にした（既定の 1 s だとセンサがないとき毎回 1 s 止まる）
- タイムスタンプは `nh.now()` の代わりに `rmw_uros_sync_session()` で agent と同期した時刻

## 使い方

### 1. PlatformIO

micro_ros_platformio は PlatformIO 標準の仮想環境 `~/.platformio/penv` を前提にしている（無いと `cannot open .../penv/bin/activate` で止まる）。

```bash
uv venv --seed --allow-existing ~/.platformio/penv
uv pip install --python ~/.platformio/penv/bin/python platformio==6.2.0
```

### 2. ビルドと書き込み

```bash
cd mechatrobot_ros2/firmware/uno_r4_microros
~/.platformio/penv/bin/pio run -e mechatrobot -t upload
```

初回は micro-ROS のライブラリをビルドするので env ごとに 2〜3 分かかる。ROS 2 を source した端末のままでよい（`scripts/strip_ros_env.py` が `/opt/ros` を外してからビルドする）。

### 3. micro_ros_agent

apt パッケージはないので、ワークスペースでソースからビルドする（57 秒ほど）。

```bash
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
# 2026 年度の演習の版 (jazzy ブランチの 2026-11-23 の commit) に固定する
git -C micro-ROS-Agent checkout 7c932329ad5591ef23942ef1962534258c16b000
git -C micro_ros_msgs checkout b1ef85201e5545d3e07b73f4628bb15836062573
cd ~/ros2_ws
colcon build --packages-select micro_ros_msgs micro_ros_agent --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

起動（USB ブリッジをリセットしてから agent を起動する）:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
scripts/run_agent.sh
```

止めるときは Ctrl+C。

### 4. 確認

```bash
python3 scripts/check_pubsub.py        # pubsub_sample
python3 scripts/check_mechatrobot.py   # mechatrobot（サーボを 30° へ動かす）
```

## うまくいかないとき

ボードは 2 つのマイコンでできている。スケッチが動く RA4M1 と、USB と RA4M1 の UART をつなぐ ESP32-S3（USB ブリッジ）である。
今回の不具合はどれも ESP32-S3 側で起きた。RA4M1 のスケッチはその間も動き続けていた（デバッガで確認した）。

| 症状 | 対処 |
| --- | --- |
| LED がゆっくり点滅し続け、agent に `session established` が出ない | `../scripts/recover_usb.sh`。USB を抜き差ししたのと同じ状態になる |
| `pio run -t upload` が `No device found on ttyACM0` / `Device unsupported` で失敗する | `../scripts/recover_usb.sh` してからやり直す。それでもだめなら `../scripts/flash_swd.sh uno_r4_microros <env>` |

- `recover_usb.sh` は `usbreset` を使う。sudo が要らないのは、Arduino IDE の Renesas コアが入れる `/etc/udev/rules.d/60-arduino-renesas.rules`（VID 2341 を `MODE 0666`）があるとき
- `flash_swd.sh` はボード上の CMSIS-DAP から pyOCD でアプリ領域（0x4000〜）だけを書く。ブートローダ（0x0000〜0x3FFF）は消さないので、あとで普通の書き込みも使える。約 40 秒かかる。初回は Renesas のデバイスパックを取得する
- agent を `kill`（SIGTERM）で止めると、`ros2 run` の子プロセスの agent が残ってシリアルポートをつかむことがある。`ps -eo pid,args | grep micro_ros_agent` で確かめる

## 依存するもの

- micro_ros_platformio `cfee17faffaa532363b7151dd13af6a85c69d3c1`（`platformio.ini` で固定）
- micro-ROS-Agent `jazzy` ブランチ（動作確認は `7c932329ad5591ef23942ef1962534258c16b000`）
- PlatformIO renesas-ra / Arduino コア（GCC 7.2.1）
