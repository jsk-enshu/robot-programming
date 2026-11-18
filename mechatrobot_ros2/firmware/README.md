# メカトロボットのファームウェア（Arduino UNO R4 WiFi）

昨年度の rosserial 版 `mechatrobot/sketchbook/MechatrobotDriver` と同じことをする 2 つの版。
**どちらも ROS 2 側には同じトピックを出す**ので、`mechatrobot_ros2` の残り（rviz、ros2_control、手の検出 → モータ）はどちらの版でもそのまま動く。

| 版 | マイコン側 | PC 側 |
| --- | --- | --- |
| シリアル＋ブリッジ | `uno_r4_serial`：1 行 1 メッセージのテキスト通信 | `../scripts/serial_bridge.py`（rclpy）が行とトピックを相互に変換する |
| micro-ROS | `uno_r4_microros`（env `mechatrobot`）：マイコン自身が ROS 2 ノードになる | 既製の `micro_ros_agent` |

| トピック | 型 | 向き | 周期 |
| --- | --- | --- | --- |
| `range` | sensor_msgs/Range | ボード → PC | 20 Hz |
| `led/state` | std_msgs/Bool | ボード → PC | 2 Hz |
| `motor1/position` | std_msgs/Int64 | ボード → PC | 20 Hz |
| `motor1/command` | std_msgs/Int64 | PC → ボード | 送ったとき |

制御部分（サーボを `millis()` で 1° ずつ目標へ進める、`pulseIn` のタイムアウト、角度を −90〜90° に丸める）は 2 つの版で同じにしてある。違うのは通信だけ。

配線は HC-SR04 が Trig D2 / Echo D3、サーボ SG90 の信号線が D8。角度は SG90 の真ん中を 0° とする（`Servo.write()` の値 − 90）。
サーボは角度を返さないので、`motor1/position` はマイコンがサーボに書いている角度で、実測ではない。

## 比較

2026-11-18 に、UNO R4 WiFi 単体（HC-SR04・28BYJ-48 は未接続）、ROS 2 Jazzy、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` で測った。
**サーボ版に変える前の、ステッピングモータ版のファームウェアでの数字である。** 「指令から位置が変わるまで」はモータを動かす部分の作りにもよるので、サーボ版では測り直していない。
測定には `scripts/measure_mechatrobot.py` を使った（どちらの版にも同じスクリプトを使える）。

| 項目 | シリアル＋ブリッジ | micro-ROS |
| --- | --- | --- |
| 静的 RAM（32,768 B 中） | **2,952 B（9.0%）** | 19,868 B（60.6%） |
| Flash（262,144 B 中） | **38.6 KB（14.7%）** | 118.9 KB（45.4%） |
| 初回ビルド | 数秒 | env ごとに 2〜3 分（micro-ROS のライブラリをビルドする） |
| 自分で書くコード（空行・コメントを除く） | マイコン 103 行＋PC 126 行 | マイコン 165 行（PC 側は既製の agent） |
| PC 側に要るもの | pyserial（`python3-serial`） | micro_ros_agent（apt に無く、ソースからビルド） |
| `range` の周期 | 20.05 Hz、間隔の標準偏差 2.8 ms | 20.05 Hz、間隔の標準偏差 3.2 ms |
| 起動から最初の `range` まで | 0.16 s | 0.16 s |
| `motor1/command` から `motor1/position` が変わるまで（20 回、中央値） | **50.9 ms**（49.5〜90.4） | 144.4 ms（99.2〜153.5） |
| `range` のタイムスタンプ | PC が受け取った時刻 | マイコンが測った時刻（agent と時刻同期済み、PC に届くまで中央値 34 ms） |
| 送受信の中身を人が読めるか | 読める（シリアルモニタで読み書きできる） | 読めない（XRCE-DDS のバイナリ） |
| `Serial.print` でのデバッグ | 使える（`#` で始まる行はブリッジがログとして表示する） | 使えない（USB シリアルを micro-ROS が使う） |
| ブリッジ／agent の起動・停止を繰り返したとき、ボードからのデータが止まった回数 | 0 / 20 | 1 / 10（`usbreset` で復帰） |
| トピックを 1 本足すとき | マイコン・通信手順・ブリッジの 3 か所を変える | マイコンだけ。ただし `metas/*.meta` の上限を上げてライブラリをビルドし直す |
| 型と単位 | 通信手順として自分で決めて、両側でそろえる | `.msg` の定義から生成される |
| ROS から見たマイコン | ブリッジのノードがいるだけで、マイコンは見えない | マイコンが `mechatrobot_driver` ノードとして見える |

測定についての注意:

- 「指令から位置が変わるまで」には、ボードの 50 ms 周期の送信と、センサがないときの `pulseIn` のタイムアウト（約 27 ms）が含まれる。両方の版で同じ条件。micro-ROS 版が約 100 ms 長い原因は調べていない
- 「データが止まった回数」は USB ブリッジ（ESP32-S3）が止まった回数。シリアル版の 20 回のうち 2 回は 4 秒で約 28 件しか届かなかった（普段は約 80 件）。そのときのログを残していないので原因は分かっていない
- どの数字も、ボード 1 枚・PC 1 台・各条件 1 回の測定

## どちらを使うか

ボード 1 枚を PC 1 台に USB でつなぐ使い方なら、シリアル＋ブリッジのほうが素直である。
RAM の余裕、ビルドの手軽さ、デバッグのしやすさ、指令の遅れのどれもシリアル版のほうがよかった。
rosserial もこれと同じ形（マイコンはシリアルで話し、PC 側の `serial_node.py` がトピックに変換する）で、通信手順を汎用にしたものだった。

micro-ROS が効いてくるのは次のような場合である。

- マイコンが何台もあり、それぞれのためにブリッジを書きたくない
- WiFi や Ethernet で PC から離したい（micro-ROS は通信路を差し替えられる）
- マイコン上で測った時刻をそのまま使いたい
- マイコン側でサービスやパラメータを使いたい

## 使い方

### シリアル＋ブリッジ

```bash
cd mechatrobot_ros2/firmware/uno_r4_serial
~/.platformio/penv/bin/pio run -e mechatrobot_serial -t upload
ros2 run mechatrobot_ros2 serial_bridge.py --ros-args -p port:=/dev/ttyACM0
```

ブリッジを使わずに、シリアルモニタ（115200 bps）で `cmd 90` と打ってもモータが動く。

### micro-ROS

[uno_r4_microros/README.md](uno_r4_microros/README.md) を参照。

### 測定

ブリッジか agent を起動しておいてから:

```bash
python3 mechatrobot_ros2/firmware/scripts/measure_mechatrobot.py --label serial
```

## うまくいかないとき（両方の版に共通）

シリアル版は `diag` の行を送ると配線の自己診断の結果（`diag echo_us ...` など）を返す。
`ros2 run mechatrobot_ros2 check_hardware.py` がこれを使って、ポート・データ・超音波センサの配線を調べ、次にすることを表示する（`--servo` でサーボも動かす）。

UNO R4 WiFi では、USB と RA4M1 のあいだに ESP32-S3 が入っている（USB ブリッジ）。
今回の不具合はどれもこの ESP32-S3 側で起きた。RA4M1 のスケッチはその間も動き続けていた（デバッガで確認した）。

| 症状 | 対処 |
| --- | --- |
| ボードからデータが届かない | `scripts/recover_usb.sh`（USB を抜き差ししたのと同じ状態になる） |
| `pio run -t upload` が `No device found on ttyACM0` / `Device unsupported` で失敗する | `scripts/recover_usb.sh` してからやり直す。それでもだめなら `scripts/flash_swd.sh <uno_r4_serial または uno_r4_microros> <env>` |

- `recover_usb.sh` は `usbreset` を使う。sudo が要らないのは、Arduino IDE の Renesas コアが入れる `/etc/udev/rules.d/60-arduino-renesas.rules`（VID 2341 を `MODE 0666`）があるとき
- `flash_swd.sh` は、ボード上の CMSIS-DAP から pyOCD でアプリ領域（0x4000〜）だけを書く。ブートローダは消さない。約 15 秒（シリアル版）〜40 秒（micro-ROS 版）かかる。書き込んだあと必ず `recover_usb.sh` を呼ぶ（SWD でリセットしたあとは毎回 USB ブリッジが止まったため）
- このディレクトリには `COLCON_IGNORE` を置いてある。`.pio/` に `package.xml` を含むライブラリがダウンロードされるので、colcon が拾わないようにするため
