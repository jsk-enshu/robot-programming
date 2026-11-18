(day-mechatronics)=
# メカトロボットの製作とプログラミング

:::{figure} fig/mechatrobot-servo-2026.jpg
:align: center
:width: 420px
:name: fig:mechatrobot-servo-2026

本日作るメカトロボット．Arduino UNO R4 WiFi に超音波距離センサ HC-SR04（Trig D2 / Echo D3）とサーボモータ SG90（信号線 D8）をつないだもの．
:::

## 本日の演習内容

{doc}`機械系二学科 夏学期演習 マイコンプログラミング演習<micon-programming>`では，Arduino UNO R4 WiFi で超音波距離センサ（HC-SR04）を読み，サーボモータ（SG90）を回した．
そのときの処理はマイコンの中で閉じていた．
本日はこの回路を PC とつなぎ，ROS 2 のロボットシステムに組み込んだメカトロボット[^mechatro]を作る．
PC からモータに指令を送ったり，センサの値を PC で受け取って可視化したり，PC の画像処理の結果でモータを動かしたりできるようになる．

### 1・2日目の Jedy との関係

1・2日目に動かした Jedy も，PC が直接サーボを動かしていたわけではない．
Jedy の中には近藤科学の RCB-4mini というボードが入っていて，このボードが ICS 通信で各サーボに目標の角度を送り，今の角度を読み取っている．
PC は USB シリアルアダプタを通して RCB-4mini とシリアル通信し，「この角度にして」「今の角度を教えて」といったコマンドをやり取りしていた．
PC 側ではこのシリアル通信を受け持つドライバが動いていて，その上で ROS 2 のノードや `ri` が動いていたので，
1・2日目はシリアル通信を意識せずに `ri.angle_vector()` や ROS 2 のトピックでロボットを動かせた．

本日作るメカトロボットも，これと同じ形をしている（{numref}`Fig. %s <fig:mechatrobot-jedy-analogy-2026>`）．
RCB-4mini の役を Arduino UNO R4 WiFi が，ドライバの役を `serial_bridge.py`（ブリッジ）が受け持つ．
違うのは，ボードのプログラムも，PC との通信手順も，PC 側のブリッジも，すべて自分で読めて書き換えられることである．
市販のロボット用ボードとそのドライバの中で何が起きているかを，自分で作って確かめ，
自分のロボットに合わせたものを自作できるようになること，これが本日の演習の位置づけである．

:::{figure} fig/mechatrobot-jedy-analogy-2026.jpg
:align: center
:name: fig:mechatrobot-jedy-analogy-2026

上：1・2日目の Jedy．PC のドライバが RCB-4mini とシリアルで通信し，RCB-4mini がサーボを動かす．
下：本日のメカトロボット．PC のブリッジが Arduino とシリアルで通信し，Arduino がセンサとモータを動かす．
オレンジの枠が同じ役割の部分である．
:::

**本日の目標**

- Jedy の RCB-4mini とそのドライバのような「ボード＋PC 側のドライバ」の構成を，自分で作れるようになる
- マイコンと PC が USB シリアルでやり取りする仕組みを理解し，自分で通信手順を読み書きできるようになる
- シリアル通信を ROS 2 のトピックに変換するブリッジノードの役割を理解する
- ros2_control・rviz・画像処理と組み合わせて，メカトロボットを ROS 2 から動かす

## 本日のシステム構成

:::{figure} fig/mechatrobot-system-2026.jpg
:align: center
:name: fig:mechatrobot-system-2026

本日のメカトロボットのシステム構成．
左がマイコン，中央の矢印に添えた `"range 0.123"` などが USB シリアルを流れる 1 行のテキスト，
右が PC で動くブリッジ（`serial_bridge.py`），ROS 2 のトピック，それを使うノードである．
:::

マイコンは ROS のことを何も知らない．`range 0.123` や `pos 30` のような 1 行のテキストを USB シリアルに書き，`cmd 90` のような行を読むだけである．
PC で動く `serial_bridge.py`（ブリッジ）がその行を ROS 2 のトピックに変換し，逆にトピックで届いた指令を行に変換してマイコンに送る．
ブリッジから先は普通の ROS 2 のノードなので，ros2_control・rviz・画像処理のノードは，相手がマイコンであることを気にせずに使える．

| トピック | 型 | 向き | 周期 | シリアルの行 |
| --- | --- | --- | --- | --- |
| `/range` | sensor_msgs/Range | マイコン → PC | 20 Hz | `range <距離 [m]>` |
| `/motor1/position` | std_msgs/Int64 | マイコン → PC | 20 Hz | `pos <角度 [deg]>` |
| `/led/state` | std_msgs/Bool | マイコン → PC | 2 Hz | `led <0 または 1>` |
| `/motor1/command` | std_msgs/Int64 | PC → マイコン | 送ったとき | `cmd <角度 [deg]>` |

角度はサーボの真ん中を 0° とし，−90° 〜 90° の範囲で表す（Arduino の `Servo.write()` の 0〜180° から 90 を引いた値）．

```{note}
2025年度までは ROS 1 の rosserial でマイコンと通信し，ros1_bridge で ROS 2 につないでいた．
rosserial も「マイコンはシリアルで話し，PC 側の `serial_node.py` がトピックに変換する」という同じ形で，通信手順を汎用にしたものである．
本年度は ROS 2 だけで完結させ，通信手順とブリッジを自分で読める形にした．
マイコン自身を ROS 2 のノードにする micro-ROS という方法もある．比べたい人は発展課題の課題3に取り組んでほしい．
```

## 準備

(mechatrobot-circuit-2026)=
### 回路

{doc}`夏学期のマイコンプログラミング演習<micon-programming>`の演習課題3（超音波距離センサ）と発展課題5-1（サーボモータ）の回路を，両方同時につないで使う（発展課題5-2 と同じ配線）．
超音波距離センサは Trig = D2 / Echo = D3，サーボの信号線は D8 である．

| 部品 | 端子 | Arduino UNO R4 WiFi |
| --- | --- | --- |
| HC-SR04 | VCC / GND | 3.3V / GND |
| HC-SR04 | Trig / Echo | D2 / D3 |
| サーボ SG90 | 赤（真ん中）/ 茶 | 5V / GND |
| サーボ SG90 | オレンジ（信号） | D8 |

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/micon-programming/hc-sr04-connection.jpg
:align: center
:name: fig:hc-sr04-connection-2026

超音波距離センサ HC-SR04 の接続（夏学期の演習課題3と同じ）
:::

:::{grid-item}
:::{figure} fig/micon-programming/servo-motor-with-arduino.jpg
:align: center
:name: fig:servo-motor-wiring-2026

サーボモータ SG90 の接続（夏学期の発展課題5-1．写真は信号線を D2 につないだ旧配線で，本日は D8 につなぐ）
:::

::::

<span style="color:red">**配線を変えるときは，必ず Arduino の USB ケーブルを PC から抜いて電源を切った状態で行うこと．**</span>
通電したまま配線を変えると，一瞬の接触で短絡や過電流が起き，マイコンや周辺回路が壊れる．

(mechatrobot-software-2026)=
### ソフトウェア

{ref}`software-update`の手順でソフトウェアを更新してから，メカトロボットのパッケージをビルドする．
画像処理には，`jedy_vision` の `hand_detection_node`（1日目の人の認識と同じ MediaPipe で手を見つけるノード）を使う．`jedy_vision` は上のソフトウェア更新でビルドされるので，追加の作業はいらない．

```{code-block} console
$ cd ~/ros2_ws/
$ colcon build --symlink-install --packages-up-to mechatrobot_ros2
$ source ~/ros2_ws/install/setup.bash
```

ROS 2 の通信が自分の PC の中だけに閉じているかを確かめる．`LOCALHOST` と出ればよい．

```{code-block} console
$ echo $ROS_AUTOMATIC_DISCOVERY_RANGE
LOCALHOST
```

<span style="color:red">**何も出ないときは，必ず設定してから先に進むこと．**</span>
本日のトピック名（`/motor1/command` など）は全員同じなので，設定していないと同じネットワークにいる他の人の PC とつながり，
他の人が送った指令で自分のサーボが動いたり，自分の指令で他の人のサーボが動いたりする．
{doc}`environment-setup`のとおりに `~/.bashrc` に書いてあれば，新しく開いたターミナルでは自動で設定される．
まだ書いていなければ，次で書いておく（1 回だけ）．

```{code-block} console
$ echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc
$ source ~/.bashrc
```

`~/.bashrc` を書き換える前に開いたターミナルには読み込まれていないので，そのターミナルでも `source ~/.bashrc` するか，開き直すこと．

Arduino を USB でつなぎ，デバイスファイルの名前を確かめておく．
UNO R4 WiFi は `/dev/ttyACM0` のような名前になる．

```{code-block} console
$ ls /dev/ttyACM*
/dev/ttyACM0
```

## マイコンと PC のシリアル通信

### ファームウェアの書き込み

マイコンのプログラム（ファームウェア）は [mechatrobot_ros2/firmware/uno_r4_serial/src/main.cpp](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/firmware/uno_r4_serial/src/main.cpp) にある．
PlatformIO でビルドして書き込む．PlatformIO は `uvx` で呼び出すので，別にインストールしなくてよい（初回はツールチェーンのダウンロードに時間がかかる）．

```{code-block} console
$ cd ~/ros2_ws/src/robot-programming/mechatrobot_ros2/firmware/uno_r4_serial
$ uvx --from platformio pio run -t upload
```

書き込みが `No device found on ttyACM0` や `Device unsupported` で失敗するときは，{ref}`mechatrobot-troubleshooting-2026`を見ること．

### 通信手順

マイコンは 50 ms ごとに距離と角度を，500 ms ごとに LED の状態を，それぞれ 1 行ずつ USB シリアル（115200 bps）に書く．

```cpp
static void send_sensors() {
  static int tick = 0;
  Serial.print(F("range "));
  Serial.println(measure_range_m(), 3);
  Serial.print(F("pos "));
  Serial.println(current_deg);
  if (tick % LED_TOGGLE_EVERY_N_TICKS == 0) {
    led_on = !led_on;
    digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
    Serial.print(F("led "));
    Serial.println(led_on ? 1 : 0);
  }
  tick++;
}
```

PC から届いた行は改行までためてから解釈する．`cmd <角度>` ならモータの目標角度にし（−90〜90° の外の値は端に丸める），それ以外の行は `#` を付けて送り返す．
`#` で始まる行はログで，ブリッジはトピックにせずに画面に表示するだけである．デバッグに `Serial.print` を使いたいときは，行の先頭に `#` を付けるとよい．

```cpp
static void handle_line(const char *line) {
  if (strncmp(line, "cmd ", 4) == 0) {
    char *end = NULL;
    long deg = strtol(line + 4, &end, 10);
    if (end != line + 4) {
      target_deg = clamp_angle(deg);
      return;
    }
  }
  Serial.print(F("# ignored: "));
  Serial.println(line);
}
```

サーボには，`loop()` の中で 5 ms ごとに 1° ずつ目標に近づけた角度を書く（200°/s）．
SG90 は負荷がなければ 60° を約 0.1 秒（600°/s）で回れるので，この速さならサーボは遅れずについてくる．
`delay()` で待ちながら動かすと，その間は通信もセンサの計測も止まってしまうので，`millis()` で時刻を見て動かす．

サーボは今の角度をマイコンに返さない．
そのため `pos` で送るのは，最後に受けた目標ではなく，**今サーボに書いている角度**である．実際の角度を測っているわけではないので，
サーボの軸を手で押さえたり，重いものを付けたりしてサーボが追いつけないと，`pos` と実際の角度はずれる．

#### <span style="color:green">チェックポイント: シリアルモニタで通信を確かめる</span>

```{exercise} シリアルモニタで通信を確かめる
:label: ex_serial_monitor-2026

1.  Arduino IDE のシリアルモニタ（{doc}`tips/arduino-serial-monitor`）を 115200 bps で開き，`range` と `pos` の行が流れ続けることを確かめる．
    センサの前に手をかざすと `range` の値が変わることも確かめる．

2.  改行を「LF」にして `cmd 60` を送り，サーボが回って `pos` が 60 まで増えていくことを確かめる．`cmd -60` で反対向きに回し，`cmd 0` で真ん中に戻す．
    `cmd 120` のように範囲の外の値を送ると，`pos` が 90 で止まることも確かめる．

3.  `hello` のような決まっていない行を送り，`# ignored: hello` と返ってくることを確かめる．

4.  **確かめたらシリアルモニタを閉じる．** 開いたままだと，次のブリッジがポートを開けない．
```

## ブリッジで ROS 2 につなぐ

### ブリッジの中身

ブリッジは [mechatrobot_ros2/scripts/serial_bridge.py](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/scripts/serial_bridge.py) にある．
マイコンから届いた行を，先頭の単語で振り分けてトピックに変換する．

```python
key, _, value = line.partition(" ")
try:
    if key == "range":
        self.publish_range(received_at, float(value))
    elif key == "pos":
        self.position_pub.publish(Int64(data=int(value)))
    elif key == "led":
        self.led_pub.publish(Bool(data=value.strip() == "1"))
    else:
        self.get_logger().debug(f"unknown line: {line!r}")
except ValueError:
    # A partial line (e.g. right after the port opened) is not an error.
    self.get_logger().debug(f"malformed line: {line!r}")
```

逆向きは，`/motor1/command` を受け取ったら `cmd <角度>` の行をシリアルに書くだけである．

```python
def on_command(self, msg):
    line = f"cmd {int(msg.data)}\n".encode("ascii")
    with self.serial_lock:
        ...
        self.serial.write(line)
```

シリアルポートの読み取りは専用のスレッドで行い，ROS 2 への publish は executor のスレッドで行っている．
USB ケーブルが抜けたりポートを開けなかったりしたときは，1 秒ごとに開き直す．
`/range` のタイムスタンプには，PC が行を受け取った時刻を入れている．

### ブリッジの起動と確認

```{code-block} console
$ ros2 run mechatrobot_ros2 serial_bridge.py --ros-args -p port:=/dev/ttyACM0
[INFO] [mechatrobot_serial_bridge]: connected to /dev/ttyACM0 at 115200 bps
```

別のターミナルでトピックを確かめ，PC からモータに指令を送ってみよう．

```{code-block} console
$ ros2 topic list
$ ros2 topic echo /range
$ ros2 topic echo /motor1/position
$ ros2 topic pub --once /motor1/command std_msgs/msg/Int64 "{data: 90}"
```

#### <span style="color:green">チェックポイント: ブリッジで ROS 2 につなぐ</span>

```{exercise} ブリッジで ROS 2 につなぐ
:label: ex_serial_bridge-2026

1.  `ros2 topic echo /range` で，センサの前に手をかざすと `range` の値が変わることを確かめる．

2.  `ros2 topic pub` で `/motor1/command` に角度を送り，モータが回って `/motor1/position` がその角度に追いつくことを確かめる．

3.  `ros2 topic hz /range` で，20 Hz で届いていることを確かめる．
```

## ros2_control と rviz でメカトロボットを動かす

ブリッジを起動したまま，さらに 2 つのターミナルで次を起動する．

**ターミナル1: ブリッジ（起動済み）**

```{code-block} console
$ ros2 run mechatrobot_ros2 serial_bridge.py --ros-args -p port:=/dev/ttyACM0
```

**ターミナル2: ros2_control（モータ制御）**

```{code-block} console
$ ros2 launch mechatrobot_ros2 mechatrobot_controller.launch.py
```

**ターミナル3: rviz（可視化）**

```{code-block} console
$ ros2 launch mechatrobot_ros2 mechatrobot_display.launch.py
```

```{note}
ros2_control を起動すると，サーボはまず 0°（真ん中）に戻る．
コントローラが動き始めた時点ではまだ `/motor1/position` を受け取っていないので，指令が 0° から始まるためである．
UNO R4 WiFi はシリアルポートを開き直してもリセットされないので，マイコンの中の角度は前に動かしたときのまま残っている．
```

それぞれのプログラムの概要を以下に示す．より詳しく知りたい場合はファイルの中身を追っていくとよい．

- **serial_bridge.py**
  シリアル通信と ROS 2 のトピックを変換する（上で説明したもの）．

- **mechatrobot_controller.launch.py**
  ros2_control[^ros2_control]を用いてモータの制御指令を生成する．
  コントローラとして位置の軌道を生成する `position_trajectory_controller` を使っている．
  ハードウェアインタフェース（[mechatrobot_ros2/src/robot_control.cpp](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/src/robot_control.cpp)）が，関節の目標角度 [rad] を度に変換して `/motor1/command` に publish し，`/motor1/position` を関節の現在角度として読む．
  操作用の GUI（`rqt_joint_trajectory_controller`[^rqt_jtc]）も起動する．

- **mechatrobot_display.launch.py**
  ROS 2 の可視化ツール rviz2[^rviz] を起動する．
  `/led/state` を受けて rviz 上の LED の色を変えるノード（`led_controller.py`）も起動する．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/mechatrobot-rviz.jpg
:align: center
:name: fig:mechatrobot-rviz-2026

メカトロボットの rviz での表示
:::

:::{grid-item}
:::{figure} fig/mechatrobot-rviz-with-tf.jpg
:align: center
:name: fig:mechatrobot-rviz-with-tf-2026

tf を表示すると，ロボットモデルの link，joint 等の位置関係が可視化されわかりやすい場合もある．
:::

::::

### ロボットモデルの構成

ロボットモデルは [mechatrobot_ros2/urdf/robot.urdf](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/urdf/robot.urdf) に定義されている．
ロボットモデルは `link` と `joint` の連結で表されており，以下に記述の一部を紹介する．

`link` 部では `link` 名や幾何情報が記述されている．
幾何情報は既に用意されたプリミティブ形状の他，自身で作成した 3D データを読み込ませることも可能である．
`origin` には `link` の 3D モデル原点からの変位（`xyz`），回転（`rpy`）の情報を記入し，ここが rviz における `link` 原点となる．

```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.053 0.082 0.01"/>
    </geometry>
  </visual>
</link>
```

`joint` 部では `joint` 名，`joint` の種類，親と子に相当する `link` 名や `joint` の基準位置を記述する．
`origin` には親 `link` の原点からの変位（`xyz`），回転（`rpy`）の情報を記入し，ここが rviz における `joint` 原点となる．

```xml
<joint name="base_to_motor" type="fixed">
  <parent link="breadboard_link" />
  <child link="motor_link" />
  <origin xyz="0 0.08 0" rpy="0 0 0" />
</joint>
```

### アクチュエータの制御: GUI による操作インターフェース

ROS には GUI でロボットを操作するインターフェースが用意されており，本演習では `rqt_joint_trajectory_controller` を取り上げる．

下図は `rqt_joint_trajectory_controller` を起動した直後の off の状態である．
GUI 中で `/controller_manager` と `position_trajectory_controller` を選択し，電源ボタン（赤）をクリックすると，コントローラは電源ボタン（緑）の on となり，GUI からロボットを操作できるようになる．
`joint1` のバーと数値は角度 [rad] を示しており（サーボの範囲に合わせて −1.57〜1.57 rad），左右に動かすことで rviz および実物のモータが動くので試してみてほしい．
正しく起動できていれば，rviz 上に表示されているモータ回転軸に相当する座標系が回転するのが確認できる．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} fig/mechatrobot-rqt-off.jpg
:align: center
:name: fig:rqt_joint_trajectory_controller-off-2026

rqt_joint_trajectory_controller（off の状態）
:::

:::{grid-item}
:::{figure} fig/mechatrobot-rqt-on.jpg
:align: center
:name: fig:rqt_joint_trajectory_controller-on-2026

rqt_joint_trajectory_controller（on の状態）
:::

::::

### センサデータの可視化

超音波センサの認識領域は，`/range` を円錐形で可視化している．
前方物体の距離に応じて円錐が変形するので，超音波センサの前に手をかざしてみたり，実際に定規で距離を確認してみるとよい．

:::{figure} fig/range-sensor-rviz.png
:align: center
:name: fig:range-sensor-visualized-2026

rviz で測距センサを可視化している様子
:::

#### <span style="color:green">チェックポイント: メカトロボットの動作確認</span>

```{exercise} メカトロボットの動作確認
:label: ex_mechatrobot-2026

メカトロボットの動作確認を行ってみよう．

1.  rviz にメカトロボットが表示されているか．

2.  `rqt_joint_trajectory_controller` でモータ角度を制御すると，実機および rviz は動作するか．

3.  超音波センサの `/range` が rviz で可視化されており，前方物体の距離を認識できているか．
```

## 画像処理との連携

PC と通信することで，PC でのプログラムの処理結果をロボットの制御に反映させることができる．
ここでは PC のカメラに手をかざし，手が画像のどこにあるかを画像処理で見つけて，手のある側へモータを回す．
手を見つけるのは `jedy_vision` の `hand_detection_node` で，1日目に人の手を見つけたのと同じ MediaPipe の Hand Landmarker を使っている．
新しく書くのは，手の位置をモータの角度に変えるノードだけである．

前の節で起動した ros2_control（ターミナル2）は止めてから，以下を別々のターミナルで実行する[^motor_command_batching]．
ブリッジ（ターミナル1）は起動したままでよい．

**ターミナル2: カメラ**

```{code-block} console
$ ros2 run usb_cam usb_cam_node_exe
```

**ターミナル3: 手を見つけるノード**

```{code-block} console
$ ros2 run jedy_vision hand_detection_node --ros-args -p image_topic:=/image_raw
```

**ターミナル4: モータ制御ノード**

```{code-block} console
$ ros2 run mechatrobot_ros2 motor_command_by_hand.py
```

カメラの画像の右側に手を出すとモータが右に，左側に出すと左に回り，手を真ん中に置くか下ろすと止まる．
見つけた手は `rqt_image_view` で `/hand_detection/image` を選ぶと見られる（手の骨格と枠が描かれる）．
見つけた手の枠は `ros2 topic echo /hand_detection/hands` で確かめられる．

```{code-block} console
$ ros2 run rqt_image_view rqt_image_view
```

- `hand_detection_node`（[jedy_vision/scripts/hand_detection_node.py](https://github.com/jsk-enshu/robot-programming/blob/master/jedy/jedy_vision/scripts/hand_detection_node.py)）
  カメラの画像（`image_topic` で指定する．`usb_cam` は `/image_raw` に出す）から，MediaPipe の Hand Landmarker で手（1 つにつき 21 点）を見つけ，
  手ごとに「右手」「左手」のラベルと 21 点を囲む枠 [pixel] を `/hand_detection/hands`（`jedy_vision_msgs/DetectionArray`）に publish する．
  1日目の物体の認識（`/object_detection/detections`）と同じ型である．

- [motor_command_by_hand.py](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/scripts/motor_command_by_hand.py)
  `/hand_detection/hands` を受け，いちばん確かな手の枠の中心が画像の右側なら角度を 3° 増やし，左側なら 3° 減らして `/motor1/command` に送る．
  真ん中（画像の幅の 20%）にあるときは送らない．手がわずかに揺れただけでモータが左右に行ったり来たりしないようにするためである．
  `hand_detection_node` は 1 秒に 10 回ほど出すので，手を片側に出し続けると 1 秒に 30° ほど回り，サーボの端（±90°）で止まる．
  起動したときの角度を 0° として数えるので，最初の指令でサーボは真ん中付近に向かう．

### <span style="color:green">チェックポイント: 手の位置を利用したモータ制御</span>

```{exercise} 手の位置を利用したモータ制御
:label: ex_hand_motor-2026

上記のプログラムを実行して，カメラの画像の右側・左側に手を出し，モータを左右に回してみよう．
余裕があれば，`motor_command_by_hand.py` を書き換えて，手の高さ（枠の `y`）や右手・左手の違いにも動きを割り当ててみよう．
```

(mechatrobot-troubleshooting-2026)=
## うまくいかないとき

### まず配線を自動で確かめる

センサの値が変わらない，サーボが動かない，といったときは，まず次を実行する．
ポートを使うので，ブリッジ（`serial_bridge.py`）は先にそのウィンドウで `Ctrl-C` して止めておく．

```{code-block} console
$ ros2 run mechatrobot_ros2 check_hardware.py
[OK] ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST (ROS 2 は自分の PC の中だけ)
[OK] /dev/ttyACM0 を使える
[OK] ファームウェアが動いている (range の行が 2 秒で 41 行)
[OK] 超音波センサ: Trig D2 / Echo D3 で反射が返る (前方 27.9 cm)
```

ROS 2 の設定，シリアルポート（ある・権限がある・他のプログラムが開いていない），ボードからデータが届くか，を順に調べる．
超音波センサは，ファームウェアの `diag`（配線の自己診断）で，反射が返るか・Trig と Echo が逆でないか・Echo が HIGH のままでないかを調べる．
`[NG]` が出たら，その下の `->` に書いてあることを上から順に行い，1 つ直したらもう一度実行する．

サーボは角度を返さないので，電気的には動いたかが分からない．`--servo` を付けると，サーボを −45° → 45° → 元の角度と動かすので，**サーボを見ていて，左右に動いたかを確かめる**．

```{code-block} console
$ ros2 run mechatrobot_ros2 check_hardware.py --servo
...
サーボを -45° → 45° → 0° (元の角度) に動かす．サーボを見ていること
[OK] ファームウェアはサーボに角度を書いた (pos が指令どおりに変わった)
サーボは左右に動きましたか? [y/n]
```

<span style="color:red">**配線を直すときは，必ず先に Arduino の USB ケーブルを抜くこと．**</span>

### AI TA に見てもらう

AI TA（{doc}`tips/ai-ta`）に「超音波センサの値が 0 のまま」「サーボが動かない」などと聞くと，AI TA は上の `check_hardware.py` を実行して原因を探す．
サーボを動かすときは，何をどれだけ動かすかを書いてから動かし，「いま左右に動きましたか？」と聞いてくるので，見て答える．

配線が合っているかを目で見てほしいときは，**スマホで配線の写真を撮って AI TA に貼る**．

1.  明るいところで 2 枚撮る．1 枚目はブレッドボードと Arduino の全体を真上から，2 枚目は Arduino のピンの番号と，そこに刺さっている線が読めるくらい近づけて．
2.  写真を PC に送る（自分宛てのメールなど）．PC のブラウザで開き，画像を右クリックして「画像をコピー」する．
3.  `ta` の対話の画面で `Ctrl+V` で貼り，「メカトロボットの配線が資料と合っているか見て」と頼む．
    画像を保存したときは，`~/Downloads/wiring.jpg の配線を見て` のようにファイルの場所を書いてもよい．

AI TA は，資料の先頭の写真と{ref}`mechatrobot-circuit-2026`の表と比べる．
ただし，写真では細い線がブレッドボードや Arduino のどの穴に刺さっているかまでは見分けにくい．
`check_hardware.py` の結果と食い違うときは，`check_hardware.py`（電気的に測った結果）を信じる．

### よくある症状

UNO R4 WiFi では，USB と Arduino のマイコン（RA4M1）のあいだに，もう 1 つのマイコン（ESP32-S3）が USB とシリアルの変換役として入っている．
ここが止まると，マイコンのプログラムは動いているのに PC にデータが届かなくなる．

| 症状 | 対処 |
| --- | --- |
| `ls /dev/ttyACM*` で何も出ない | USB ケーブルを挿し直す．充電専用のケーブルでないか確かめる |
| ブリッジが `cannot open /dev/ttyACM0` を出し続ける | シリアルモニタなど，ポートを開いている他のプログラムを閉じる．`Permission denied` なら {doc}`tips/arduino-device-detection` を見る |
| ブリッジは `connected` と出るのにトピックが届かない（Arduino の LED は点滅している） | マイコンは動いているが，USB とシリアルの変換役が止まっている．Arduino の USB ケーブルを抜いて挿し直す（下の `recover_usb.sh` でも同じ）．そのあとブリッジを起動し直す |
| 書き込みが `No device found on ttyACM0` / `Device unsupported` で失敗する | `recover_usb.sh` を実行してから書き込み直す．それでも失敗するなら下の `flash_swd.sh` で書き込む |
| `range` が `0.000` のまま変わらない | 超音波が返ってきていない．`check_hardware.py` で原因を調べる．Trig と Echo が逆でないか，夏学期の発展課題5-2 の古い資料の配線（Trig = D3 / Echo = D4）のままでないかを確かめる．Trig = D2，Echo = D3 である．4.5 m 以内に何もないときも 0 になる |
| `pos` は変わるのにサーボが動かない | サーボは角度を返さないので，`pos` は動かそうとしている角度でしかない．`check_hardware.py --servo` で動かして確かめる．信号線（オレンジ）が D8 か，赤が 5V・茶が GND かを確かめる |
| しばらく PC から離れていたら，ブリッジは `connected` のままトピックが届かない | ノート PC がバッテリーで動いていると，触らずに 15 分ほどたつとサスペンド（スリープ）する．復帰しても Arduino からのデータは止まったままなので，Arduino の USB ケーブルを抜いて挿し直す．演習中は AC アダプタをつないでおく |
| 何も送っていないのにサーボが動く．`ros2 node list` に自分が起動していないノードが出る | 同じネットワークの他の人の PC とつながっている．`echo $ROS_AUTOMATIC_DISCOVERY_RANGE` が `LOCALHOST` か確かめる（{ref}`mechatrobot-software-2026`）．設定したら，ブリッジなどのノードをすべて止めて起動し直す |

```{code-block} console
$ cd ~/ros2_ws/src/robot-programming/mechatrobot_ros2/firmware
$ scripts/recover_usb.sh                                       # USB を抜き差ししたのと同じ状態にする（権限が無いと失敗する．そのときは抜き差しする）
$ scripts/flash_swd.sh uno_r4_serial mechatrobot_serial        # ボード上のデバッガ経由で書き込む（約 15 秒）
```

`flash_swd.sh` はアプリケーションの領域だけを書き換え，Arduino のブートローダは消さないので，あとで普通の書き込みも使える．
詳しくは [mechatrobot_ros2/firmware/README.md](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/firmware/README.md) を参照すること．

## 本日の発展課題

発展課題は必須ではない．
課題1は本日のメカトロボットで LiDAR を作る課題，課題2・課題3はメカトロボットの通信を発展させる課題，課題4〜課題9は四脚ロボット JEFY を題材にした課題である．

(mechatrobot-lidar-2026)=
### 課題1(発展): サーボと超音波センサで LiDAR を作る

自動運転の車やロボット掃除機に載っている LiDAR は，レーザの向きを回しながら距離を測り，周りの物の位置を点の集まり（点群）として出す．
ここでは超音波センサをサーボの軸の先に付けて左右に振り，同じことをする．
向きを変えながら測った距離を，その向きに置いた点にすれば，周りの物の形が見えてくる．

#### センサをサーボに付ける

<span style="color:red">**USB ケーブルを抜いてから付けること．**</span>
超音波センサをサーボの軸（ホーン）の先に付ける（{numref}`Fig. %s <fig:mechatrobot-servo-lidar-2026>`．写真ではホルダーで留めている）．
センサのホルダーは TA が配るので，欲しい人は `ta call "センサのホルダーがほしい"` で TA を呼ぶか，近くの TA に声をかけること．
配線は{ref}`mechatrobot-circuit-2026`のままでよい．
センサの線は，サーボが −90° から 90° まで回っても引っ張られないように，長さに余裕を持たせる．

:::{figure} fig/mechatrobot-servo-lidar-2026.jpg
:align: center
:width: 360px
:name: fig:mechatrobot-servo-lidar-2026

サーボの軸の先に超音波センサを付けたところ．サーボを回すとセンサの向きが変わる．
:::

#### 動かす

ros2_control（`mechatrobot_controller.launch.py`）や `motor_command_by_hand.py` が動いていたら止めておく（同じ `/motor1/command` にサーボの角度を送るため）．

**ターミナル1: ブリッジ（起動済みならそのまま）**

```{code-block} console
$ ros2 run mechatrobot_ros2 serial_bridge.py --ros-args -p port:=/dev/ttyACM0
```

**ターミナル2: サーボを振って点群を出すノードと rviz**

```{code-block} console
$ ros2 launch mechatrobot_ros2 servo_scan.launch.py
```

サーボが左右に振れ続け，rviz に点がたまっていく（{numref}`Fig. %s <fig:mechatrobot-servo-lidar-rviz-2026>`）．
rviz は真上から見た図で，原点がサーボの軸，**画面の右（赤い軸の向き）がサーボ 0° のときセンサが向く方向**である．
緑の線が，いま測っている向きと距離を表す．点の色は距離（近いほど赤い）．

:::{figure} fig/mechatrobot-servo-lidar-rviz-2026.png
:align: center
:name: fig:mechatrobot-servo-lidar-rviz-2026

rviz で見た点群．往復するたびに点が増えていく（点は 20 秒で消える）．
:::

[servo_scan.py](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/scripts/servo_scan.py) は次のことをしている．

- 50 ms ごとにサーボの目標角度を 7.3° ずつ進め，−90° と 90° で折り返す（`/motor1/command`）．
- マイコンは 50 ms ごとに距離を測って `range` を送り，すぐ後にそのとき書いていた角度を `pos` で送る．
  ノードは `range` を受けたら次の `pos` を待ち，その角度の向きに，その距離の点を 1 つ作って `/servo_scan/points`（`sensor_msgs/PointCloud2`）に出す．
  1 つ前の `pos` を使うと 50 ms 前の角度（最大 10° ずれる）になってしまう．
- 7.3° は 180° を割り切らないので，往復するたびに前の往復とは少しずれた向きで測る．
  rviz は点を 20 秒残すので（PointCloud2 の Decay Time），待つほど点の間が埋まって形がはっきりする．
  Livox などの LiDAR も，同じ向きばかりを測らない走査で，時間をかけるほど細かく見えるようにしている．

超音波は 15° ほどの幅に広がって進むので，レーザの LiDAR のように細かい形は分からない．
斜めのつるつるした面では音が別の向きに反射して返ってこず，点が出ないこともある．
センサが机に近いと，机の面で反射した音を拾い，どの向きでも同じくらいの距離の点が出る．そのときはセンサを少し上に向けるか，サーボを台に載せる．

#### 物を目の前に置いて確かめよう

<div class="screen">

1.  センサの正面（サーボ 0° の向き）20〜30 cm のところに箱などを置き，rviz で画面の右，原点から 20〜30 cm のところに点が集まることを確かめる．

2.  箱をセンサから見て左に動かすと，rviz では点が画面の上に，右に動かすと画面の下に移ることを確かめる．
    逆になるときは，サーボの回る向きが逆である．ターミナル2を `Ctrl-C` で止め，`ros2 launch mechatrobot_ros2 servo_scan.launch.py direction:=-1.0` で起動し直す．

3.  しばらく待って点が増え，箱の面の形が見えてくることを確かめる．箱を取り除くと，その点は 20 秒で消える．

</div>

余裕があれば，`servo_scan.launch.py step_deg:=10.0` で起動し（180° を割り切るので，毎回同じ向きで測る），点の増え方を比べてみよう．

### 課題2(発展): 通信手順を自分で拡張する

本日のメカトロボットに，新しいトピックを 1 本足してみよう．
トピックを足すには，マイコン（`firmware/uno_r4_serial/src/main.cpp`）・通信手順（行の書式）・ブリッジ（`scripts/serial_bridge.py`）の 3 か所をそろえて変える必要がある．

**実装内容の例：**

- PC から LED を点けたり消したりする（`/led/command` を受けて `led_cmd 1` の行を送る）
- サーボの速さ [deg/s]（`SERVO_DEG_PER_SEC`）を PC から変える
- 夏学期の LED マトリクスに，PC から送った文字や数字を表示する

<div class="screen">

1.  まず通信手順（どんな行を，どちら向きに，どの周期で送るか）を決める．

2.  マイコン側を書き換えて書き込み，シリアルモニタで新しい行を手で送って動作を確かめる．

3.  ブリッジに publish または subscribe を足し，`ros2 topic pub` / `ros2 topic echo` で確かめる．

</div>

### 課題3(発展): micro-ROS 版と比べる

本日はシリアルの行を PC のブリッジでトピックに変換した．
これに対して micro-ROS は，マイコン自身を ROS 2 のノードにする方法である（PC 側では既製の `micro_ros_agent` が中継する）．
同じトピックを出す micro-ROS 版のファームウェアが [mechatrobot_ros2/firmware/uno_r4_microros](https://github.com/jsk-enshu/robot-programming/tree/master/mechatrobot_ros2/firmware/uno_r4_microros) にある．
トピックの名前と型が同じなので，ros2_control や rviz はどちらの版でもそのまま動く．

<div class="screen">

1.  [uno_r4_microros/README.md](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/firmware/uno_r4_microros/README.md) の手順で `micro_ros_agent` をビルドし，env `mechatrobot` を書き込む．
    micro-ROS のライブラリのビルドに 2〜3 分かかる．

2.  ブリッジの代わりに `micro_ros_agent` を起動し，本日のチェックポイントが同じように動くことを確かめる．

3.  `firmware/scripts/measure_mechatrobot.py` を両方の版で実行し，トピックの周期や，指令を送ってから位置が変わるまでの時間を比べる．

</div>

両方の版で次のことを比べ，それぞれの方法が向いている場面を考えてみよう．
[firmware/README.md](https://github.com/jsk-enshu/robot-programming/blob/master/mechatrobot_ros2/firmware/README.md) に，TA が測った結果がある．

- マイコンの RAM と Flash の使用量（ビルドの最後に表示される）
- 自分で書くコードの量と，トピックを 1 本足すときに変える場所
- 通信の中身を人が読めるか，`Serial.print` でデバッグできるか
- マイコンが何台もあるとき，WiFi で PC から離したいとき，にどうなるか

### 四脚ロボット JEFY について

課題4〜課題9では，Arduino UNO R4 WiFi と近藤科学の KRS サーボで組む四脚ロボット JEFY を題材に，
組み立て → 強化学習 → 自分の方策で実機を歩かせる → IMU を付ける → CAD から URDF を作る → 自分のロボットを作る，
と順に進められるようにした．

**JEFY**（ジェフィー）は **J**SK **E**ducational **F**our-legged walk**Y** robot の略で，[四脚ロボット JEFY の組み立て](https://iory.github.io/build-your-own-arduino-rtos/hardware/index.html)にまとめられている四脚ロボットの呼び名である．

課題4から順にやると，ロボットを一から作って歩かせるところまでの流れをひととおり体験できる．
課題5・課題8・課題9は実機がなくても取り組める．

<video width="100%" controls preload="metadata" poster="fig/krs-quad-walk-2026-poster.jpg">
  <source src="fig/krs-quad-walk-2026.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** KRS-3302 ICS ×8 と RCB-4mini で組んだ四脚ロボット．Ni-MH バッテリー（ROBO パワーセル E タイプ，7.2 V）1 本だけで，ケーブルをつながずに歩いている．

### 課題4(発展): 四脚ロボット JEFY を組み立てて歩かせる

動画の四脚ロボット JEFY を自分で組み立て，Arduino UNO R4 WiFi から歩かせてみよう．
希望者は TA から部品一式を受け取ること．
手順はすべて次のページにまとめてある．

- 組み立て・配線・書き込み・校正: [近藤科学の KRS サーボで組む（RCB-4mini 版）](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html)
- 部品表: [部品表（BOM）— KRS 版 8 自由度四脚ロボット](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs_bom.html)

**TA から受け取るもの：**

- KRS-3302 ICS ×8，ホーン，ネジ
- 近藤科学 RCB-4mini（サーボをまとめる基板）
- 胴体・脚・蓋の 3D プリント部品
- Dual USB アダプター HS（サーボの ID を振るときに使う）
- 電源（ROBO パワーセル E タイプ と充電器，または近藤科学の AC アダプター）

Arduino UNO R4 WiFi は TA からは配らない．各自が持っているものを使うこと．

以下の手順では `uv` を使う．入っていなければ {doc}`environment-setup` の「初回環境構築」にある `curl -LsSf https://astral.sh/uv/install.sh | sh` で入れておくこと．

<div class="screen">

1.  `ics-manager`（`uv tool install rcb4` で入る）で8個のサーボに ID を振り，設定を確かめる．

2.  サーボを真ん中に合わせてからホーンと脚を付け，ページの 3D ビューアを見ながら胴体に組み付ける．

3.  サーボを RCB-4mini の SIO ポートに，RCB-4mini の COM ポートを Arduino の **D8 / D9** につなぐ（D0 / D1 は空けておく）．

4.  コードを取ってきて書き込み，ゼロ点と回る向きを測って `include/quad_calib_rcb4.h` に書く．

    ```bash
    git clone https://github.com/iory/build-your-own-arduino-rtos
    cd build-your-own-arduino-rtos/code
    uv run pio run -d 13_quadruped -e uno_r4_wifi_rcb4 -t upload
    ```

5.  `QUAD_RCB4_CALIBRATED` を `1` にしてもう一度書き込み，吊るしたまま `rl 0.06 0` を送って足踏みを確かめてから床で歩かせる．
    この env の方策（`include/arduino_quad_policy.h`）は IMU を使わない．

</div>

:::{warning}
校正が終わったファームウェアは，起動するとすぐに立ち姿勢へ動く．
最初は必ず機体を吊るした状態で動かし，足踏みすることを確かめてから床に下ろすこと．
歩いている最中に Arduino がリセットするときは，バッテリーの電圧（6 V を切っていないか）を確かめる．
:::

### 課題5(発展): 強化学習で歩行方策を学習する

課題4で使った方策は，物理シミュレータ MuJoCo の中で強化学習（PPO）によって学習したものである．
[歩行を学習させる（強化学習）](https://iory.github.io/build-your-own-arduino-rtos/hardware/walk.html)を読んで，
自分で歩行方策を学習してみよう．実機がなくても取り組める．

<div class="screen">

1.  まずページ内のブラウザのシミュレーションで，sin/cos で作った歩行と学習済みの方策を比べてみる．

2.  観測・行動・報酬が何か（ページの「そもそも強化学習とは」「方策の中身」）を確かめる．

3.  [歩行方策を自分で学習する](https://iory.github.io/build-your-own-arduino-rtos/hardware/train.html)の手順で `ArduinoQuad-Walk` を学習する．
    学習の入口は次の3つで，どれで学習してもチェックポイントの形式は同じである．
    - NVIDIA GPU がある PC: mjlab 版
    - GPU がない場合: Google Colab（無料枠の T4）のノートブック
    - GPU がない場合: CPU 版（`code/13_quadruped/rl/cpu` で `uv sync` のあと `uv run train.py`）

4.  学習した方策をシミュレーションで歩かせ，動画（`record_video.py` や `evaluate.py --video`）に書き出す．

</div>

余裕があれば，報酬の重みや速度指令の上限を変えて学習し直し，歩き方がどう変わるかを比べてみよう．

### 課題6(発展): 自分で学習した方策で実機を歩かせる

（課題4・課題5ができてから挑戦することをお勧めする．）
課題5で学習した方策を Arduino に書き込み，課題4で組み立てた JEFY を歩かせてみよう．

<div class="screen">

1.  `host/export_quad_policy.py` でチェックポイントから Arduino 用のヘッダを書き出す（[歩行方策を自分で学習する](https://iory.github.io/build-your-own-arduino-rtos/hardware/train.html)の「実機に持っていく」）．

2.  書き出した `arduino_quad_policy.h` で `code/13_quadruped/include/arduino_quad_policy.h` を置き換える．

3.  課題4と同じ `-e uno_r4_wifi_rcb4` で書き込み，**吊るした状態で**足踏みを確かめてから床で歩かせる．

</div>

配布の方策と自分の方策で，同じ速度指令を与えたときの歩き方（速さ・まっすぐ進むか・押されたときに倒れないか）を比べてみよう．

`ArduinoQuad-Walk` の機体モデルは Feetech STS3215 のサーボを前提にしていて，KRS-3302 のトルクはその4分の1以下である．
シミュレーションと実機で歩き方がどう違ったかを観察し，その差がどこから来るのかを考えてみよう．

### 課題7(発展): IMU を付けて KRS 専用の方策で歩かせる

（課題4または課題6で JEFY が歩いた人向け．）
JEFY が歩いたら TA に申し出て，IMU 基板（QMI8658）と Qwiic ケーブルを受け取ること．
IMU（慣性計測装置）で胴体の傾きと回転の速さを測り，その値も見る方策で歩かせてみよう．

これまでの方策は関節角と関節速度しか見ていないので，胴体が傾いても方策にはそれが直接は伝わらない．
近藤科学の KRS-3302 に合わせて機体のモデル・関節の範囲・質量を作り直し，IMU の値も観測に入れて学習した KRS 専用の方策がある（[KRS 版の歩行方策](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html)）．

<div class="screen">

1.  [IMU で姿勢を見て踏ん張らせる](https://iory.github.io/build-your-own-arduino-rtos/hardware/imu.html)を読み，IMU 基板を胴体に取り付けて Arduino の Qwiic コネクタにつなぐ．

2.  ファームウェアは IMU を決まった向きに付けた前提で，チップの軸を胴体の軸（x 前・y 左・z 上）へ回している（`src/quad_imu.cpp` の `R_BODY_CHIP`）．
    JEFY の胴体に付けた向きに合わせてここを直し，ページの「向きを確かめる」で合っているか確かめる．

3.  KRS 専用の方策（`include/krs_quad_policy.h`）で書き込み，吊るして足踏みを確かめてから床で歩かせる．

    ```bash
    uv run pio run -d 13_quadruped -e uno_r4_wifi_rcb4_krs -t upload
    ```

</div>

IMU なしの方策と比べて，歩いている途中や止まっているときに胴体を横から軽く押すと，どう違うかを確かめてみよう．

さらに余裕があれば，KRS 専用の方策を自分で学習し直してみよう．
タスクは `KrsQuad-Walk` で，mjlab 版で学習するため NVIDIA GPU が必要である．
手元に GPU がなければ，課題5の Google Colab（無料枠の T4）で学習してみるとよい．
Colab のノートブックは `ArduinoQuad-Walk` 用なので，学習と書き出しのセルのタスク名を `KrsQuad-Walk` に変え，書き出しには `--name krs_quad_policy` を付ける．
手順は [code/13_quadruped/rl/README.md](https://github.com/iory/build-your-own-arduino-rtos/blob/main/code/13_quadruped/rl/README.md) の「KRS 版（KrsQuad-*）」にある．

<div class="screen">

1.  `code/13_quadruped` で学習環境を用意し（README の「KRS 版」にある機体モデルの作成も先に行う），`KrsQuad-Walk` を学習する．

    ```bash
    ./rl/scripts/setup.sh
    source rl/scripts/env.sh
    ./rl/scripts/train.sh KrsQuad-Walk --env.scene.num-envs=4096 --agent.max-iterations 3000
    ```

2.  `rl/scripts/krs_eval.py` で，速度指令への追従・胴体の傾き・関節とトルクの余裕を数値で確かめる．

3.  `host/export_quad_policy.py` に `--name krs_quad_policy` を付けて Arduino 用のヘッダを書き出し，`include/krs_quad_policy.h` を置き換えて，上と同じ `-e uno_r4_wifi_rcb4_krs` で書き込む．

</div>

### 課題8(発展): SolidWorks で組んだ四脚ロボットから URDF を書き出す

[sw2robot（solidworks_urdf_exporter2）](https://github.com/jsk-ros-pkg/solidworks_urdf_exporter2)は，SolidWorks のアセンブリの合致（メイト）からリンクの木構造と関節軸を読み取り，URDF を書き出すツールである．
[sw2robot の日本語ドキュメント](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/ja/)の
[チュートリアル: 4脚ロボットをゼロから作る](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/ja/tutorial/)に沿って，
四脚ロボットを SolidWorks でアセンブリし，URDF を書き出してみよう．

SolidWorks は Windows でしか動かないので，この課題は Windows に切り替えて行うこと．

<div class="screen">

1.  PowerShell で sw2robot のエディタを入れる．

    ```powershell
    powershell -ExecutionPolicy ByPass -c "irm https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/install.ps1 | iex"
    ```

2.  SolidWorks で body・thigh・shin の3つの部品を作り，材料を設定する（材料から URDF の質量・慣性が決まる）．

3.  アセンブリで「同心円 + 一致」の合致を使って股関節と膝を組む．脚をドラッグして，軸まわりにだけ回ることを確かめる．

4.  sw2robot でアセンブリを抽出し，ブラウザのエディタ（`sw2robot-web`）で関節の名前・向き・可動範囲を整えて URDF を書き出す．

5.  書き出した URDF を RViz などで表示し，関節を動かして確かめる．

</div>

抽出（`.SLDASM` の読み込み）には SolidWorks 2024 以降が必要である．

余裕があれば，実際のサーボを SolidWorks でアセンブリしてみよう．
KRS-3302 を含む KRS-3300 シリーズは，近藤科学がフレーム設計用の 3D データ（STEP）を公開している
（[KRS-3300シリーズ 寸法図](https://kondo-robot.com/faq/krs-3300dimention)の
[KRS3300_scale_step.zip](https://kondo-robot.com/kpc/wp-content/uploads/KRS3300_scale_step.zip)．
商品ページ: [KRS-3304R2 ICS](https://kondo-robot.com/product/krs-3304r2-ics)）．
STEP のサーボを部品として取り込み，サーボの出力軸とフレームの間に合致を付けて組むと，寸法や関節の位置が実機の JEFY に近い URDF ができる．
JEFY の 3D プリント部品の STL は[部品表](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs_bom.html)の「3D プリント部品」にある．

さらに余裕があれば，[MuJoCo で歩かせる](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/ja/walk/)に沿って，
自分で書き出したロボットを課題5の CPU 版の学習コードで歩かせてみよう．

### 課題9(発展): 自分のオリジナルロボットの URDF を作る

課題8の流れを使って，自分で考えたオリジナルのロボットを SolidWorks で設計し，sw2robot で URDF を作ってみよう．
脚の数・関節の数・形は自由である（アーム，車輪ロボット，脚の数が違う歩行ロボットなど）．

- 実際に使うサーボがあれば，課題8と同じようにメーカーの STEP を取り込んでアセンブリする．
- 部品ごとに材料を設定し，合致で関節を作る．
- エディタで関節の種類（動く関節か固定か）・可動範囲・名前を整える．
- 書き出した URDF を RViz で表示し，各関節を動かしたスクリーンショットを撮って URDF と一緒に提出すること．

さらに余裕があれば，作ったロボットを Gazebo や MuJoCo に読み込んで動かしてみよう．

[^mechatro]: メカトロニクスとロボティクスを組み合わせた造語で，機械・電子・制御・情報などの要素技術を統合したロボットシステムを指す．

[^ros2_control]: ros2_control は，ROS 2 でロボットのコントローラを実装するためのフレームワークである．詳細は [ros2_control のドキュメント](https://control.ros.org/jazzy/index.html)を参照のこと．

[^rqt_jtc]: rqt_joint_trajectory_controller は，関節軌道制御のための GUI ツールである．

[^rviz]: rviz は，ROS の 3 次元可視化ツールである．センサデータやロボットの状態を可視化できる．

[^motor_command_batching]: ros2_control も `/motor1/command` に指令を送り続けるので，起動したままだと手の位置からの指令とぶつかってしまうためである．
