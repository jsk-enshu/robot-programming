# ジョイスティックコントローラによる操縦

ジョイスティックコントローラ (ゲームパッド) で Jedy を操縦する方法を紹介する．
2026 年度の Jedy は，2025 年度と同じ **PS3 のコントローラ (DUALSHOCK 3)** で操縦する (PS4 のコントローラ (DUALSHOCK 4) でも同じ割り当てで動く)．
コントローラは USB ケーブルでつないでも，Bluetooth で無線でつないでもよい (下の「Bluetooth で無線接続する」)．
コントローラが無くても，台車はキーボードや `ri.go_velocity()` で動かせる ({doc}`../robot-programming-1-2026` の「台車を動かす」)．

この章の端末では，毎回次を実行しておく ({doc}`../robot-programming-1-2026` のとおりに `~/.bashrc` に書いてあれば不要)．

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

## しくみ

コントローラの入力は次の 2 つのノードで台車と首の指令になる．

| ノード | 中身 |
| --- | --- |
| `joy_linux_node` (`joy_linux` パッケージ) | `/dev/input/js0` を読み，ボタンとスティックの状態を `/joy` (`sensor_msgs/Joy`) に出す |
| `jedy_joy` (`jedy_bringup` パッケージ) | `/joy` を読み，台車の速度を `/cmd_vel` (`geometry_msgs/TwistStamped`) に，首の目標を `/jedy_fullbody/head_controller/joint_trajectory` に出す．サーボの ON / OFF もする |

台車 (`mecanum_drive_controller`) は実機でも Gazebo でも `/cmd_vel` を聞いている．
キーボード操縦 (`teleop_twist_keyboard`) や `ri.go_velocity()` も同じ `/cmd_vel` に送る．
このように，操縦する道具が違っても，共通のトピックで台車を動かしている．

この 2 つのノードは `ros2 launch jedy_bringup jedy_joy.launch.py` で起動する．

## 実機での起動

実機を起動する `jedy_fullbody_all.launch.py` は，`joy:=true` (既定) で `jedy_joy.launch.py` も起動する．
コントローラを USB で PC につなげば，それだけで操縦できる．
コントローラを後から挿しても，`joy_linux_node` が見つけて開く (見つかるまで `Couldn't open joystick /dev/input/js0. Will retry every second.` と出続ける)．

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true
```

コントローラが `/dev/input/js0` 以外 (たとえば `/dev/input/js1`) になったときは，`joy:=false` で起動し，別の端末でデバイスを指定して起動する．

```{code-block} console
$ ls /dev/input/js*
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true joy:=false
$ ros2 launch jedy_bringup jedy_joy.launch.py dev:=/dev/input/js1   # 別の端末で
```

```{warning}
`jedy_fullbody_all.launch.py` を起動すると，台車の車輪だけはすぐにサーボが入る．
L1 を押しながらスティックを倒すと，そのまま走り出す．初めて動かすときは車輪を床から浮かせておく．
```

## ボタンとスティックの割り当て

`jedy_joy` の割り当ては次のとおりである (`jedy/jedy_bringup/scripts/jedy_joy.py` と `jedy_teleop/mapping.py`)．

| 操作 | 動き |
| --- | --- |
| L1 (押し続ける) | デッドマンスイッチ．押していないと台車も首も動かない．離すと台車は止まる |
| L1 + 左スティック 上下 | 前進・後退 |
| L1 + 左スティック 左右 | 左右へ横に動く (メカナムホイール) |
| L1 + 右スティック 左右 | その場で旋回 |
| L1 + R1 + スティック | ゆっくり動く |
| L1 + 十字キー 左右 / 上下 | 首を左右に振る / 上下に向ける |
| L1 + L2 + 十字キー | 首を 2 倍の速さで動かす (台車は速くならない) |
| L1 + R1 + × を 1 秒押し続ける | 全身を初期姿勢 (`home`) へ 5 秒かけて動かす |
| ○ | 全身のサーボを OFF (脱力) にする．L1 は要らない |
| △ | サーボを ON にする |

- 台車の速さはスティックの倒し方で決まる．いちばん倒すと前後 0.16 m/s 程度，R1 を押すと 0.09 m/s 程度．
  実機の車輪は遅すぎる指令では回らないので，少しだけ倒しても回る速さに広げてある．
- ○は L1 なしで効く．危ないと思ったら○を押す．
- △は，手で動かされた腕の目標を今の姿勢に合わせてからサーボを入れる．
- `/joy` が 0.3 秒来ないと (コントローラが抜けた，電池が切れた)，`jedy_joy` は台車を止め，首を止める．

`/joy` の番号 (`axes[i]`，`buttons[i]`) は次のとおりである (Ubuntu 24.04 のカーネル 6.8 の hid-sony ドライバの場合)．
PS3 と PS4 で違うのは十字キーだけで，PS3 ではボタン (13〜16)，PS4 では軸 (6，7) になる．`jedy_joy` はどちらも読む．

| 番号 | `axes` | `buttons` (PS3) | `buttons` (PS4) |
| --- | --- | --- | --- |
| 0 | 左スティック 左右 (左が +) | × | × |
| 1 | 左スティック 上下 (上が +) | ○ | ○ |
| 2 | L2 (アナログ) | △ | △ |
| 3 | 右スティック 左右 (左が +) | □ | □ |
| 4 | 右スティック 上下 | L1 | L1 |
| 5 | R2 (アナログ) | R1 | R1 |
| 6 | (PS4 のみ) 十字キー 左右 | L2 | L2 |
| 7 | (PS4 のみ) 十字キー 上下 | R2 | R2 |
| 8 | | SELECT | SHARE |
| 9 | | START | OPTIONS |
| 10 | | PS | PS |
| 11 | | L3 (左スティック押し込み) | L3 |
| 12 | | R3 (右スティック押し込み) | R3 |
| 13 | | 十字キー 上 | |
| 14 | | 十字キー 下 | |
| 15 | | 十字キー 左 | |
| 16 | | 十字キー 右 | |

番号はコントローラの種類やドライバで変わることがある．
次で，押したボタンや倒したスティックの番号と，`jedy_joy` がそれを何に使うかを表示できる．

```{code-block} console
$ ros2 run jedy_bringup jedy_joy_monitor   # 全部の axes と buttons を表示し続ける
$ ros2 run jedy_bringup jedy_joy_buttons   # 動かしたものだけを 1 行ずつ表示する
```

## Gazebo での操縦

Gazebo (`jedy_gazebo.launch.py`) はコントローラのノードを起動しない．

### キーボード

{doc}`../robot-programming-1-2026` と同じく，`teleop_twist_keyboard` を使う．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true   # 別の端末で
```

`stamped:=true` を忘れると時刻なしの `Twist` が送られ，台車は受け取らない．
キーの意味は {doc}`../robot-programming-1-2026` の表を見よ (`i` 前進，`J` / `L` 横，`j` / `l` 旋回，`k` 止まる)．

### コントローラ

PC にコントローラを USB でつなげば，Gazebo でも同じ割り当てで操縦できる．
Gazebo を起動した後，別の端末で次を起動する．

```{code-block} console
$ ros2 launch jedy_bringup jedy_joy.launch.py
```

Gazebo にもサーボの ON / OFF があるので，○と△も実機と同じに動く．

## 動作の確認

### コントローラが見えているか

まず，コントローラが `/dev/input/js0` として見えているかを確かめる．

```{code-block} console
$ ls /dev/input/js*
/dev/input/js0
```

`jstest-gtk` を使うと，ボタンやスティックの状態を画面で見られる．

```{code-block} console
$ sudo apt install jstest-gtk
$ jstest-gtk
```

### `/joy` と `/cmd_vel`

`jedy_joy.launch.py` (または `jedy_fullbody_all.launch.py`) を起動した状態で，別の端末で `/joy` を見る．

```{code-block} console
$ ros2 topic echo /joy
header:
  ...
axes:
- 0.0
- 1.0
...
buttons:
- 0
- 0
- 0
- 0
- 1
...
```

`axes` にスティックの値 (-1〜1)，`buttons` にボタンの状態 (0 か 1) が入る．
上の例は L1 (`buttons[4]`) を押しながら左スティックを上 (`axes[1]`) に倒したところである．

台車への指令は `/cmd_vel` で見る．

```{code-block} console
$ ros2 topic echo /cmd_vel
header:
  ...
twist:
  linear:
    x: 0.1562975
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
```

`jedy_joy` は L1 を押している間だけ `/cmd_vel` を出す (離したときに止める指令を 1 回出す)．
L1 を押していなければ何も出ないので，プログラムの `ri.go_velocity()` とぶつからない．

## Python で `/joy` を読む

`jedy_skrobot` には `/joy` を読むメソッドは無い．rclpy で `/joy` を subscribe する．
`ri` は rclpy のノードでもあるので，`ri.create_subscription()` で受け取れる．
コールバックは `ri` の中の別のスレッドで呼ばれる．

```{code-block} python
from jedy_skrobot import connect
from sensor_msgs.msg import Joy

robot, ri = connect()
last = {'msg': None}


def on_joy(msg):
    last['msg'] = msg   # 最後に来た /joy を覚えておく


ri.create_subscription(Joy, '/joy', on_joy, 10)

while True:
    msg = last['msg']
    if msg is not None and msg.buttons[0] == 1:   # × ボタン
        print('× が押された')
        break
    ri.sleep(0.05)
```

ファイルに書いたときは，jedy_skrobot の仮想環境の python で実行する．

```{code-block} console
$ $(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/python ~/enshu/my_joy.py
```

`jedy_joy` が動いていると，L1 + スティックや○ / △はそちらでも効く．
自分のプログラムでは，`jedy_joy` が使っていないボタン (× だけ，□，SHARE，OPTIONS など) を使うとよい．
台車まで自分のプログラムで動かしたいときは，`joy:=false` で起動し，`joy_linux_node` だけを起動する．

```{code-block} console
$ ros2 run joy_linux joy_linux_node
```

ロボットを使わず，`/joy` を表示するだけなら rclpy だけで書ける．

```{code-block} python
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


def on_joy(msg):
    print('axes', [round(a, 2) for a in msg.axes], 'buttons', list(msg.buttons))


rclpy.init()
node = Node('my_joy_reader')
node.create_subscription(Joy, '/joy', on_joy, 10)
try:
    rclpy.spin(node)   # Ctrl-C で終わる
except (KeyboardInterrupt, ExternalShutdownException):
    pass
```

## Bluetooth で無線接続する

USB の代わりに Bluetooth でつなぐこともできる．PS3 のコントローラは，最初に一度 USB でつないでペアリングする．

### ClassicBondedOnly の設定 (1 回だけ)

PS3 のコントローラは Ubuntu の既定の設定では Bluetooth でつながらず，{numref}`fig:ps3-bluetooth-paring-failed` のような PIN の確認画面が出る．
`/etc/bluetooth/input.conf` の `[General]` に `ClassicBondedOnly=false` を書き，Bluetooth を再起動する．

:::{figure} ../fig/ps3-bluetooth-paring-failed.jpg
:align: center
:name: fig:ps3-bluetooth-paring-failed

この PIN の確認画面が出るときは，ClassicBondedOnly の設定が要る
:::

```{code-block} console
$ sudoedit /etc/bluetooth/input.conf     # [General] の下に ClassicBondedOnly=false を書く
$ grep ClassicBondedOnly /etc/bluetooth/input.conf
ClassicBondedOnly=false
$ sudo systemctl restart bluetooth
```

### ペアリング

1. ターミナルで `gnome-control-center bluetooth` を実行し，Bluetooth を ON にする．
2. コントローラを USB で PC につなぎ，PS ボタン (中央のボタン) を押す．
3. {numref}`fig:ps3-bluetooth-paring` のような確認が出たら「Allow」を押す．
4. USB を抜き，もう一度 PS ボタンを押す．LED が点灯すれば Bluetooth でつながっている．

:::{figure} ../fig/ps3-bluetooth-paring.jpg
:align: center
:name: fig:ps3-bluetooth-paring

Bluetooth の接続の確認．「Allow」を押す
:::

PS4 のコントローラは，SHARE ボタンと PS ボタンを同時に，ライトバーが速く点滅するまで長押しし，設定画面のデバイスの一覧に出る「Wireless Controller」を選んでペアリングする．

つながると，USB のときと同じく `/dev/input/js0` ができる．`ls /dev/input/js*` と `ros2 topic echo /joy` で確かめる．
Bluetooth では，ボタンの番号が USB のときと同じかを `ros2 run jedy_bringup jedy_joy_monitor` で確かめてから走らせること．
