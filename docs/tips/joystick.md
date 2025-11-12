# ジョイスティックコントローラによる操縦

無線ジョイスティックコントローラを用いた操縦について紹介する．
本演習では，DualShock 3（PS3コントローラー）やその互換コントローラーを使用してロボットを操作する．

::::{grid} 2
:gutter: 2

:::{grid-item}
:::{figure} ../fig/ps3joy_battery_chargings.jpg
:align: center
:name: fig:ps3joy-battery-charging

PS3ジョイスティックのバッテリー充電
:::

:::{grid-item}
:::{figure} ../fig/ps3joy-controllers.jpg
:align: center
:name: fig:ps3joy-controllers

PS3ジョイスティックコントローラ．真ん中の丸いボタンが`Pairing`ボタンである．
:::

::::

## 有線接続での動作確認

Bluetooth接続の前に，まず有線（USB）接続でコントローラーが正常に動作することを確認する．
これにより，コントローラー自体の動作確認とROS 2との連携テストを行うことができる．

### コントローラーの接続

{numref}`fig:ps3joy-battery-charging`のようにDualShockコントローラーをUSBケーブルでPCに接続する．

### jstest-gtkでの動作確認

`jstest-gtk`を使用してコントローラーのボタンやスティックが正しく認識されているか確認する．

```{code-block} console
$ sudo apt install jstest-gtk
$ jstest-gtk
```

jstest-gtkのウィンドウが開くと，{numref}`fig:ps3-jstest-gtk-select`のように接続されているコントローラーの一覧が表示される．

:::{figure} ../fig/ps3-jstest-gtk-select.jpg
:align: center
:name: fig:ps3-jstest-gtk-select

jstest-gtkの起動画面．接続されているコントローラー一覧が表示される
:::

コントローラーを選択すると，{numref}`fig:ps3-jstest-gtk`のような画面が表示される．
各ボタンやスティックを操作すると，画面上で対応する軸やボタンの状態がリアルタイムに更新され，正常に動作していることを確認できる．

:::{figure} ../fig/ps3-jstest-gtk.jpg
:align: center
:name: fig:ps3-jstest-gtk

jstest-gtkでのコントローラー動作確認画面．ボタンを押したりスティックを倒すと反応が表示される
:::

### ROS 2での動作確認

`teleop_twist_joy`パッケージを使用して，コントローラーからの入力がROSトピックとして出力されることを確認する．

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup joystick_teleop.launch.py
```

別のターミナルで`/joy`トピックをモニタする：

```{code-block} console
$ ros2 topic echo /joy
```

コントローラーのボタンやスティックを操作すると，`sensor_msgs/Joy`メッセージが出力される．
`axes`配列にはスティックの値，`buttons`配列にはボタンの状態（0または1）が格納される．

### ロボット操作の確認

DualShock 3コントローラーでロボットを操作する場合，以下の操作方法となる：

- **L1ボタン（enable button）を押しながら**左スティックを操作する
  - 左スティック縦（上下）：前進・後退
  - 左スティック横（左右）：回転

L1ボタンを押していない状態では，スティックを倒してもロボットは動作しない（安全機構）．

:::{figure} ../fig/jedy-gazebo-joystick.jpg
:align: center
:name: fig:jedy-gazebo-joystick

Gazebo上でL1ボタンを押しながら左スティックを倒すとロボットが動作する様子
:::

{numref}`fig:jedy-gazebo-joystick`は，Gazebo上でコントローラーを使用してロボットを操作している様子を示している．
L1ボタンを押しながら左スティックを操作することで，シミュレータ内のロボットを動かすことができる．

メカナムホイールロボットを操作する場合の起動例：

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup joystick_teleop.launch.py
```

## Bluetooth接続の設定

有線接続での動作確認が完了したら，Bluetooth接続の設定を行う．

### BluetoothデーモンのClassicBondedOnly設定

DualShock 3コントローラーをBluetooth接続する際，{numref}`fig:ps3-bluetooth-paring-failed`のようなPIN確認画面が表示されることがある．
この画面が表示される場合，`/etc/bluetooth/input.conf`で`ClassicBondedOnly`を`false`に設定する必要がある．

:::{figure} ../fig/ps3-bluetooth-paring-failed.jpg
:align: center
:name: fig:ps3-bluetooth-paring-failed

Bluetoothペアリング時のPIN確認画面．この画面が表示される場合は設定変更が必要
:::

```{code-block} console
$ sudo vim /etc/bluetooth/input.conf
# もしくは
$ sudo emacs -nw /etc/bluetooth/input.conf
```

以下のように設定する：

```
[General]
ClassicBondedOnly=false
```

設定後，Bluetoothサービスを再起動する：

```{code-block} console
$ sudo systemctl daemon-reload
$ sudo systemctl restart bluetooth
```

設定内容を確認する：

```{code-block} console
$ head /etc/bluetooth/input.conf
# Configuration file for the input service

# This section contains options which are not specific to any
# particular interface
[General]

ClassicBondedOnly=false
# Set idle timeout (in minutes) before the connection will
# be disconnect (defaults to 0 for no timeout)
#IdleTimeout=30
```

### Bluetoothが有効になっているかの確認

ターミナルで以下を実行する：

```{code-block} console
$ gnome-control-center
```

「Bluetooth」を選択し，ウィンドウの右上に「ON」と表示されているか確認する．
「OFF」と表示されている場合は，「OFF」の表示をクリックして「ON」に切り替える．

### コントローラーのペアリング

1. gnome-control-centerのBluetoothウィンドウで，「デバイス」の一覧を確認する
2. DualShock 3コントローラーのPSボタン（中央のプレステマークのボタン）を長押しして，ペアリングモードにする
3. PC側でコントローラーが検出されたら，{numref}`fig:ps3-bluetooth-paring`のような確認ダイアログが表示される．「Allow」をクリックして接続を許可する

:::{figure} ../fig/ps3-bluetooth-paring.jpg
:align: center
:name: fig:ps3-bluetooth-paring

Bluetooth接続確認ダイアログ．「Allow」をクリックして接続を許可する
:::

接続が成功すると，コントローラーのLEDが点灯する．

### 接続確認

Bluetooth接続後，有線接続時と同様にコントローラーの動作を確認する．

まず，`jstest-gtk`で無線接続されたコントローラーが認識されているか確認する：

```{code-block} console
$ jstest-gtk
```

{numref}`fig:ps3-bluetooth-jstest`のように，無線接続されたコントローラーがデバイス一覧に表示されれば，Bluetooth接続が成功している．

:::{figure} ../fig/ps3-bluetooth-jstest.jpg
:align: center
:name: fig:ps3-bluetooth-jstest

Bluetooth接続が成功すると，jstest-gtkで無線接続されたコントローラーが認識される
:::

次に，`joy_linux`ノードを起動する：

```{code-block} console
$ ros2 run joy_linux joy_linux_node
```

以下のようなメッセージが表示され，ジョイスティックが認識されたことが確認できる：

```
[WARN] [timestamp] [joy_node]: Couldn't open joystick force feedback: Bad file descriptor
[INFO] [timestamp] [joy_node]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
```

または，`teleop_twist_joy`パッケージを使用して確認する：

```{code-block} console
$ ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3'
```

別のターミナルで`/joy`トピックを確認：

```{code-block} console
$ ros2 topic echo /joy
```

Bluetooth接続でも`/joy`トピックが正常に出力されれば，設定完了である．

## ジョイスティック操縦

ジョイスティックで台車ロボットを操縦する方法について説明する．

jedy_bringup.launch.pyが起動している状態で，ロボットPCの別のターミナルで以下を実行する：

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup joystick_teleop.launch.py
```

`/dev/input/js0`が見つからず，ジョイスティックが`/dev/input/js1`として認識されている場合は，以下のように指定する：

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

ジョイスティックコントローラの**L1を押しながら左ジョイスティックを前後左右にすると**，前進後退，旋回を行う．

本プログラムでも，別のターミナルで以下を実行すると，指令速度がトピックとして送られていることが確認できる：

```{code-block} console
$ ros2 topic echo /cmd_vel
```

このように，異なる操縦デバイスであっても，共通のトピックを使いロボットの移動行動を実現している．

## ジョイスティックのEusLispサンプル

ジョイスティックに関するトピックである`/joy`をEusLispでsubscribeするサンプルについて紹介する．
これは次回以降の講義で，アームの操縦プログラムを開発する際に必要となる．

ロボットPCのターミナルで以下を実行する：

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch jedy_bringup joystick_teleop.launch.py
```

別のターミナルで以下を実行する：

```{code-block} console
$ roscd jedyeus/euslisp
$ roseus joy-sample.l
```

circle-buttonに応答するサンプルプログラムが起動する．Enterで終了する．

EusLisp REPLで以下のコマンドを実行すると，全ボタンの状態を表示できる：

```{code-block} lisp
;; 全ボタンの状態を表示
irteusgl$ (sample-ps3joy)

;; :circle-buttonの他に，ジョイスティック・十字キー・ボタンに対応したメソッド一覧を表示
irteusgl$ (send *joy* :axis-names)
irteusgl$ (send *joy* :button-names)
```

EusLispでps3joyの値がとれて表示されているのがわかる．
あとは，この値をEusLisp上で何らかの処理と対応付ければ，ジョイスティックに応じてロボットが動くプログラムが作れる．

どのボタンの値がとれるかは，`(send *joy* :button-names)`や`(send *joy* :axis-names)`で調べたり，`(sample-ps3joy)`関数の出力を見ながら調べるとよい．
