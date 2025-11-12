# rosserialトラブルシューティング

## 概要

rosserial_pythonを使用してArduino（Seeeduino Nano）とROSを接続する際によく発生するトラブルとその解決方法をまとめる．

## デバイスが見つからないエラー

rosserial_pythonを起動した際に以下のようなエラーが表示される場合がある．

```{code-block} console
$ rosrun rosserial_python serial_node.py
[INFO] [1762030385.203196]: ROS Serial Python Node
[INFO] [1762030385.209614]: Connecting to /dev/ttyUSB0 at 57600 baud
[ERROR] [1762030385.210762]: Error opening serial: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0'
```

このエラーは主に以下の原因で発生する．

### 原因1: デバイスファイルが存在しない

Seeeduino NanoがPCに接続されていないか認識されていない可能性がある．

**確認方法：**

1. USBケーブルが正しく接続されているか確認する
2. 以下のコマンドでデバイスファイルの存在を確認する

```{code-block} console
$ ls /dev/ttyUSB*
```

デバイスが認識されていれば`/dev/ttyUSB0`や`/dev/ttyUSB1`などと表示される．何も表示されない場合はデバイスが認識されていない．

### 原因2: デバイスファイル名が異なる

環境によってはSeeeduino Nanoが`/dev/ttyUSB1`や`/dev/ttyACM0`として認識される場合がある．

**解決方法：**

正しいデバイスファイル名を`_port`引数で指定する．

```{code-block} console
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1
```

または

```{code-block} console
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
```

### 原因3: ボーレートの不一致

ArduinoスケッチとROSノードのボーレート（通信速度）が一致していない可能性がある．

**解決方法：**

Arduinoスケッチ内の`Serial.begin()`で指定されているボーレートを確認し同じ値を`_baud`引数で指定する．

rosserialのデフォルトボーレートは57600bpsだがArduinoスケッチで115200bpsを使用している場合は以下のように指定する．

```{code-block} console
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
```

## デバイス接続状況の確認方法

Seeeduino Nanoがどのデバイスファイル名で認識されているか確実に確認するには`dmesg`コマンドを使用する．

### dmesgによるリアルタイム監視

以下のコマンドでデバイスの接続状況をリアルタイムで監視できる．

```{code-block} console
$ sudo dmesg --follow
```

このコマンドを実行した状態でSeeeduino NanoのUSBケーブルを抜き差しすると以下のようなログが表示される．

**接続時のログ例：**

```
[12196.298656] usb 3-5: new full-speed USB device number 8 using xhci_hcd
[12196.424562] usb 3-5: New USB device found, idVendor=10c4, idProduct=ea60, bcdDevice= 1.00
[12196.424573] usb 3-5: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[12196.424577] usb 3-5: Product: CP2102N USB to UART Bridge Controller
[12196.424580] usb 3-5: Manufacturer: Silicon Labs
[12196.424583] usb 3-5: SerialNumber: 9020ed9b1e76ee11b763719db16f70a1
[12196.427283] cp210x 3-5:1.0: cp210x converter detected
[12196.430313] usb 3-5: cp210x converter now attached to ttyUSB0
```

最後の行に`cp210x converter now attached to ttyUSB0`と表示されており`/dev/ttyUSB0`として認識されたことが分かる．

### 確認手順

1. 別のターミナルで`sudo dmesg --follow`を実行する

2. Seeeduino NanoのUSBケーブルを一度抜く

3. 再度USBケーブルを接続する

4. ログに表示される`attached to ttyUSBX`の部分を確認する

5. 確認できたら`Ctrl+C`で`dmesg --follow`を終了する

6. 確認したデバイスファイル名を使用してrosserial_pythonを起動する

```{code-block} console
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
```

## その他のトラブルシューティング

### roscoreが起動していない

rosserial_pythonを起動した際に以下のようなメッセージが繰り返し表示される場合がある．

```{code-block} console
$ rosrun rosserial_python serial_node.py
Unable to register with master node [http://localhost:11311]: master may not be running yet. Will keep trying.
```

このエラーはROSのマスターノード（roscore）が起動していないことを示している．

**解決方法：**

別のターミナルでroscoreを起動する．

```{code-block} console
$ source /opt/ros/one/setup.bash
$ roscore
```

roscoreが起動すると自動的にrosserial_pythonがマスターノードに接続される．

### パーミッションエラー

デバイスファイルへのアクセス権限がない場合以下のようなエラーが表示されることがある．

```
[ERROR] Permission denied: '/dev/ttyUSB0'
```

**解決方法：**

ユーザーを`dialout`グループに追加する．

```{code-block} console
$ sudo usermod -aG dialout $USER
```

この後ログアウトして再ログインする必要がある．

### Arduino IDEとの競合

Arduino IDEのシリアルモニタを開いたままrosserial_pythonを起動すると通信ポートが競合してエラーが発生する．

**解決方法：**

Arduino IDEのシリアルモニタを閉じてからrosserial_pythonを起動する．

## まとめ

rosserialの接続トラブルが発生した場合は以下の順序で確認する．

1. roscoreが起動しているか（`Unable to register with master node`エラー）
2. デバイスファイルが存在するか（`ls /dev/ttyUSB*`）
3. 正しいデバイスファイル名を指定しているか（`dmesg --follow`で確認）
4. ボーレートが一致しているか（Arduinoスケッチと`_baud`引数）
5. パーミッションは正しいか（`dialout`グループに所属しているか）
6. 他のプログラムとポートが競合していないか（Arduino IDEのシリアルモニタなど）
