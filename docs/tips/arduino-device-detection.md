# Arduinoが認識されないときの確認方法

Arduino UNO R4 WiFi をUSBで接続したのにArduino IDEに表示されない場合や，`/dev/ttyACM0`が見つからない場合の確認手順をOS別にまとめる．

## Ubuntu の場合

### デバイスファイルの確認

接続後に `/dev/ttyACM*` または `/dev/ttyUSB*` が現れているかを確認する．

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Arduino UNO R4 WiFi は通常 `/dev/ttyACM0` として認識される．何も表示されない場合は次の `dmesg` を確認する．

### dmesg でカーネルログを確認

ターミナルを2つ開く，または `--follow` を使ってリアルタイムにログを監視する：

```bash
sudo dmesg --follow
```

この状態でArduinoのUSBケーブルを抜き差しすると，次のようなログが表示されるはずである：

```text
usb 1-1: new full-speed USB device number 5 using xhci_hcd
usb 1-1: New USB device found, idVendor=2341, idProduct=1002
usb 1-1: Product: UNO R4 WiFi
cdc_acm 1-1:1.0: ttyACM0: USB ACM device
```

- `idVendor=2341` がArduinoの公式ベンダーID（=`0x2341`）である
- `idProduct=1002` がUNO R4 WiFiの製品ID（=`0x1002`）である
- `ttyACM0` の行が出ていれば，デバイスファイル `/dev/ttyACM0` として正常に認識されている
- 一切ログが出ない場合は，USBケーブル（充電専用ケーブルになっていないか）やUSBハブを疑う

```{note}
Arduino UNO R4 WiFi はESP32-S3モジュール側のファームウェアによって `Product:` の表示が変わる：

- 通常のCDCシリアル設定（出荷時）: `UNO R4 WiFi`
- CMSIS-DAPデバッガ設定: `UNO WiFi R4 CMSIS-DAP`

どちらでも `idVendor=2341, idProduct=1002` は変わらない．`ttyACM*` も同様に作成されるので，書き込み・シリアル通信に支障はない．
```

### lsusb でUSBデバイス一覧を確認

```bash
lsusb | grep -i arduino
```

`Bus 001 Device 005: ID 2341:1002 Arduino SA` のような表示があればOSはデバイスを認識している．

### 権限エラー (`Permission denied`) の対処

`"can't open device "/dev/ttyACM0": Permission denied"` のようなエラーが出る場合，ユーザを `dialout` グループに追加する：

```bash
sudo usermod -a -G dialout $USER
```

実行後はログアウトし直すか，再起動が必要．

## macOS の場合

### デバイスファイルの確認

macOSでは `/dev/cu.usbmodem*` または `/dev/tty.usbmodem*` として認識される．`*` の部分にはArduinoのシリアル番号＋インタフェース番号が入る（例: `/dev/cu.usbmodemB43A45B9F1CC2`）．

```bash
ls /dev/cu.usbmodem* /dev/tty.usbmodem* 2>/dev/null
```

シリアル通信プログラムからは原則として `/dev/cu.usbmodem*` を使う（`tty.usbmodem*` はキャリア信号を待つためブロックする場合がある）．Arduino IDE で表示されるポート名がそのまま使える．

### system_profiler でUSBデバイスを確認

macOS 15 Sequoia / 26 Tahoe 以降では `SPUSBHostDataType` を使う：

```bash
system_profiler SPUSBHostDataType | grep -B 1 -A 10 -i "uno\|arduino"
```

macOS 14 Sonoma 以前では `SPUSBDataType` を使う：

```bash
system_profiler SPUSBDataType | grep -B 1 -A 10 -i "uno\|arduino"
```

認識されていると次のような表示になる：

```text
UNO WiFi R4 CMSIS-DAP:    # または "UNO R4 WiFi"（ESP32側のファームウェアによる）
  Manufacturer: Arduino
  Serial Number: B43A45B9F1CC
  USB Vendor ID: 0x2341
  USB Product ID: 0x1002
```

### log show でUSBイベントを確認

ログをリアルタイムに監視する場合（実行後にUSBを抜き差し）：

```bash
log stream --predicate 'subsystem == "com.apple.iokit.IOUSBHostFamily"' --info
```

直近1分のUSBログを確認する場合：

```bash
log show --predicate 'subsystem == "com.apple.iokit.IOUSBHostFamily"' --info --last 1m
```

### ioreg でUSBツリーを確認

```bash
ioreg -p IOUSB -l -w 0 | grep -B 1 -A 8 -iE "uno|arduino|cmsis"
```

`USB Product Name`，`USB Vendor Name = "Arduino"`，`idVendor = 9025`（=`0x2341`），`idProduct = 4098`（=`0x1002`）が見つかれば認識済み．

## Windows の場合

1. デバイスマネージャーを開く（スタートメニューで「デバイスマネージャー」と検索）
2. 「ポート (COM と LPT)」の項目を展開
3. `Arduino UNO R4 WiFi (COM*)` のような表示があれば認識済み
4. 「ほかのデバイス」や「不明なデバイス」として黄色い警告マーク付きで表示される場合は，ドライバが正しくインストールされていない

ドライバが入っていない場合，Arduino IDE 経由で「Arduino UNO R4 Boards」をインストールすることで自動的にドライバも導入される．

## それでも認識されないとき

1. **別のUSBケーブルで試す**：充電専用ケーブルではデータ通信ができない．ケーブルが原因のトラブルが最も多い
2. **別のUSBポートで試す**：特にUSBハブを経由している場合はPC直挿しで確認する
3. **別のPCで試す**：PC側ではなくArduino側の問題かを切り分ける
4. **二度押しリセットでブートローダーモードに入る**：スケッチがプロセッサをロックしてUSBが見えなくなった場合は，リセットボタンを素早く2回押すと書き込み可能な状態に戻る．詳細は{doc}`bootloader`を参照
