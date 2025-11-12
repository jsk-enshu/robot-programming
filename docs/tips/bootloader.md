# Arduinoのブートローダの書き込み直し

マイコンには起動直後に自動的に実行されるプログラム（ブートローダ）が焼き込まれている．
このブートローダが何らかの理由で壊れて書き込み直しする必要がある場合がある．
ブートローダが壊れてしまったArduinoは，別の正常なArduinoを使ってブートローダを書き込み直すことができる[^1].
（正常なArduinoをブートローダ書き込み機として使うことになる．）
以下にArduinoのブートローダの書き込み方法を示す．（コンデンサが必要になる．）

## 1. 正常なArduinoをブートローダ書き込み機にする

1.  Arduino IDEを起動する．

2.  「ツール」→「書き込み装置」→「Arduino as ISP」を選択する[^2].

3.  「ファイル」→「スケッチ例」→「ArduinoISP」→「ArduinoISP」で書き込み機用のスケッチを開く[^3].

4.  正常なArduinoをUSBでPCに接続する．

5.  Arduino IDEで「ツール」→「ボード」→「Arduino Nano」を選択して書き込み機となるArduino基板の種類を選択する．

6.  「ツール」→「プロセッサー」→「ATmega328P (Old Bootloader)」を選択して書き込み機上のマイコンの種類を選択する．

7.  「マイコンボードへ書き込む」（右矢印ボタン）で書き込み機用のプログラムを書き込む．

## 2. 壊れたArduinoへのブートローダの書き込み

1.  書き込み機用Arduinoと壊れたArduinoをブレッドボードに差す．

2.  書き込み機用ArduinoのGNDとResetの間にコンデンサ（10μF）を接続する．

3.  下表に従って2つのArduinoのピンを接続する[^4].

    | 書き込み機用Arduinoのピン | 壊れたArduinoのピン |
    |:-------------------------:|:--------------------:|
    |            5V             |          5V          |
    |            GND            |         GND          |
    |            D10            |         RST          |
    |            D11            |         D11          |
    |            D12            |         D12          |
    |            D13            |         D13          |

4.  Arduino IDEで「ツール」→「ボード」→「Arduino Nano」を選択して壊れたArduino基板の種類を設定する．

5.  「ツール」→「プロセッサー」→「ATmega328P (Old Bootloader)」を選択して壊れた基板のマイコンの種類を設定する．

6.  「ツール」→「ブートローダを書き込む」を押して壊れた基板にブートローダを書き込む[^5].
    2つのArduinoのLEDが点滅して書き込みが成功すれば完了である．

### 接続方法の図

:::{figure} ../fig/writing-boot-loader-of-arduino-throu-arduino.jpg
:name: arduino-bootloader-connection
:align: center

Arduinoのブートローダの書き込み時の接続方法
:::

## 注意事項

- Arduino IDE 2.x系を使用する場合は，メニューの位置が異なる場合がある．その場合は，画面上部の検索機能を使って該当する機能を探すこと．
- Seeeduino Nanoの場合も基本的に同じ手順で行えるが，ボード選択時に「Seeeduino Nano」を選択する必要がある．
- コンデンサは極性があるため，プラス側を5V，マイナス側をGNDに接続すること．
- ブートローダの書き込みが失敗する場合は，配線を再確認し，コンデンサの容量（10μF程度）が適切であることを確認すること．

## 参考資料

[^1]: <https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP>

[^2]: <https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP#use-arduino-as-isp>

[^3]: <https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP#load-the-sketch>

[^4]: <https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP#how-to-wire-your-boards>

[^5]: <https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP#program-the-bootloader>
