# Arduinoへのプログラム書き込み方法

Arduinoへのプログラムの書き込みは，Arduino IDEを用いて行う．Arduino
IDEはArduinoの統合開発環境であり，
プログラムエディタ，プログラムのコンパイル，マイコンへの書き込み，シリアル通信の表示機能などが統合されている．
本付録では，Arduino IDEを用いてPCとUSB接続されたArduinoマイコンへプログラムを焼きこむ方法として，配布教材であるSeeeduino
Nano（Arduino Nano完全互換機）を具体例として説明する． なお，説明は，Arduino IDEはすでにPCにインストールされているものとしている．（インストールされていない場合はに従ってインストールする．）
また，Arduinoではプログラムのことをスケッチと呼ぶため，適宜読み替えること．

## Seeeduino Nano（Arduino Nano互換機）を動かす準備

本演習で使用するSeeeduino Nanoは，Arduino Nanoの完全互換機であるため，Arduino IDEで使用するには専用のボード定義をインストールする必要がある．以下の手順でセットアップを行う．

### ボードマネージャのURL追加

1. Arduino IDEを起動する

2. 「ファイル」→「環境設定」を開く

3. 「追加のボードマネージャのURL」欄に以下のURLを貼り付ける：

```
https://raw.githubusercontent.com/Seeed-Studio/Seeed_Platform/master/package_legacy_seeeduino_boards_index.json
```

4. 「OK」をクリックして設定を保存する

### Seeeduino AVRボードのインストール

1. 「ツール」→「ボード」→「ボードマネージャ」を開く

2. 検索欄に「Seeeduino AVR」と入力

3. 検索結果に表示される「Seeeduino AVR Boards」を見つけて「インストール」をクリック

4. インストール完了まで待機する

5. インストールが完了すると，以下のように「Seeeduino AVR」がインストール済みと表示される

    ```{figure} ../fig/arduino-ide-seeeduino-installed.png
    :name: arduino-ide-seeeduino-installed
    :width: 600px

    Seeeduino AVRボードのインストール完了画面
    ```

### 参考情報

Seeeduino Nanoの詳細については，[公式ページ](https://wiki.seeedstudio.com/Seeeduino-Nano/)を参照のこと：

## プログラムの書き込み手順

### 1. Arduino IDEの起動

### 2. 書き込み設定

Arduinoマイコンへプログラムを焼きこむための設定として以下の項目を設定する．

#### シリアルポートの選択

「ツール」→「シリアルポート」→「ポート (/dev/ttyUSBx)」を選択する．

- Seeeduino NanoをUSBで接続すると，通常 `/dev/ttyUSB0` または `/dev/ttyUSB1` として認識される
- どのポートかわからない場合はUSBケーブルを抜き差ししてメニューに表示されるポートの変化を確認する

#### ボードの選択

**Arduino IDE 2.x系の場合:**

画面上部の「Select Board」ドロップダウンメニューをクリックすると，ボードとポートを同時に選択できるダイアログが表示される．

```{figure} ../fig/arduino-ide-select-board.png
:name: arduino-ide-select-board
:width: 600px

Arduino IDE 2.x系でのボードとポートの選択画面
```

1. BOARDSセクションの検索欄に「seeeduino nano」と入力
2. 表示された「Seeeduino Nano」を選択
3. PORTSセクションで対応するシリアルポート（例: `/dev/ttyUSB0 Serial Port (USB)`）を選択
4. 「OK」ボタンをクリックして設定を確定

**Arduino IDE 1.8.x系の場合:**

「ツール」→「ボード」→「Seeeduino AVR Boards」→「Seeeduino Nano」を選択する．

これにより，Seeeduino Nano用のコンパイル設定が適用される．

### 3. プログラムの実装（スケッチを開く）

IDEのエディタでプログラムを実装する．
既存のスケッチファイルを開く場合は，「ファイル」→「開く」からファイル場所を指定してプログラムを開く．

### 4. プログラムの検証及び，マイコンへの書き込み

#### 検証（コンパイル）

IDEの「検証」（チェックマークボタン）をクリックすると，プログラムのコンパイルが始まりエラーがないかの検証が行われる．

- エラーが表示される場合は該当箇所を修正する
- エラーがない場合はIDE下部に「コンパイルが完了しました．」と表示される

#### 書き込み

コンパイルが成功したらIDE左上の「マイコンボードへ書き込む」（右矢印ボタン）を押すと，マイコンへの書き込みが行われる．

- 書き込み中はSeeeduino Nano上のLEDが点滅する
- 「ボードへの書き込みが完了しました．」（Arduino IDE 1.8.x系）または「Done uploading.」（Arduino IDE 2.x系）と表示されれば成功
- エラーが表示される場合はシリアルポートやボードの設定を再確認する

```{figure} ../fig/arduino-ide-done-uploading.png
:name: arduino-ide-done-uploading
:width: 600px

Arduino IDE 2.x系での書き込み完了画面
```

書き込み完了後，出力ウィンドウにはプログラムのメモリ使用量が表示される．

#### 動作確認

書き込みが完了したマイコンは，電源が供給されている間，`loop()`関数を周期的に実行する．意図する挙動となっているかを確かめる．

## トラブルシューティング

書き込み時のエラーや問題が発生した場合は，[Arduino IDEでのエラー等の対処法](arduino-errors.md)を参照すること．
