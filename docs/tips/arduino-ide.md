# Arduino IDEのインストール方法

Arduino IDEには最新版（2.x系）とLegacy版（1.8.x系）の2種類が存在する．以下では両方のインストール方法を説明する．

## 最新版（AppImage形式）のインストール

最新版のArduino IDE 2.xは，AppImage形式で提供されている．以下の手順でインストールする．

1.  <https://www.arduino.cc/en/software>から「Linux AppImage 64 bits (X86-64)」をダウンロードする

2.  AppImageの実行に必要なライブラリをインストールする

    ```bash
    sudo apt install libfuse2
    ```

3.  ダウンロードした`arduino-ide_<version>_Linux_64bit.AppImage`に実行権限を付与する

    ```bash
    chmod +x arduino-ide_<version>_Linux_64bit.AppImage
    ```

4.  AppImageを実行する

    ```bash
    ./arduino-ide_<version>_Linux_64bit.AppImage
    ```

    または，ファイルマネージャーでAppImageファイルをダブルクリックして起動することもできる．

    ```bash
    xdg-open . &  # ファイルマネージャーを開く
    ```

5.  起動後のArduino IDE 2.xは以下のような画面になる

    ```{figure} ../fig/arduino-ide-2.x.png
    :name: arduino-ide-2-x
    :width: 600px

    Arduino IDE 2.3.6の起動画面
    ```

## Legacy IDE (1.8.19) のインストール

古いバージョンのArduino IDE 1.8.19を使用する場合は，以下の手順でインストールする．

1.  <https://www.arduino.cc/en/software>の「Legacy IDE (1.8.19)」セクションから「Linux 64 bits」をダウンロードする

2.  ダウンロードした`arduino-<version>-linux64.tar.xz`を展開する（`<version>`はダウンロードしたバージョン番号）

    ```bash
    tar -xf arduino-<version>-linux64.tar.xz
    ```

3.  展開したディレクトリ内にある`install.sh`をsudoで実行する

    ```bash
    cd arduino-<version>
    sudo ./install.sh
    ```

## どちらを使うべきか

- **最新版（2.x系）**: 新機能やUIの改善が含まれており，基本的にはこちらを推奨する
- **Legacy版（1.8.19）**: 古いライブラリやボードとの互換性が必要な場合や，動作が軽量なIDEを求める場合に使用する

