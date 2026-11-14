# 環境構築

本演習を開始する前に，必要なソフトウェア環境を構築する必要がある．
本ページでは，初回のワークスペース構築方法と，演習開始時のソフトウェア更新方法について説明する．

## 前提条件

2026 年度の演習は **ROS 2 (Jazzy) と Python** だけを使う (ROS 1・EusLisp・ros1_bridge は使わない)．
Ubuntu 24.04 に ROS 2 Jazzy が入っていることを前提とする．入っていなければ {doc}`tips/ros2-jazzy-installation` を見よ．

## 初回環境構築

```{code-block} console
$ sudo apt update
$ sudo apt upgrade -y   # 入っている ROS 2 を新しくする (古いとビルドが止まる)
$ sudo apt install -y python3-vcstool python3-rosdep ros-jazzy-realsense2-camera byobu
$ curl -LsSf https://astral.sh/uv/install.sh | sh   # Python の仮想環境を作る uv (1 回だけ)
$ source $HOME/.local/bin/env
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ source /opt/ros/jazzy/setup.bash
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.jazzy-skrobot -O- | vcs import
```

### AI TA の設定

演習では，演習資料をもとに質問に答える AI TA を使う．AI TA は上の `vcs import` で取ってきた演習資料を読むので，
**ここで設定しておくと，この後のビルドでエラーが出たときに `ta fix` で聞ける．**
演習のアンケートで登録した GitHub アカウントが必要である．

```{code-block} console
$ curl -fsSL https://raw.githubusercontent.com/iory/ai-ta-for-student/main/install.sh | bash
$ source ~/.bashrc
$ ta login    # 表示されたコードを https://github.com/login/device に入力して承認する
$ ta doctor   # 全部 ✓ になればよい
```

作業は byobu ({doc}`tips/byobu`) の中のターミナルで行うとよい．`ta fix` がターミナルの画面も AI TA に渡せる．

使い方と注意は {doc}`tips/ai-ta` を見ること．

### ビルド

```{code-block} console
$ cd ~/ros2_ws/
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

- `sudo apt upgrade` を省くと，入っている ROS 2 が古いままになる．`rosdep install` は足りないパッケージを入れるだけで，入っているものは新しくしない．
  古い ROS 2 Jazzy では `speech_recognition_msgs` のビルドが `Unknown CMake command "rosidl_auto_generate_interfaces"` で止まる．
- `--packages-up-to` の後に並べた，演習に使うパッケージ (ロボットとシミュレータ，`robot, ri`，カメラの認識，音声認識) とそれが使うものだけをビルドする．
  ワークスペースには演習で使わないパッケージ (jsk_3rdparty の他のパッケージ，メカトロボットなど) も入っているが，それらはビルドしない．
- 音声合成 (VOICEVOX) は，エンジン (約 1.3 GB) のダウンロードに時間がかかるので，ここではビルドしない．
  ロボットに声でしゃべらせるときに，{doc}`robot-programming-1-2026` の「声でしゃべる・聞く」の手順でビルドする．
- ビルドの最後に `N packages had stderr output` と出ることがあるが，uv の進み具合などの表示で，失敗ではない．
  失敗したときは `Failed` と出て，`Summary` に `failed` のパッケージが並ぶ．
- `--skip-keys` の 3 つは，本演習で使わないパッケージ (`mechatrobot_ros2`，`jsk_data`) の依存で，rosdep では入らない．
  付けないと `rosdep install` が最後に `ERROR` を出すが，演習には影響しない．

```{important}
**初回のビルドは時間がかかるので，演習の前に済ませておくこと．**
Python のパッケージ (scikit-robot，カメラの認識に使う torch・YOLOE・MediaPipe など) を uv で仮想環境に入れ，
認識のモデルをダウンロードする．
カメラの認識の `jedy_vision` だけで 15 分ほどかかった (ノート PC，ダウンロード済みのものを除く)．
```

ビルドしたら，どのターミナルでも最初に読み込む設定を `~/.bashrc` に書いておく (**1 回だけ**．2 回実行すると同じ行が 2 回書かれる)．
これで，新しく開いたターミナルでは自動で読み込まれる．

```{code-block} console
$ echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
$ echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc   # ROS 2 の通信を自分の PC の中だけにする
$ source ~/.bashrc   # 今開いているターミナルにも読み込む
```

書けたかは `tail -2 ~/.bashrc` で確かめられる．

:::{figure} fig/jedy-ki-build.gif
:align: center
:name: fig:jedy-ki-build

学科 PC で `rosdep install` から `~/.bashrc` への書き込みまで (ビルドは 4 分 20 秒，10 倍速)
:::

`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` は，ROS 2 の通信を自分の PC の中だけに閉じる設定である．
ロボット (実機) は自分の PC に USB でつなぎ，ノードはすべて自分の PC の中で動くので，
同じネットワークにいる他の人のロボットやシミュレータとトピックが混ざらない．

## 演習開始時のソフトウェア更新

演習の日には，最新のソフトウェアを取得してビルドし直す．

```{code-block} console
$ cd ~/ros2_ws/src
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.jazzy-skrobot -O- | vcs import
$ vcs pull
$ rosdep update
$ cd ~/ros2_ws/
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

## トラブルシューティング

### `colcon build` が「`uv` not found on PATH」で失敗する

uv が入っていないか，PATH に無い．上の手順の `curl -LsSf https://astral.sh/uv/install.sh | sh` と
`source $HOME/.local/bin/env` を実行してから，もう一度 `colcon build` する．

### `apt upgrade` の後のビルドが `No rule to make target '/opt/ros/jazzy/lib/...'` で止まる

```{code-block} console
gmake[2]: *** No rule to make target '/opt/ros/jazzy/lib/libfastcdr.so.2.2.5', needed by '...'.  Stop.
```

前のビルドが覚えている ROS 2 のライブラリが，`apt upgrade` で新しい版に置き換わった．ビルドの途中結果を消して，ビルドし直す．

```{code-block} console
$ cd ~/ros2_ws/
$ rm -rf build install log
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

VOICEVOX をビルドしてあった人は，`install` を消すと VOICEVOX も消えるので，使うときにもう一度ビルドする ({doc}`robot-programming-1-2026` の「声でしゃべる・聞く」)．

### より詳細なトラブルシューティング

その他の問題については，[環境構築トラブルシューティング](tips/troubleshooting.md)を参照する．

## 次のステップ

環境構築が完了したら，以下の演習に進む：

- {doc}`robot-programming-1-2026`
- {doc}`robot-programming-2-2026`
- {doc}`robot-programming-3-2026`
