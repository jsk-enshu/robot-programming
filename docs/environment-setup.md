# 環境構築

本演習を開始する前に，必要なソフトウェア環境を構築する必要がある．
本ページでは，初回のワークスペース構築方法と，演習開始時のソフトウェア更新方法について説明する．

## 前提条件

2026 年度の演習は **ROS 2 (Jazzy) と Python** だけを使う (ROS 1・EusLisp・ros1_bridge は使わない)．
Ubuntu 24.04 に ROS 2 Jazzy が入っていることを前提とする．入っていなければ {doc}`tips/ros2-jazzy-installation` を見よ．

## 初回環境構築

```{code-block} console
$ sudo apt update
$ sudo apt install python3-vcstool python3-rosdep ros-jazzy-realsense2-camera
$ curl -LsSf https://astral.sh/uv/install.sh | sh   # Python の仮想環境を作る uv (1 回だけ)
$ source $HOME/.local/bin/env
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ source /opt/ros/jazzy/setup.bash
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.jazzy-skrobot -O- | vcs import
$ echo "yaml file://$HOME/ros2_ws/src/robot-programming/jedy/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-jedy.list
$ rosdep update
$ cd ~/ros2_ws/
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
$ colcon build --symlink-install
$ echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc   # ROS 2 の通信を自分の PC の中だけにする
```

- `50-jedy.list` は，ROS の rosdep の一覧に無い依存 (発話の吹き出しの日本語フォント `fonts-noto-cjk`) の定義を rosdep に教える (1 回だけ)．
- `--skip-keys` の 3 つは，本演習で使わないパッケージ (`mechatrobot_ros2`，`jsk_data`) の依存で，rosdep では入らない．
  付けないと `rosdep install` が最後に `ERROR` を出すが，演習には影響しない．

```{important}
**初回のビルドは時間がかかるので，演習の前に済ませておくこと．**
Python のパッケージ (scikit-robot，カメラの認識に使う torch・YOLOE・MediaPipe など) を uv で仮想環境に入れ，
音声合成のエンジン (約 1.3 GB) と認識のモデルをダウンロードする．
カメラの認識の `jedy_vision` だけで 15 分ほどかかった (ノート PC，ダウンロード済みのものを除く)．
```

ビルドしたら，どのターミナルでも最初に次を実行する (`~/.bashrc` に書いておくとよい)．

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
```

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
$ colcon build --symlink-install
```

## トラブルシューティング

### `colcon build` が「`uv` not found on PATH」で失敗する

uv が入っていないか，PATH に無い．上の手順の `curl -LsSf https://astral.sh/uv/install.sh | sh` と
`source $HOME/.local/bin/env` を実行してから，もう一度 `colcon build` する．

### より詳細なトラブルシューティング

その他の問題については，[環境構築トラブルシューティング](tips/troubleshooting.md)を参照する．

## 次のステップ

環境構築が完了したら，以下の演習に進む：

- {doc}`robot-programming-1-2026`
- {doc}`robot-programming-2-2026`
- {doc}`robot-programming-3-2026`
