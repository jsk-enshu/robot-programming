# ROS 1 Bridge

## 概要

**ros1_bridge**はROS 1とROS 2の間でメッセージをブリッジ（橋渡し）するためのパッケージである．このパッケージを使用することでROS 1のノードとROS 2のノードを同時に実行し相互に通信させることができる．

本演習ではrosserial（ROS 1）とros2_control（ROS 2）を組み合わせたシステムを構築するためにros1_bridgeを使用する．

## 必要な環境

- ROS 1（ROS-O）がインストールされていること
- ROS 2（Jazzy）がインストールされていること

## インストール方法

以下の
[事前ビルド済みパッケージからのインストール（推奨）](#事前ビルド済みパッケージからのインストール（推奨）)か
[ソースからのビルド](#ソースからのビルド)の**いずれか**を実行する．

### 事前ビルド済みパッケージからのインストール（推奨）

ros1_bridgeをソースからビルドすると時間がかかるため，事前にビルドされた .deb パッケージを使用することを推奨する．

#### Ubuntu 24.04 (amd64) 用パッケージ

以下のGoogle Driveリンクから .deb ファイルをダウンロードする．

- [ros-jazzy-ros1-bridge_0.10.3-0noble_amd64.deb](https://drive.google.com/file/d/1jXZlvovTGa_PU6stJOznvv5ZYkarB62n/view?usp=sharing)

ダウンロード後，以下のコマンドでインストールする．

```{code-block} console
$ sudo dpkg -i ros-jazzy-ros1-bridge_0.10.3-0noble_amd64.deb
```

依存関係のエラーが出た場合は以下のコマンドで解決する．

```{code-block} console
$ sudo apt-get install -f
```

### ソースからのビルド

事前ビルド済みパッケージが使用できない場合やカスタムメッセージを使用する場合は，ソースからビルドする必要がある．

#### 1. ワークスペースの作成

```{code-block} console
$ mkdir -p ~/ros2/bridge/src
$ cd ~/ros2/bridge/src
```

#### 2. ros1_bridgeのクローン

```{code-block} console
$ git clone https://github.com/ros-o/ros1_bridge
```

#### 3. 依存パッケージのインストール

両方のROSディストリビューションの環境をsourceしてから依存パッケージをインストールする．

```{code-block} console
$ cd ~/ros2/bridge
$ source /opt/ros/one/setup.bash
$ source /opt/ros/jazzy/setup.bash
$ rosdep install --from-paths src -i -y -r
```

#### 4. ビルド

```{code-block} console
$ colcon build
```

ビルドには数分かかる場合がある．

#### 5. セットアップファイルのsource

```{code-block} console
$ source ~/ros2/bridge/install/setup.bash
```

## 使用方法

### 基本的な起動方法

ros1_bridgeを使用するには以下の3つのターミナルが必要である．

**注意：** 事前ビルド済みパッケージ（．deb）をインストールした場合は，`~/ros2/bridge/install/setup.bash` のsourceは不要である．

**ターミナル1: ROS 1のroscore**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ roscore
```

**ターミナル2: ros1_bridge**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2/bridge/install/setup.bash
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

`--bridge-all-topics`オプションを指定するとROS 1とROS 2の全てのtopicが自動的にブリッジされる．

**ターミナル3以降: ROS 1またはROS 2のノード**

ROS 1のノードを起動する場合：

```{code-block} console
$ source /opt/ros/one/setup.bash
$ rosrun <package_name> <node_name>
```

ROS 2のノードを起動する場合：

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ ros2 run <package_name> <node_name>
```

### 動作確認

ros1_bridgeが正しく動作しているか確認するにはROS 1とROS 2の両方でtopicリストを確認する．

**ROS 1でのtopic確認：**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ rostopic list
```

**ROS 2でのtopic確認：**

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ ros2 topic list
```

両方のコマンドで同じtopicが表示されていればブリッジが正しく動作している．

### ブリッジのオプション

`dynamic_bridge`には以下のオプションがある．

| オプション | 説明 |
|:-----------|:-----|
| `--bridge-all-topics` | 全てのtopicを自動的にブリッジする |
| `--bridge-all-1to2-topics` | ROS 1からROS 2へのtopicのみブリッジする |
| `--bridge-all-2to1-topics` | ROS 2からROS 1へのtopicのみブリッジする |
| `--help` | ヘルプを表示する |

ヘルプの確認：

```{code-block} console
$ ros2 run ros1_bridge dynamic_bridge --help
```

## メカトロボットでの使用例

本演習のメカトロボットではrosserial（ROS 1）でArduinoと通信しros2_control（ROS 2）でモータ制御を行う．これらを連携させるためにros1_bridgeを使用する．

**ターミナル1: roscore（ROS 1）**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ roscore
```

**ターミナル2: ros1_bridge**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2/bridge/install/setup.bash
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

**ターミナル3: rosserial（ROS 1）**

```{code-block} console
$ source /opt/ros/one/setup.bash
$ rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

**ターミナル4: ros2_control（ROS 2）**

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
$ ros2 launch mechatrobot mechatrobot_controller.launch.py
```

これによりROS 1のrosserialで受信したセンサデータをROS 2のros2_controlで使用したりROS 2の制御指令をROS 1経由でArduinoに送信したりできる．

## トラブルシューティング

### ビルドエラー

ros1_bridgeのビルド時には必ずROS 1とROS 2の両方の環境をsourceする必要がある．

```{code-block} console
$ source /opt/ros/one/setup.bash
$ source /opt/ros/jazzy/setup.bash
$ colcon build
```

### bridgeが起動しない

ros1_bridgeの起動時にもROS 1とROS 2の両方の環境とビルドしたワークスペースをsourceする必要がある．

```{code-block} console
$ source /opt/ros/one/setup.bash
$ source /opt/ros/jazzy/setup.bash
$ source ~/ros2/bridge/install/setup.bash
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### roscoreが起動していないエラー

ros1_bridgeを起動した際に以下のようなエラーメッセージが大量に表示される場合がある．

```
Error in XmlRpcClient::writeRequest: write error (Connection refused).
Error in XmlRpcDispatch::work: couldn't find source iterator
Error in XmlRpcClient::writeRequest: write error (Connection refused).
Error in XmlRpcDispatch::work: couldn't find source iterator
...
```

このエラーはros1_bridgeを起動する前にroscoreが起動していないことを示している．

**解決方法：**

別のターミナルでroscoreを起動してからros1_bridgeを起動する．

```{code-block} console
# ターミナル1: roscoreを起動
$ source /opt/ros/one/setup.bash
$ roscore
```

roscoreが起動するとエラーメッセージが停止しros1_bridgeが正常に動作する．

**注意：** ros1_bridgeは起動順序が重要である．必ず以下の順序で起動すること．

1. roscore（ROS 1）を起動
2. ros1_bridgeを起動
3. ROS 1/ROS 2のノードを起動

### topicがブリッジされない

- roscoreが起動しているか確認する
- ros1_bridgeが正しく起動しているか確認する
- メッセージ型がROS 1とROS 2の両方で定義されているか確認する

## 参考資料

- [ros1_bridge GitHub リポジトリ](https://github.com/ros-o/ros1_bridge)
- [ROS 2 Documentation - ros1_bridge](https://github.com/ros2/ros1_bridge)
