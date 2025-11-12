# ROS 2 コマンドラインツール

## 概要

ROS 2では，さまざまなコマンドラインツールを使用してノードやトピック，サービスなどを管理・デバッグできる．ROS 2はマスターレスアーキテクチャを採用しているため，ROS 1で必要だった`roscore`の起動は不要である．

ROS 2のコマンドラインツールについての詳細は，[公式ドキュメント](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Command-Line-Tools.html)を参照すること．

## 主要コマンド

### ros2 run

`ros2 run`コマンドは，任意のパッケージの中の実行ファイルを，その場所に移動することなく実行するためのコマンドである．

```{code-block} console
# 使用法
$ ros2 run <package_name> <executable_name>

# 例：talkerノードを実行
$ ros2 run demo_nodes_cpp talker

# 例：listenerノードを実行
$ ros2 run demo_nodes_cpp listener
```

### ros2 launch

`ros2 launch`コマンドは，launchファイルに記載された複数のノードを一度に起動するためのコマンドである．ROS 2では，launchファイルをPython，XML，YAMLで記述できる．

```{code-block} console
# 使用法
$ ros2 launch <package_name> <launch_file>

# 例：MoveIt2のデモを起動
$ ros2 launch moveit2_tutorials demo.launch.py
```

詳細は[Launch Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)を参照すること．

### ros2 node

`ros2 node`コマンドは，実行中のノード情報やノード間の接続を確認するためのコマンドである．

```{code-block} console
# 現在実行中のノード一覧を表示
$ ros2 node list

# ノードの詳細情報を表示（購読・出版しているトピックなど）
$ ros2 node info <node_name>

# ライフサイクル管理に対応したノードを終了
$ ros2 lifecycle set <node_name> shutdown
```

通常のノードはCtrl+Cで終了することも可能である．

### ros2 topic

`ros2 topic`コマンドは，実行中のトピックの情報やトピックに送られたメッセージを表示できる．

```{code-block} console
# トピック一覧を表示
$ ros2 topic list

# トピックのメッセージをリアルタイムで表示
$ ros2 topic echo <topic_name>

# トピックの詳細情報を表示
$ ros2 topic info <topic_name>
$ ros2 topic info <topic_name> -v  # より詳細な情報

# トピックにメッセージを送信
$ ros2 topic pub <topic_name> <msg_type> <data>

# 例：chatterトピックのメッセージを表示
$ ros2 topic echo /chatter
```

### ros2 service

`ros2 service`コマンドは，実行中のサービスに関する情報を表示し，サービスを呼び出すことができる．

```{code-block} console
# サービス一覧を表示
$ ros2 service list

# サービスの型を表示
$ ros2 service type <service_name>

# サービスを呼び出す
$ ros2 service call <service_name> <service_type> <data>

# 例：2つの整数を加算するサービスを呼び出す
$ ros2 service call /add_two_ints \
  example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### ros2 interface

`ros2 interface`コマンドは，メッセージ，サービス，アクションのデータ構造を表示する．ROS 2では統一された`interface`コマンドでこれらすべてを扱える．

```{code-block} console
# 利用可能なインターフェース一覧を表示
$ ros2 interface list

# インターフェースの詳細を表示
$ ros2 interface show <interface_name>

# 例：画像メッセージの構造を表示
$ ros2 interface show sensor_msgs/msg/Image

# 例：サービスの構造を表示
$ ros2 interface show example_interfaces/srv/AddTwoInts
```

### ros2 pkg

`ros2 pkg`コマンドは，パッケージに関する情報を取得するためのコマンドである．

```{code-block} console
# インストール済みパッケージ一覧を表示
$ ros2 pkg list

# パッケージのパスを表示
$ ros2 pkg prefix <package_name>

# パッケージの実行可能ファイル一覧を表示
$ ros2 pkg executables <package_name>

# 例：demo_nodes_cppパッケージの情報を取得
$ ros2 pkg prefix demo_nodes_cpp
$ ros2 pkg executables demo_nodes_cpp
```

### ros2 param

`ros2 param`コマンドは，ノードのパラメータに値をセットしたり取得したりできる．パラメータはYAML形式で扱われる．

```{code-block} console
# パラメータ一覧を表示
$ ros2 param list

# パラメータの値を取得
$ ros2 param get <node_name> <param_name>

# パラメータの値を設定
$ ros2 param set <node_name> <param_name> <value>

# 例：teleop_turtleノードのパラメータを操作
$ ros2 param get /teleop_turtle scale_linear
$ ros2 param set /teleop_turtle scale_linear 3.0
```

### ros2 bag

`ros2 bag`コマンドは，トピックの記録や再生を行うためのコマンドである．トピックをバッグファイルに保存し，後で再生することができる．

```{code-block} console
# トピックを記録
$ ros2 bag record <topic_name>

# 複数のトピックを記録
$ ros2 bag record <topic1> <topic2> <topic3>

# すべてのトピックを記録
$ ros2 bag record -a

# バッグファイルを再生
$ ros2 bag play <bag_file>

# バッグファイルの情報を表示
$ ros2 bag info <bag_file>

# 例：cmd_velトピックを記録して再生
$ ros2 bag record /turtle1/cmd_vel
$ ros2 bag play my_bag
$ ros2 bag info my_bag
```

### ros2 daemon

`ros2 daemon`コマンドは，ROS 2のバックグラウンドデーモンを管理する．デーモンはノードディスカバリーを支援する．

```{code-block} console
# デーモンの状態を確認
$ ros2 daemon status

# デーモンを停止
$ ros2 daemon stop

# デーモンを開始
$ ros2 daemon start
```

## ヘルプの表示

各コマンドの詳細なヘルプは，`--help`オプションで表示できる．

```{code-block} console
# 例：ros2 runのヘルプを表示
$ ros2 run --help

# 例：ros2 topicのサブコマンド一覧を表示
$ ros2 topic --help
```

## 参考情報

- [ROS 2 Command Line Tools](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Command-Line-Tools.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
