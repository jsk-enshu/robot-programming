# ROS 1 コマンドラインツール

## 概要

ROS 1では，さまざまなコマンドラインツールを使用してノードやトピック，サービスなどを管理・デバッグできる．ROS 1はマスター・スレーブアーキテクチャを採用しているため，**ROSプログラムを実行する前に必ず`roscore`を起動する必要がある**.

**本演習では，ROS 1としてROS-O (ROS One)を使用する．**環境設定については，[ROS-Oインストール](ros-one-installation.md)を参照すること．

## roscore

`roscore`は，ROSのコアとなるプログラム（Master，Parameter Server，rosoutなど）を実行するコマンドである．**roscoreを起動しないとノード間の通信が行えない**ため，ROSプログラムを立ち上げる前に必ず実行する必要がある．

```{code-block} console
# roscoreの起動
$ roscore
```

roscoreを起動すると，以下のようなメッセージが表示される：

```
started roslaunch server http://hostname:xxxxx/
ros_comm version 1.xx.x

SUMMARY
========

PARAMETERS
 * /rosdistro: one
 * /rosversion: 1.xx.x

NODES

auto-starting new master
process[master]: started with pid [xxxxx]
ROS_MASTER_URI=http://localhost:11311/
...
```

## rosrun

`rosrun`コマンドは，任意のパッケージの中の実行ファイルを，その場所に移動することなく実行するためのコマンドである．

```{code-block} console
# 使用法
$ rosrun <package_name> <executable_name>

# 例：roscpp_tutorialsパッケージのtalkerを実行
$ rosrun roscpp_tutorials talker

# 例：rospy_tutorialsパッケージのlistenerを実行
$ rosrun rospy_tutorials listener
```

## roslaunch

`roslaunch`コマンドは，[roslaunch XML形式](http://wiki.ros.org/ja/roslaunch/XML)の設定ファイルに記載されたノードを開始する．複数のノードを立ち上げるシステムを使うときに必要である．

```{code-block} console
# 使用法
$ roslaunch <package_name> <launch_file>

# 例：turtlebot_gazeboパッケージのlaunchファイルを実行
$ roslaunch turtlebot_gazebo turtlebot_world.launch
```

**注意：**`roslaunch`は内部で`roscore`を自動的に起動するため，`roslaunch`を使用する場合は事前に`roscore`を起動する必要はない．

## rosnode

`rosnode`コマンドは，実行中のノード情報やノード間の接続を確認するためのコマンドである．

```{code-block} console
# 現在実行中のノード一覧を表示
$ rosnode list

# ノードの詳細情報を表示（購読・出版しているトピックなど）
$ rosnode info <node_name>

# 実行中のノードを終了
$ rosnode kill <node_name>

# ノードの接続状態を確認
$ rosnode ping <node_name>
```

詳細は[rosnode Wiki](http://wiki.ros.org/rosnode)を参照すること．

## rostopic

`rostopic`コマンドは，実行中のトピックの情報やトピックに送られたメッセージを表示できる．

```{code-block} console
# トピック一覧を表示
$ rostopic list

# トピックのメッセージをリアルタイムで表示
$ rostopic echo <topic_name>

# トピックの詳細情報を表示
$ rostopic info <topic_name>

# トピックのメッセージ型を表示
$ rostopic type <topic_name>

# トピックにメッセージを送信
$ rostopic pub <topic_name> <msg_type> <data>

# トピックの周波数を表示
$ rostopic hz <topic_name>

# 例：/chatterトピックのメッセージを表示
$ rostopic echo /chatter
```

`rostopic`コマンドと`rosmsg`コマンドを組み合わせると便利である：

```{code-block} console
# トピックの型を調べて，その型の詳細を表示
$ rostopic type /rosout | rosmsg show
```

詳細は[rostopic Wiki](http://wiki.ros.org/rostopic)を参照すること．

## rosmsg

`rosmsg`コマンドは，メッセージのデータ構造を表示する．

```{code-block} console
# メッセージの詳細を表示
$ rosmsg show <message_type>

# メッセージ一覧を表示
$ rosmsg list

# 例：sensor_msgs/CameraInfoの構造を表示
$ rosmsg show sensor_msgs/CameraInfo
```

詳細は[rosmsg Wiki](http://wiki.ros.org/rosmsg)を参照すること．

## rosservice

`rosservice`コマンドは，実行中のサービスに関する情報を表示し，サービスを呼び出すことができる．

```{code-block} console
# サービス一覧を表示
$ rosservice list

# サービスの型を表示
$ rosservice type <service_name>

# サービスを呼び出す
$ rosservice call <service_name> <args>

# サービスの詳細情報を表示
$ rosservice info <service_name>

# 例：add_two_intsサービスを呼び出す
$ rosservice call /add_two_ints 1 2
```

詳細は[rosservice Wiki](http://wiki.ros.org/rosservice)を参照すること．

## rossrv

`rossrv`コマンドは，サービスのデータ構造を表示する．使い方は`rosmsg`と同じである．

```{code-block} console
# サービスの詳細を表示
$ rossrv show <service_type>

# サービス一覧を表示
$ rossrv list

# 例：AddTwoIntsサービスの構造を表示
$ rossrv show std_srvs/AddTwoInts
```

詳細は[rossrv Wiki](http://wiki.ros.org/rossrv)を参照すること．

## rospack

`rospack`コマンドは，ROSパッケージに関する情報を取得するためのコマンドである．

```{code-block} console
# パッケージ一覧を表示
$ rospack list

# パッケージのパスを表示
$ rospack find <package_name>

# パッケージの依存関係を表示
$ rospack depends <package_name>

# 例：roscpp_tutorialsパッケージのパスを表示
$ rospack find roscpp_tutorials
```

## rosparam

`rosparam`コマンドは，ROSパラメータサーバーに保存されているパラメータを取得・設定できる．

```{code-block} console
# パラメータ一覧を表示
$ rosparam list

# パラメータの値を取得
$ rosparam get <param_name>

# パラメータの値を設定
$ rosparam set <param_name> <value>

# パラメータをYAMLファイルから読み込む
$ rosparam load <yaml_file>

# パラメータをYAMLファイルに保存
$ rosparam dump <yaml_file>

# 例：すべてのパラメータを表示
$ rosparam get /
```

## rosbag

`rosbag`コマンドは，トピックの記録や再生を行うためのコマンドである．

```{code-block} console
# トピックを記録
$ rosbag record <topic_name>

# 複数のトピックを記録
$ rosbag record <topic1> <topic2> <topic3>

# すべてのトピックを記録
$ rosbag record -a

# バッグファイルを再生
$ rosbag play <bag_file>

# バッグファイルの情報を表示
$ rosbag info <bag_file>

# 例：/cmd_velトピックを記録
$ rosbag record /cmd_vel
```

## rosclean

`rosclean`コマンドは，ROSのログファイルを管理するためのコマンドである．

```{code-block} console
# ローカルディスク上のログファイルの容量を確認
$ rosclean check

# ログファイルを削除
$ rosclean purge
```

## ヘルプの表示

各コマンドの詳細なヘルプは，`-h`オプションで表示できる．

```{code-block} console
# 例：rostopicのヘルプを表示
$ rostopic -h

# 例：rosrunのヘルプを表示
$ rosrun -h
```

## 参考情報

- [ROS Command Line Tools](http://wiki.ros.org/ja/ROS/CommandLineTools)
- [ROS Tutorials](http://wiki.ros.org/ja/ROS/Tutorials)
