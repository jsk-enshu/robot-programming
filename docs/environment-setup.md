# 環境構築

本演習を開始する前に，必要なソフトウェア環境を構築する必要がある．
本ページでは，初回のワークスペース構築方法と，演習開始時のソフトウェア更新方法について説明する．

## 前提条件

本演習は岡田先生のロボットシステムの講義で扱った`Euslisp`や`ROS`の環境が正しく構築されていることを前提として進める．

## 初回環境構築

ロボットシステムで`~/enshu_ws`というワークスペース(作業ディレクトリのこと)を準備したと思う．
下記の環境構築方法に従ってワークスペースを作成または更新する．
`~/ros_ws/src` がある場合は下記の代わりに[最新バージョンのソフトウェアを取得](#最新バージョンのソフトウェアを取得)を実行する．

### ROS 2 セットアップ

```{code-block} console
$ sudo apt update
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ source /opt/ros/jazzy/setup.bash
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.jazzy -O- | vcs import
$ rosdep update
$ cd ~/ros2_ws/
$ rosdep install --from-paths src --ignore-src -y -r
$ colcon build --symlink-install jedy_bringup
```

### ROS 1 セットアップ
```{code-block} console
$ sudo apt update
$ sudo apt install python3-vcstool python3-catkin-tools
$ mkdir -p ~/ros_ws/src
$ cd ~/ros_ws/src
$ source /opt/ros/one/setup.bash
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.one -O- | vcs import
$ rosdep update
$ cd ~/ros_ws
$ rosdep install --from-paths src --ignore-src -y -r
$ catkin build jedyeus
```

### ROS1 bridge セットアップ

[ros1_bridge](tips/ros1-bridge.md)を参考にしてros1 bridgeをインストールする．


## 次のステップ

環境構築が完了したら，以下の演習に進む：

- [1日目：双腕移動台車ロボットを用いた認識操作プログラミング](robot-programming-1-2025.md)
- [2日目：ロボットの全身行動プログラミング](robot-programming-2-2025.md)
- [3日目：組込みデバイスとの統合](robot-programming-3-2025.md)
