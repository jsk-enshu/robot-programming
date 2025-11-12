# 環境構築トラブルシューティング

## 概要

本演習の環境構築時によく発生するトラブルとその解決方法をまとめる．

## catkin build時にjedyeusパッケージが見つからないエラー

### 症状

`catkin build jedyeus`を実行すると以下のようなエラーが表示される．

```{code-block} console
$ catkin build jedyeus
[build] Error: Given package 'jedyeus' is not in the workspace and pattern does not match any package
```

ワークスペース内に`jedyeus`パッケージが存在するにもかかわらずcatkinがパッケージを認識していない．

```{code-block} console
$ ls src/robot-programming/jedy/jedyeus/
CMakeLists.txt  COLCON_IGNORE   euslisp/        package.xml     README.md
```

### 原因

`jedyeus`パッケージのディレクトリに`COLCON_IGNORE`ファイルが存在するため，catkin buildがパッケージをスキップしている．

`COLCON_IGNORE`ファイルは本来ROS 2のビルドツール（colcon）用のファイルであるが，catkin-toolsのバージョンによってはこのファイルの存在によってパッケージがスキップされてしまう．

catkin-toolsのバージョンが0.9.4の場合，`COLCON_IGNORE`ファイルが存在するパッケージが正しくビルドされない．

```{code-block} console
$ catkin --version
catkin_tools 0.9.4 (C) 2014-2025 Open Source Robotics Foundation
```

### 解決方法

catkin-toolsをバージョン0.9.5にアップグレードする必要がある．

#### 手順1: 既存のcatkin-toolsパッケージを削除する

古い`catkin-tools`パッケージと新しい`python3-catkin-tools`パッケージの間で競合が発生する可能性があるため，まず既存のパッケージを削除する．

```{code-block} console
$ sudo apt remove catkin-tools
```

#### 手順2: python3-catkin-toolsを再インストールする

```{code-block} console
$ sudo apt install python3-catkin-tools --reinstall
```

インストール時に以下のようなエラーが発生する場合がある．

```
dpkg: error processing archive /var/cache/apt/archives/python3-catkin-tools_0.9.5-1_all.deb (--unpack):
 trying to overwrite '/usr/bin/catkin', which is also in package catkin-tools 0.9.4+ds-1
```

この場合は，手順1の`sudo apt remove catkin-tools`を実行してから再度インストールする．

#### 手順3: バージョンを確認する

インストールが成功したら，バージョンを確認する．

```{code-block} console
$ catkin --version
catkin_tools 0.9.5 (C) 2014-2025 Open Source Robotics Foundation
```

`catkin_tools 0.9.5`と表示されれば成功である．

#### 手順4: 再度ビルドを実行する

```{code-block} console
$ cd ~/ros_ws
$ catkin build jedyeus
```
