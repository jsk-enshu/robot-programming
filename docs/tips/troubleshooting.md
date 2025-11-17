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

## rosdep installでlibgazebo11とlibdartのインストールに失敗する

### 症状

`rosdep install`を実行すると，`libgazebo11*`と`libdart*`パッケージのインストールに失敗し，以下のようなメッセージが表示される．

```{code-block} console
$ cd ~/ros_ws
$ rosdep install --from-path src --ignore-src
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
...
rosdep key 'libgazebo11-dev' cannot be resolved
rosdep key 'libdart6-dev' cannot be resolved
```

その後，`apt --fix-broken install`を実行するように指示される場合がある．

### 原因

`/etc/apt/sources.list.d/`ディレクトリに不要または競合するGazebo関連のリポジトリが追加されている可能性がある．具体的には以下のようなファイルが存在する場合がある．

- `gazebo-stable.list`
- `packages.ros.org.list`（`packages.ros.org/ros/ubuntu noble main`などを含む）

これらのリポジトリがシステムのパッケージ管理と競合し，rosdepが正しく依存関係を解決できなくなる．

### 解決方法

以下の手順で不要なリポジトリを削除し，既存のGazebo/DARTパッケージをクリーンアップしてから再度rosdepを実行する．

#### 手順1: 不要なリポジトリファイルを確認・削除する

まず，`/etc/apt/sources.list.d/`内のGazebo関連のリポジトリファイルを確認する．

```{code-block} console
$ ls /etc/apt/sources.list.d/ | grep -E 'gazebo|packages.ros'
```

Gazebo関連のファイル（例: `gazebo-stable.list`, `packages.ros.org.list`）が存在する場合は削除する．

```{code-block} console
$ sudo rm /etc/apt/sources.list.d/gazebo-stable.list
$ sudo rm /etc/apt/sources.list.d/packages.ros.org.list
```

または，必要に応じてファイルをコメントアウトすることもできる．

```{code-block} console
$ sudo vi /etc/apt/sources.list.d/packages.ros.org.list
# 内容をコメントアウトする
# deb http://packages.ros.org/ros/ubuntu noble main
```

#### 手順2: パッケージリストを更新する

リポジトリファイルを削除・変更した後，パッケージリストを更新してエラーが発生しないことを確認する．

```{code-block} console
$ sudo apt update
```

エラーが表示されないことを確認する．もしエラーが表示される場合は，エラーメッセージに従って追加のリポジトリファイルを削除する必要がある．

#### 手順3: 既存のlibgazebo11とlibdartパッケージを削除する

競合する可能性のある既存のGazeboとDARTパッケージを削除する．

```{code-block} console
$ sudo apt remove 'libgazebo11*' 'libdart*'
```

パッケージが見つからない場合は，このステップをスキップして次に進む．

#### 手順4: 再度rosdep installを実行する

```{code-block} console
$ cd ~/ros_ws
$ rosdep install --from-path src --ignore-src -r -y
```

`-r`オプションはエラーが発生しても続行するオプション，`-y`オプションは確認なしでインストールを実行するオプションである．

#### 補足: apt --fix-broken installについて

`apt --fix-broken install`を実行するように指示された場合は，上記の手順1〜3を実行した後に実行する．ただし，多くの場合は上記の手順でリポジトリの競合を解決することで問題が解消される．

```{code-block} console
$ sudo apt --fix-broken install
```
