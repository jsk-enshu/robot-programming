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

## ROS 2のsetup.bashがROS 1の環境を読み込んでしまう

### 症状

ROS 2環境（`/opt/ros/jazzy/setup.bash`）をsourceすると，以下のような警告が2回表示される．

```{code-block} console
$ source /opt/ros/jazzy/setup.bash
ROS_DISTRO was set to 'jazzy' before. Please make sure that the environment does not mix paths from different distributions.
ROS_DISTRO was set to 'jazzy' before. Please make sure that the environment does not mix paths from different distributions.
```

`~/ros2_ws/install/setup.bash`の内容を確認すると，ROS 1のパスが含まれている．

```{code-block} bash
# 誤った設定例
COLCON_CURRENT_PREFIX="/opt/ros/one"
_colcon_prefix_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"
```

### 原因

`colcon build`を実行する前に`source /opt/ros/one/setup.bash`（ROS 1）を実行すると，ビルド時に現在の環境変数が埋め込まれてしまう．その結果，生成された`setup.bash`にROS 1のパスが含まれてしまい，ROS 2とROS 1の環境が混在してしまう．

### 解決方法

#### 方法1: setup.bashの該当行をコメントアウトする（一時的）

`~/ros2_ws/install/setup.bash`を編集し，ROS 1のパスを含む行をコメントアウトする．

```{code-block} console
$ vi ~/ros2_ws/install/setup.bash
```

以下のような行を探してコメントアウトする：

```{code-block} bash
# COLCON_CURRENT_PREFIX="/opt/ros/one"
# _colcon_prefix_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"
```

#### 方法2: ワークスペースを再構築する（推奨）

時間がある場合は，正しい手順でワークスペースを再構築することを推奨する．

```{code-block} console
# 既存のビルド結果を削除
$ cd ~/ros2_ws
$ rm -rf build install log

# ROS 1の環境をsourceしない状態で再ビルド
# 新しいターミナルを開くか，現在のターミナルをリセット
$ source /opt/ros/jazzy/setup.bash
$ colcon build --symlink-install
```

**重要**: `colcon build`を実行する際は，ROS 1の環境（`/opt/ros/one/setup.bash`）をsourceしないこと．

### 予防策

- `colcon build`を実行する前に，ROS 1の環境がsourceされていないことを確認する
- `.bashrc`にROS 1とROS 2の両方の環境をsourceする設定を記述しないこと
- ビルド時は必要な環境のみをsourceする

## Gazebo起動時にロボットモデルが表示されない

### 症状

`ros2 launch jedy_bringup jedy_gazebo.launch.py`を実行すると，以下のようなエラーが繰り返し表示され，Gazeboにロボットモデルが表示されない．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
package 'topic_tools' not found, searching: ['/home/leus/ros2_ws/install/mechatrobot_ros2', '/home/leus/ros2_ws/install/opencv_apps', '/home/leus/ros2_ws/install/jedy_bringup', '/home/leus/ros2_ws/install/jedy_description', '/opt/ros/jazzy']
```

### 原因

必要な依存パッケージ（`topic_tools`など）がインストールされていない．環境構築の手順で`rosdep install`コマンドをスキップした場合に発生する．

### 解決方法

`rosdep`を使用して必要な依存パッケージをインストールする．

```{code-block} console
$ cd ~/ros2_ws
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r
```

インストール完了後，再度Gazeboを起動する．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
```

### 予防策

環境構築時は，演習資料の手順を省略せず，すべてのコマンドを実行すること．特に`rosdep install`は依存関係を自動的に解決するため重要である．

## jedy_bringupパッケージが見つからない

### 症状

`ros2 launch jedy_bringup jedy_gazebo.launch.py`を実行すると，以下のエラーが表示される．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py
Package 'jedy_bringup' not found: 'package 'jedy_bringup' not found, searching: ['/home/mech-user/ros2_ws/install/opencv_apps', '/opt/ros/jazzy']'
```

ワークスペース内に`jedy`パッケージが存在するにもかかわらず，colconがパッケージを認識していない．

```{code-block} console
$ ls ~/ros2_ws/src/robot-programming/jedy/jedy_bringup/
CMakeLists.txt  package.xml  launch/  ...
```

### 原因

`~/ros2_ws/src/robot-programming/jedy/`ディレクトリに`CATKIN_IGNORE`ファイルが存在するため，colcon buildがこのディレクトリ全体をスキップしている．

`CATKIN_IGNORE`ファイルは本来ROS 1のビルドツール（catkin）用のファイルであるが，colconもこのファイルが存在するディレクトリをビルド対象から除外してしまう．

### 解決方法

`CATKIN_IGNORE`ファイルをリネームまたは削除してから，再度ビルドを実行する．

```{code-block} console
$ mv ~/ros2_ws/src/robot-programming/jedy/CATKIN_IGNORE \
     ~/ros2_ws/src/robot-programming/jedy/CATKIN_IGNORE.bak
$ cd ~/ros2_ws
$ colcon build --symlink-install
```

ビルドが成功すると，`jedy_bringup`と`jedy_description`パッケージがコンパイルされる．

### 確認

ビルド後，パッケージが正しく認識されることを確認する．

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ ros2 pkg list | grep jedy
jedy_bringup
jedy_description
```

## ROS 1とROS 2のトピックリストが一致しない

### 症状

ROS 1/ROS 2ブリッジ（`ros2 run ros1_bridge dynamic_bridge --bridge-all-topics`）を起動した状態で，ROS 1とROS 2のトピックリストを確認すると，数が一致しない．

```{code-block} console
# ROS 1のトピック数
$ rostopic list | wc -l
40

# ROS 2のトピック数
$ ros2 topic list | wc -l
60
```

### 原因

これは**正常な動作**である．ROS 2にはROS 1に存在しない追加のトピックが含まれている．

**ROS 2固有のトピック例：**
- `/controller_manager/*`（コントローラマネージャの内部状態）
- `/parameter_events`（パラメータ変更イベント）
- `/dynamic_joint_states`（動的関節状態）
- `/map`, `/map_updates`（地図関連）

また，アクショントピックの命名規則も異なる：
- ROS 1: `follow_joint_trajectory/*`
- ROS 2: `controller_state`, `joint_trajectory`など

### 解決方法

この不一致は問題ではない．ブリッジは必要なトピックのみを変換しており，完全一致は必須ではない．

実際にブリッジが機能しているかを確認するには，以下を実行する：

```{code-block} console
# ROS 2側でトピックをpublish
$ ros2 topic pub --once /test std_msgs/msg/String "data: 'hello'"

# ROS 1側でトピックをsubscribe
$ rostopic echo /test
```

データが正しく受信できれば，ブリッジは正常に動作している．

## colcon buildでGitマージコンフリクトマーカーが原因で失敗する

### 症状

`colcon build --symlink-install --packages-select jedy_bringup jedy_description`を実行すると，以下のようなエラーが表示される．

```{code-block} console
$ colcon build --symlink-install --packages-select jedy_bringup jedy_description
ament_cmake_symlink_install_programs() can't find '/home/mech-user/ros2_ws/src/robot-programming/jedy/jedy_bringup/<<<<<<<'
```

パスに`<<<<<<<`という不正な文字列が含まれている．

### 原因

この問題には2つの原因が考えられる．

#### 原因1: 誤ったディレクトリでcolcon buildを実行している

`colcon build`をワークスペースのルート（`~/ros2_ws`）ではなく，ソースディレクトリ（`~/ros2_ws/src/robot-programming`）内で実行してしまっている．ソースディレクトリ内に`build/`，`install/`，`log/`ディレクトリが存在すると，ビルドシステムが混乱する．

#### 原因2: Gitのマージコンフリクトが未解決

`<<<<<<<`はGitのマージコンフリクトマーカーである．`CMakeLists.txt`などのファイルにマージコンフリクトが残っている状態でビルドを実行すると，ビルドシステムがこれを不正なパスとして解釈してしまう．

### 解決方法

#### 手順1: 正しいディレクトリでビルドを実行する

必ずワークスペースのルートディレクトリに移動してからビルドを実行する．

```{code-block} console
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select jedy_bringup jedy_description
```

#### 手順2: Gitのマージコンフリクトを解決する

`<<<<<<<`が含まれるファイルを探して，マージコンフリクトを解決する．

```{code-block} console
# マージコンフリクトマーカーを含むファイルを検索
$ cd ~/ros2_ws/src/robot-programming
$ grep -r "<<<<<<< " .
$ grep -r "=======" .
$ grep -r ">>>>>>> " .
```

該当ファイルをエディタで開き，以下のようなマージコンフリクトマーカーを手動で解決する：

```
<<<<<<< HEAD
現在のブランチの内容
=======
マージしようとしているブランチの内容
>>>>>>> branch-name
```

正しい内容を選択し，マージコンフリクトマーカー（`<<<<<<<`，`=======`，`>>>>>>>`）を削除する．

#### 手順3: ソースディレクトリ内のビルド成果物を削除する

もしソースディレクトリ内に誤ってビルド成果物が作成されている場合は削除する．

```{code-block} console
$ cd ~/ros2_ws/src/robot-programming
$ rm -rf build install log
```

#### 手順4: 再度ビルドを実行する

```{code-block} console
$ cd ~/ros2_ws
$ colcon build --symlink-install
```

### 予防策

- `colcon build`は必ずワークスペースのルート（`~/ros2_ws`）で実行する
- Gitでブランチをマージした後は，必ずマージコンフリクトが解決されていることを確認してからビルドする
- `git status`でマージコンフリクトの有無を確認する習慣をつける

## opencv_appsのビルドでリンカエラーが発生する

### 症状

`colcon build --symlink-install`を実行すると，`opencv_apps`パッケージのビルド時に以下のようなリンカエラーが発生する．

```{code-block} console
$ colcon build --symlink-install
...
undefined reference to `class_loader::impl::AbstractMetaObjectBase::~AbstractMetaObjectBase()'
undefined reference to methods in libimage_transport.so
...
make[2]: *** [opencv_apps/nodelet/libopencv_apps] Error 1
```

### 原因

ROS 2 Jazzy環境の`class_loader`や`image_transport`ライブラリが破損しているか，互換性のないバージョンがインストールされている．

### 解決方法

#### 手順1: システムパッケージを更新する

まず，システム全体のパッケージを更新する．

```{code-block} console
$ sudo apt update
$ sudo apt dist-upgrade
```

#### 手順2: 問題のあるパッケージを再インストールする

`class_loader`と`image_transport`パッケージを再インストールする．

```{code-block} console
$ sudo apt install --reinstall ros-jazzy-class-loader
$ sudo apt install --reinstall ros-jazzy-image-transport
```

#### 手順3: ワークスペースをクリーンビルドする

既存のビルド成果物を削除してから再ビルドする．

```{code-block} console
$ cd ~/ros2_ws
$ rm -rf build install log
$ colcon build --symlink-install
```

#### 手順4: 依存関係を再インストールする

`rosdep`を使って依存関係を再確認・再インストールする．

```{code-block} console
$ cd ~/ros2_ws
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r
```

### 診断方法

詳細なログを出力してエラーの詳細を確認する場合は，以下のコマンドを実行する．

```{code-block} console
$ colcon build --symlink-install --event-handlers console_direct+
```

### 予防策

- 定期的に`sudo apt update && sudo apt dist-upgrade`を実行してシステムを最新に保つ
- ROSパッケージをインストールする際は，公式リポジトリから行う
- 複数のROSバージョンを混在させない（ROS 1とROS 2を明確に分ける）
