# 環境構築トラブルシューティング

## 概要

本演習（2026年度，ROS 2 Jazzy）の環境構築時によく発生するトラブルとその解決方法をまとめる．
ビルドとソフトウェア更新の手順は{doc}`../environment-setup`にあり，そこにもよくあるトラブル（`uv` not found，`No rule to make target`）がある．
エラーが出たら，AI TAの`ta fix`で聞くこともできる（{doc}`ai-ta`）．

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
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
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
Package 'jedy_bringup' not found: ...
```

### 原因

次のどれかである．

- ワークスペースをまだビルドしていない，またはビルドが途中で失敗した．
- このターミナルで`~/ros2_ws/install/setup.bash`を読み込んでいない（`~/.bashrc`に書いていない，または書いた後に開いたターミナルではない）．

### 解決方法

{doc}`../environment-setup`の「ビルド」の手順でビルドし，`Summary`に`failed`が無いことを確かめる．
そのあと設定を読み込む．

```{code-block} console
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
$ source ~/ros2_ws/install/setup.bash
$ tail -2 ~/.bashrc   # source ~/ros2_ws/install/setup.bash が書かれているか
```

### 確認

```{code-block} console
$ ros2 pkg list | grep jedy
jedy_bringup
jedy_description
jedy_skrobot
jedy_vision
jedy_vision_msgs
```

## colcon buildでGitマージコンフリクトマーカーが原因で失敗する

### 症状

`colcon build`を実行すると，以下のようなエラーが表示される．

```{code-block} console
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
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
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
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
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

### 予防策

- `colcon build`は必ずワークスペースのルート（`~/ros2_ws`）で実行する
- Gitでブランチをマージした後は，必ずマージコンフリクトが解決されていることを確認してからビルドする
- `git status`でマージコンフリクトの有無を確認する習慣をつける

## opencv_appsのビルドでリンカエラーが発生する

### 症状

`colcon build`を実行すると，`opencv_apps`パッケージのビルド時に以下のようなリンカエラーが発生する．

```{code-block} console
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
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
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

#### 手順4: 依存関係を再インストールする

`rosdep`を使って依存関係を再確認・再インストールする．

```{code-block} console
$ cd ~/ros2_ws
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
```

### 診断方法

詳細なログを出力してエラーの詳細を確認する場合は，以下のコマンドを実行する．

```{code-block} console
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition --event-handlers console_direct+
```

### 予防策

- 定期的に`sudo apt update && sudo apt dist-upgrade`を実行してシステムを最新に保つ
- ROSパッケージをインストールする際は，公式リポジトリから行う
