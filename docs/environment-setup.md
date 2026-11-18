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
$ sudo apt install -y python3-colcon-common-extensions python3-vcstool python3-rosdep ros-jazzy-realsense2-camera byobu
$ sudo curl -fsSL -o /etc/udev/rules.d/99-realsense-libusb.rules \
    https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.58.4/config/99-realsense-libusb.rules   # 頭のカメラ (D405) を開けるようにする
$ sudo udevadm control --reload-rules && sudo udevadm trigger
$ curl -LsSf https://astral.sh/uv/install.sh | sh   # Python の仮想環境を作る uv (1 回だけ)
$ export PATH="$HOME/.local/bin:$PATH"   # 今のターミナルで uv を使えるようにする
$ uv --version
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
$ source ~/.bashrc    # zsh なら source ~/.zshrc
$ ta login    # 表示されたコードを https://github.com/login/device に入力して承認する
$ ta doctor   # ✓ と - だけで ✗ が無ければよい
```

演習の資料は bash で書いてある．AI TA は zsh でも使えるが，fish など，ほかのシェルを普段使っている人は，演習中はターミナルで `bash` と打って bash で作業する ({doc}`tips/ai-ta`)．
ほかのシェルに対応させたい人は，AI を使ってぜひ PR を送ってほしい．

作業は byobu ({doc}`tips/byobu`) の中のターミナルで行うとよい．`ta fix` がターミナルの画面も AI TA に渡せる．

使い方と注意は {doc}`tips/ai-ta` を見ること．

### ビルド

```{code-block} console
$ cd ~/ros2_ws/
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
$ source /opt/ros/jazzy/setup.bash
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

- `rosdep install` の後の `source /opt/ros/jazzy/setup.bash` は省かない．`rosdep install` で入った Gazebo のパッケージ (`ros-jazzy-gz-*-vendor`) は，`source` し直すまでビルドから見えない ({ref}`troubleshooting-gz-sim8`)．
- `sudo apt upgrade` を省くと，入っている ROS 2 が古いままになる．`rosdep install` は足りないパッケージを入れるだけで，入っているものは新しくしない．
  古い ROS 2 Jazzy では `speech_recognition_msgs` のビルドが `Unknown CMake command "rosidl_auto_generate_interfaces"` で止まる．
- `--packages-up-to` の後に並べた，演習に使うパッケージ (ロボットとシミュレータ，`robot, ri`，カメラの認識，音声認識) とそれが使うものだけをビルドする．
  ワークスペースには演習で使わないパッケージ (jsk_3rdparty の他のパッケージなど) も入っているが，それらはビルドしない．
  メカトロボット (`mechatrobot_ros2`) は {doc}`robot-programming-3-2026` の日にビルドする．
- 音声合成 (VOICEVOX) は，エンジン (約 1.3 GB) のダウンロードに時間がかかるので，ここではビルドしない．
  ロボットに声でしゃべらせるときに，{doc}`robot-programming-1-2026` の「声でしゃべる・聞く」の手順でビルドする．
- ビルドの最後に `N packages had stderr output` と出ることがあるが，uv の進み具合などの表示で，失敗ではない．
  失敗したときは `Failed` と出て，`Summary` に `failed` のパッケージが並ぶ．
- `--skip-keys` の 3 つは，ワークスペースのパッケージの依存のうち，rosdep では入らないものである．
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

(software-update)=
## 演習開始時のソフトウェア更新

演習の日には，最新のソフトウェアを取得してビルドし直す．

```{code-block} console
$ cd ~/ros2_ws/src
$ wget https://raw.githubusercontent.com/jsk-enshu/robot-programming/refs/heads/master/.repos.jazzy-skrobot -O- | vcs import
$ vcs pull
$ rosdep update
$ cd ~/ros2_ws/
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
$ source /opt/ros/jazzy/setup.bash
$ colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

AI TA（{doc}`tips/ai-ta`）を使う人は，AI TA も新しくする．
AI TA の本体（`ta`）と AI TA への指示は `~/ros2_ws` の外に入っているので，上の手順では新しくならない．
インストールのコマンドをもう一度実行すれば新しい版になり，キーと設定はそのまま残る．

```{code-block} console
$ curl -fsSL https://raw.githubusercontent.com/iory/ai-ta-for-student/main/install.sh | bash
$ source ~/.bashrc
$ ta version
```

- **`vcs pull` の出力を読んでから先に進む．** どこかに `error:` が出ていたら，そのリポジトリは更新されていない．
  そのまま `colcon build` すると古いままのコードで動き，別のエラーになる．
  よくあるのは自分で変えたファイルとぶつかる場合で，直し方は {ref}`vcs-pull-local-changes`．
- **`rosdep install` を飛ばさない．** 新しく必要になった apt のパッケージ (実機のジョイスティックの `ros-jazzy-joy-linux`，
  音声認識の `flac` など) はここで入る．入っていないと，起動したときに `package 'joy_linux' not found` などで止まる．
- **`~/ros2_ws` の中にパッケージのバックアップを置かない．** 古い版を `jedy_skrobot.bak` などとして残すときは
  `~/ros2_ws` の外 (例: `~/backup/`) に移す．中にあると同じ名前のパッケージが 2 つになり，`colcon build` が止まる
  ({ref}`colcon-duplicate-package`)．

## トラブルシューティング

(vcs-pull-local-changes)=
### `vcs pull` が `Your local changes to the following files would be overwritten by merge` で止まる

演習のリポジトリのファイルを自分で変えていて (例: ロボットシステムの授業で `jedy/jedy_bringup/CMakeLists.txt` に自分のスクリプトを書き足した)，
同じファイルが更新されたときに出る．自分で足したファイルと同じ名前のファイルが追加されたときは
`The following untracked working tree files would be overwritten by merge` と出る．どちらも最後に `Aborting` と出て，このリポジトリは更新されていない．
自分の変更をいったん退避してから更新する．退避した変更は消えない．

```{code-block} console
$ cd ~/ros2_ws/src/robot-programming
$ git status                                   # 自分が変えたファイルを確かめる
$ git stash push -u -m "before-update"         # 変更と追加したファイルを退避する
$ cd ~/ros2_ws/src && vcs pull                 # 今度は error が出ないことを確かめる
```

その後，上の手順の `rosdep install` と `colcon build` をやり直す．
退避した変更は `git -C ~/ros2_ws/src/robot-programming stash list` で見られ，`git stash show -p` で中身を確かめられる．
`jedy/jedy_bringup/scripts/track_box_to_cmd_vel_*.py` などは今はリポジトリに入っているので，退避したものを戻す必要はない．
自分のプログラムは，これからは演習のリポジトリの外 (`~/enshu` など) に置く．

(colcon-duplicate-package)=
### `colcon build` が `Duplicate package names not supported` で止まる

```{code-block} console
ERROR:colcon:colcon build: Duplicate package names not supported:
- jedy_skrobot:
  - src/robot-programming/jedy/jedy_skrobot
  - jedy_skrobot.bak
```

`~/ros2_ws` の下に，同じ名前のパッケージ (`package.xml` の `<name>` が同じ) が 2 つある．
`colcon build` は `~/ros2_ws` の下を全部探すので，`src` の外 (`~/ros2_ws/jedy_skrobot.bak` など) に置いたバックアップも見つかる．
このエラーでビルドが止まると，前にビルドした古いもの (または何も) が使われ，「`jedy_skrobot` が見つからない」などになる．
バックアップを `~/ros2_ws` の外に移してからビルドし直す．

```{code-block} console
$ mkdir -p ~/backup && mv ~/ros2_ws/jedy_skrobot.bak ~/backup/
$ cd ~/ros2_ws && colcon build --symlink-install --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

### `colcon build` が「`uv` not found on PATH」で失敗する

uv が入っていないか，PATH に無い．上の手順の `curl -LsSf https://astral.sh/uv/install.sh | sh` と
`export PATH="$HOME/.local/bin:$PATH"` を実行し，`uv --version` で版が表示されることを確かめてから，もう一度 `colcon build` する．

uv のインストーラが最後に `source $HOME/.local/bin/env` を実行するように表示することがあるが，実行しなくてよい (上の `export` と同じことをする)．
`~/.local/bin` がもともと PATH に入っている PC (Ubuntu はログイン時に `~/.local/bin` があれば PATH に入れる) では `env` ファイルは作られないので，
`source` すると「そのようなファイルやディレクトリはありません」と出る．

### ビルドが `No rule to make target '/opt/ros/jazzy/lib/...'` で止まる

```{code-block} console
gmake[2]: *** No rule to make target '/opt/ros/jazzy/lib/libfastcdr.so.2.2.5', needed by '...'.  Stop.
```

前のビルドが覚えている ROS 2 のライブラリが，新しい版に置き換わった (`apt upgrade` や，演習の日の更新の `rosdep install` のあとに起きる)．
まず，CMake の設定をやり直してビルドする (VOICEVOX は消えない．模擬の受講者の環境では 45 秒で通った)．

```{code-block} console
$ cd ~/ros2_ws/
$ colcon build --symlink-install --cmake-force-configure --packages-up-to jedy_bringup jedy_skrobot jedy_vision ros_speech_recognition
```

それでも同じエラーで止まるときは，ビルドの途中結果を消して，ビルドし直す．

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
