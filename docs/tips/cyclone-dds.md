(cyclone-dds)=
# Cyclone DDS に切り替える (任意)

ROS 2 Jazzy は，標準では Fast DDS で通信する．
これを Cyclone DDS に替えると，サンプルを起動してからロボットにつながるまでの待ちが短くなる．
切り替えは**任意**である．標準の Fast DDS のままでも演習はすべてできる．

## なぜ速くなるのか

新しく起動したプログラムは，まず相手のノードやサービスを見つける (ディスカバリ) 必要がある．
Fast DDS では，同じ PC の中 (localhost) でも，これに 0.1〜3.5 秒かかることがある．
Cyclone DDS では，同じ条件で毎回 0.02 秒ほどだった．

Gazebo (`jedy_gazebo.launch.py`) で，サンプルの始めから終わりまでの時間を比べた (同じ PC，同じワークスペース)．

| サンプル | Cyclone DDS | Fast DDS |
|---|---|---|
| `sample_reset_pose` | 6.9 秒 | 9.4 秒 |
| `sample_ik` | 9.6 秒 | 12.7 秒 |
| `show_camera` | 5.6 秒 | 12.9 秒 |
| `approach_tag` | 23.4 秒 | 29.3 秒 |

ロボットの動きそのものは変わらない．短くなったのは起動時の待ちである．

## Cyclone DDS で動作を確かめたもの

Gazebo では，次のものがすべて Cyclone DDS で動いた．

- `jedy_skrobot` のサンプル 13 個 (`sample_*`，`checkpoint_*`，`track_box` など) と `show_camera`
- 物体認識 (`recognition.launch.py` と `ri.what_do_you_see`)
- AprilTag (`tag_pose`，`approach_tag`)
- 深度画像 (1.2 MB) と点群 (9.6 MB) のトピック．どちらも Fast DDS と同じくらいの周期 (約 5 Hz) で届いた．

**まだ確かめていないもの：** 実機 (サーボのノード)，音声の認識と合成，`vlm_navigate`，複数の PC をまたぐ通信．
これらで困ったら，Fast DDS に戻す ({ref}`cyclone-dds-revert`)．

## 切り替える手順

### 1. パッケージを入れる

ワークスペースで `rosdep install` をし直すと入る (`jedy_bringup` の依存に入っている)．

```{code-block} console
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy --skip-keys "opencv_apps python3-gdown-pip micro_ros_agent"
```

`sudo apt install ros-jazzy-rmw-cyclonedds-cpp` でもよい．
学科 PC で `sudo` が使えない場合は，入っているかを次のコマンドで確かめる．

```{code-block} console
$ ls /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so
```

無いと言われたら，TA に伝える．

### 2. `~/.bashrc` に 1 行足す

`~/.bashrc` の `source /opt/ros/jazzy/setup.bash` の前に，次の 1 行を足す．

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 3. 全部起動し直す

開いているターミナルを**すべて**閉じて，新しく開き直す．
Gazebo や実機の launch，IPython，RViz も全部起動し直す．
最後に，裏で動いている ROS 2 のデーモンを止める．

```{code-block} console
$ ros2 daemon stop
```

### 4. 確かめる

新しいターミナルで次を実行し，`rmw_cyclonedds_cpp` と表示されればよい．

```{code-block} console
$ echo $RMW_IMPLEMENTATION
rmw_cyclonedds_cpp
```

## 注意：Fast DDS のプログラムとは通信できない

Fast DDS で動いているプログラムと Cyclone DDS で動いているプログラムは，お互いを見つけられない．
例えば，Gazebo を Fast DDS のまま動かして，サンプルだけ Cyclone DDS で起動すると，
サンプルはサービスを呼べずにタイムアウトする．

次のようなときに起きやすい．

- `~/.bashrc` を書き換える前に開いたターミナルが残っている．
- 実機の PC (ロボットの launch を動かす PC) と，サンプルを動かす PC とで設定が違う．
- 班のメンバーの PC と通信するのに，片方だけ切り替えた．

「トピックは見えるのにつながらない」「サービスがタイムアウトする」ときは，
すべてのターミナルで `echo $RMW_IMPLEMENTATION` が同じになっているかを確かめる．

(cyclone-dds-revert)=
## Fast DDS に戻す

`~/.bashrc` に足した `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` の行を消し，
手順 3 と同じように，すべて起動し直してから `ros2 daemon stop` を実行する．

## 参考

- Jazzy の標準は Fast DDS で，Cyclone DDS も公式に Tier 1 として扱われている．
  - <https://docs.ros.org/en/jazzy/Installation/RMW-Implementations.html>
  - <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Different-Middleware-Vendors.html>
- Cyclone DDS の入れ方と使い方：<https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html>
- MoveIt 2 は Cyclone DDS を勧めている：<https://moveit.ai/install-moveit2/source/>
- 複数の PC で大きいメッセージ (点群など) を送るときは，受信バッファの設定が要ることがある：<https://docs.stereolabs.com/docs/integrations/ros-2/dds-and-network-tuning.md>
- 違う DDS 実装どうしで通信できないことは {doc}`ros-network` にもある．
