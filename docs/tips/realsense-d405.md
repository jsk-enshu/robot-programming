# Intel RealSense D405 (頭のカメラ)

Jedy の頭のカメラは Intel RealSense Depth Camera D405 である．
カラー画像と距離画像 (深度) を同時に撮れる．PC とは USB でつなぐ．

この章の端末では，毎回次を実行しておく ({doc}`../robot-programming-1-2026` のとおりに `~/.bashrc` に書いてあれば不要)．

```{code-block} console
$ source ~/ros2_ws/install/setup.bash
$ export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

## D405 について

演習で使う D405 の性質は次のとおりである．

| 項目 | 値 |
| --- | --- |
| 深度を測れる距離 | 0.07〜0.5 m くらい．それより遠い所は測れない (`ri.depth_image()` では nan) |
| 画角 | 横 87° 程度 |
| 演習での解像度と速さ | 640×480，30 Hz (`jedy_fullbody_all.launch.py` の設定) |

近い物を細かく測るためのカメラなので，**0.5 m より遠い人や物までの距離は測れない**．
遠い物の距離を使うプログラムは，近づいてから測るように書く ({doc}`../robot-programming-1-2026` の「カメラで見る」)．

[Intel の製品ページ (仕様)](https://www.intel.co.jp/content/www/jp/ja/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html)

### ステレオカメラの測距の原理

D405 は 2 つのカメラ (ステレオカメラ) で距離を測る．人間の両眼と同じ原理である．

1. 2 つのカメラで同じ物を撮ると，2 枚の画像の上で物の位置がずれる．このずれを視差 (disparity) という．
2. 2 つのカメラの間の距離 (ベースライン) $B$ と焦点距離 $f$ がわかっていれば，視差 $d$ から距離 $Z$ を三角測量で求められる．

$$
Z = \frac{B f}{d}
$$

近い物ほど視差が大きい．D405 は近い物を測るように作られているので，近距離では細かく測れるが，遠い物は視差が小さすぎて測れない．
模様の無い白い壁のように，左右の画像で同じ点を見つけにくい所も測れないことがある．

## インストール

ROS 2 のドライバ (`realsense2_camera`) を apt で入れる ({doc}`../environment-setup` の「初回環境構築」で入れていれば不要)．

```{code-block} console
$ sudo apt install -y ros-jazzy-realsense2-camera
```

ライブラリ (librealsense2) も一緒に入る．ソースからビルドする必要は無い．

D405 を一般ユーザで開けるように，udev ルールを 1 回だけ入れる (apt のパッケージには入っていない)．
入れたら D405 を挿し直す．

```{code-block} console
$ sudo curl -fsSL -o /etc/udev/rules.d/99-realsense-libusb.rules \
    https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.58.4/config/99-realsense-libusb.rules
$ sudo udevadm control --reload-rules && sudo udevadm trigger
```

(d405-cable)=
## ケーブルのつなぎ方

D405 は付属の USB 3.0 ケーブル (カメラ側が L 字の USB Type-C で，抜け止めのネジが付いている) で PC とつなぐ．
カメラの USB 端子は，`intel REALSENSE` の文字がある側面にある ({numref}`fig:d405-cable-port`)．

```{figure} ../fig/d405-cable-port.jpg
---
name: fig:d405-cable-port
width: 50%
---
Jedy の頭の D405．`intel REALSENSE` の文字がある側面に USB 端子がある (右下に写っているのがケーブルの L 字のプラグ)
```

1. プラグのネジがカメラの端子の横の小さな穴に合う向きにして，端子にまっすぐ奥まで挿す ({numref}`fig:d405-cable-insert`)．
2. ケーブルが下 (胴体の方) へ伸びる向きになっていることを確かめる ({numref}`fig:d405-cable-connected`)．
3. プラグの後ろのネジのつまみを指で回して締める．工具は使わず，軽く止まるところまででよい．
   ネジを締めておかないと，首を動かしたときにケーブルに引っぱられて抜けやすい．
4. 首を左右・上下に動かしても突っぱらないくらいケーブルにたるみを持たせて，PC の USB ポートに挿す．
   USB 2.0 のポートや USB ハブにつなぐと，画像が来なかったり途切れたりすることがある．

```{figure} ../fig/d405-cable-insert.jpg
---
name: fig:d405-cable-insert
width: 50%
---
プラグを端子に挿すところ．プラグの後ろのつまみがネジになっている
```

```{figure} ../fig/d405-cable-connected.jpg
---
name: fig:d405-cable-connected
width: 50%
---
挿し終わったところ．ケーブルが下へ伸び，ネジが締まっている
```

つないだら，PC からカメラが見えているかを確かめる．

```{code-block} console
$ rs-enumerate-devices --compact
```

`Intel RealSense D405` の行が出ればつながっている．何も出ないときは，プラグが奥まで入っているか，ネジが締まっているかを見て挿し直す．
抜くときは，先にネジを緩めてからプラグを持って抜く (ケーブルを引っぱらない)．

(d405-no-frames)=
### 画像が来ないとき

launch のログに `No RealSense devices were found!` や `Frames didn't arrived within 5 seconds` が出続けるときは，
PC が USB でカメラを見つけているか，見つけていても通信が途切れていないかを確かめる．

1. `lsusb` でカメラが見えているかを見る．`Intel(R) RealSense(TM) Depth Camera 405` の行が無ければ，USB でつながっていない．

   ```{code-block} console
   $ lsusb | grep -i realsense
   Bus 002 Device 023: ID 8086:0b5b Intel Corp. Intel(R) RealSense(TM) Depth Camera 405
   ```

2. `lsusb -t` で，どの速さでつながっているかを見る．`Driver=uvcvideo` の行の最後が `5000M` なら USB 3.0，`480M` なら USB 2.0 である．
   `480M` のときや，カメラが `Class=Hub` の行の下にある (USB ハブを通っている) ときは，PC の USB 3.0 のポートに直接挿し直す．

3. 別のターミナルで `sudo dmesg --follow` を動かしたまま，カメラのケーブルを PC 側で抜いて挿し直す ({doc}`arduino-device-detection` の「dmesg でカーネルログを確認」と同じやり方)．

   ```{code-block} console
   $ sudo dmesg --follow
   ```

   挿したときに `New USB device found, idVendor=8086, idProduct=0b5b` と `Product: Intel(R) RealSense(TM) Depth Camera 405` が出れば，PC はカメラを見つけている．
   何も出なければ，ケーブルかカメラ側の端子 (ネジが締まっているか) を見る．
   見つけたあとで `uvcvideo ...: Non-zero status (-71) in video completion handler.` が出るときは，USB の通信がうまくいっていない．
   USB ハブを外して PC に直接挿すか，ほかのポートやケーブルに替える．
   `sudo` が使えないときは `journalctl -kf` でも同じログが見られる．

挿し直したら launch も起動し直す．ケーブルを抜き差しするときは TA を呼び，launch を Ctrl-C で止めてから行う．

## 実機での起動

実機では，ロボットを起動するときに `camera:=true` を付ける．

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true
```

ドライバが入っていないと，`camera:=true needs the RealSense ROS driver: sudo apt install ros-jazzy-realsense2-camera` と出て止まる．

`camera:=true` は `realsense2_camera_node` を次の設定で起動する (`jedy/jedy_bringup/launch/jedy_fullbody_all.launch.py` の `_camera()`)．

- カラーと深度をどちらも 640×480，30 Hz で出す．
- `align_depth.enable` は `false` (距離画像をカラー画像に合わせる処理はしない)．
  D405 のカラー画像は左の深度カメラと同じセンサから作られるので，同じ解像度ならカラーと深度の画素がもともと揃っている．
  `ri.depth_image()[v, u]` は `ri.image()` の画素 `(u, v)` の距離である．
- TF は出さない．カメラのフレーム (`camera_link` など) はロボットの URDF から `robot_state_publisher` が出す．

トピックは次のとおりである．ドライバはカラー画像を `/camera/color/image_raw` に出すが，
Gazebo や `jedy_skrobot` と同じ名前にするため，`/camera/color/image_rect_raw` に名前を変えている．

| トピック | 型 | 中身 |
| --- | --- | --- |
| `/camera/color/image_rect_raw` | `sensor_msgs/Image` | カラー画像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | カラー画像の内部パラメータ |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 距離画像 (`16UC1`，単位 mm) |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 距離画像の内部パラメータ |

```{code-block} console
$ ros2 topic hz /camera/color/image_rect_raw
$ ros2 topic echo /camera/color/camera_info --once
```

### Gazebo のカメラ

Gazebo (`jedy_gazebo.launch.py`) の頭のカメラも，同じ名前のトピックに画像を出す
(`/camera/color/image_rect_raw`，`/camera/color/camera_info`，`/camera/depth/image_rect_raw`)．
Gazebo のカメラは既定 (`camera:=wide`) では横の画角が 60° で，深度は 10 m まで測れる．
`camera:=d405` を付けると，D405 と同じ横の画角 87° と深度 0.07〜0.5 m になる．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py camera:=d405
```

Gazebo の頭のカメラは 1 秒に 5 枚である (`camera_rate:=5`，既定)．`camera_rate:=30` で実機と同じ 30 Hz になる．

## カメラだけを起動する

ロボットを起動せずにカメラだけを試すときは，`realsense2_camera` の launch ファイルを使う．
`jedy_fullbody_all.launch.py camera:=true` と同時には起動しない (1 台のカメラを 2 つのドライバが取り合う)．

```{code-block} console
$ ros2 launch realsense2_camera rs_launch.py
```

このときのノードは `/camera/camera` (名前空間 `camera`，ノード名 `camera`) になり，トピックにも `/camera/camera/` が付く．
カラー画像の名前も変えないので，`jedy_fullbody_all.launch.py` のときとは名前が違う．

| トピック | 中身 |
| --- | --- |
| `/camera/camera/color/image_raw` | カラー画像 |
| `/camera/camera/color/camera_info` | カラー画像の内部パラメータ |
| `/camera/camera/depth/image_rect_raw` | 距離画像 |
| `/camera/camera/depth/camera_info` | 距離画像の内部パラメータ |

`ri.image()` などは `/camera/color/image_rect_raw` を読むので，この起動のしかたでは `ri` で画像は取れない．

カメラが見つからないと，`No RealSense devices were found!` と出続ける．
USB ケーブルを挿し直し，次でカメラが見えているかを確かめる．

```{code-block} console
$ rs-enumerate-devices --compact
```

`realsense-viewer` を使うと，ROS を使わずにカメラの画像と設定を見られる．

```{code-block} console
$ realsense-viewer
```

## 画像を見る

### rqt_image_view

```{code-block} console
$ ros2 run rqt_image_view rqt_image_view /camera/color/image_rect_raw
```

左上のメニューで，見るトピックを選び直せる．距離画像 (`/camera/depth/image_rect_raw`) は mm の値のままなので，ほとんど黒く見える．

### RViz

`jedy_fullbody_all.launch.py` は RViz も開く (`rviz:=false` で開かない)．
画像を見るには，左下の `Add` → `By topic` で `/camera/color/image_rect_raw` の `Image` を選ぶ．

### jedy_skrobot の show_camera

カラー画像と距離画像を並べて表示し続ける．

```{code-block} console
$ ros2 run jedy_skrobot show_camera
```

## Python で画像を取る

`ri` で，カラー画像と距離画像を numpy の配列として取れる ({doc}`../robot-programming-1-2026` の「カメラで見る」)．

```{code-block} python
from jedy_skrobot import connect

robot, ri = connect()
rgb, stamp = ri.image()            # (480, 640, 3) uint8, RGB
depth, stamp = ri.depth_image()    # (480, 640) float32 [m]．測れない画素は nan
K, D = ri.camera_info()            # カメラ行列 3x3 と歪み係数
print(depth[240, 320])             # 画像の真ん中までの距離 [m]
```

`ri.depth_image()` は，D405 の mm の値をメートルに直し，測れなかった画素 (値が 0) を nan にして返す．
実機で部屋の中を写したときは，nan だったのは 5% ほどで，0.5 m より遠い所にも値 (数 m〜数十 m) が入っていた．
D405 が得意なのは 0.07〜0.5 m なので，それより遠い値の精度は保証されない ({ref}`jedy-real-run`)．

台車を動かした後の画像がほしいときは，`newer_than` に時刻を渡すと，その時刻より後に撮った画像を待って返す．

```{code-block} python
ri.tuck_arms()                                  # 走る前に腕をたたむ
ri.go_velocity(0.1, 0.0, 0.0, duration=1.0)
stopped = ri.now()
rgb, _ = ri.image(newer_than=stopped)          # 止まってから撮った画像
depth, _ = ri.depth_image(newer_than=stopped)
```

画素からロボットの座標への変換は {doc}`coordinate_systems` を見よ．

## 参考資料

- [Intel RealSense D405 製品ページ (仕様)](https://www.intel.co.jp/content/www/jp/ja/products/sku/229218/intel-realsense-depth-camera-d405/specifications.html)
- [realsense-ros (ROS 2 のドライバ)](https://github.com/IntelRealSense/realsense-ros)
- [librealsense](https://github.com/IntelRealSense/librealsense)
