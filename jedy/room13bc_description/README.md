# room13bc_description

東京大学 工学部2号館 1階 13BC 室（通称「メカスタ」）と、その中の KOKUYO MT-GFDM10（天板跳ね上げ式の台形テーブル）48 台の URDF。
写真から作ったテクスチャつきのメッシュ（OBJ ＋ MTL ＋ PNG）で、RViz2 と Gazebo（gz sim）で表示する。

Ubuntu 24.04 ＋ ROS 2 Jazzy ＋ Gazebo Harmonic で使っている。`jedy_bringup` の `jedy_gazebo.launch.py`
（`world:=room13bc`、既定）がこの部屋を読み込み、Gazebo でテクスチャつきで表示される。

## 中身

`urdf/` と `meshes/` はこのディレクトリには無い．ビルド時に
[scikit-robot-models](https://github.com/iory/scikit-robot-models) の `room13bc_description.tar.gz` を
ダウンロードし（SHA256 で確認），インストール先の `share/room13bc_description/` に置く．
scikit-robot（`skrobot.data.room13bc_urdfpath()`）や scikit-robot-web と同じものを使う．

| | |
| --- | --- |
| `urdf/room13bc_scene.urdf` | 部屋＋机 48 台。机の天板は 1 台ずつ `desk_XX_n_hinge`（revolute、0 で平ら、−1.5708 で跳ね上げ） |
| `urdf/room13bc.urdf` | 部屋だけ（床・壁・天井・ガラスの間仕切り・ホワイトボードの壁。すべて固定） |
| `urdf/desk.urdf` | 机 1 台 |
| `meshes/*.obj` | 見た目：OBJ ＋ MTL ＋ PNG（`urdf/*.urdf`） |
| `meshes/*.glb` | 見た目：glTF（Y 上の標準どおり、テクスチャ埋め込み。`urdf/*_glb.urdf`） |
| 当たり判定 | モデルの直方体と円柱（机の天板だけ凸包の STL） |
| launch の `mesh:=obj\|glb` | 見た目の形式。既定は RViz2 が obj、Gazebo が glb |
| `launch/display.launch.py` | RViz2 |
| `launch/gazebo.launch.py` | Gazebo（ros_gz_sim、空の世界に spawn、/clock を bridge） |

座標：Z 上、床 z = 0、m。X が部屋の長手（木のキャビネット側 0 → 歯車の壁 26.83）、Y が短手
（入口側のガラスの間仕切り 0 → 収納の白い扉 9.56）。原点の `world` に全部が固定されている。

## 使い方

```bash
# ROS 2 Humble / Jazzy のワークスペースへ
cp -r room13bc_description ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select room13bc_description && source install/setup.bash

ros2 launch room13bc_description display.launch.py                 # RViz2、部屋＋机
ros2 launch room13bc_description display.launch.py model:=desk gui:=true   # 机 1 台を、スライダーで畳む
ros2 launch room13bc_description gazebo.launch.py                  # Gazebo
```

要るもの：`robot_state_publisher`、`joint_state_publisher(_gui)`、`rviz2`、Gazebo は `ros_gz_sim` と `ros_gz_bridge`
（Jazzy なら Harmonic、Humble なら Fortress の `ros-humble-ros-gz`）。

## 既知の制限

- GLB は glTF の決まりどおり Y 上で書いてある。RViz2 がそれを Z 上に直さないと部屋が横倒しに出るので、RViz2 の既定は obj にしてある。
- ガラス（MTL の `d 0.15`）は RViz2 では透けないことがある。
- 部屋＋机 48 台はリンク 102、メッシュは部屋 6・机 2 種類。机の天板は revolute のヒンジに載っていて、
  重さで上限 0（平ら）に当たって止まる。`jedy_gazebo.launch.py` はヒンジを 0 で固定し、部屋を static にして
  物理の負荷を減らしている。

OBJ は全部品が頂点・uv・法線を同じ番号で持つように書き出している（trimesh の OBJ 書き出しは、
テクスチャのない部品の後で uv の番号がずれるため使っていない）。

## 作り方

このリポジトリに含まれていない生成スクリプトで、SolidWorks からの書き出し（URDF と GLB）、部屋と机の
モデルの直方体、机の配置から作った。
