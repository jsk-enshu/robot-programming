# scikit-robotでの対話的な作業

ロボットのプログラムは，少し書いて動かし，見て直す，を何度も繰り返して作る．
このページでは，その繰り返しを速くするための道具と手順をまとめる．

- ipython で対話的に試す (履歴，補完，`%run`，自分のモジュールの自動再読み込み)
- PyrenderViewer の操作と，目印 (座標軸・球・箱) の出し入れ
- モデルとロボットの姿勢をそろえる
- モデルで試してからロボットへ送る
- 姿勢をファイルに保存して読み込む
- 安全に止める

2025 年度までの roseus と irtviewer ({doc}`roseus-workflow`，{doc}`irtviewer`) に当たる内容である．
`robot` と `ri` の基本，ビューワの開き方は {doc}`../robot-programming-1-2026` を先に読むこと．
以下の例は Gazebo と jedy_skrobot の venv (scikit-robot 0.4.20) で動かして確かめた．

## ipython で試す

jedy_skrobot の venv には ipython が入っている．

```{code-block} console
$ $(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/ipython
```

roseus に rlwrap を付けて得ていた機能は，ipython には最初からある．

| 操作 | 働き |
| --- | --- |
| 上 / 下キー | 前 / 次の入力．途中まで打ってから押すと，その文字で始まる入力だけをたどる |
| Ctrl-R | 入力の履歴を検索する |
| Tab | 補完．`ri.wa` Tab で `ri.wait_interpolation` など，`robot.rarm.` Tab でメソッドの一覧 |
| `ri.angle_vector?` | 説明 (docstring) と引数を見る |
| `ri.angle_vector??` | ソースを見る |
| `%history -n 1-20` | 今の入力の 1〜20 行目を番号付きで出す (ファイルに写すのに便利) |
| `%history -g inverse` | 前に起動したときの入力も含めて `inverse` を含む行を探す |
| Ctrl-A / Ctrl-E / Ctrl-K / Ctrl-Y | 行頭 / 行末 / 行末まで消す / 貼り付け ({doc}`roseus-workflow` の Emacs のキーと同じ) |

履歴は ipython を終了しても残る (`~/.ipython/profile_default/history.sqlite`)．

### 接続は 1 回だけにして，ファイルを `%run -i` する

`connect()` は呼ぶたびに新しい `robot` と `ri` を作る (2 回呼ぶと別のものが 2 組できた)．
接続はファイルに書いて最初に 1 回だけ実行し，動きのプログラムは `connect()` を呼ばないファイルにして，`%run -i` で実行する．
`-i` を付けると，ipython にある `robot` や `ri` をそのまま使える．

```{code-block} python
# ~/enshu/setup_robot.py
from jedy_skrobot import connect
import numpy as np

robot, ri = connect()
ri.servo_on()
```

```{code-block} python
In [1]: %run -i ~/enshu/setup_robot.py
In [2]: %run -i ~/enshu/wave.py      # robot と ri を使う．直したらもう一度 %run -i
```

### 自分のモジュールを自動で読み直す (autoreload)

よく使う動きは関数にして自分のモジュール (ファイル) に書いておく．
ふつうは `import` したモジュールを書き換えても，ipython を起動し直すまで古いままである．
`autoreload` を使うと，書き換えたファイルが次の入力のときに読み直される．

```{code-block} python
# ~/enshu/mymotion.py
import numpy as np


def raise_hand(robot, ri, dz=0.03, time=2.0):
    """右手を dz [m] 上げる．解けなければ False を返し，ロボットには何も送らない．"""
    target = robot.rarm.end_coords.copy_worldcoords()
    target.translate([0.0, 0.0, dz], wrt='world')
    if robot.rarm.inverse_kinematics(target, rotation_mask=False) is False:
        return False
    ri.angle_vector(robot.angle_vector(), time)
    ri.wait_interpolation()
    return True
```

```{code-block} python
In [1]: %load_ext autoreload
In [2]: %autoreload 2
In [3]: import sys; sys.path.insert(0, '/home/student/enshu')   # 自分のディレクトリに合わせる
In [4]: %run -i ~/enshu/setup_robot.py
In [5]: import mymotion
In [6]: mymotion.raise_hand(robot, ri)    # 3 cm 上がる
        # ここでエディタで dz=0.03 を dz=0.05 に書き換えて保存
In [7]: mymotion.raise_hand(robot, ri)    # 起動し直さなくても 5 cm 上がる
```

Gazebo で確かめると，右手の高さは 1 回目で 0.022 m から 0.052 m，書き換えたあとの 2 回目で 0.101 m になった．
関数の引数に `robot` と `ri` を渡す形にしておくと，同じ関数をプログラムのファイルからもそのまま使える．

## PyrenderViewer の操作

開き方は {doc}`../robot-programming-1-2026` の「モデルを画面で見る」．`connect(load_meshes=True)` を忘れると何も表示されない．

| マウス | 動き |
| --- | --- |
| 左ドラッグ | 回転 |
| Ctrl + 左ドラッグ | 視線の向きを軸に回転 |
| Shift + 左ドラッグ，または中ボタンでドラッグ | 平行移動 |
| ホイール，または右ドラッグ | 拡大・縮小 |

キーはビューワの窓を選んでから押す．

| キー | 働き |
| --- | --- |
| `z` | 最初の視点に戻す (ロボットを見失ったとき．irtviewer の `:look-all` に当たる) |
| `j` | 関節の位置 (青い球) と回転軸 (赤い棒) を出す / 消す |
| `v` | 見た目のメッシュと衝突判定用のメッシュを切り替える |
| `w` | ワイヤーフレーム表示を切り替える |
| `i` | 座標軸の表示を切り替える (なし → 世界の原点 → …) |
| `s` | 今の画面を画像に保存する (ファイルを選ぶ窓が出る) |
| `q` | ビューワを閉じる |

```{warning}
`q` や窓の × でビューワを閉じると，同じ Python (ipython) の中では開き直せない．
`viewer.close()` のあとに `PyrenderViewer()` と `show()` をし直すと，`GLError ... invalid operation` が出て描画されなかった．
閉じてしまったら ipython を終了して起動し直す (`%run -i ~/enshu/setup_robot.py` で元に戻せるようにしておくとよい)．
```

画面はモデルを変えただけでは描き直されない．`viewer.redraw()` を呼ぶ．
今の画面をプログラムから保存するには `viewer.save_image('pose.png')` を使う．

### 目印を置く・消す

目標の位置に球を置いたり，手先に座標軸をくっつけたりすると，IK の目標とのずれや，向きが見やすくなる．
`skrobot.model.primitives` の形を作り，`viewer.add` で足し，`viewer.delete` で消す．

```{code-block} python
from jedy_skrobot import connect
from skrobot.coordinates import Coordinates
from skrobot.model.primitives import Axis, Box, Sphere
from skrobot.viewers import PyrenderViewer
import numpy as np

robot, ri = connect(load_meshes=True)
viewer = PyrenderViewer()
viewer.add(robot)
viewer.show()
robot.angle_vector(ri.angle_vector())

origin = Axis(axis_radius=0.004, axis_length=0.15)            # base_link の原点の座標軸
viewer.add(origin)

hand = Axis(axis_radius=0.003, axis_length=0.06)              # 手先の座標軸
hand.newcoords(robot.rarm.end_coords.copy_worldcoords())
robot.rarm.end_coords.assoc(hand)                             # 手先にくっつける (腕と一緒に動く)
viewer.add(hand)

goal = robot.rarm.end_coords.worldpos() + [0.0, 0.0, 0.08]    # 右手の 8 cm 上
ball = Sphere(radius=0.015, color=[255, 0, 0], pos=goal)      # 赤い球
viewer.add(ball)
viewer.redraw()
```

座標軸の色は，赤が x，緑が y，青が z である．
机や箱のように手が入ってはいけない物は `Box(extents=[奥行き, 幅, 高さ], face_colors=[R, G, B, 255], pos=[x, y, z])` で置ける (位置は箱の中心)．
ビューワの中では手は箱を突き抜けて表示されるので，突き抜けていたらその姿勢は送らない．

```{code-block} python
viewer.delete(ball)                     # 消す
robot.rarm.end_coords.dissoc(hand)      # 手先から外す (外さないと腕と一緒に動き続ける)
viewer.delete(hand)
viewer.redraw()
```

:::{figure} ../fig/jedy-skrobot-viewer-markers.png
:align: center
:name: fig:jedy-skrobot-viewer-markers

右前から見たところ．左: 手先にくっつけた座標軸と，目標の赤い球 (手先の 8 cm 上)．
右: 位置だけの IK で手先を球に合わせ，モデルの上で動かしたあと．座標軸は手先についてきて，向きが変わったことが分かる
:::

## モデルとロボットの姿勢をそろえる

`robot` はプログラムの中の計算用のモデルで，`ri` の向こうのロボットとは自動ではそろわない．
次のときは `robot.angle_vector(ri.angle_vector())` で今のロボットの角度をモデルに入れる．

- IK を「今の姿勢から」解く前 (前の動きが途中で止まった，重力で腕が下がった，など)
- `ri.servo_off()` のあと (脱力した腕は重力で動く)
- 実機で，サーボを OFF にして手で腕を動かしたあと
- 動いたあとに手先の位置を確かめるとき ({doc}`skrobot-ik` の「ロボットへ送る」)

ビューワをロボットに付いてこさせるには，読み込んで描き直すことを繰り返す．Ctrl-C で止める．

```{code-block} python
try:
    while True:
        robot.angle_vector(ri.angle_vector())
        viewer.redraw()
        ri.sleep(0.1)
except KeyboardInterrupt:
    pass
```

Gazebo で reset_pose から右肘 (`rarm_joint3`) だけを `ri.servo_off(joints=['rarm_joint3'])` すると，
0.5 秒後には肘が -119.2° から -85.7° まで垂れ，手先が 0.12 m 下がった．
モデルを読み直さずにここから IK を解くと，実際とは違う姿勢から計算することになる．

## モデルで試してからロボットへ

`robot` だけを変えてもロボットは動かない．新しい動きは，まずモデルだけで作ってビューワで見て，よければ `ri` に送る．
ロボットの補間と同じように，始めと終わりの姿勢の間をモデルの上で動かして見ることもできる (途中で体に当たらないかが分かる)．

```{code-block} python
import time

robot.reset_pose()
start = robot.angle_vector().copy()
if robot.rarm.inverse_kinematics(Coordinates(pos=goal), rotation_mask=False) is False:
    print('IK failed')
end = robot.angle_vector().copy()

for s in np.linspace(0.0, 1.0, 30):          # モデルの上で 1.5 秒かけて動かす
    robot.angle_vector((1 - s) * start + s * end)
    viewer.redraw()
    time.sleep(0.05)
```

```{code-block} python
ri.angle_vector(end, 3.0)                     # 見てよければ送る
ri.wait_interpolation()
```

実際のロボットの補間は直線 (上のループ) とは少し違うが，大きな動きのあらましは分かる．
ロボットにつながずにモデルだけで試すなら `ros2 run jedy_skrobot keyboard_ik --model-only` も使える．
`Jedy()` (scikit-robot のモデル) でも試せるが，base_link の高さが `connect()` の `robot` と違うので，数字で書いた目標の位置はずれる ({doc}`skrobot-ik` の「手先の座標」)．

## 姿勢をファイルに保存する・読み込む

うまくいった姿勢は保存しておくと，あとでプログラムから使える．関節の名前と角度の組で JSON に書く．
名前で書いておくと，関節の並び順に頼らない．

```{code-block} python
import json


def save_pose(path):
    robot.angle_vector(ri.angle_vector())          # 今のロボットの姿勢
    pose = {j.name: float(j.joint_angle()) for j in robot.joint_list}
    with open(path, 'w') as f:
        json.dump(pose, f, indent=1)


def load_pose(path):
    with open(path) as f:
        pose = json.load(f)
    for name, angle in pose.items():
        getattr(robot, name).joint_angle(angle)    # robot.rarm_joint0 などを名前で引く
```

```{code-block} python
save_pose('/home/student/enshu/pose_wave.json')
# ...
load_pose('/home/student/enshu/pose_wave.json')
ri.angle_vector(robot.angle_vector(), 2.0)
ri.wait_interpolation()
```

ファイルは `{"rarm_joint0": 0.8708, "rarm_joint1": 0.0, ...}` のようになる (rad，24 関節)．
首を 20°，右肩を 50° にして保存し，reset_pose に戻してから読み込むと，19.9° と 49.9° に戻った (保存したのはロボットの実際の角度)．
`np.save('pose.npy', robot.angle_vector())` でも保存できるが，こちらは関節の並び順に頼る．

実機では，サーボを OFF にして手で腕をよい姿勢に動かし (腕を支えること)，`save_pose` で保存すると，教えた姿勢を再生できる．

## 安全に止める

**Ctrl-C で止まるのは Python だけである．** 送ってある腕の動きは止まらない．
Gazebo で右肩を 6 秒かけて 90° 回す途中 (1.5 秒) で `ri.wait_interpolation()` を Ctrl-C で止めると，
`KeyboardInterrupt` で入力に戻ったが，その後の 2 秒で肩は 17.0° から -25.2° まで動き続けた．

腕をその場で止めるには `ri.cancel_angle_vector()` を呼ぶ．
同じ動きで 1.5 秒後に呼ぶと，肩は 17.3° で止まり，2 秒後も 17.5° のままだった．
`controller_type='right_arm_controller'` を付けるとそのコントローラだけを止める．
台車は `ri.stop()` で止まる．

ipython ではすぐ打てるように，最初に関数を作っておくとよい．

```{code-block} python
def stop_all():
    ri.cancel_angle_vector()   # 腕と首をその場で止める
    ri.stop()                  # 台車を止める
```

ファイルのプログラムでは `finally` に書いて，Ctrl-C やエラーで止まっても必ず呼ばれるようにする ({doc}`../robot-programming-2-2026` の「プログラムの形」)．

```{code-block} python
try:
    ...                        # 動きのプログラム
finally:
    ri.cancel_angle_vector()
    ri.stop()
```

それでも危ないとき (腕が何かに当たって押し続けている，など) は `ri.servo_off()` で脱力させるか，実機なら背面のスイッチでサーボの電源を切る．
どちらも腕が重力で落ちるので，支えられる人がそばにいるときに行う．
