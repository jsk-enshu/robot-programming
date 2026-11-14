# scikit-robotの逆運動学

{doc}`../robot-programming-1-2026` の「手先の位置で動かす (逆運動学)」では，`robot.rarm.inverse_kinematics(target, rotation_axis=False)` で手先の位置だけを合わせた．
このページでは，その先として次を扱う．

- `inverse_kinematics` の引数 (手先の向きをどこまで合わせるか，どの点を合わせるか，どの関節を使うか，収束の判定)
- 解けないときにどうするか
- 解をビューワで確かめてから送る

2025 年度までの EusLisp の `:inverse-kinematics` ({doc}`euslisp-ik`) と考え方は同じで，引数の名前と単位 (m, rad) が違う．
対応は {doc}`euslisp-to-skrobot` にもある．

以下の例は scikit-robot 0.4.20 (jedy_skrobot の venv に入っているもの) と Gazebo で動かして確かめた．
数値はそのときの出力である．どの例も，最初に次をしてある前提で書く．

```{code-block} python
from jedy_skrobot import connect
from skrobot.coordinates import Coordinates
import numpy as np

robot, ri = connect()
ri.servo_on()
```

## 引数の一覧

`help(robot.rarm.inverse_kinematics)` や，ipython なら `robot.rarm.inverse_kinematics?` で全部見られる．よく使うのは次である．

| 引数 | 既定 | 意味 | EusLisp |
| --- | --- | --- | --- |
| `target_coords` (1 番目) | | 目標の座標 (位置と向き)．`Coordinates` | 1 番目の引数 |
| `move_target` | `robot.rarm.end_coords` | 目標に合わせる点 | `:move-target` |
| `rotation_mask` | `True` | 向きをどの軸まわりで合わせるか (下の節) | `:rotation-axis` (意味が逆) |
| `position_mask` | `True` | 位置をどの軸で合わせるか (`'xy'` なら x, y だけ) | `:translation-axis` (意味が逆) |
| `joint_list` | 腕の 7 関節 | 動かす関節 | `:link-list` |
| `invariant_joint_list` | なし | 動かさない関節 | |
| `stop` | 50 | 繰り返しの最大回数 | `:stop` |
| `thre` / `rthre` | 0.001 m / 1° (rad で指定) | 「解けた」とみなす位置と向きの誤差 | `:thre` [mm] / `:rthre` |
| `revert_if_fail` | `True` | 解けなかったら関節を元に戻す | `:revert-if-fail` |
| `warn_if_fail` | `True` | 解けなかったときに警告を出す | |

戻り値は，解けたら動かした関節の角度 (`robot.rarm` なら 7 個の配列．全身の 24 個ではない)，解けなければ `False` である．
モデル `robot` の関節も解に変わるので，ロボットへは `robot.angle_vector()` (全身) を送る．
配列は `if av:` と書けないので，`if av is False:` で判定する．
1 日目の `rotation_axis=False` は古い書き方で，今も `rotation_mask=False` と同じに動く．

## 手先の座標

目標は `robot` の **base_link** から見た座標で書く (x が前，y が左，z が上，単位 m)．
手先の座標 (`end_coords`) は `reset_pose` では向きが base_link と同じで，x 軸が前を向いている．

```{code-block} python
robot.reset_pose()
c = robot.rarm.end_coords
print(c.parent.name, np.round(c.worldpos(), 3))   # rarm_link6 [ 0.192 -0.078  0.022]
print(np.round(c.worldrot(), 2))                  # ほぼ単位行列 (base_link と同じ向き)
```

```{warning}
`connect()` の `robot` は，起動しているロボット (Gazebo / 実機) の URDF から作る．
`from skrobot.models.jedy import Jedy; robot = Jedy()` (ロボットにつながずモデルだけ) は scikit-robot に入っている別の URDF で，
base_link の原点の高さが違う (reset_pose の右手の z が `connect()` では 0.022 m，`Jedy()` では 0.149 m)．
`Coordinates(pos=[0.2, -0.1, 0.1])` のように数字で書いた目標は，片方で解けてももう片方ではずれる．
目標は今の手先から相対的に作るか，`connect()` の `robot` で決めること．
```

## 手先の向きをどこまで合わせるか (`rotation_mask`)

Jedy の腕は 7 関節だが，手先の位置と向きを全部 (6 自由度) 自由に選べるわけではない．
関節の可動範囲が狭いので，向きまで合わせると解けない目標が多い．
そこで `rotation_mask` で「どの軸まわりの回転を合わせるか」を選ぶ．
軸は**手先の座標の**軸である (x が指の向き，z が上)．

| `rotation_mask` | 合わせる回転 | 自由な回転 | 結果 | 旧 `rotation_axis` |
| --- | --- | --- | --- | --- |
| `True` (既定) | x, y, z まわり | なし | 位置と向きが全部合う | `True` |
| `'yz'` | y, z まわり | x まわり | 手先の **x 軸の向き** (指の向き) が合う．指の向きを軸にねじるのは自由 | `'x'` |
| `'xz'` | x, z まわり | y まわり | 手先の y 軸の向きが合う | `'y'` |
| `'xy'` | x, y まわり | z まわり | 手先の z 軸の向きが合う | `'z'` |
| `False` | なし | すべて | 位置だけ合う | `False` |

```{important}
`rotation_mask` は「**合わせる**」軸，古い `rotation_axis` (と EusLisp の `:rotation-axis`) は「**自由にする**」軸を書く．
`rotation_axis='z'` と `rotation_mask='xy'` が同じで，`rotation_mask='z'` と書くと z まわりだけを合わせる別の意味になる．
```

reset_pose の右手を前に 4 cm，上に 2 cm 動かし，さらに手先をそれぞれの軸まわりに 60° 回した目標で試すと，次のようになった
(「誤差」は手先の x, y, z 軸の向きの目標とのずれ)．

| 目標の回し方 | `True` | `'yz'` | `'xz'` | `'xy'` | `False` |
| --- | --- | --- | --- | --- | --- |
| y まわり (指先を下へ) 60° | 失敗 | 失敗 | 解けた．y 軸だけ合う | 失敗 | 解けた |
| x まわり (手首をねじる) 60° | 解けた．全部合う | 解けた．x 軸だけ合う | 解けた | 解けた | 解けた |
| z まわり (指先を横へ) 60° | 失敗 | 失敗 | 失敗 | 解けた．z 軸だけ合う | 解けた |

手首のねじり (x まわり) は自由に回せるが，指先を下や横へ大きく向けることは苦手である．
指先を下へ向ける角度を変えて全部を合わせると，30° までは解け，40° からは解けなかった．

```{code-block} python
robot.reset_pose()
target = robot.rarm.end_coords.copy_worldcoords()
target.translate([0.04, 0.0, 0.02], wrt='world')   # base_link の向きで前に 4 cm，上に 2 cm
target.rotate(np.deg2rad(30), 'y')                 # 手先の y 軸まわりに 30° (指先が下を向く)

av = robot.rarm.inverse_kinematics(target)          # 位置と向きを全部合わせる
print(av is not False)                                                # True
print(np.round(np.rad2deg([j.joint_angle() for j in robot.rarm.joint_list]), 1))
# [-71.5  19.5 -23.9 -55.6  42.5  60.2  14.7]
```

`translate(..., wrt='world')` は base_link の向きで動かす．`wrt` を付けないと手先の向き (`'local'`) で動かす．
`rotate` は既定で手先の軸まわりに回す．

:::{figure} ../fig/jedy-skrobot-ik-rotation-mask.png
:align: center
:name: fig:jedy-skrobot-ik-rotation-mask

PyrenderViewer で右前から見たところ．大きい座標軸が目標 (指先を 30° 下へ)，小さい座標軸が手先 (赤が x，緑が y，青が z)．
左: `reset_pose`．中: `rotation_mask=False` (位置だけ合い，向きは前のまま)．
右: 既定の `rotation_mask=True` (向きも合うが，肘を外へ張った姿勢になる)
:::

上の例では，手先は 4.5 cm しか動かないのに，`rarm_joint0` が 30° から -71.5° へ 100° 以上回り，肘を外に張った姿勢になった (図の右)．
`rarm_joint5` は上限 (60.2°) に張り付いている．向きまで合わせると，こうした大きく違う姿勢が解になりやすい．
送る前に必ずビューワで見る (下の「ビューワで確かめる」)．

## 合わせる点 (`move_target`)

既定では `end_coords` (手先の座標) を目標に合わせる．指先など別の点を合わせたいときは，その点の座標を作って手先のリンクにくっつけ (`assoc`)，
`move_target` に渡す．

```{code-block} python
from skrobot.coordinates import CascadedCoords

robot.reset_pose()
tip = CascadedCoords(name='rarm_tip')
tip.newcoords(robot.rarm.end_coords.copy_worldcoords())
tip.translate([0.03, 0.0, 0.0])            # 手先の x 方向 (指の向き) に 3 cm
robot.rarm.end_coords.parent.assoc(tip)    # 手先のリンク (rarm_link6) にくっつける

target = Coordinates(pos=robot.rarm.end_coords.worldpos() + [0.05, 0.0, 0.0])
robot.rarm.inverse_kinematics(target, move_target=tip, rotation_mask=False)
print(np.round(tip.worldpos(), 3), np.round(robot.rarm.end_coords.worldpos(), 3))
# [ 0.241 -0.078  0.022] [ 0.211 -0.078  0.023]   指先の点が目標に着き，end_coords は 3 cm 手前
```

`CascadedCoords(parent=robot.rarm.end_coords)` のように `end_coords` を親にすると，`inverse_kinematics` が
`TypeError: Support only Link class.` で止まる．親はリンク (`end_coords.parent`) にする．

手先を今の位置から少し動かすだけなら，`move_end_pos` が短い．

```{code-block} python
robot.reset_pose()
robot.rarm.move_end_pos([0.0, 0.0, 0.03], 'world', rotation_mask=False)   # 上に 3 cm
```

## 使う関節 (`joint_list`)

Jedy には腰 (胴の関節) が無い．`robot.rarm.inverse_kinematics` は右腕の 7 関節 (`rarm_joint0`〜`rarm_joint6`) だけを動かし，
頭や左腕は変えない．EusLisp で「腰を使う・使わない」と `:link-list` を選んでいたのに当たる選択は，Jedy では次の 3 つである．

**関節を減らす**: `joint_list` で動かす関節を指定する．`invariant_joint_list` で指定した関節は動かさない．
前に 3 cm，上に 3 cm の目標 (位置だけ) で試すと，次のようになった．

```{code-block} python
robot.reset_pose()
target = robot.rarm.end_coords.copy_worldcoords().translate([0.03, 0.0, 0.03], wrt='world')
robot.rarm.inverse_kinematics(target, rotation_mask=False,
                              joint_list=robot.rarm.joint_list[:4])         # 肩と肘の 4 関節: 解けた
robot.reset_pose()
robot.rarm.inverse_kinematics(target, rotation_mask=False,
                              joint_list=robot.rarm.joint_list[:3])         # 3 関節: False
robot.reset_pose()
robot.rarm.inverse_kinematics(target, rotation_mask=False,
                              invariant_joint_list=[robot.rarm_joint0])     # 肩の根元を固定: False
```

手首を動かしたくない (持った物を傾けたくない) ときなどに使う．減らすほど解ける目標は減る．

**両腕**: 腰が無いので右腕と左腕は関節を共有しない．両手を動かすときは，片腕ずつ 2 回解けばよい．

```{code-block} python
robot.reset_pose()
tr = robot.rarm.end_coords.copy_worldcoords().translate([0.0, 0.0, 0.06], wrt='world')
tl = robot.larm.end_coords.copy_worldcoords().translate([0.0, 0.0, 0.06], wrt='world')
ok_r = robot.rarm.inverse_kinematics(tr, rotation_mask=False) is not False
ok_l = robot.larm.inverse_kinematics(tl, rotation_mask=False) is not False
if ok_r and ok_l:
    ri.angle_vector(robot.angle_vector(), 3.0)   # 両腕を同時に送る
    ri.wait_interpolation()
```

Gazebo では両手とも 6 cm 上がった (右手の z が 0.022 m から 0.082 m)．

`robot.inverse_kinematics([tr, tl], move_target=[robot.rarm.end_coords, robot.larm.end_coords])` と 2 つの目標を一度に渡す書き方もあるが，
このバージョンでは `rotation_mask` を付けても付けなくても `list length differ : position_mask 1, rotation_mask 1, move_target 2 ...` と出て `False` になった．

**台車も使う**: 腕だけでは届かない目標に，台車をどれだけ動かせば届くかを求められる．
`robot.inverse_kinematics` (`robot.rarm` ではない) に `use_base='planar'` を付けると，台車の前後・左右・旋回も未知数に入れて解き，`robot` 全体を動かす．

```{code-block} python
robot.reset_pose()
far = Coordinates(pos=robot.rarm.end_coords.worldpos() + [0.20, 0.0, 0.0])  # 腕だけでは届かない
robot.inverse_kinematics(far, move_target=robot.rarm.end_coords,
                         rotation_mask=False, use_base='planar')
print(np.round(robot.worldpos(), 3))   # [0.199 0.    0.   ]  台車を前に 0.2 m 動かせば届く
robot.newcoords(Coordinates())         # モデルの台車の位置を元に戻す (戻さないと以後の目標がずれる)
```

求まるのはモデルの上での台車の位置だけである．実際に動かすには `ri.go_velocity` で走らせる ({doc}`../robot-programming-1-2026` の「台車を動かす」)．
台車は指令どおりの距離を進まないので，走ったあとは目標を見直してから腕の IK を解き直す．

## 収束の判定 (`stop`, `thre`, `rthre`, `revert_if_fail`)

IK は今の関節角度から少しずつ関節を動かして目標に近づける．`stop` 回繰り返しても誤差が `thre` (位置) と `rthre` (向き) 以下にならなければ失敗である．

```{code-block} python
robot.reset_pose()
far = Coordinates(pos=robot.rarm.end_coords.worldpos() + [0.20, 0.0, 0.10])   # 前に 20 cm，上に 10 cm
print(robot.rarm.inverse_kinematics(far, rotation_mask=False))
# inverse_kinematics failed after 50 iterations: rarm_end_coords: position error 0.2211 m (threshold 0.0010 m),
# rotation error 0.0 deg (threshold 1.0 deg); joint angles were reverted to the initial pose. ...
# False
```

- **`stop`**: 前に 5 cm，左に 3 cm，上に 8 cm (位置だけ) は `stop=5` では失敗，`stop=10` 以上で解けた．既定の 50 で足りないことは少ない．
- **`thre`, `rthre`**: 「指先を 40° 下へ」の目標 (上の表の続き) は既定では解けず，`revert_if_fail=False` で最後の姿勢を見ると位置が 10.8 mm 足りなかった．
  `thre=0.015` (15 mm) にすると `True` が返り，そのときの位置の誤差は 14.6 mm だった．
  ゆるめた分だけ手先は目標からずれるので，つかむ・触るなどずれが困る動きでは使わない．
- **`revert_if_fail=False`**: 失敗しても関節を戻さず，近づけたところまでの姿勢を返す．上の遠い目標 (最初は 0.224 m 離れている) では，0.221 m 残して止まった．
  「どちらへ届かないのか」をビューワで見るのに便利だが，戻り値が配列なので `is False` では失敗が分からない．送る前に誤差を自分で測ること．

```{code-block} python
robot.reset_pose()
robot.rarm.inverse_kinematics(far, rotation_mask=False, revert_if_fail=False, warn_if_fail=False)
print(np.linalg.norm(robot.rarm.end_coords.worldpos() - far.worldpos()))   # 0.2211 [m]
```

## 解けないとき

IK は今のモデルの関節角度から解き始めるので，同じ目標でも始めの姿勢で解けたり解けなかったりする．
解けないときは次を順に試す．

1. **届く範囲か確かめる**: reset_pose の右手から真っすぐ前へ位置だけで試すと，6 cm 先までは解け，8 cm 先は解けなかった (base_link から前に約 0.25 m)．
   `revert_if_fail=False` で最後の姿勢をビューワで見ると，腕が伸び切っているか，関節が範囲の端 (`robot.rarm_joint1.max_angle` など) に当たっているかが分かる．
2. **reset_pose から解く**: 前の IK の結果から続けて解くと，変な姿勢から始まって解けないことがある．`robot.reset_pose()` してから解く．
3. **向きをゆるめる**: `rotation_mask=True` → `'yz'` (指の向きだけ) → `False` (位置だけ) の順に．
4. **途中の点 (経由点) を通る**: 遠い目標は，途中の点を先に解き，そこから目標を解く．
5. **`batch_inverse_kinematics` で初期姿勢を変えて何度も解く** (下)．

1〜3 をまとめると，例えば次のようになる．

```{code-block} python
def solve_ik(robot, arm, target):
    """IK を順に条件をゆるめて解く．解けた条件を返し，解けなければ None (モデルは元のまま)．"""
    start = robot.angle_vector().copy()
    for seed in ('current', 'reset_pose'):
        for mask in (True, 'yz', False):      # 向きを全部 → 指の向きだけ → 位置だけ
            robot.angle_vector(start)
            if seed == 'reset_pose':
                robot.reset_pose()
            if arm.inverse_kinematics(target, rotation_mask=mask,
                                      warn_if_fail=False) is not False:
                return seed, mask
    robot.angle_vector(start)
    return None
```

指先を 60° 下へ向ける目標では `('current', False)` (位置だけで解けた)，30° では `('current', True)`，30 cm 先の目標では `None` が返った．
どの条件で解けたかを表示しておくと，あとで「向きが合っていない」ことに気づける．

**経由点**: reset_pose の右手から前に 1 cm，右に 4 cm，上に 12.5 cm の目標は，reset_pose から直接では解けなかった．
途中の点を先に解き，2 つの姿勢を `ri.angle_vector_sequence` で続けて送ると届いた．

```{code-block} python
robot.reset_pose()
start = robot.rarm.end_coords.worldpos()
goal = Coordinates(pos=start + [0.01, -0.04, 0.125])
via = Coordinates(pos=(start + goal.worldpos()) / 2)      # 真ん中の点

avs = []
for c in [via, goal]:
    if robot.rarm.inverse_kinematics(c, rotation_mask=False) is False:
        raise RuntimeError('IK failed at {}'.format(c.worldpos()))
    avs.append(robot.angle_vector().copy())              # 経由点の解から次を解く
ri.angle_vector_sequence(avs, [2.0, 2.0])                # 経由点まで 2 秒，そこから 2 秒
ri.wait_interpolation()                                  # 4.06 秒 (シミュレーション時刻) で着いた
```

**`batch_inverse_kinematics`**: 初期姿勢を少しずつ変えて何度も解き直す．上の経由点の例の目標も，これなら直接解けた．
返り値の `solutions` は腕の 7 関節だけで，モデルは変わらないので，自分で `robot.rarm.angle_vector(...)` に入れる．

```{code-block} python
robot.reset_pose()
res = robot.rarm.batch_inverse_kinematics([goal], rotation_mask=False, attempts_per_pose=20)
print(res.success_flags)                 # [True]
if res.success_flags[0]:
    robot.rarm.angle_vector(res.solutions[0])   # 右腕の 7 関節を解に
```

## ビューワで確かめる

IK の解は，関節の範囲は守るが，**体や台車にぶつからないことは確かめていない**．
ロボットに送る前に PyrenderViewer で姿勢を見る (ビューワの使い方は {doc}`skrobot-workflow`)．
目標の位置と向きに座標軸 (`Axis`) を置き，手先にも座標軸をくっつけておくと，ずれがすぐ分かる．

```{code-block} python
from skrobot.model.primitives import Axis
from skrobot.viewers import PyrenderViewer

robot, ri = connect(load_meshes=True)    # ビューワにはメッシュが要る
viewer = PyrenderViewer()
viewer.add(robot)
viewer.show()

robot.reset_pose()
target = robot.rarm.end_coords.copy_worldcoords()
target.translate([0.04, 0.0, 0.02], wrt='world')
target.rotate(np.deg2rad(30), 'y')

target_axis = Axis(axis_radius=0.004, axis_length=0.06,
                   pos=target.worldpos(), rot=target.worldrot())    # 目標
viewer.add(target_axis)
hand_axis = Axis(axis_radius=0.0025, axis_length=0.04)
hand_axis.newcoords(robot.rarm.end_coords.copy_worldcoords())
robot.rarm.end_coords.assoc(hand_axis)                              # 手先についてくる
viewer.add(hand_axis)

av = robot.rarm.inverse_kinematics(target)
viewer.redraw()     # ここで姿勢を見る．おかしければ送らずに目標や rotation_mask を変える
```

見るところは次である．

- 手先の座標軸が目標の座標軸に重なっているか (位置だけのときは原点だけ重なる)
- 腕が胴・頭・反対の腕・台車にめり込んでいないか
- 今の姿勢から大きく違う姿勢になっていないか (上の例の 100° 回る肩など．動く途中で何かに当たりやすい)
- 机などがあるなら，`Box` を置いて手が突き抜けていないか ({doc}`skrobot-workflow`)

## ロボットへ送る

```{code-block} python
t0 = ri.now()
ri.angle_vector(robot.angle_vector(), 3.0)   # 3 秒で
ri.wait_interpolation()
print(ri.now() - t0)                          # 3.06 [s]

plan = robot.angle_vector().copy()
robot.angle_vector(ri.angle_vector())         # 実際の角度をモデルに入れて
reached = robot.rarm.end_coords.worldpos()
print(np.linalg.norm(reached - target.worldpos()) * 1000)   # 0.9 [mm]
print(np.max(np.abs(np.rad2deg(plan - robot.angle_vector()))))   # 0.3 [deg]
```

Gazebo では，上の「指先を 30° 下へ」の解を 3 秒で送ると，実際の手先と目標の差は 0.9 mm，送った角度と実際の角度の差は最大 0.3° だった．
実機では重力や物との接触でもっと差が出る．`robot.angle_vector(ri.angle_vector())` で実際の角度をモデルに戻してから手先の位置を測ると，どれだけずれたかが分かる．

- 時間は動く量に合わせて長めにする．速すぎると関節の速さの上限で時間が伸ばされる ({doc}`../robot-programming-2-2026` の「実機に出す前のチェック」)．
  Gazebo では上の 100° 回る解を 1 秒で送っても伸ばされなかった (1.06 秒で着いた) が，実機で肩が 1 秒に 100° 振れるのは危ない．
- 腕だけを送るなら `controller_type='right_arm_controller'` を付ける．頭など他のコントローラの動きを止めない ({doc}`../robot-programming-1-2026` の「頭と腕を別々に動かす」)．
- 次の IK は，送ったあとのモデル (または `robot.angle_vector(ri.angle_vector())` で読み直したモデル) から解く．
