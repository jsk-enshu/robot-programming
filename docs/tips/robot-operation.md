# 実機の使用方法

本ページでは，双腕移動台車ロボットJedyの実機を使用する際の基本的な操作方法について説明する．

## サーボバッテリーの接続と電源ON

実機を動かすには，まずサーボ制御基板にバッテリーを接続し，電源をONにする必要がある．

### バッテリーの接続手順

1. **バッテリーの接続**：サーボバッテリーの白いVH2ピンコネクタをサーボ制御基板のVH2ピンに接続する
2. **電源ON**：ロボット背面にあるサーボ制御基板のスイッチを押すと，青いLEDが点灯し，数秒後にブザー音が鳴る

```{figure} ../fig/how_to_switch_on_jedy_servo.png
---
name: fig:how_to_switch_on_jedy_servo
---
サーボ制御基板の電源ON方法
```

### 電源ONの確認

電源が正しくONになると，以下のような状態になる：

- 青いLEDが点灯する
- 数秒後にブザー音（ビープ音）が鳴る
- サーボモータに電力が供給される

## サーボバッテリーの充電方法

:::{danger}
**サーボバッテリーの充電に関する重要な注意事項**

サーボバッテリーを充電する際は，**必ず専用のACアダプタとUSBケーブルを使用すること**．

<span style="color:red; font-weight:bold;">絶対にPCのUSBポートには接続しないこと！</span>

PCに接続すると，PCやバッテリーが破損する可能性がある．
:::

### 正しい充電手順

1. **専用ACアダプタの使用**：配布された専用のACアダプタをコンセントに接続する
2. **USBケーブルの接続**：専用のUSBケーブルでACアダプタとバッテリーを接続する
3. **充電状態の確認**：バッテリーのLEDインジケータで充電状態を確認する
   - LED点滅：待機中または充電できないバッテリー
   - LED点灯：充電中
   - LED消灯：充電完了
   - LED高速点滅：エラー

```{figure} ../fig/how_to_charge_battery.png
---
name: fig:how_to_charge_battery
---
サーボバッテリーの正しい充電方法（専用ACアダプタを使用）
```

:::{seealso}
充電器の詳細な仕様や使用方法については，[BX-32MH充電器マニュアル（PDF）](https://kondo-robot.com/w/wp-content/uploads/BX-32MH-Manual.pdf)を参照のこと．
:::

(jedy-real-run)=
## 実機を起動して動かす

1 日目の資料のプログラムは，Gazebo と同じものが実機でもそのまま動く．
下の動画は，{doc}`../robot-programming-1-2026` の「腕と首を動かす」と「台車を動かす」のコードを，実機でそのまま実行したところである．

<video width="100%" controls preload="metadata" poster="../fig/jedy-real-arm.jpg">
  <source src="../fig/jedy-real-arm.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** 「腕と首を動かす」のコードを実機で実行したところ (実時間)．
`ri.servo_on()` のあと，頭だけ OFF / ON，`reset_pose` へ 3 秒，首を回して傾け，うなずきを 3 回，逆運動学で右手を前・上へ 3 cm，頭 (2 秒) と右腕 (4 秒) を同時に動かす．

<video width="100%" controls preload="metadata" poster="../fig/jedy-real-base.jpg">
  <source src="../fig/jedy-real-base.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** `ri.tuck_arms()` のあと，`ri.go_velocity()` で前・後ろ・右・左に 0.1 m/s で 2 秒ずつ，その場で左・右に 1.0 rad/s で 1.5 秒ずつ動かしたところ (実時間)．

### 起動

1. 電源を入れる (上の「サーボバッテリーの接続と電源ON」)．
2. USB シリアルアダプタ (Kondo DUAL USB ADAPTER HS) と頭のカメラ (D405) の USB を PC に挿す．
3. 起動する．

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true
```

起動できたかは，ログに次の行が出たかで分かる．

```{code-block} text
[actuo_bus_node] created shm 'jedy_fullbody' (22 joints); servicing at 30.0 Hz
...
Configured and activated mecanum_drive_controller
```

コントローラがそろっているかは別のターミナルで確かめる (7 つが `active` になる)．

```{code-block} console
$ ros2 control list_controllers -c /jedy_fullbody/controller_manager
```

### 最初に `ri.servo_on()` する

**実機はサーボ OFF で起動する** (台車の車輪だけは ON)．OFF の関節に角度を送っても動かず，エラーも出ない．
腕や首が動かないときは，まず `ri.servo_on()` をしたか確かめる．
`ros2 run jedy_vision tag_pose --tilt 25` のように首を動かすコマンドも，サーボが OFF だと首が下を向かず，タグが見えない (`no tag in view`)．

```{code-block} python
from jedy_skrobot import connect
robot, ri = connect()
ri.servo_on()       # 今の姿勢のまま固まる (腕が飛ぶことはない)
```

### Gazebo と違うところ

実機で 1 日目の資料を通したときに見つかった違いである．

| | Gazebo | 実機 |
| --- | --- | --- |
| 起動したときのサーボ | ON | **OFF** (車輪だけ ON) |
| `ri.servo_off()` | 腕が垂れる | 腕が**床や体に落ちる**．支える人がいるときだけ使う |
| `connect()` の警告 | 出ない | サーボ OFF で腕が垂れていると `rarm_joint2: ... exceeds max ...; clipped to max` が出る．脱力した腕がモデルの関節の範囲の外にあるだけで，害はない |
| `reset_pose` を送ったときの誤差 | ほぼ 0 | 両腕の `joint3` が重力で 8° ほど下がる．ほかの関節は 1° 以内 |
| 逆運動学で動かした手先 | 目標どおり | 目標から 5〜7 mm (3 cm 前・3 cm 上の目標で測った) |
| 台車の速さ | どの速さでも動く | 前後・左右 0.078〜0.156 m/s，旋回 0.83〜1.66 rad/s 程度．遅すぎると止まる |

```{note}
**実機の深度は 0.5 m より遠くても値が入る．** 部屋の中を写した `ri.depth_image()` で nan だったのは 5% ほどで，
残りには 0.47〜37 m の値が入っていた．`ri.detection_position(d)` も遠い物で `None` にならず，3 m 先の椅子で 3.03 m を返した．
D405 が得意なのは 0.07〜0.5 m なので，それより遠い値の精度は保証されない．遠い物の位置を使うときは，近づいてから測り直す．
```

頭のカメラは床から低いので，床に置いた物を見るには首を下に向ける (下の写真は `head_joint1` が 20°)．

```{figure} ../fig/jedy-real-detect-cube.jpg
---
name: fig:jedy-real-detect-cube
---
実機で `ri.what_do_you_see(["rubik's cube"])` を呼んだときの `/object_detection/image`．
床のルービックキューブ 2 つが見つかり，`ri.detection_position(d)` は [0.58, 0.09, −0.07] と [0.41, −0.29, −0.07] (m)．
床の AprilTag (64 mm) は `tag_pose --size 0.064 --tilt 25` で 0.5〜0.7 m 先に見つかる
```

```{note}
`approach_tag` はタグが立っていて，ロボットの方を向いていることを前提にしている (タグの面から `--standoff` だけ手前を目標にする)．
**床に寝かせたタグには使わない．** 目標がタグの真上になり，タグに乗り上げる向きに走る．
```

### 起動できないとき

エラーが出ても **launch は終わらない**．そのあと `waiting for service /jedy_fullbody/controller_manager/list_controllers` などが流れ続けるので，
起動しないときはログを上にさかのぼって `ABORT:` で始まる行を探す．

| ログに出ること | 原因と直し方 |
| --- | --- |
| `` `/dev/actuo_kondo_rcb4mini` を待っています `` | USB シリアルアダプタが挿さっていない，または udev ルールが入っていない ({doc}`../robot-programming-1-2026` の「実機 (RCB-4mini)」) |
| `ABORT: board answers at none of 625000, 115200, 625000, 1250000 bit/s` | アダプタの先の制御基板が答えない．電源が入っているか，アダプタの LED が緑か (シリアルモード)，ケーブルが挿さっているかを確かめる．アダプタを挿し直すと直ることがある |
| `ABORT: Configured servos did not answer: head_joint1 (id 34)` | その関節のサーボが答えない．**そのサーボのケーブル**を確かめる．電源を入れ直しても直らなかったときは，ケーブルを替えて直った |
| `segment 'jedy_fullbody' not ready within 10.0s` | 上の 2 つのどれかで actuo が止まった結果．その前の `ABORT:` を見る |
| `No RealSense devices were found!` / `Frames didn't arrived within 5 seconds` | 頭のカメラ (D405) の USB が抜けている．首まわりのケーブルを触ったあとに抜けることがある．挿し直すと戻る |

ケーブルを抜き差しするときは TA を呼び，launch を Ctrl-C で止めてから行う．

### 終わるとき

サーボを OFF にすると腕が落ちるので，**先に腕を下ろした姿勢に戻してから** OFF にする．
起動した直後 (サーボ OFF で腕が垂れている) の姿勢を覚えておき，最後にそこへ戻すとよい．

```{code-block} python
robot, ri = connect()
rest = ri.angle_vector().copy()   # 起動直後の，腕を下ろした姿勢
ri.servo_on()
# ... ここで動かす ...
ri.angle_vector(rest, 4.0)        # 4 秒かけて下ろした姿勢へ
ri.wait_interpolation()
ri.servo_off()
```

そのあと launch を Ctrl-C で止め，電源を切る．

(jedy-gripper-hold-relax)=
## グリッパでつかんだ物が落ちるとき

### つかんだあと，グリッパは弱く握り直す

グリッパで物をつかむと，グリッパは物に当たって止まり，指令した角度まで閉じられない．
そのままだとサーボは全力で押し続けて熱くなるので，実機 (RCB-4mini) のグリッパは **指令が 1 秒変わらず，指令から 3° 以上離れたまま** になると，
**今の角度から 2° だけ閉じる側** を目標にして弱く握り直す (actuo の `hold_relax`)．
新しい指令 (開く，別の角度にする) を送ると元に戻る．

2026 年 11 月に測ったときは，物に当たって押している間はサーボの電流が待機中より約 0.9 A 多く，弱く握り直したあとは約 0.05 A 多いだけだった．
その状態で物を 18 秒つかんだままにしても，グリッパは開かなかった．

これは実機だけの動きで，Gazebo のグリッパは握り直さない．

### 弱く握り直しているかを確かめる

```{code-block} console
$ ros2 topic echo /diagnostics | grep -A1 "bus/hold_relax"
```

`message` が `Held lighter than commanded: rarm_gripper_joint commanded ...` なら弱く握り直している．
`No servo is held lighter than commanded.` なら握り直していない (指令した角度で全力で握っている)．

### 落ちるときに変えるところ

まず，つかむ位置が浅くないか，物が重すぎないかを確かめる．握り直す前 (つかんで 1 秒以内) に落ちるなら，握る強さではなく，つかみ方の問題である．

握り直したあとに落ちるなら，`jedy/jedy_bringup/config/jedy_fullbody_rcb4.yaml` の `rarm_gripper_joint` / `larm_gripper_joint` にある次の行を変える．

```{code-block} yaml
      hold_relax: {error_deg: 3.0, seconds: 1.0, squeeze_deg: 2.0}
```

| 値 | 意味 | 変えるとどうなるか |
| --- | --- | --- |
| `squeeze_deg` | 握り直すとき，今の角度からどれだけ閉じる側を目標にするか [deg] | 大きくすると強く握る．そのぶんサーボは熱くなりやすい．まず 3〜4 にして試す |
| `error_deg` | 指令からこれ以上離れていたら握り直す [deg] | **`squeeze_deg` より大きくする** (同じか小さいと actuo が起動時にエラーにする) |
| `seconds` | 指令が何秒変わらなかったら握り直すか [s] | 長くすると全力で握る時間が延びる |

- 左手 (`larm_gripper_joint`) は閉じる向きが負の角度で，何もつかまずに閉じると約 −22° で止まる．
  閉じる側の可動範囲 (`position_limit_deg` の −28.6°) までは約 6° しかないので，`error_deg` を 6 以上にすると左手は握り直さなくなる (全力で握り続けて熱くなる)．
- 握り直さないようにするには，その関節の `hold_relax:` の行を消す．全力で握り続けるので，長くつかんだままにしない．
- 変えたら `ros2 launch jedy_bringup jedy_fullbody_all.launch.py` を Ctrl-C で止めて起動し直す．
  `colcon build --symlink-install` でビルドしていれば，ビルドし直さなくてよい．

## 2 台以上の Jedy で教示する

1 台の Jedy を手で動かして，ほかの Jedy にその姿勢をまねさせるやり方は {doc}`jedy-teach-follow` にある．
