# 実機の使用方法

本ページでは，双腕移動台車ロボットJedyの実機を使用する際の基本的な操作方法について説明する．

## サーボ制御基板 (RCB-4mini)

2026 年度から，Jedy のサーボ制御基板は近藤科学の [RCB-4mini](https://kondo-robot.com/product/rcb4_mini) になった．
ロボットの胴体の中にあり，頭・両腕・グリッパ・車輪のサーボ 22 個はすべてここにつながっている．
PC とは USB シリアルアダプタ (Kondo DUAL USB ADAPTER HS) でつなぐ．

::::{grid} 2
:::{grid-item}
```{figure} ../fig/jedy-rcb4mini-board.jpg
---
name: fig:jedy-rcb4mini-board
---
RCB-4mini
```
:::
:::{grid-item}
```{figure} ../fig/jedy-rcb4mini-in-body.jpg
---
name: fig:jedy-rcb4mini-in-body
---
胴体に付けた RCB-4mini (右上の基板) と電源スイッチ (下の黒いスイッチ)
```
:::
::::

## 電源の入れ方

電源は **安定化電源を 7.4 V に設定して** 使う．バッテリーで動かすこともできる．

:::{danger}
**安定化電源の出力を ON にする前に，必ず電圧が 7.4 V になっていることを確かめること．**

前に使った人の設定が残っていて，電圧が高いまま出力を ON にすると，サーボを壊すおそれがある
(RCB-4mini 自体の電源電圧は 6〜12 V だが，つながっているサーボに合わせて 7.4 V にする)．
電圧を変えるのは出力が OFF の間だけにし，表示が 7.4 V になっていることを目で確かめてから ON にする．
:::

### 安定化電源で動かす

1. 安定化電源の出力が OFF になっていることを確かめる．
2. 電圧を **7.4 V** に設定し，表示が 7.4 V になっていることを確かめる．
3. 安定化電源をロボットの電源コネクタにつなぐ．
4. 安定化電源の出力を ON にする．
5. ロボットの電源スイッチを ON にする．RCB-4mini の LED が **3 回点滅** すれば起動している (下の動画)．

<video width="360" controls muted preload="metadata" poster="../fig/jedy-rcb4mini-power-on.jpg">
  <source src="../fig/jedy-rcb4mini-power-on.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** 電源を入れたところ．RCB-4mini (右上の基板) の LED が最初に 3 回点滅する．

### バッテリーで動かす

1. サーボバッテリーの白い VH2 ピンコネクタを，ロボットの電源コネクタにつなぐ．
2. ロボットの電源スイッチを ON にする．RCB-4mini の LED が 3 回点滅すれば起動している．

```{figure} ../fig/jedy-power-connector-switch.png
---
name: fig:jedy-power-connector-switch
---
電源をつなぐ VH2 ピンコネクタと電源スイッチの位置 (バッテリーで動かすとき)
```

起動しない (LED が点滅しない) ときは，安定化電源の出力が ON か，電圧が 7.4 V か，バッテリーが充電されているか，コネクタが挿さっているかを確かめる．
それでも LED が 3 回点滅しないときは，次の「LED が 3 回点滅しないとき」で RCB-4mini に設定を書き込む．
基板は起動したのに PC から動かせないときは，下の「起動できないとき」を見る．

(jedy-rcb4mini-flat-startup)=
### LED が 3 回点滅しないとき (RCB-4mini に設定を書き込む)

電源・電圧・コネクタに問題がないのに LED が 3 回点滅しないときは，RCB-4mini に起動時の設定が書き込まれていない
(新しい基板や，近藤科学の HeartToHeart4 で書き換えた基板)．
次のコマンドで，起動時の設定を RCB-4mini に書き込む．
この設定にすると，電源を入れてもサーボには何も命令せず (サーボは力が抜けたまま)，緑の LED を 3 回点滅させるだけになる．
書き込みは 1 回でよく，電源を切っても消えない．

1. launch が動いていたら Ctrl-C で止める．書き込みにも同じ USB シリアルアダプタを使うので，launch が動いていると書き込めない．
2. ロボットの電源を入れたまま，USB シリアルアダプタ (Kondo DUAL USB ADAPTER HS) を PC に挿す．
   アダプタの LED が **緑** (シリアルモード) になっていることを確かめる．
3. 書き込む．

   ```{code-block} console
   $ $(ros2 pkg prefix actuo_ros)/share/actuo_ros/venv/bin/rcb4-rom flat-startup --blink 3
   Opened ftdi://Kondo:DUAL USB ADAPTER/1 at 625000 baud
   Opened ftdi://Kondo:DUAL USB ADAPTER/1 at 115200 baud
   Switched ftdi://Kondo:DUAL USB ADAPTER/1 to 625000 baud
   Saved 262144 bytes of ROM to rcb4-rom-20261115-234259.bin (undo with: rcb4-rom restore rcb4-rom-20261115-234259.bin)
   Servo data: 0 bytes written
   Startup program: 233 bytes written
   Takes effect at the next power-up
   ```

4. ロボットの電源を切って入れ直し，LED が 3 回点滅することを確かめる (設定は次に電源を入れたときから効く)．

`rcb4-rom` は `actuo_ros` と一緒に入る (ワークスペースをビルドしてあれば入っている)．
書き込む前に，今の ROM の中身を今いるディレクトリに `rcb4-rom-<日時>.bin` として保存する．
元に戻すときは，出力に書かれている `rcb4-rom restore rcb4-rom-<日時>.bin` を実行する．
`rcb4-rom` の詳しい使い方 (`rcb4-rom show` で今の設定を見る，など) は [rcb4 の README (RCB-4 board ROM)](https://github.com/iory/rcb4#rcb-4-board-rom-rcb4-rom) を参照すること．

::::{grid} 2
:::{grid-item}
```{figure} ../fig/jedy-rcb4mini-write-rom.jpg
---
name: fig:jedy-rcb4mini-write-rom
---
基板だけで書き込むときのつなぎ方．USB シリアルアダプタのケーブルを RCB-4mini の COM 端子 (青いコネクタ) に挿し，白い VH2 コネクタから電源を入れる
```
:::
:::{grid-item}
```{figure} ../fig/jedy-usb-adapter-serial-led.jpg
---
name: fig:jedy-usb-adapter-serial-led
---
USB シリアルアダプタの LED が緑のときがシリアルモード (RCB-4mini と通信できる)
```
:::
::::

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

1. 電源を入れる (上の「電源の入れ方」)．
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

(jedy-real-rviz-check)=
### RViz で関節角度が正しいか確かめる

上の launch を起動すると RViz も開く (開かないようにするには `rviz:=false`)．
3D の画面のロボットのモデルは，実機の関節角度 (サーボが測った角度) で描かれる．
左の **Servo パネル**では関節ごとにトルクの ON/OFF ができ (ALL ON / ALL OFF もある)，
`pointcloud:=true` を付けて起動すると頭のカメラの点群も 3D の画面に出る．

**最初に，トルクが OFF のまま (起動した直後はすべて OFF) Jedy の首や腕を手でゆっくり動かしてみよう．**
RViz のモデルが実機と同じ姿勢になり，同じ向きに動けば，組み立てと設定は正しい．

```{figure} ../fig/jedy-real-rviz-torque-off.jpg
---
name: fig:jedy-real-rviz-torque-off
---
トルク OFF のまま首を下に向け，両腕を下ろしたところ．左：Servo パネル (すべて OFF)，中：RViz のモデル (後ろから見ている)，右：実機 (前から見ている)
```

モデルと実機の姿勢が違うとき (関節の向きが逆，角度がずれている，動かした関節と別の関節が動く) は，組み立てを間違えている．
{doc}`jedy-assembly` を見ながら，ドライバーでネジを外して組み立て直そう．
組み立て直すときは，launch を Ctrl-C で止め，電源を切ってから行う．

### 最初に `ri.servo_on()` する

**実機はサーボ OFF で起動する** (台車の車輪だけは ON)．OFF の関節に角度を送っても動かず，エラーも出ない．
腕や首が動かないときは，まず `ri.servo_on()` をしたか確かめる．
`ros2 run jedy_vision tag_pose --tilt 25` のように首を動かすコマンドも，サーボが OFF だと首が下を向かず，タグが見えない (`no tag in view`)．

```{code-block} python
from jedy_skrobot import connect
robot, ri = connect()
ri.servo_on()       # 今の姿勢のまま固まる (腕が飛ぶことはない)
```

(jedy-real-rviz-servo-panel)=
### 動かしている間の RViz を見る

<video width="100%" controls preload="metadata" poster="../fig/jedy-real-rviz-arm.jpg">
  <source src="../fig/jedy-real-rviz-arm.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** {doc}`../robot-programming-1-2026` の「腕と首を動かす」のコードを実機で実行したところ (左：RViz，右：実機，実時間)．
起動した直後は Servo パネルの torque がすべて OFF (赤) で，`ri.servo_on()` で ON (緑) になる．

Servo パネルの **away from command** は，コントローラが送った角度と，サーボが測った今の角度の差である．
ふつうは数度以内に収まる．
0.35 rad (約 20°) 以上離れた関節は赤で表示され，パネルに `Red: moved ≥0.35 rad from the controller's command.` と出る．
この動画では，右肩の `rarm_joint1` が送った角度まで動かず，`reset_pose` に戻したあとも 27.7° 離れたままになっている．
プログラムからは `ri.angle_vector()` で読んだ角度と送った角度を比べると分かる (`reset_pose` で 28° の差)．
腕が体や床，ケーブルに当たっている，サーボホーンの取り付けがずれている，などが考えられる．
その関節に力がかかり続けてサーボが熱くなるので，プログラムを止め，その関節だけ `ri.servo_off(joints=[...])` するか電源を切ってから，腕の周りと組み立てを確かめる．

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

- グリッパは左右とも閉じる向きが負の角度で，何もつかまずに閉じると約 −22° で止まる．
  閉じる側の可動範囲 (`position_limit_deg` の −28.6°) までは約 6° しかないので，`error_deg` を 6 以上にすると握り直さなくなる (全力で握り続けて熱くなる)．
- 握り直さないようにするには，その関節の `hold_relax:` の行を消す．全力で握り続けるので，長くつかんだままにしない．
- 変えたら `ros2 launch jedy_bringup jedy_fullbody_all.launch.py` を Ctrl-C で止めて起動し直す．
  `colcon build --symlink-install` でビルドしていれば，ビルドし直さなくてよい．

## 2 台以上の Jedy で教示する

1 台の Jedy を手で動かして，ほかの Jedy にその姿勢をまねさせるやり方は {doc}`jedy-teach-follow` にある．
