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

(jedy-teach-follow)=
## 2 台の Jedy で教示する (手で動かした姿勢をもう 1 台にまねさせる)

Jedy を 2 台，それぞれの Dual USB アダプタで 1 台の PC につなぐと，**片方 (教える側) をトルク OFF のまま手で動かし，もう片方 (まねる側) をトルク ON にしてその姿勢を追わせる**ことができる．
手で動かした姿勢を記録して，あとで再生することもできる．

### アダプタを見分ける

Dual USB アダプタはどれも同じ名前 (`/dev/actuo_kondo_rcb4mini`) とシリアル番号を持つので，2 つ挿すと名前では区別できない．
**PC のどの USB の口に挿したか** (`/dev/serial/by-path/`) で区別する．
1 つずつ挿して，増えた行を確かめる．

```{code-block} console
$ ls -l /dev/serial/by-path/
... pci-0000:00:14.0-usb-0:1.1:1.0-port0 -> ../../ttyUSB0   # 教える側のアダプタ
... pci-0000:00:14.0-usb-0:1.2:1.0-port0 -> ../../ttyUSB1   # まねる側のアダプタ
```

アダプタを別の口に挿し替えると名前が変わるので，そのときはもう一度確かめる．

### 2 台を起動する

それぞれ別の端末で，**別の `namespace:=`** と，そのロボットのアダプタ (`port:=`) を付けて起動する．
同じ `namespace:=` で 2 つ起動しようとすると，「すでに起動しています」と出て止まる．

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py namespace:=jedy_leader \
    port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1:1.0-port0 rviz:=false joy:=false wheels:=false
```

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py namespace:=jedy_follower \
    port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2:1.0-port0 rviz:=false joy:=false wheels:=false
```

- `namespace:=jedy_leader` で起動したロボットの topic などは `/jedy_leader/joint_states` のように `/jedy_leader` の下に出る．サーボの ON/OFF は `/jedy_leader/actuo_bus_servicer/servo/set` になる．
  Python からは `connect(namespace='/jedy_leader')` でつなぐ．
- `namespace:=` を付けないときは，これまでどおり `/jedy_fullbody` (サーボの ON/OFF は `/actuo_bus_servicer`) になる．
- `namespace:=` を付けたロボットで確かめたのは頭・腕・グリッパだけで，RViz・ジョイスティック・台車は確かめていない．教示には要らないので `rviz:=false joy:=false wheels:=false` にする．

### 教示する

3 つ目の端末で次を実行する．

```{code-block} console
$ ros2 run jedy_skrobot teach_follow --record poses.yaml
```

1. まねる側の頭・両腕・両グリッパのトルクが ON になり，**5 秒かけて教える側と同じ姿勢まで動く** (`--approach` で秒数を変えられる)．まねる側の周りに物や人がいないことを確かめてから実行する．
2. そのあとは，教える側を手で動かすと，まねる側が同じように動く．教える側の姿勢を 1 秒に `--rate` 回 (既定 25 回) 送り，まねる側は前に送られた姿勢から次の姿勢まで一定の速さで動くので，少し遅れてついてくる．
3. **Enter** を押すと，そのときの教える側の姿勢が `poses.yaml` に追記される．**q** で終わる．

- 教える側のトルクは起動したときから OFF で，`teach_follow` は ON にしない．腕は重力で下がるので，手で支えながら動かす．
- 終わってもまねる側はトルク ON のまま最後の姿勢を保つ．脱力させるときは launch を Ctrl-C で止めるか，`ri.servo_off()` を使う．
- 動かす部位は `--parts` で選べる (`head`, `rarm`, `larm`, `rhand`, `lhand` をカンマ区切り．例: `--parts rarm,rhand`)．
- グリッパは機体ごとに閉じきる角度が少し違う．教える側が閉じきった角度にまねる側が届かないときは，まねる側は {ref}`jedy-gripper-hold-relax` の弱い握り直しになる．

### 記録した姿勢を再生する

```{code-block} console
$ ros2 run jedy_skrobot teach_follow --play poses.yaml --follower /jedy_follower --time 1.5
```

`poses.yaml` の姿勢を順に，1 つあたり `--time` 秒かけて再生する．1 台だけ (`namespace:=` なし) で起動したロボットで再生するときは `--follower /jedy_fullbody` にする．
`poses.yaml` は関節名と角度 [deg] の YAML なので，自分のプログラムから読んで `ri.angle_vector()` に渡すこともできる．
