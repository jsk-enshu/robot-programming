(jedy-teach-follow)=
# 複数の Jedy で動きをまねる (教示)

1 台の Jedy を手で動かすと，ほかの Jedy が同じ姿勢をまねる．
ほかの Jedy を使っている人と組んで，**1 人の Jedy (教える側) の動きを，何台もの Jedy (まねる側) にそろって再現させる**ことができる．
手で動かした姿勢を記録して，あとで再生することもできる．

<video width="100%" controls preload="metadata" poster="../fig/jedy-teach-follow.jpg">
  <source src="../fig/jedy-teach-follow.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** 実機の Jedy 3 台を 1 台の PC につないで `teach_follow` を動かしたところ (実時間)．
左の Jedy (教える側，トルク OFF) を手で動かすと，真ん中と右の Jedy (まねる側) が頭・両腕・両グリッパで同じ姿勢をとる．

## しくみ

- Jedy ごとに Dual USB アダプタを 1 つ使い，全部を 1 台の PC につなぐ．
- それぞれの Jedy を `jedy_fullbody_all.launch.py` で**別の名前空間** (`namespace:=`) に起動する．
  `namespace:=jedy_leader` で起動したロボットの topic は `/jedy_leader/joint_states` のように `/jedy_leader` の下に出て，サーボの ON/OFF は `/jedy_leader/actuo_bus_servicer/servo/set` になる．
  Python からは `connect(namespace='/jedy_leader')` でつなぐ．
- `teach_follow` が教える側の関節角を読み，1 秒に 25 回，まねる側全員のコントローラに送る．
  教える側のトルクは OFF のままなので，手で自由に動かせる．

`namespace:=` を付けないときは，これまでどおり `/jedy_fullbody` (サーボの ON/OFF は `/actuo_bus_servicer`) になる．

## アダプタを見分ける

Dual USB アダプタはどれも同じ名前 (`/dev/actuo_kondo_rcb4mini`) とシリアル番号を持つので，いくつも挿すと名前では区別できない．
**PC のどの USB の口に挿したか** (`/dev/serial/by-path/`) で区別する．
1 つずつ挿して，増えた行を確かめる．

```{code-block} console
$ ls -l /dev/serial/by-path/
... pci-0000:00:14.0-usb-0:1.1:1.0-port0 -> ../../ttyUSB0   # 教える側のアダプタ
... pci-0000:00:14.0-usb-0:1.2:1.0-port0 -> ../../ttyUSB1   # まねる側 1 のアダプタ
... pci-0000:00:14.0-usb-0:1.3:1.0-port0 -> ../../ttyUSB2   # まねる側 2 のアダプタ
```

アダプタを別の口に挿し替えると名前が変わるので，そのときはもう一度確かめる．

## 起動する

Jedy ごとに端末を 1 つ開き，**別の `namespace:=`** と，その Jedy のアダプタ (`port:=`) を付けて起動する．
同じ `namespace:=` で 2 つ起動しようとすると，「すでに起動しています」と出て止まる．

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py namespace:=jedy_leader \
    port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1:1.0-port0 rviz:=false joy:=false wheels:=false
```

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py namespace:=jedy_follower1 \
    port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.2:1.0-port0 rviz:=false joy:=false wheels:=false
```

```{code-block} console
$ ros2 launch jedy_bringup jedy_fullbody_all.launch.py namespace:=jedy_follower2 \
    port:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.3:1.0-port0 rviz:=false joy:=false wheels:=false
```

- **1 台ずつ，10 秒ほどあけて起動する．** コントローラを起動するプログラム (spawner) は PC 全体で 1 つのロックを取り合うので，同時に起動すると「Failed to acquire lock」と出て待たされ，台数が多いと失敗することがある．
- `namespace:=` を付けたロボットで確かめたのは頭・腕・グリッパだけで，RViz・ジョイスティック・台車は確かめていない．教示には要らないので `rviz:=false joy:=false wheels:=false` にする．

## 教示する

もう 1 つ端末を開き，まねる側をカンマで区切って並べる．

```{code-block} console
$ ros2 run jedy_skrobot teach_follow --leader /jedy_leader \
    --follower /jedy_follower1,/jedy_follower2 --record poses.yaml
```

1. まねる側全員の頭・両腕・両グリッパのトルクが ON になり，**5 秒かけて教える側と同じ姿勢まで動く** (`--approach` で秒数を変えられる)．まねる側の周りに物や人がいないことを確かめてから実行する．
2. そのあとは，教える側を手で動かすと，まねる側が同じように動く．教える側の姿勢を 1 秒に `--rate` 回 (既定 25 回) 送り，まねる側は前に送られた姿勢から次の姿勢まで一定の速さで動くので，少し遅れてついてくる．
3. **Enter** を押すと，そのときの教える側の姿勢が `poses.yaml` に追記される．**q** で終わる．

- 教える側のトルクは起動したときから OFF で，`teach_follow` は ON にしない．腕は重力で下がるので，手で支えながら動かす．
- 動かす部位は `--parts` で選べる (`head`, `rarm`, `larm`, `rhand`, `lhand` をカンマ区切り．例: `--parts rarm,rhand`)．最初は片腕から試すとよい．
- 終わってもまねる側はトルク ON のまま最後の姿勢を保つ．脱力させるときは launch を Ctrl-C で止めるか，`ri.servo_off()` を使う．
- 教える側の関節が可動範囲の外にあるとき (グリッパを強く握り込んだときなど)，まねる側には範囲の端の角度を送る．
- グリッパは機体ごとに閉じきる角度が少し違う．教える側が閉じきった角度にまねる側が届かないときは，まねる側は {ref}`jedy-gripper-hold-relax` の弱い握り直しになる．

### うまくいかないとき

- **`torque refused` と出て止まる**：まねる側のどこかの関節が可動範囲から大きく外れている (手で回しておいたなど)．
  `jedy_fullbody_all.launch.py` は，範囲の外 20° (`max_start_jump:=0.35` [rad]) までならトルクを入れて範囲の端まで戻す．それより外れていたら，その関節を手で範囲の中に戻してから実行し直す．
- **`the leader stopped answering` と出る**：教える側の基板が応答しなくなった．まねる側は今の姿勢で止まって待つ．基板の電源とケーブルを確かめる．
- **`follower ... stopped answering` と出て終わる**：まねる側の基板が応答しなくなった (トルクは OFF になる)．基板の電源とケーブルを確かめてから，実行し直す．

## 記録した姿勢を再生する

```{code-block} console
$ ros2 run jedy_skrobot teach_follow --play poses.yaml --follower /jedy_follower1,/jedy_follower2 --time 1.5
```

`poses.yaml` の姿勢を順に，1 つあたり `--time` 秒かけて，並べたロボット全員で再生する．
1 台だけ (`namespace:=` なし) で起動したロボットで再生するときは `--follower /jedy_fullbody` にする．
`poses.yaml` は関節名と角度 [deg] の YAML なので，自分のプログラムから読んで `ri.angle_vector()` に渡すこともできる．

## 何台までつなげるか

確かめたのは，1 台の PC に Jedy 3 台 (教える側 1 台，まねる側 2 台) までである．
それより多いときに起こりうることを書いておく．

- 1 つの `ROS_DOMAIN_ID` に ROS のプロセスがとても多いと，あとから起動したロボットのデータがほかのプロセスに届かなくなることがあった (Jedy の基板 11 台を 1 台の PC で動かしたとき)．
  原因は確かめていないが，ROS の通信 (DDS) の設定で直せる見込みがある．
- PC の負荷が高いと，サーボを読む周期 (30 Hz) を保てなくなる．
