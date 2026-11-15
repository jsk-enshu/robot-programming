(ta-remote-debugging)=
# TA 向け：学生の PC で実機の問題を一緒に調べる

学生が実機やプログラムで詰まったときに，TA が AI と一緒に原因を調べるやり方をまとめる．
学生の PC に SSH で入って調べる方法と，実機が思ったとおりに動かないときに見る順番を書く．
実際に AI と調べた例は {doc}`ai-robot-programming` にある．

## まず学生に頼むこと

遠隔で入る前に，学生自身にできることがある．多くの場合はこれで足りる．

- `ta fix`：直前のエラー (失敗したコマンド，launch のログ，Traceback など) を集めて AI TA に直し方を聞く ({doc}`ai-ta`)
- `ta call`：エラーの情報と一緒に TA を呼ぶ
- **画面やカメラの画像を AI TA に見せる**：AI TA は画像も読める．スクリーンショットを会話に貼るか，
  `ta 'ここ (~/enshu/camera.png) を見て，何が写っているか教えて'` のようにファイルの場所を書いて頼む．
  (頭のカメラの画像で試したところ，どちらのやり方でも，床のタグ 3 枚・ルービックキューブ 2 つ・画面の下のグリッパを正しく答えた．
  ただし 640×480 の画像の中の小さな文字は読めなかった．)
- 何をしたくて，何を試して，何が起きたかを AI TA に文章で説明する．説明するだけで自分で気づくことも多い ({ref}`confessional-debugging`)

## SSH で学生の PC に入る

学生の画面を横から見ながらでは時間がかかるとき，TA の PC から SSH で入って調べる．
**学生の同意を得て，学生がそばにいるときだけ行う**．終わったら必ず入れないように戻す．

### 学生の PC で (学生に打ってもらう)

1. SSH のサーバを入れる (1 回だけ)．

   ```{code-block} console
   $ sudo apt install -y openssh-server
   ```

2. TA の公開鍵を，**あとで消せる印を付けて**足す．`<TA>` は TA の GitHub アカウント名．
   GitHub に登録してある公開鍵が `https://github.com/<TA>.keys` で取れる．

   ```{code-block} console
   $ mkdir -p ~/.ssh && chmod 700 ~/.ssh
   $ wget https://github.com/<TA>.keys -O- -q | sed 's/$/ enshu-ta-temp/' >> ~/.ssh/authorized_keys
   $ chmod 600 ~/.ssh/authorized_keys
   ```

   行の末尾の ` enshu-ta-temp` は鍵のコメント欄に入る印で，鍵の働きには影響しない．後で消すときにこの印を使う．

3. PC の IP アドレスを調べて TA に伝える．

   ```{code-block} console
   $ hostname -I
   ```

### TA の PC から

TA の PC を学生の PC と同じネットワーク (同じ部屋の有線 LAN や Wi-Fi) につなぎ，入る．

```{code-block} console
$ ssh mech-user@<学生の PC の IP アドレス>
```

つながらないときは，IP アドレスが合っているか，同じネットワークにいるか，学生の PC で SSH のサーバが動いているか
(`systemctl status ssh`) を確かめる．
ネットワークによっては端末どうしの通信が許されていないことがあり，その場合は別のネットワークにつなぐ．

### 終わったら (必ず)

学生の PC で，足した鍵を消す．印の付いた行だけが消え，学生がもともと持っていた鍵は残る．

```{code-block} console
$ sed -i '/ enshu-ta-temp$/d' ~/.ssh/authorized_keys
```

`grep enshu-ta-temp ~/.ssh/authorized_keys` で何も出なければ消えている．
SSH のサーバも要らなければ止める (`sudo systemctl disable --now ssh`)．

```{warning}
- 学科 PC の `mech-user` は共有のアカウントである．鍵を消し忘れると，その TA は演習が終わったあとも入れてしまう．
- 入ると学生のファイルがすべて見える．AI TA のキー (`~/.config/ai-ta/env`) など，キーやパスワードの入ったファイルは開かない．AI に調べさせるときも，表示しないように伝える．
- 実機を動かすコマンドを打つ前に，学生に声をかけ，周りに人や物がないことを確かめてもらう．
```

## AI と一緒に調べる

SSH で入れたら，TA が使っている AI (Claude Code などのエージェント) に，学生の PC で調べてもらうこともできる．
たとえば次のように頼む．

```{code-block} text
ssh mech-user@<IP アドレス> で学生の PC に入れる．
ros2 launch jedy_bringup jedy_fullbody_all.launch.py camera:=true で起動すると腕が動かない．
ログと Diagnostics を見て原因を調べて．
ロボットを動かすコマンドを実行する前には必ず聞いて．キーやパスワードは表示しないで．
```

任せるときの注意は {doc}`ai-robot-programming` の「B. AI エージェントに実機を任せる」にまとめてある
(動かす前に説明させる，見張りを入れる，証拠で報告させる，PC を重くしすぎない，故障を疑うのは人)．

## 実機が思ったとおりに動かないときに見る順番

上から順に見ると，どこで止まっているかを絞り込める．

### 1. 起動したか

- RCB-4mini の LED が電源を入れたときに 3 回点滅するか ({doc}`robot-operation` の「電源の入れ方」)
- launch のログを上にさかのぼって `ABORT:` で始まる行を探す．エラーが出ても launch は終わらず，
  `waiting for service ...` が流れ続けるので見落としやすい (直し方は {ref}`jedy-real-run` の「起動できないとき」)
- どのサーボが応答するかを ID ごとに調べる．launch を止めてから実行する (サーボのトルクは入らないので，ロボットは動かない)．

  ```{code-block} console
  $ PYTHONNOUSERSITE=1 $(ros2 pkg prefix actuo_ros)/share/actuo_ros/venv/bin/python \
      ~/ros2_ws/src/actuo/actuo/cli/kondo_rcb4_probe.py --channel /dev/actuo_kondo_rcb4mini --scan
  ```

  `ICS assigned : [0, 1, 2, ...]` に応答した番号が並ぶ．設定にある番号 (頭は 32・34，腕とグリッパは 0〜15，車輪は 20〜23) が抜けていれば，そのサーボのケーブルか電源を疑う．

### 2. カメラ

- launch のログに `No RealSense devices were found!` や `Frames didn't arrived within 5 seconds` が続く → 頭のカメラ (D405) の USB が抜けている．首まわりのケーブルを触ったあとに抜けやすい

### 3. 見えているか

- {ref}`show-3d` で，RViz に点群・認識した物の箱・タグを重ねて，点群の中の物と箱が重なっているかを見る
- 位置が怪しいときは，深度を使わない方法 (画像の画素を通る光線と床の交点，`jedy_vision.pixel_on_floor`) とも比べる

### 4. 関節が思ったとおりか

- トルク OFF のまま腕や首を手で動かし，RViz のモデルが同じ姿勢になるかを見る ({ref}`jedy-real-rviz-check`)．違えば組み立ての間違い ({doc}`jedy-assembly`)
- プログラムが計算した姿勢 (逆運動学の解) を PyrenderViewer で描いて，腕が体や床に当たっていないかを見る
- 送った角度と測った角度を関節ごとに比べる．1 つの関節だけが大きくずれていれば，その関節が何かに当たっているか，サーボが力を出していない

  ```{code-block} python
  import numpy as np
  ri.angle_vector(av, 3.0)
  ri.wait_interpolation()
  for j, sent, got in zip(robot.joint_list, av, ri.angle_vector()):
      if abs(np.rad2deg(got - sent)) > 5:
          print(j.name, 'sent', np.rad2deg(sent), 'measured', np.rad2deg(got))
  ```

### 5. PC が重すぎないか

- RViz の Diagnostics に `loop is not keeping up` が出る，`uptime` のロードアベレージが CPU の数を大きく超える
  → サーボを動かすプロセスが周期 (30 Hz) に間に合っていない．
  認識・RViz・録画などを同時にたくさん動かすと，このプロセスが止まり，サーボのトルクが切れて腕が落ちたことがある．
  要らないプログラムを止める

### 6. 人が見る・触る

AI にもプログラムにもできないことである．

- ケーブルの抜け，コネクタの向き，USB アダプタの挿し直し
- サーボの不調 (手で動かしたときの引っかかり，異音，熱)
- ロボットが倒れそうでないか，周りに人や物がないか
