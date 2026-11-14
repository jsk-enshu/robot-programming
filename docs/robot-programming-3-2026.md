(day-mechatronics)=
# メカトロボットの製作とプログラミング

:::{note}
**Coming soon**

2026年度の資料は準備中である．公開までは{doc}`2025年度版の資料<robot-programming-3-2025>`を参照してほしい．
下の「本日の発展課題」は2026年度版として先に公開している．
:::

## 本日の発展課題

発展課題は必須ではない．
今年度は，Arduino UNO R4 WiFi と近藤科学の KRS サーボで組む四脚ロボット JEFY を題材に，
組み立て → 強化学習 → 自分の方策で実機を歩かせる → IMU を付ける → CAD から URDF を作る → 自分のロボットを作る，
と順に進められるようにした．

**JEFY**（ジェフィー）は **J**SK **E**ducational **F**our-legged walk**Y** robot の略で，[四脚ロボット JEFY の組み立て](https://iory.github.io/build-your-own-arduino-rtos/hardware/index.html)にまとめられている四脚ロボットの呼び名である．

課題1から順にやると，ロボットを一から作って歩かせるところまでの流れをひととおり体験できる．
課題2・課題5・課題6は実機がなくても取り組める．

<video width="100%" controls preload="metadata" poster="fig/krs-quad-walk-2026-poster.jpg">
  <source src="fig/krs-quad-walk-2026.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** KRS-3302 ICS ×8 と RCB-4mini で組んだ四脚ロボット．Ni-MH バッテリー（ROBO パワーセル E タイプ，7.2 V）1 本だけで，ケーブルをつながずに歩いている．

### 課題1(発展): 四脚ロボット JEFY を組み立てて歩かせる

動画の四脚ロボット JEFY を自分で組み立て，Arduino UNO R4 WiFi から歩かせてみよう．
希望者は TA から部品一式を受け取ること．
手順はすべて次のページにまとめてある．

- 組み立て・配線・書き込み・校正: [近藤科学の KRS サーボで組む（RCB-4mini 版）](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html)
- 部品表: [部品表（BOM）— KRS 版 8 自由度四脚ロボット](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs_bom.html)

**TA から受け取るもの：**

- KRS-3302 ICS ×8，ホーン，ネジ
- 近藤科学 RCB-4mini（サーボをまとめる基板）
- 胴体・脚・蓋の 3D プリント部品
- Dual USB アダプター HS（サーボの ID を振るときに使う）
- 電源（ROBO パワーセル E タイプ と充電器，または近藤科学の AC アダプター）

Arduino UNO R4 WiFi は TA からは配らない．各自が持っているものを使うこと．

以下の手順では `uv` を使う．入っていなければ {doc}`environment-setup` の「初回環境構築」にある `curl -LsSf https://astral.sh/uv/install.sh | sh` で入れておくこと．

<div class="screen">

1.  `ics-manager`（`uv tool install rcb4` で入る）で8個のサーボに ID を振り，設定を確かめる．

2.  サーボを真ん中に合わせてからホーンと脚を付け，ページの 3D ビューアを見ながら胴体に組み付ける．

3.  サーボを RCB-4mini の SIO ポートに，RCB-4mini の COM ポートを Arduino の **D8 / D9** につなぐ（D0 / D1 は空けておく）．

4.  コードを取ってきて書き込み，ゼロ点と回る向きを測って `include/quad_calib_rcb4.h` に書く．

    ```bash
    git clone https://github.com/iory/build-your-own-arduino-rtos
    cd build-your-own-arduino-rtos/code
    uv run pio run -d 13_quadruped -e uno_r4_wifi_rcb4 -t upload
    ```

5.  `QUAD_RCB4_CALIBRATED` を `1` にしてもう一度書き込み，吊るしたまま `rl 0.06 0` を送って足踏みを確かめてから床で歩かせる．
    この env の方策（`include/arduino_quad_policy.h`）は IMU を使わない．

</div>

:::{warning}
校正が終わったファームウェアは，起動するとすぐに立ち姿勢へ動く．
最初は必ず機体を吊るした状態で動かし，足踏みすることを確かめてから床に下ろすこと．
歩いている最中に Arduino がリセットするときは，バッテリーの電圧（6 V を切っていないか）を確かめる．
:::

### 課題2(発展): 強化学習で歩行方策を学習する

課題1で使った方策は，物理シミュレータ MuJoCo の中で強化学習（PPO）によって学習したものである．
[歩行を学習させる（強化学習）](https://iory.github.io/build-your-own-arduino-rtos/hardware/walk.html)を読んで，
自分で歩行方策を学習してみよう．実機がなくても取り組める．

<div class="screen">

1.  まずページ内のブラウザのシミュレーションで，sin/cos で作った歩行と学習済みの方策を比べてみる．

2.  観測・行動・報酬が何か（ページの「そもそも強化学習とは」「方策の中身」）を確かめる．

3.  [歩行方策を自分で学習する](https://iory.github.io/build-your-own-arduino-rtos/hardware/train.html)の手順で `ArduinoQuad-Walk` を学習する．
    学習の入口は次の3つで，どれで学習してもチェックポイントの形式は同じである．
    - NVIDIA GPU がある PC: mjlab 版
    - GPU がない場合: Google Colab（無料枠の T4）のノートブック
    - GPU がない場合: CPU 版（`code/13_quadruped/rl/cpu` で `uv sync` のあと `uv run train.py`）

4.  学習した方策をシミュレーションで歩かせ，動画（`record_video.py` や `evaluate.py --video`）に書き出す．

</div>

余裕があれば，報酬の重みや速度指令の上限を変えて学習し直し，歩き方がどう変わるかを比べてみよう．

### 課題3(発展): 自分で学習した方策で実機を歩かせる

（課題1・課題2ができてから挑戦することをお勧めする．）
課題2で学習した方策を Arduino に書き込み，課題1で組み立てた JEFY を歩かせてみよう．

<div class="screen">

1.  `host/export_quad_policy.py` でチェックポイントから Arduino 用のヘッダを書き出す（[歩行方策を自分で学習する](https://iory.github.io/build-your-own-arduino-rtos/hardware/train.html)の「実機に持っていく」）．

2.  書き出した `arduino_quad_policy.h` で `code/13_quadruped/include/arduino_quad_policy.h` を置き換える．

3.  課題1と同じ `-e uno_r4_wifi_rcb4` で書き込み，**吊るした状態で**足踏みを確かめてから床で歩かせる．

</div>

配布の方策と自分の方策で，同じ速度指令を与えたときの歩き方（速さ・まっすぐ進むか・押されたときに倒れないか）を比べてみよう．

`ArduinoQuad-Walk` の機体モデルは Feetech STS3215 のサーボを前提にしていて，KRS-3302 のトルクはその4分の1以下である．
シミュレーションと実機で歩き方がどう違ったかを観察し，その差がどこから来るのかを考えてみよう．

### 課題4(発展): IMU を付けて KRS 専用の方策で歩かせる

（課題1または課題3で JEFY が歩いた人向け．）
JEFY が歩いたら TA に申し出て，IMU 基板（QMI8658）と Qwiic ケーブルを受け取ること．
IMU（慣性計測装置）で胴体の傾きと回転の速さを測り，その値も見る方策で歩かせてみよう．

これまでの方策は関節角と関節速度しか見ていないので，胴体が傾いても方策にはそれが直接は伝わらない．
近藤科学の KRS-3302 に合わせて機体のモデル・関節の範囲・質量を作り直し，IMU の値も観測に入れて学習した KRS 専用の方策がある（[KRS 版の歩行方策](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs.html)）．

<div class="screen">

1.  [IMU で姿勢を見て踏ん張らせる](https://iory.github.io/build-your-own-arduino-rtos/hardware/imu.html)を読み，IMU 基板を胴体に取り付けて Arduino の Qwiic コネクタにつなぐ．

2.  ファームウェアは IMU を決まった向きに付けた前提で，チップの軸を胴体の軸（x 前・y 左・z 上）へ回している（`src/quad_imu.cpp` の `R_BODY_CHIP`）．
    JEFY の胴体に付けた向きに合わせてここを直し，ページの「向きを確かめる」で合っているか確かめる．

3.  KRS 専用の方策（`include/krs_quad_policy.h`）で書き込み，吊るして足踏みを確かめてから床で歩かせる．

    ```bash
    uv run pio run -d 13_quadruped -e uno_r4_wifi_rcb4_krs -t upload
    ```

</div>

IMU なしの方策と比べて，歩いている途中や止まっているときに胴体を横から軽く押すと，どう違うかを確かめてみよう．

さらに余裕があれば，KRS 専用の方策を自分で学習し直してみよう．
タスクは `KrsQuad-Walk` で，mjlab 版で学習するため NVIDIA GPU が必要である．
手元に GPU がなければ，課題2の Google Colab（無料枠の T4）で学習してみるとよい．
Colab のノートブックは `ArduinoQuad-Walk` 用なので，学習と書き出しのセルのタスク名を `KrsQuad-Walk` に変え，書き出しには `--name krs_quad_policy` を付ける．
手順は [code/13_quadruped/rl/README.md](https://github.com/iory/build-your-own-arduino-rtos/blob/main/code/13_quadruped/rl/README.md) の「KRS 版（KrsQuad-*）」にある．

<div class="screen">

1.  `code/13_quadruped` で学習環境を用意し（README の「KRS 版」にある機体モデルの作成も先に行う），`KrsQuad-Walk` を学習する．

    ```bash
    ./rl/scripts/setup.sh
    source rl/scripts/env.sh
    ./rl/scripts/train.sh KrsQuad-Walk --env.scene.num-envs=4096 --agent.max-iterations 3000
    ```

2.  `rl/scripts/krs_eval.py` で，速度指令への追従・胴体の傾き・関節とトルクの余裕を数値で確かめる．

3.  `host/export_quad_policy.py` に `--name krs_quad_policy` を付けて Arduino 用のヘッダを書き出し，`include/krs_quad_policy.h` を置き換えて，上と同じ `-e uno_r4_wifi_rcb4_krs` で書き込む．

</div>

### 課題5(発展): SolidWorks で組んだ四脚ロボットから URDF を書き出す

[sw2robot（solidworks_urdf_exporter2）](https://github.com/jsk-ros-pkg/solidworks_urdf_exporter2)は，SolidWorks のアセンブリの合致（メイト）からリンクの木構造と関節軸を読み取り，URDF を書き出すツールである．
[sw2robot の日本語ドキュメント](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/ja/)の
[チュートリアル: 4脚ロボットをゼロから作る](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/ja/tutorial/)に沿って，
四脚ロボットを SolidWorks でアセンブリし，URDF を書き出してみよう．

SolidWorks は Windows でしか動かないので，この課題は Windows に切り替えて行うこと．

<div class="screen">

1.  PowerShell で sw2robot のエディタを入れる．

    ```powershell
    powershell -ExecutionPolicy ByPass -c "irm https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/install.ps1 | iex"
    ```

2.  SolidWorks で body・thigh・shin の3つの部品を作り，材料を設定する（材料から URDF の質量・慣性が決まる）．

3.  アセンブリで「同心円 + 一致」の合致を使って股関節と膝を組む．脚をドラッグして，軸まわりにだけ回ることを確かめる．

4.  sw2robot でアセンブリを抽出し，ブラウザのエディタ（`sw2robot-web`）で関節の名前・向き・可動範囲を整えて URDF を書き出す．

5.  書き出した URDF を RViz などで表示し，関節を動かして確かめる．

</div>

抽出（`.SLDASM` の読み込み）には SolidWorks 2024 以降が必要である．

余裕があれば，実際のサーボを SolidWorks でアセンブリしてみよう．
KRS-3302 を含む KRS-3300 シリーズは，近藤科学がフレーム設計用の 3D データ（STEP）を公開している
（[KRS-3300シリーズ 寸法図](https://kondo-robot.com/faq/krs-3300dimention)の
[KRS3300_scale_step.zip](https://kondo-robot.com/kpc/wp-content/uploads/KRS3300_scale_step.zip)．
商品ページ: [KRS-3304R2 ICS](https://kondo-robot.com/product/krs-3304r2-ics)）．
STEP のサーボを部品として取り込み，サーボの出力軸とフレームの間に合致を付けて組むと，寸法や関節の位置が実機の JEFY に近い URDF ができる．
JEFY の 3D プリント部品の STL は[部品表](https://iory.github.io/build-your-own-arduino-rtos/hardware/krs_bom.html)の「3D プリント部品」にある．

さらに余裕があれば，[MuJoCo で歩かせる](https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/ja/walk/)に沿って，
自分で書き出したロボットを課題2の CPU 版の学習コードで歩かせてみよう．

### 課題6(発展): 自分のオリジナルロボットの URDF を作る

課題5の流れを使って，自分で考えたオリジナルのロボットを SolidWorks で設計し，sw2robot で URDF を作ってみよう．
脚の数・関節の数・形は自由である（アーム，車輪ロボット，脚の数が違う歩行ロボットなど）．

- 実際に使うサーボがあれば，課題5と同じようにメーカーの STEP を取り込んでアセンブリする．
- 部品ごとに材料を設定し，合致で関節を作る．
- エディタで関節の種類（動く関節か固定か）・可動範囲・名前を整える．
- 書き出した URDF を RViz で表示し，各関節を動かしたスクリーンショットを撮って URDF と一緒に提出すること．

さらに余裕があれば，作ったロボットを Gazebo や MuJoCo に読み込んで動かしてみよう．
