(day-whole-body)=
# ロボットの全身行動プログラミング

## 本日の演習内容

今日は {doc}`robot-programming-1-2026` で使えるようになった道具 (台車・腕と首・カメラ・認識・声) を組み合わせて，
**自分がロボットにさせたい動き**を作る．何を作るかは自由である．

<div class="screen">

1. 作るものを決めて計画を書く (〜30 分)
2. シミュレータ (Gazebo) で作る
3. 実機で動かして直す
4. 発表する

</div>

```{important}
各チェックポイントを達成しながら進捗報告シートへの記入を進めること．各チェックポイントの結果をスクリーンショット等で保存しておくこと．
```

## 課題の条件

- 台車・腕と首・カメラ (画像・認識)・声のうち **2 つ以上**を使い，ロボットが**周りの様子 (人・物・声) に応じて**動くこと．決まった動きを再生するだけのものは不可．
- まず Gazebo で動かし，そのあと実機で動かすこと (実機が空いていないときは Gazebo だけでもよい．その場合は発表で理由を言う)．
- 一人で作っても，2〜3 人で 1 つを作ってもよい．複数人の場合は誰が何を作ったかを発表で言うこと．

## 作品の例

迷ったら例から始めて，自分なりに変えるとよい．

| 例 | 使う道具 | 使うメソッド |
| --- | --- | --- |
| じゃんけんロボット: 「じゃんけんぽん」と言って手を読み，勝ち負けを言って腕で喜ぶ・悲しむ | 声・認識・腕 | `ri.say`, `ri.wait_until_janken`, `ri.angle_vector` |
| 物を探すロボット: 「ペットボトルを探して」と言われたら旋回して探し，見つけたら近づいて指さす | 声・認識・台車・腕 | `ri.listen`, `ri.what_do_you_see`, `ri.go_velocity`, `inverse_kinematics` |
| 声で操縦するロボット: 「前」「右」「止まって」で台車を動かし，「手を振って」で腕を振る | 声・台車・腕 | `ri.listen`, `ri.go_velocity` |
| 案内ロボット: 人が来たらあいさつし，「いいね」をされたらお辞儀する | 認識・声・首 | `ri.wait_until_sees('人')`, `ri.gesture`, `ri.say` |
| 人について行くロボット | カメラ・台車・首 | `ri.what_do_you_see(['人'])`, `ri.go_velocity` |

### すでにあるデモ

1 日目に扱わなかったことも，動くデモがすでにある．作りたいものに近いデモを動かして，中身を読むとよい．
`ros2 run jedy_vision demos` で一覧と起動方法が出る (中身は `jedy/jedy_vision/scripts/` と `jedy/jedy_skrobot/scripts/enshu/`)．
Gazebo では `jedy_gazebo.launch.py demo:=true person:=true camera:=d405` がこれらのデモの舞台になる．

| したいこと | 起動するもの | 中身 (1 日目に出てこない話題) |
| --- | --- | --- |
| 物を探して近づき，指さす | `ros2 run jedy_vision find_object cup` | 見回して探す，床との交点で距離を推定する |
| 手で物に触る | `ros2 run jedy_vision touch_tag` | カメラ座標からロボット座標へ，逆運動学，手前で一度止まる経由点 |
| 物をつかんで持ち上げる | `ros2 run jedy_vision grasp_tag` (遠くからでも台車で寄る) | 複数のタグから物の姿勢を求める，姿勢付きの逆運動学，つかめたかの確認 |
| 首で人を追う | `ros2 run jedy_vision look_at_person` | 首のコントローラだけに指令を送る |
| 人について行く | `ros2 run jedy_vision follow_person` | 見え方で台車を動かすフィードバック |
| 人の腕のまねをする | `ros2 run jedy_vision mimic_arms` | 姿勢推定の結果から逆運動学で腕の姿勢を作る |
| 手を挙げた人に手を振り返す | `ros2 run jedy_vision wave_back` | 姿勢から合図を判定し，動作を作る |
| 声でデモを切り替える | `ros2 run jedy_vision voice_demo` | 音声認識の結果で行動を選ぶ |
| 色の付いた物の 3 次元の位置 | `ros2 run jedy_skrobot color_clusters` | 色で画素を選び，深度画像で 3 次元に戻す |
| チェッカーボードの位置と向き | `ros2 run jedy_skrobot checkerboard_pose` | 既知の模様から姿勢を求める (PnP) |
| 傾いたら止まる | `ros2 run jedy_skrobot checkpoint_go_forward_imu` | IMU (Gazebo のみ．実機の RCB-4mini には無い) |

<video width="100%" controls preload="metadata">
  <source src="fig/jedy-gazebo-grasp-tag.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** Gazebo のデモの舞台で `ros2 run jedy_vision approach_tag` のあと `ros2 run jedy_vision grasp_tag` を実行したところ．
左が横から見た Gazebo，右上の小窓がロボットの頭のカメラの画像，右がターミナル．
台の AprilTag の正面に止まり，立方体の各面のタグから立方体の位置と向きを求めて，片手でつかんで持ち上げてから元に戻す．

## 計画を書く

作り始める前に，次を書いて TA に見せる．

<div class="screen">

- **何をするロボットか** (1〜2 文)
- **流れ**: 「〜が見えたら / 聞こえたら → 〜する」を順に並べる
- **使う道具とメソッド**
- **うまくいかなそうな所** (例: 遠いと認識できない，台車が止まりきれない) と，そのときどうするか

</div>

```{exercise} 計画
:label: ex_plan_2026

上の 4 つを書いて TA に見せ，進捗報告シートに記入せよ．
```

## 作る

### プログラムの置き場所と実行

プログラムは自分のディレクトリ (例: `~/enshu/`) に置き，次で実行する．

```{code-block} console
$ $(ros2 pkg prefix jedy_skrobot)/share/jedy_skrobot/venv/bin/python ~/enshu/my_robot.py
```

ロボット (Gazebo / 実機) に加えて，使う道具のノードを別のターミナルで起動しておく．

| 使う道具 | 起動するもの |
| --- | --- |
| カメラの認識 | `ros2 launch jedy_vision recognition.launch.py` |
| しゃべる | `ros2 launch voicevox voicevox_texttospeech.launch.xml` |
| 聞く | `ros2 launch ros_speech_recognition speech_recognition.launch.xml language:=ja-JP launch_sound_play:=false energy_threshold:=300.0 dynamic_energy_threshold:=false tts_action_names:="[robotsound_jp]"` |

### プログラムの形

```{code-block} python
from jedy_skrobot import connect
import numpy as np

robot, ri = connect()
ri.servo_on()                      # 実機はサーボ OFF で起動するので最初に ON

try:
    ri.say('はじめます')
    while True:
        text = ri.listen(timeout=10.0)
        if text is None:
            continue
        if 'おわり' in text:
            break
        # ここに「聞こえた / 見えた → 動く」を書く
finally:
    ri.stop()                      # Ctrl-C で止めても台車を止める
    ri.say('おわります')
```

```{tip}
- **小さく作って動かす**: 全部書いてから動かすのではなく，1 つの動作 (例: 手を読むだけ) ごとに動かして確かめる．
- **関数に分ける**: `def cheer():` (喜ぶ動き)，`def find(word):` (探す) のように動作を関数にすると，組み合わせや直しが楽になる．
- **待つ**: 腕を動かしたら `ri.wait_interpolation()`，しゃべるなら `ri.say()` (既定でしゃべり終わるまで待つ)．待たずに次の指令を送ると前の動きが途中で変わる．
- **見えない・聞こえないときのことを書く**: `ri.listen()` は聞こえなければ `None`，`ri.janken()` は手が無ければ `None` を返す．
```

### <span style="color:green">チェックポイント: シミュレータ</span>

```{exercise} シミュレータで動かす
:label: ex_sim_2026

作ったプログラムを Gazebo で最後まで動かし，その様子 (画面) を保存せよ．
Gazebo に無いもの (人・ペットボトル・手) は，PC の Web カメラやマイクで代わりに試してよい (認識のノードを `image_topic:=/image_raw` で起動する)．
```

## 実機で動かす

### 実機に出す前のチェック

<div class="screen">

- Gazebo で最後まで動いたか
- **台車**: 速さは前後・左右 0.078〜0.156 m/s，旋回 0.83〜1.66 rad/s の範囲か (遅すぎると動かない)．走る前に `ri.tuck_arms()` で腕をたたんだか．周りに人や机が無いか
- **腕**: 自分の体や台車にぶつかる姿勢を送っていないか (Gazebo やモデルで確かめる)．速すぎないか (`ri.angle_vector(av, 時間)` の時間を短くしすぎない)
- **止め方**: Ctrl-C で止まるか．止まらないときに背面のスイッチでサーボの電源を切れる人がそばにいるか

</div>

```{warning}
実機を動かすときは**必ずもう 1 人がそばにいて**，危ないときにすぐ電源を切れるようにすること．
腕が何かに当たって止まったまま力を出し続けるとサーボが焼ける．
```

### 実機とシミュレータの違い

- 実機は**サーボ OFF で起動する** (プログラムの最初で `ri.servo_on()`)．
- 台車は遅い指令では動かず，止まるまでに少し進む．床で滑るので，指令どおりの距離は進まない．
- 頭のカメラ (D405) の深度は 0.07〜0.5 m くらいしか測れない．
- 照明や背景で認識の結果が変わる．`/object_detection/image` で何が見えているかを確かめる．
- 応答しないサーボがあると `connect()` でエラーになる．ケーブルと電源を確認する (抜き差しは TA を呼んでから)．
- 腕に大きな力がかかったときなどに，ロボットのボードが止まることがある．そのときは端末に「ロボットのボード (RCB-4mini) が応答していません」と出て，`ri.angle_vector()` などがエラーになる (止まる前の角度を返し続けることはない)．サーボはすべて OFF になるので腕を支え，ロボットの電源を入れ直す (分からなければ TA を呼ぶ)．「ボードが応答を再開しました」と出たら，`ri.servo_on()` で ON にしてから続ける．

### <span style="color:green">チェックポイント: 実機</span>

```{exercise} 実機で動かす
:label: ex_real_2026

作ったプログラムを実機で動かし，その様子を動画で保存せよ．
シミュレータと違ったところと，どう直したかを書け．
```

## 発表と提出

最後に 1 人 (1 グループ) 3 分程度で発表する．

<div class="screen">

- 何をするロボットか (動画を見せる，または実演)
- 工夫したところ・うまくいかなかったところ
- シミュレータと実機で違ったところ

</div>

```{exercise} 本日の課題
:label: ex_day2_2026

計画・プログラム・Gazebo と実機の様子 (スクリーンショット・動画へのリンク)・発表の内容を pdf にまとめて提出せよ．
プログラムは全文を載せること．
```

## 付録: `ri` の早見表

| やりたいこと | メソッド |
| --- | --- |
| 関節角度を読む | `ri.angle_vector()` [rad] |
| 姿勢を送る / 到着を待つ | `ri.angle_vector(av, 秒)` / `ri.wait_interpolation()` |
| 一部の関節だけ送る | `ri.angle_vector(av, 秒, controller_type='head_controller')` |
| サーボ ON / OFF | `ri.servo_on()` / `ri.servo_off()` (`joints=[...]` で一部) |
| 台車 | `ri.go_velocity(前後, 左右, 旋回, duration=秒)` / `ri.stop()` / `ri.tuck_arms()` |
| 時間 | `ri.now()` / `ri.sleep(秒)` (シミュレータの時計にも合う) |
| 画像 / 深度 / 内部パラメータ | `ri.image()` / `ri.depth_image()` / `ri.camera_info()` |
| 物体を探す | `ri.sees('単語')` / `ri.what_do_you_see(['単語', ...])` / `ri.wait_until_sees('単語', timeout=秒)` |
| ジェスチャ | `ri.gesture()` / `ri.janken()` / `ri.wait_until_gesture('チョキ')` / `ri.wait_until_janken()` |
| しゃべる / 聞く | `ri.say('文')` (`wait=False`, `speaker='ずんだもん-ノーマル'`) / `ri.listen(timeout=秒)` |
| モデル: 初期姿勢 / 関節 | `robot.reset_pose()` / `robot.head_joint1.joint_angle(rad)` |
| モデル: 手先 / 逆運動学 | `robot.rarm.end_coords.copy_worldcoords()` / `robot.rarm.inverse_kinematics(target, rotation_axis=False)` |

使える関節名は `[j.name for j in robot.joint_list]`，コントローラは `ri.controller_names`．
