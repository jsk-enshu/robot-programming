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
| 人について行くロボット | カメラ・台車・首 | `ri.what_do_you_see(['人'])`, `ri.go_velocity` (人までの距離は下の注意) |
| 実況するロボット: 「何が見える?」と聞かれたら，見えているものを AI に聞いてしゃべる | 声・カメラ (AI) | `ri.listen`, `ri.ask`, `ri.say` |

:::{note}
実機の D405 は 0.5 m くらいより遠くの深度を測れない．そのため，それより遠い人や物には `ri.detection_position()` が None を返す．
`follow_person` (`jedy/jedy_vision/scripts/follow_person.py`) は，深度が無いとき，人の枠の下端 (足元) の画素と床の交点から距離を求める．
足元が画像の外なら，枠の上端を床から 1.6 m (頭) とみなして求める．
自分のプログラムでも，次のように足元の床の点を求められる (Gazebo の 3.5 m 先の人で 3.43 m と出た)．

```{code-block} python
from jedy_vision import FLOOR_Z, pixel_on_floor
K, _ = ri.camera_info()
x1, y1, x2, y2 = ri.what_do_you_see(['人'])[0].box
robot.angle_vector(ri.angle_vector())
feet = pixel_on_floor(robot, K, (x1 + x2) / 2, y2, FLOOR_Z)   # base_link [m] か None
print(np.hypot(feet[0], feet[1]))                             # 距離 [m]
```
:::

### すでにあるデモ

1 日目に扱わなかったことも，動くデモがすでにある．作りたいものに近いデモを動かして，中身を読むとよい．
`ros2 run jedy_vision demos` で jedy_vision のデモの一覧と起動方法が出る (中身は `jedy/jedy_vision/scripts/`)．
表の下の 4 つ (`ros2 run jedy_skrobot ...`) は jedy_skrobot のデモで，この一覧には出ない (中身は `jedy/jedy_skrobot/scripts/enshu/`)．

Gazebo では，次の「舞台」が jedy_vision のデモに合わせてある (AprilTag の付いた台と立方体，椅子，コップ，人)．

```{code-block} console
$ ros2 launch jedy_bringup jedy_gazebo.launch.py demo:=true person:=true camera:=d405
```

ほかのデモは，表の「Gazebo」の列のとおりに引数を変えて Gazebo を起動する．
ロボットや物を動かしたデモのあとは，Gazebo を起動し直すと元の配置に戻る．

| したいこと | 起動するもの | Gazebo | 中身 (1 日目に出てこない話題) |
| --- | --- | --- | --- |
| 物を探して近づき，指さす | `ros2 run jedy_vision find_object cup` | 舞台 | 見回して探す，床との交点で距離を推定する |
| タグの位置と向き | `ros2 run jedy_vision tag_pose` | 舞台 | カメラ内部パラメータ，PnP，座標変換 |
| タグの正面に止まる | `ros2 run jedy_vision approach_tag` | 舞台 | 極座標の姿勢制御，閉ループ |
| 手で物に触る | `ros2 run jedy_vision touch_tag` | 舞台．先に `approach_tag` | カメラ座標からロボット座標へ，逆運動学，手前で一度止まる経由点 |
| 物をつかんで持ち上げる | `ros2 run jedy_vision grasp_tag` | 舞台 (遠くからでも台車で寄る．先に `approach_tag --standoff 0.17` をしておくと確実) | 複数のタグから物の姿勢を求める，姿勢付きの逆運動学，つかめたかの確認 |
| 画像の人と骨格を調べる | `ros2 run jedy_vision detect_people --pose` | 舞台 | YOLO と MediaPipe．結果を画像に保存する |
| 首で人を追う | `ros2 run jedy_vision look_at_person` | 舞台 | 首のコントローラだけに指令を送る (追い続けるなら `--keep`) |
| 人について行く | `ros2 run jedy_vision follow_person` | `person:=true` だけ (`demo:=true` を付けない) | 見え方で台車を動かすフィードバック |
| 人の腕のまねをする | `ros2 run jedy_vision mimic_arms` | 舞台 | 姿勢推定の結果から逆運動学で腕の姿勢を作る |
| 手を挙げた人に手を振り返す | `ros2 run jedy_vision wave_back` | 舞台 | 姿勢から合図を判定し，動作を作る |
| 声でデモを切り替える | `ros2 run jedy_vision voice_demo` | 舞台と声のノード | 音声認識の結果で行動を選ぶ |
| 言葉で言った目標へ走る | `ros2 run jedy_skrobot vlm_navigate 赤い箱に近づいて` | 舞台に `box:=true` を足す．VLM のキー | カメラの画像を VLM に送って次の一手を決める ({ref}`day-recognition-manipulation` の「VLM に聞いて動く」) |
| 色の付いた物の 3 次元の位置 | `ros2 run jedy_skrobot color_clusters` | `box:=true` だけ (カメラは既定のまま) | 色で画素を選び，深度画像で 3 次元に戻す |
| チェッカーボードの位置と向き | `ros2 run jedy_skrobot checkerboard_pose` | `checkerboard:=true` | 既知の模様から姿勢を求める (PnP) |
| 傾いたら止まる | `ros2 run jedy_skrobot checkpoint_go_forward_imu` | どれでもよい (傾かないと止まらない) | IMU (Gazebo のみ．実機の RCB-4mini には無い) |

- 「舞台」は上のコマンドのこと．「`person:=true` だけ」は `ros2 launch jedy_bringup jedy_gazebo.launch.py person:=true` と起動する．
- `touch_tag` は台の正面に寄ってから動かす．先に `approach_tag` を動かさないと，手が届かずに終わる．
  `grasp_tag` は立方体が見えれば自分で台車を寄せる (舞台を起動した直後から動かして持ち上げられた)．立方体が見えないときは，先に `approach_tag --standoff 0.17` で台の正面に寄っておく．
- `follow_person` は舞台では人の手前にある台に乗り上げて止まる．
- `color_clusters` は `camera:=d405` だと，赤い箱 (前に約 1.2 m，左に約 0.5 m) が D405 の深度 (0.5 m まで) の外になり，`0 cluster(s)` と出る．
- `vlm_navigate` には，AI TA のキーとカメラ画像を送ることへの同意，または自分の Google AI Studio のキー (`GOOGLE_API_KEY`．{ref}`vlm-google-ai-studio`，{doc}`tips/google-ai-studio`) が要る．キーが無くても `--dry-run left,forward,done` で VLM に聞かずに流れだけ試せる．
- `checkpoint_go_forward_imu` は傾いたときだけ止まる．Gazebo の平らな床ではロボットは傾かないので，止まらずに 30 秒で時間切れになる．

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

この Python (jedy_skrobot の venv) には YOLO (`ultralytics`) が入っていない．
そのため，デモの `ObjectDetector` や `PersonDetector` を自分のプログラムで使うと `ModuleNotFoundError: No module named 'ultralytics'` になる．
物や人を探すときは，認識のノード (下の表) を起動して `ri.what_do_you_see` や `ri.sees` を使う．
`jedy_vision` のうち YOLO を使わない関数 (`pixel_on_floor` など) は，この Python でも import できる．

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

声を使わないときは，`while` の中の `ri.listen` から `break` までを `ri.what_do_you_see` などに置き換え，`ri.say` の 2 行を消す．
VOICEVOX を起動していないと `ri.say` は `RuntimeError` になり，`finally` の中の `ri.say` も同じエラーを出す (エラーが 2 つ重なって出る)．
`ri.listen` も，音声認識を起動していないと `RuntimeError` になる．

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
Gazebo の画面は，Ubuntu の PrtSc キー (GNOME のスクリーンショット) で保存する．ロボットが写っていなければ，左の Entity Tree で jedy を右クリックして「Move to」を選ぶ．
頭のカメラの画像は `ros2 run jedy_skrobot show_camera --once --save ~/enshu/camera.png` で保存できる．
人は `person:=true` で Gazebo に出せ，舞台 (`demo:=true`) にはコップがある．
Gazebo に無いもの (ペットボトル・手のジェスチャ・手を挙げる人・声) は，PC の Web カメラやマイクで代わりに試してよい (認識のノードを `image_topic:=/image_raw` で起動する)．
```

## 実機で動かす

### 実機に出す前のチェック

<div class="screen">

- Gazebo で最後まで動いたか
- **台車**: 速さは前後・左右 0.078〜0.156 m/s，旋回 0.83〜1.66 rad/s の範囲か (遅すぎると動かない．{ref}`base-velocity`)．走る前に `ri.tuck_arms()` で腕をたたんだか．周りに人や机が無いか
- **腕**: 自分の体や台車にぶつかる姿勢を送っていないか (Gazebo やモデルで確かめる)．速すぎないか (`ri.angle_vector(av, 時間)` の時間は，{ref}`joint-angle-motion` の例のように 1 回の振りに 1〜2 秒以上かける．
  0.4 秒の振りは Gazebo でも関節の速さの上限を超え，`Time has been changed from 0.4 to 0.71 due to joint velocity limit` と出て伸ばされた)
- **止め方**: Ctrl-C で止まるか．Ctrl-C で止まるのはプログラムで，`finally` の `ri.stop()` が台車を止める．
  送ってある腕の動きは止まらず，目標の姿勢まで動いてそこに残る．腕もその場で止めるなら，`finally` に `ri.cancel_angle_vector()` を書く (Gazebo で確かめた)．
  それでも止まらないときに背面のスイッチでサーボの電源を切れる人がそばにいるか (切ると腕は重力で垂れるので支える)

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
| 見つけた物の 3D 位置 (base_link) | `ri.detection_position(d)` [m] か None (D405 は 0.5 m より遠いと None．上の「作品の例」の注意) |
| AI に画像について聞く | `ri.ask('質問')` / `ri.ask('質問', [画像1, 画像2])` (`model='vlm-er'`) |
| AI に選ばせる / 指さしてもらう | `ri.ask('質問', choices=['左', '右'])` / `ri.point('赤い箱')` → `(u, v)` か None |
| 画素の 3D 位置 (base_link) | `ri.pixel_position(u, v, depth)` [m] か None |
| ジェスチャ | `ri.gesture()` / `ri.janken()` / `ri.wait_until_gesture('チョキ')` / `ri.wait_until_janken()` |
| しゃべる / 聞く | `ri.say('文')` (`wait=False`, `speaker='ずんだもん-ノーマル'`) / `ri.listen(timeout=秒)` |
| モデル: 初期姿勢 / 関節 | `robot.reset_pose()` / `robot.head_joint1.joint_angle(rad)` |
| モデル: 手先 / 逆運動学 | `robot.rarm.end_coords.copy_worldcoords()` / `robot.rarm.inverse_kinematics(target, rotation_axis=False)` |

使える関節名は `[j.name for j in robot.joint_list]`，コントローラは `ri.controller_names`．
