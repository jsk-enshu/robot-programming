# AI TA の使い方

この演習では，演習資料をもとに質問に答える **AI TA** を使える．
ターミナルで `ta "質問"` と打つと AI (Claude Code) が起動し，この資料を読んで答える．
`ta` だけを打つと Claude Code の対話の画面が開き，続けて頼みごとをしたり，画像を貼ったりできる (下の「対話の例」)．
エラーが出たときは `ta fix` で，直前のエラー・ターミナルの画面・ROS のログをまとめて AI TA に渡せる．

AI TA のプログラムは [iory/ai-ta-for-student](https://github.com/iory/ai-ta-for-student) にある．
使い方の最新の情報はこのリポジトリの README を見ること．

## 必要なもの

- {doc}`../environment-setup` の `vcs import` まで済んでいること (`~/ros2_ws/src/robot-programming` に演習資料がある．ビルドの前に設定してよい)
- インターネット接続
- GitHub アカウント．**演習のアンケートで登録したもの**を使う．
  AI TA は GitHub のユーザ名で受講者を確かめるので，登録していないアカウントでは使えない．
- Ubuntu の bash または zsh．fish など，ほかのシェルには対応していない (`ta` は動くが，失敗したコマンドが `ta fix` に渡らない．ROS 2 にも fish 用の設定ファイルは無い)．
  演習中はターミナルで `bash` と打ち，bash で作業する．ほかのシェルに対応させたい人は，AI を使ってぜひ [ai-ta-for-student](https://github.com/iory/ai-ta-for-student) に PR を送ってほしい．

```{note}
**ROS の環境が無くても試せる．** AI TA は演習資料を読んで答えるので，資料さえ手元にあれば，
環境構築をしていない PC (TA の PC など) でも質問できる．資料を次の場所に clone してから，下の「設定」の手順を行う．

    $ git clone https://github.com/jsk-enshu/robot-programming ~/robot-programming

AI TA は `~/ros2_ws/src/robot-programming/docs` か `~/robot-programming/docs` にある資料を自動で見つける．
資料は 200 MB ほどあるので，`git clone --depth 1 ...` にすると速い．
教員からキーのファイルを受け取った TA は，`ta login` の代わりに `ta setup <キーのファイル>` でも設定できる．
ROS が無いので，`ta fix` が集めるのはターミナルの画面と Python の Traceback だけになり，Gazebo での確認もできない．
```

(ai-ta-setup)=
## 設定

```{code-block} console
$ curl -fsSL https://raw.githubusercontent.com/iory/ai-ta-for-student/main/install.sh | bash
$ source ~/.bashrc    # zsh なら source ~/.zshrc
$ ta login    # 表示されたコードを https://github.com/login/device に入力して承認する
$ ta doctor   # 接続・キー・資料の場所を確認する (✗ が無ければよい)
```

- `install.sh` は `ta` コマンド，ターミナル (bash と zsh) のフック，Python の例外を記録するフック，Claude Code (入っていなければ) を入れる．
  中身を見てから入れたいときは `git clone https://github.com/iory/ai-ta-for-student` して `./install.sh` を実行しても同じ．
- `ta login` は GitHub のアカウントで本人確認をして，自分専用のキーを受け取る．
  「登録されていない」と言われたら，アンケートで登録した GitHub アカウントでログインしているか確かめ，それでも駄目なら TA に伝える．
- 学科の PC と自分の PC など，1 人 3 台まで，それぞれの PC で `ta login` できる (ta 0.2.9 以降．PC ごとにキーが出る)．4 台目で `ta login` すると，いちばん古い PC のキーが無効になる．
  キーが無効になった PC では `ta` が「この PC の AI TA のキーが無効になっています」と表示するので，その PC で `ta login` をやり直す．
- `ta login` の途中で，2 つの同意を `y` / `n` で聞かれる．Enter だけを押すと「同意しない」になる．
  - AI TA の記録 (下の「記録について」) への同意: **必須**．同意しないとキーを受け取れない．
  - カメラ画像を AI (Gemini) に送る機能への同意: 任意．同意すると {doc}`../robot-programming-1-2026` の `ri.ask` などが使える．同意しなくても AI TA は使える．同意しないと `ta doctor` のカメラ画像の行は `-` になる (エラーではない)．
  答えを変えたいときは `ta login` をやり直す．
- ホーム (`~`) の外のフォルダで初めて `ta` を起動すると「このフォルダを信頼するか」と聞かれる．
  最初は **No, exit** が選ばれているので，矢印キーで **Yes, I trust this folder** を選んでから Enter を押す (Enter だけだと終わる)．
  ホームとその中のフォルダでは聞かれない．
- `ta: command not found` と出るときは `~/.local/bin` が PATH に入っていない．
  `echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc` をしてから `source ~/.bashrc` する．
- 演習資料が見つからないと言われたら，`AITA_DOCS=~/ros2_ws/src/robot-programming/docs` のように場所を指定する．
- 新しい版にするには，上の `curl ... | bash` をもう一度実行する (キーと設定はそのまま残る)．入っている版は `ta version` で分かる．

## 使い方

| コマンド | すること |
|---|---|
| `ta` / `ta "質問"` | AI TA と会話を始める．演習資料を読んで答える |
| `ta fix [メモ]` | 直前のエラーを集めて直し方を聞く (最後に失敗したコマンド，最近打ったコマンドとその成否，最新の `colcon build` で失敗したパッケージのエラー，`ta run` の出力，tmux / byobu の画面，最新の launch のログ，Python の Traceback，ROS の環境変数．どれも 30 分以内のもの) |
| `ta run <コマンド>` | コマンドの出力を全部記録しながら実行する．長い出力を `ta fix` に渡したいときに使う |
| `ta review <ファイル.py>` | 実機で動かす前のチェック項目に照らしてコードを見てもらう |
| `ta call [メッセージ]` | 人の TA・教員を呼ぶ．エラーの情報も一緒に届く |
| `ta context` | `ta fix` が送る内容を表示するだけ (何も送らない) |
| `ta usage` | 今日の利用額と上限 |
| `ta uninstall` | AI TA を消す (演習が終わったら．下の「アンインストール」) |

例:

```{code-block} console
$ ta "Gazebo で台車を前に 0.3 m 動かすには？"
$ ros2 launch jedy_bringup jedy_gazebo.launch.py   # ここでエラーが出たら
$ ta fix
```

- 質問に `$` や `!` を含めるときは `'...'` (シングルクォート) で囲む．`"..."` の中では bash が `$1` などを置き換えてしまい，
  AI TA に届く文が変わる (`ta "上限 $10 は?"` は「上限 0 は?」になる)．
- コマンドが失敗すると `[ai-ta] exit N — "ta fix" で…` という 1 行が出る．`export AITA_QUIET=1` で消せる (`~/.bashrc` に書けばずっと消える．コマンドの前に `AITA_QUIET=1` を付けるだけでは消えない)．
- AI TA は **auto mode** で起動する (ta 0.2.13 以降．画面の下に `⏵⏵ auto mode on` と出る)．
  AI TA が実行するコマンドやファイルの書き換えは，安全かを AI が判定し，問題ないものは確認なしで行う．Gazebo や実機を動かすコマンドも含む．
  キー (`~/.config/ai-ta/env`) を読むことなどは設定で禁止してある．
- 1 つずつ確かめてから許可したいときは，`Shift+Tab` を何回か押して `manual mode on` にする．AI TA がコマンドを実行したりファイルを書き換えたりするたびに確認を求められる
  (`~/enshu` の中のファイルの書き換えと，資料を読むだけのコマンドは確認なし)．
- AI TA は，示したコードが動くかを Gazebo で確かめることがある (Gazebo の起動やプログラムの実行)．
- ターミナルの画面は byobu / tmux の中で作業しているときだけ `ta fix` に渡る ({doc}`byobu`)．

(ai-ta-interactive)=
## 対話の例 (画像を作って見る・画像を見せる)

`ta` で対話の画面を開き，やりたいことを文章で頼む．AI TA はプログラムを書いて実行し，結果を画像にして画面に開くこともできる．
画像を見ながら「本当にそうなったか」を自分で確かめるのに使える．

<video width="100%" controls muted preload="metadata" poster="../fig/ai-ta-demo-auto.jpg">
  <source src="../fig/ai-ta-demo-auto.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** auto mode での例 (Gazebo，待ち時間を詰めて 1.5 倍速．画像の場面は 5 秒止めている)．頼みごとは次の 2 回だけで，確認は 1 回も求められていない．

```{code-block} text
Gazebo で Jedy を動かしています（実機は使っていません）．右手を今の位置から前に 5 cm，上に 5 cm 動かす逆運動学を解いて，
解いた姿勢を ~/enshu/ik.png に描いて feh で開いてから，その姿勢を Gazebo のロボットに 3 秒で送ってください．
送ったあとの実際の姿勢も ~/enshu/ik_after.png に描いて feh で開き，目標とのずれを教えてください．
```

```{code-block} text
次に，頭のカメラの画像を ~/enshu/camera.png に保存して feh で開き，何が写っているか教えてください．
そのあと ri.ask で「赤い物は画面のどちら側にある？」と聞き，ri.point で赤い箱の位置に印を付けた画像を
~/enshu/vlm_point.png に保存して feh で開いてください．
```

AI TA は，送る前に Gazebo だけが動いていること (実機の launch が無いこと) を確かめ，
IK の姿勢を PyrenderViewer で描いて開き，送ったあとの姿勢も描いて，目標とのずれ (0.87 mm) を答えた．
カメラの画像では，写っている物を説明し，`ri.ask` の答え (「赤い物は画面の左側」) と，`ri.point` が指した位置に印を付けた画像を開いた．

<video width="100%" controls muted preload="metadata" poster="../fig/ai-ta-demo-manual.jpg">
  <source src="../fig/ai-ta-demo-manual.mp4" type="video/mp4">
  お使いのブラウザは動画タグをサポートしていません．
</video>

**動画：** manual mode での例 (1 つずつ頼み，許可しながら進める)．最後に，実機の RViz の画面を **Ctrl+V で貼って**
「緑と青の箱のどちらが合っている？」と聞くと，AI TA は画像を見て「青」と答えた ({doc}`ai-robot-programming` の例)．

- **画像を見せる**：スクリーンショットなどをクリップボードにコピーして，対話の画面で `Ctrl+V` で貼る．
  ファイルの場所 (`~/enshu/camera.png` など) を書いて「見て」と頼んでもよい．
- **結果を画像にしてもらう**：IK の姿勢 (PyrenderViewer)，カメラの画像，認識の結果などを画像に保存して開くように頼むと，
  数字だけでなく目で確かめられる．
- **auto mode でも中身は見る**：AI TA が書いたプログラムは `~/enshu` に残る．何をしたかを画面で追い，分からないところは「このプログラムを説明して」と聞く．
- 動画の `feh` は画像を開くだけのソフトで，窓を閉じるまで終わらない．見終わったら窓を閉じる (`q`)．

(ai-ta-debugging)=
## デバッグ中に使うとき

エラーを調べているうちに，`ls` や `ros2 pkg list` などを何回も打つことが多い．
`ta fix` は次のものも AI TA に渡すので，困っているエラーの後にほかのコマンドを打っていても，そのまま `ta fix` してよい (ta 0.2.10 以降．`ta version` で確かめる)．

- 30 分以内に打ったコマンドと，それぞれが成功したか失敗したか
- 最新の `colcon build` で失敗したパッケージのエラー．colcon はビルドの出力を `~/ros2_ws/log/latest_build/<パッケージ名>/` に全部残すので，画面が流れたあとでも，`clear` したあとでも渡る

それでもうまく伝わらないときは，次のようにする．

- **AI TA は最初から byobu の別のウィンドウで開いておく．** `ta` は起動したターミナルを使うので，同じ画面ではコマンドを打てない．
  byobu で `F2` を押して新しいウィンドウを作り，そこで `ta` や `ta fix` を実行し，作業は元のウィンドウ (`F3` / `F4` で行き来する) で続ける．
  byobu の中なら，`ta fix` はほかのウィンドウの画面も読む．
- **会話の途中で新しいエラーが出たら**，エラーの行をそのまま会話に貼る．
  会話の中では，起動したときに集めた情報しか AI TA に届いていない．集め直すときは会話を終えて (`/exit`) `ta fix` をもう一度実行する．
- **何が渡るかは `ta context` で確かめる．** 困っているエラーが入っていなければ，そのエラーを貼るか，下の `ta run` を使う．
- **出力が長いコマンドは `ta run` を付けて実行する** (`ta run colcon build --symlink-install`)．出力が全部記録され，次の `ta fix` に渡る．

## ほかの AI (ChatGPT など) に聞くとき

ほかの AI に聞くときも，**最後のエラーの数行だけでなく，そこまでに何をしたかとエラーの全文を渡す**と正しい答えが返りやすい．

- `ta context > ~/context.md` で，`ta fix` が集める情報 (コマンドの履歴，colcon のエラー，画面，ログ，環境) をファイルにできる (何も送らない)．キーやパスワードらしい文字列は伏せ字になっている．
- `colcon build` のエラーの全文は `~/ros2_ws/log/latest_build/<パッケージ名>/stderr.log` にある．
- byobu / tmux の画面の過去の分は `tmux capture-pane -p -S - > ~/screen.txt` でファイルにできる．
- AI TA を入れていない PC では，最初に `script -f ~/debug.log` を実行してから作業すると，そのターミナルの入出力が全部 `~/debug.log` に残る (終わったら `exit`)．

ほかの AI は演習の環境を知らないので，資料と違うやり方 (別の apt リポジトリを足す，パッケージを消す，など) を勧めることがある．
`sudo` や `apt` で何かを足したり消したりする前に，資料 ({doc}`troubleshooting`) を見るか，AI TA か TA に聞くこと．

## 注意

- 実機で動かす前に，`ta review` でコードを見てもらうとよい．
  AI TA が実機を動かすコマンドを実行するときは，その前に，周りに人や机が無いか・止まらないときに電源を切れる人がそばにいるかを聞かれる．
  確かめてから答えること．auto mode では，そのあとのコマンドは確認なしで実行されるので，ロボットが動いている間はそばで見ていて，危なければ止める．
- 焦げたにおい・煙・発熱・けがなど危ないときは，AI に聞く前にロボットの電源を切り，`ta call` か近くの TA を呼ぶ．
- AI の答えは間違えることがある．資料と食い違うときや，おかしいと思ったときは TA に聞く．
- 1 日に使える額には上限がある．上限の額と今日の利用額は `ta usage` で分かる (AI TA とカメラ画像の AI の合計)．
  カメラ画像の AI の分は，そのうち 1 日 $2 まで．上限に達すると，その日はそれ以上使えない．1 回の質問は $0.05〜0.1 ほど (`ta fix` や長い会話ほど高い)．

## 記録について

- 質問と AI TA の答えは記録され，教員と TA が見る (演習の改善と，困っている人を見つけるため)．成績には使わない．
  AI TA に貼った画像と，`ri.ask` などで送ったカメラ画像も保存される．
- 保存の期間や，消してほしいものがあるときは，教員に聞く．
- `ta fix` / `ta call` で送る前に，キーやパスワードらしい文字列は伏せ字にする．
- キーは `~/.config/ai-ta/env` にある．人に見せたり，ファイルやチャットに貼ったりしない．

## アンインストール

**演習の最後の日が終わったら，AI TA を入れた PC (学科 PC も自分の PC も) で次を実行する．**
消さないと，コマンドが失敗するたびに `[ai-ta] exit N` の 1 行が出続ける．

```{code-block} console
$ ta uninstall    # 消すものが表示されるので，y を入力する
```

- `~/.bashrc` に足された ai-ta の行，Python の例外を記録するフック，キー (`~/.config/ai-ta`)，`ta` コマンドなどを消す．
  元の `~/.bashrc` は `~/.bashrc.ai-ta-backup` に残る．
- Claude Code は，AI TA のインストールで入ったときだけ一緒に消える．前から入っていたものは残る．
- `ta version` が 0.2.4 以前のときは `ta uninstall` が無い (AI TA への質問として扱われる)．先に {ref}`設定 <ai-ta-setup>` の `curl ... | bash` で新しくしてから実行する．
- 消し忘れても，キーの有効期限が過ぎると `[ai-ta] exit N` の表示は止まる．
- サーバーにある質問と答えの記録は消えない．
