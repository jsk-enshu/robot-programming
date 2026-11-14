# AI TA の使い方

この演習では，演習資料をもとに質問に答える **AI TA** を使える．
ターミナルで `ta "質問"` と打つと AI (Claude Code) が起動し，この資料を読んで答える．
エラーが出たときは `ta fix` で，直前のエラー・ターミナルの画面・ROS のログをまとめて AI TA に渡せる．

AI TA のプログラムは [iory/ai-ta-for-student](https://github.com/iory/ai-ta-for-student) にある．
使い方の最新の情報はこのリポジトリの README を見ること．

## 必要なもの

- {doc}`../environment-setup` の `vcs import` まで済んでいること (`~/ros2_ws/src/robot-programming` に演習資料がある．ビルドの前に設定してよい)
- インターネット接続
- GitHub アカウント．**演習のアンケートで登録したもの**を使う．
  AI TA は GitHub のユーザ名で受講者を確かめるので，登録していないアカウントでは使えない．
- Ubuntu (bash)．

```{note}
**ROS の環境が無くても試せる．** AI TA は演習資料を読んで答えるので，資料さえ手元にあれば，
環境構築をしていない PC (TA の PC など) でも質問できる．資料を次の場所に clone してから，下の「設定」の手順を行う．

    $ git clone https://github.com/jsk-enshu/robot-programming ~/robot-programming

AI TA は `~/ros2_ws/src/robot-programming/docs` か `~/robot-programming/docs` にある資料を自動で見つける．
資料は 200 MB ほどあるので，`git clone --depth 1 ...` にすると速い．
教員からキーのファイルを受け取った TA は，`ta login` の代わりに `ta setup <キーのファイル>` でも設定できる．
ROS が無いので，`ta fix` が集めるのはターミナルの画面と Python の Traceback だけになり，Gazebo での確認もできない．
```

## 設定

```{code-block} console
$ curl -fsSL https://raw.githubusercontent.com/iory/ai-ta-for-student/main/install.sh | bash
$ source ~/.bashrc
$ ta login    # 表示されたコードを https://github.com/login/device に入力して承認する
$ ta doctor   # 接続・キー・資料の場所を確認する (全部 ✓ になればよい)
```

- `install.sh` は `ta` コマンド，ターミナルのフック，Python の例外を記録するフック，Claude Code (入っていなければ) を入れる．
  中身を見てから入れたいときは `git clone https://github.com/iory/ai-ta-for-student` して `./install.sh` を実行しても同じ．
- `ta login` は GitHub のアカウントで本人確認をして，自分専用のキーを受け取る．
  「登録されていない」と言われたら，アンケートで登録した GitHub アカウントでログインしているか確かめ，それでも駄目なら TA に伝える．
- `ta login` の途中で，2 つの同意を `y` / `n` で聞かれる．Enter だけを押すと「同意しない」になる．
  - AI TA の記録 (下の「記録について」) への同意: **必須**．同意しないとキーを受け取れない．
  - カメラ画像を AI (Gemini) に送る機能への同意: 任意．同意すると {doc}`../robot-programming-1-2026` の `ri.ask` などが使える．同意しなくても AI TA は使える．
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
| `ta fix [メモ]` | 直前のエラーを集めて直し方を聞く (最後に失敗したコマンド，`ta run` の出力，tmux / byobu の画面，最新の launch のログ，Python の Traceback，ROS の環境変数．どれも 30 分以内のもの) |
| `ta run <コマンド>` | コマンドの出力を全部記録しながら実行する．長い出力を `ta fix` に渡したいときに使う |
| `ta review <ファイル.py>` | 実機で動かす前のチェック項目に照らしてコードを見てもらう |
| `ta call [メッセージ]` | 人の TA・教員を呼ぶ．エラーの情報も一緒に届く |
| `ta context` | `ta fix` が送る内容を表示するだけ (何も送らない) |
| `ta usage` | 今日の利用額と上限 |

例:

```{code-block} console
$ ta "Gazebo で台車を前に 0.3 m 動かすには？"
$ ros2 launch jedy_bringup jedy_gazebo.launch.py   # ここでエラーが出たら
$ ta fix
```

- 質問に `$` や `!` を含めるときは `'...'` (シングルクォート) で囲む．`"..."` の中では bash が `$1` などを置き換えてしまい，
  AI TA に届く文が変わる (`ta "上限 $10 は?"` は「上限 0 は?」になる)．
- コマンドが失敗すると `[ai-ta] exit N — "ta fix" で…` という 1 行が出る．`AITA_QUIET=1` で消せる．
- AI TA がコマンドを実行したりファイルを書き換えたりするときは，毎回確認を求められる．内容を読んでから許可すること．
  ただし，`~/enshu` の中のファイルの書き換えと，資料を読むだけのコマンドは確認なしで行う．
- AI TA は，示したコードが動くかを Gazebo で確かめることがある (Gazebo の起動やプログラムの実行)．
- ターミナルの画面は byobu / tmux の中で作業しているときだけ `ta fix` に渡る ({doc}`byobu`)．

## 注意

- 実機で動かす前に，`ta review` でコードを見てもらうとよい．
  AI TA が実機を動かすコマンドを実行するときは，その前に，周りに人や机が無いか・止まらないときに電源を切れる人がそばにいるかを聞かれる．
  確かめてから答え，実行のたびに内容を読んでから許可すること．
- 焦げたにおい・煙・発熱・けがなど危ないときは，AI に聞く前にロボットの電源を切り，`ta call` か近くの TA を呼ぶ．
- AI の答えは間違えることがある．資料と食い違うときや，おかしいと思ったときは TA に聞く．
- 1 日に使える額には上限がある．上限の額と今日の利用額は `ta usage` で分かる (AI TA とカメラ画像の AI の合計)．
  カメラ画像の AI の分は，そのうち 1 日 $2 まで．上限に達すると，その日はそれ以上使えない．1 回の質問は $0.05 ほど．

## 記録について

- 質問と AI TA の答えは記録され，教員と TA が見る (演習の改善と，困っている人を見つけるため)．成績には使わない．
  AI TA に貼った画像と，`ri.ask` などで送ったカメラ画像も保存される．
- 保存の期間や，消してほしいものがあるときは，教員に聞く．
- `ta fix` / `ta call` で送る前に，キーやパスワードらしい文字列は伏せ字にする．
- キーは `~/.config/ai-ta/env` にある．人に見せたり，ファイルやチャットに貼ったりしない．

## アンインストール

`~/.local/bin/ta`，`~/.local/share/ai-ta`，`~/.config/ai-ta`，`~/.bashrc` に足された ai-ta の行，
Python のユーザ site-packages (`python3 -m site --user-site`) の `aita_pyhook.py` と `aita-pyhook.pth` を消す．
