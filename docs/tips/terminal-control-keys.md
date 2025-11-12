# ターミナルの制御キー

## 概要

UNIXやLinuxのターミナルでは特定のキーボードショートカット（制御キー）を使用してプログラムの実行を制御できる．これらの制御キーはCtrlキーと組み合わせて使用するため[Caps LockをCtrlに変更](caps-to-ctrl.md)することで操作が格段に楽になる．

本ページではターミナルで頻繁に使用する制御キーについて解説する．

## 主要な制御キー

### Ctrl+C: プログラムの中断

**機能：**
- 実行中のプログラムに割り込みシグナル（SIGINT）を送信してプログラムを終了させる

**使用例：**
```{code-block} console
$ roscore
# roscoreが起動中．..
# Ctrl+Cを押す
^C  # ← Ctrl+Cを押した表示
[roscore] killing on exit
```

**よく使う場面：**
- 無限ループしているプログラムを停止したい
- ROSノードを終了したい
- 長時間実行されるコマンドをキャンセルしたい
- 誤って実行したコマンドを中断したい

**注意：**
- Ctrl+Cは強制終了ではなくプログラムに「終了してください」という要求を送る
- プログラムによってはこのシグナルを無視することもある
- データの保存などクリーンアップ処理が実行される場合がある

### Ctrl+Z: プログラムの一時停止（サスペンド）

**機能：**
- 実行中のプログラムを一時停止してバックグラウンドに移動させる
- プログラムは終了せず停止状態（Suspended）になる

**使用例：**
```{code-block} console
$ roscore
# roscoreが起動中．..
# Ctrl+Zを押す
^Z  # ← Ctrl+Zを押した表示
[1]+  Stopped                 roscore

# プロンプトが戻ってくる
$
```

**停止したプログラムの確認：**
```{code-block} console
$ jobs
[1]+  Stopped                 roscore
```

**プログラムの再開方法：**

1. **フォアグラウンドで再開：**
   ```{code-block} console
   $ fg
   # または特定のジョブ番号を指定
   $ fg %1
   ```

2. **バックグラウンドで再開：**
   ```{code-block} console
   $ bg
   # または特定のジョブ番号を指定
   $ bg %1
   ```

**よく使う場面：**
- エディタで作業中に一時的にターミナルに戻りたい
- 実行中のプログラムを終了せずに別のコマンドを実行したい
- 複数のプログラムを1つのターミナルで切り替えながら使いたい

**注意：**
- 停止したプログラムは実行を完全に停止している
- ROSノードなど通信を行うプログラムを停止すると他のノードとの接続が切れる可能性がある
- 長時間停止したままにすると問題が発生することがある

### Ctrl+D: 入力終了（EOF）

**機能：**
- 標準入力に対してEOF（End Of File）を送信する
- シェルやプログラムに「入力が終わった」ことを伝える

**使用例：**

1. **シェルの終了：**
   ```{code-block} console
   $ # Ctrl+Dを押す
   # ターミナルが閉じるかログアウトする
   ```

2. **catコマンドでの入力終了：**
   ```{code-block} console
   $ cat > test.txt
   Hello, World!
   This is a test.
   # Ctrl+Dを押す
   $ cat test.txt
   Hello, World!
   This is a test.
   ```

3. **Pythonインタラクティブシェルの終了：**
   ```{code-block} console
   $ python3
   >>> print("Hello")
   Hello
   >>> # Ctrl+Dを押す
   $
   ```

**よく使う場面：**
- ターミナルやSSHセッションを終了したい
- catコマンドで標準入力から入力を終了したい
- Pythonやその他の対話型シェルを終了したい

**注意：**
- Ctrl+Dは「入力終了」であって「プログラム終了」ではない
- プログラムの途中でCtrl+Dを押すと予期しない動作をする場合がある

### Ctrl+S / Ctrl+Q: 画面出力の制御

**Ctrl+S: 画面出力の一時停止**
- ターミナルの出力をフリーズさせる
- 大量の出力をスクロールさせずに確認したい場合に使用

**Ctrl+Q: 画面出力の再開**
- Ctrl+Sで停止した出力を再開する

**使用例：**
```{code-block} console
$ dmesg
# 大量のログが表示される
# Ctrl+Sを押すと出力が停止
# ゆっくり読める
# Ctrl+Qを押すと出力が再開
```

**注意：**
- Ctrl+Sを押したまま忘れると「ターミナルがフリーズした」と勘違いしやすい
- 現代ではページャ（lessなど）を使う方が一般的

### Ctrl+L: 画面のクリア

**機能：**
- ターミナルの画面をクリアする
- `clear`コマンドと同じ効果

**使用例：**
```{code-block} console
$ # 画面が情報で埋まっている
# Ctrl+Lを押す
# 画面がクリアされてプロンプトが一番上に表示される
```

**よく使う場面：**
- ターミナルの表示をすっきりさせたい
- 新しい作業を始める前に画面を整理したい

### Ctrl+U / Ctrl+K: 行の削除

**Ctrl+U: カーソル位置から行頭まで削除**
```{code-block} console
$ cd /home/user/very/long/path
         ↑ここにカーソルがある状態でCtrl+U
$ cd
# カーソルより前が削除される
```

**Ctrl+K: カーソル位置から行末まで削除**
```{code-block} console
$ cd /home/user/very/long/path
         ↑ここにカーソルがある状態でCtrl+K
$ cd /home
# カーソルより後が削除される
```

**よく使う場面：**
- 長いコマンドを打ち間違えた時
- コマンドの一部を素早く削除したい時

### Ctrl+W: 単語の削除

**機能：**
- カーソルの直前の単語を削除する

**使用例：**
```{code-block} console
$ git commit -m "Fix bug in sensor module"
                                 ↑ここにカーソルがある状態でCtrl+W
$ git commit -m "Fix bug in sensor "
# "module"が削除される
```

### Ctrl+A / Ctrl+E: カーソル移動

**Ctrl+A: 行頭に移動**
```{code-block} console
$ cd /home/user/very/long/path
                              ↑ここにカーソルがある状態でCtrl+A
$ cd /home/user/very/long/path
  ↑行頭に移動
```

**Ctrl+E: 行末に移動**
```{code-block} console
$ cd /home/user/very/long/path
  ↑ここにカーソルがある状態でCtrl+E
$ cd /home/user/very/long/path
                              ↑行末に移動
```

**よく使う場面：**
- 長いコマンドの先頭や末尾に素早く移動したい
- コマンドの編集を効率的に行いたい

## 制御キーの一覧表

| キー | 機能 | 説明 |
|:-----|:-----|:-----|
| Ctrl+C | プログラム中断 | 実行中のプログラムを終了 |
| Ctrl+Z | プログラム一時停止 | プログラムをバックグラウンドに移動 |
| Ctrl+D | 入力終了（EOF） | シェルやプログラムを終了 |
| Ctrl+S | 画面出力停止 | ターミナル出力をフリーズ |
| Ctrl+Q | 画面出力再開 | 停止した出力を再開 |
| Ctrl+L | 画面クリア | ターミナルをクリア |
| Ctrl+U | 行頭まで削除 | カーソルから行頭まで削除 |
| Ctrl+K | 行末まで削除 | カーソルから行末まで削除 |
| Ctrl+W | 単語削除 | 直前の単語を削除 |
| Ctrl+A | 行頭に移動 | カーソルを行頭に移動 |
| Ctrl+E | 行末に移動 | カーソルを行末に移動 |
| Ctrl+R | 履歴検索 | コマンド履歴を検索 |

## ジョブ制御の詳細

### ジョブとは

UNIXシェルにおける「ジョブ」とはシェルから起動されたプログラムやプロセスの集まりを指す．

**ジョブの状態：**
- **フォアグラウンド**：現在実行中でターミナルを占有している状態
- **バックグラウンド**：バックグラウンドで実行されている状態
- **停止（Suspended）**：Ctrl+Zで一時停止している状態

### ジョブ制御コマンド

**jobs: ジョブ一覧の表示**
```{code-block} console
$ jobs
[1]   Running                 roscore &
[2]-  Stopped                 vim test.txt
[3]+  Stopped                 rosrun rviz rviz
```

表示の意味：
- `[1]`, `[2]`, `[3]`: ジョブ番号
- `+`: カレントジョブ（最後にアクセスしたジョブ）
- `-`: 前のジョブ
- `Running`: 実行中
- `Stopped`: 停止中

**fg: フォアグラウンドで再開**
```{code-block} console
$ fg          # カレントジョブをフォアグラウンドで再開
$ fg %2       # ジョブ番号2をフォアグラウンドで再開
```

**bg: バックグラウンドで再開**
```{code-block} console
$ bg          # カレントジョブをバックグラウンドで再開
$ bg %2       # ジョブ番号2をバックグラウンドで再開
```

**kill: ジョブの終了**
```{code-block} console
$ kill %1     # ジョブ番号1を終了
$ kill %2     # ジョブ番号2を終了
```

### プログラムを最初からバックグラウンドで起動

コマンドの最後に`&`を付けるとバックグラウンドで起動する：

```{code-block} console
$ roscore &
[1] 12345
$ # プロンプトがすぐに戻ってくる
```

## 実践例

### 例1: ROSノードの操作

```{code-block} console
# roscoreを起動
$ roscore
# ... roscoreが実行中 ...

# Ctrl+Zで一時停止
^Z
[1]+  Stopped                 roscore

# 別のコマンドを実行
$ rostopic list

# roscoreをバックグラウンドで再開
$ bg
[1]+ roscore &

# 別のノードを起動
$ rosrun rviz rviz
# ... rvizが実行中 ...

# Ctrl+Cでrvizを終了
^C

# roscoreも終了
$ fg
# Ctrl+C
^C
```

### 例2: エディタとターミナルの切り替え

```{code-block} console
# ファイルを編集
$ vim program.py
# ... 編集中 ...

# Ctrl+Zでvimを一時停止
^Z
[1]+  Stopped                 vim program.py

# プログラムを実行してテスト
$ python3 program.py
# ... 結果確認 ...

# vimに戻る
$ fg
# ... 編集を続ける ...

# 再度Ctrl+Zで停止
^Z

# またテスト
$ python3 program.py

# vimに戻って編集完了
$ fg
# ... 保存して終了 ...
```

## なぜCaps LockをCtrlに変更すると便利なのか

これらの制御キーはすべてCtrlキーと組み合わせて使用する．標準的なキーボード配置ではCtrlキーが左下隅にあり小指を大きく伸ばす必要がある．

[Caps LockをCtrlに変更](caps-to-ctrl.md)することでホームポジションから指を動かさずにこれらの制御キーを使用できるようになり：

- **Ctrl+C, Ctrl+Z, Ctrl+D**などの頻繁に使う操作が楽になる
- **長時間作業しても手首や小指が疲れにくい**
- **タイピング速度が向上する**

特にターミナルを頻繁に使用するロボット開発やプログラミングにおいては作業効率が大幅に向上する．

## まとめ

ターミナルの制御キーを使いこなすことで効率的にプログラムを操作できる．

**最も重要な制御キー：**
- **Ctrl+C**: プログラムを中断
- **Ctrl+Z**: プログラムを一時停止
- **Ctrl+D**: シェルやプログラムを終了

**作業効率を上げる制御キー：**
- **Ctrl+L**: 画面をクリア
- **Ctrl+A / Ctrl+E**: 行頭・行末に移動
- **Ctrl+U / Ctrl+K**: 行の削除

これらの制御キーは[Caps LockをCtrlに変更](caps-to-ctrl.md)することでさらに使いやすくなる．ぜひ両方を組み合わせて活用してほしい．

## 参考資料

- [GNU Bash Manual - Job Control](https://www.gnu.org/software/bash/manual/html_node/Job-Control.html)
- [Linux Documentation - Terminal Control Keys](https://www.kernel.org/doc/html/latest/)
- [Caps LockをCtrlキーに変更する](caps-to-ctrl.md)
