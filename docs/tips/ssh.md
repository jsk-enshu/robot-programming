# SSHによるロボットPCへのアクセス

本演習では`SSH`（Secure Shell）という方法を使いロボットPCへアクセスすることでロボットPCの操作を行う．
`SSH`は，ネットワーク越しに他のPCに安全にアクセスするための手段である．`SSH`を使用することによりリモートPCのターミナルに直接アクセスしコマンド実行やファイルのやり取りを行うことが可能となる．以下に`SSH`接続の基本的な方法を示す．

## SSHの基本コマンド

`SSH`を用いてリモートPCに接続する際の基本的な書式は以下の通りである．

```{code-block} console
$ ssh ユーザー名@リモートPCのホスト名またはIPアドレス
```

本演習ではリモートPCがロボットPC，ユーザー名が `jedy` となる．
たとえばロボットPCのIPアドレスが `192.168.1.100`である場合，次のコマンドで接続する．

```{code-block} console
$ ssh jedy@192.168.1.100
# 初めてアクセスする場合は以下のような表示がでるのでyesと答える．
The authenticity of host 'XXX.XXX.XXX.XX (XXX.XXX.XXX.XX)' can't be established.
RSA key fingerprint is XXXXXXXXXXXXXXXXXXX.
Are you sure you want to continue connecting (yes/no)?
```

ロボットPCのログインパスワードは`jedy`である．

## SSH接続後のリモートPC操作例

`SSH`接続が成功するとリモートPCのターミナルに直接入ることができる．このターミナル上でリモートPCのファイル操作やプログラムの実行が可能である．以下にその例を示す．

```{code-block} console
$ ssh jedy@192.168.1.100
# 接続に成功すると以下のようにターミナルの行頭に表示されるPS1文字列が変わる．
jedy@XXXX-XXXX:~$ ls
# リモートPCのディレクトリを一覧表示する
jedy@XXXX-XXXX:~$ top
# リモートPCのプロセス一覧を表示する
```

## tmuxの活用

`SSH`接続でリモートPCを操作する際，`tmux`（terminal multiplexer）を使用すると作業効率が大きく向上する．`tmux`は以下のような利点を持つ．

- **セッションの永続化**: SSH接続が切れても`tmux`セッション内のプロセスは継続して実行される
- **複数ウィンドウ・ペインの管理**: 1つのSSH接続で複数のターミナルを同時に使用できる
- **セッションの再接続**: 一度切断しても，再度SSH接続すれば以前の作業状態に戻れる

特にロボットPCでROSノードを長時間実行する場合や，複数のターミナルで同時に作業する場合に有用である．

`tmux`の詳しい使い方については，本演習資料の[tmux・byobuの利用](byobu.md)を参照すると良い．また，公式ドキュメントとして[tmux入門](https://github.com/tmux/tmux/wiki/Getting-Started)や[tmux cheat sheet](https://tmuxcheatsheet.com/)なども有用である．

## SSH接続の終了

`SSH`接続を終了するには，以下のコマンドを入力することで`SSH`セッションが閉じられる．もしくは`Ctrl + D`でも`SSH`セッションを閉じることができる．

```{code-block} console
jedy@XXXX-XXXX:~$ exit
# SSHセッションを閉じるとPS1文字列も元に戻る
mech-user@ki00XXX:~$
```

`Ping`と`SSH`を用いることでネットワーク上のリモートPCにアクセスして安全かつ迅速に操作を行うことが可能である．初学者にはまず`Ping`で接続の確認を行い，続いて`SSH`で実際の操作に移る手順を推奨する．

## GitHubの公開鍵を使った認証

GitHubに登録されている公開鍵を使用してSSH認証を設定することができる．GitHubでは`https://github.com/<username>.keys`というURLで各ユーザーの公開鍵を取得できる．これを利用することで，パスワード入力なしでSSH接続が可能となる．

以下の手順でGitHubの公開鍵をリモートPCに登録する．

```{code-block} console
# リモートPCにSSH接続する
$ ssh jedy@192.168.1.100
# .sshディレクトリが存在しない場合は作成する
jedy@XXXX-XXXX:~$ mkdir -p ~/.ssh
# 適切なパーミッションを設定する
jedy@XXXX-XXXX:~$ chmod 700 ~/.ssh
# GitHubからioryユーザーの公開鍵を取得してauthorized_keysに追加する
jedy@XXXX-XXXX:~$ wget -qO- https://github.com/iory.keys >> ~/.ssh/authorized_keys
# authorized_keysのパーミッションを設定する
jedy@XXXX-XXXX:~$ chmod 600 ~/.ssh/authorized_keys
# SSH接続を終了する
jedy@XXXX-XXXX:~$ exit
```

上記の設定後，手元のPCにGitHubに登録した秘密鍵があれば，パスワード入力なしでSSH接続できるようになる．

```{code-block} console
# パスワード入力なしで接続できる
$ ssh jedy@192.168.1.100
```

**注意事項**:
- GitHubに公開鍵を登録していない場合は，事前にGitHubの設定ページ（Settings > SSH and GPG keys）から公開鍵を登録しておく必要がある．
- `wget -qO-`の`-q`オプションは静かなモード（進捗表示なし），`-O-`は標準出力への出力を意味する．
- `>>`は追記を意味するため，既存の`authorized_keys`ファイルがある場合でも内容を保持したまま追加される．
- 複数の公開鍵を登録する場合は，同じコマンドを異なるユーザー名で繰り返し実行すれば良い．

## 覚えておくと便利なオプション

- **`-p`** :
  デフォルト以外のポート番号で接続する場合に使用する．たとえばポート2200で接続する場合は以下の通りである．

```{code-block} console
  $ ssh -p 2200 jedy@192.168.1.100
```

- **`-i`** :
  特定の秘密鍵を指定して接続する場合に使用する．秘密鍵のパスを指定することで認証を行う．

```{code-block} console
  $ ssh -i ~/.ssh/id_rsa jedy@192.168.1.100
```
