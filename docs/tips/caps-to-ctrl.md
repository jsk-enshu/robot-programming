# Caps LockキーをCtrlキーに変更する

## コラム：なぜCaps LockをCtrlに変更するのか

プログラミングやロボット開発の現場では，テキストエディタやターミナルを長時間使用する．その際，Ctrlキーを頻繁に使用するが，多くのキーボードではCtrlキーが左下隅にあり，小指を大きく伸ばす必要がある．この動作を何千回も繰り返すと，手首や小指に負担がかかり，作業効率も低下する．

そこで，**Caps LockキーをCtrlキーに変更する**ことで，ホームポジションから指を大きく動かさずにCtrlキーを使用できるようになる．この設定は「UNIXハッカーのため，プログラマーのため」と言われており，長年プログラマーコミュニティで受け継がれてきた知恵である．

### 歴史的背景

キーボードの配置には興味深い歴史がある．初期のUNIXワークステーション用キーボードでは，Ctrlキーの配置が現在とは大きく異なっていた：

- **Sun Type-1**：VT100に準拠し，Aの左側がCaps Lock，Controlキーは左上部に配置
- **Sun Type-2**：Caps Lockが廃止され，Controlキーが左上部（Aの左側付近）に配置
- **Sun Type-3**：Type-2の配置を継続，Controlキーが使いやすい位置に
- **Sun Type-4**：Type-3と同じ配置を維持
- **Sun Type-5以降**：IBM 101キーボード仕様に変更され，Ctrlキーが左下隅に移動

この変更は，vi，Emacs，シェルなどのUNIXツールでCtrlキーを頻繁に使用するプログラマーにとって不便なものだった．そのため，多くのUNIXユーザーは，Caps LockをCtrlに変更することで，Sun Type-3のような使いやすい配置を再現している．

PFU社のHappy Hacking Keyboard（HHKB）は，Sun Type-1～Type-3のレイアウトを参考にした設計を採用しており，Ctrlキーをホームポジションに近い位置に配置している．この設計思想は，多くのプログラマーに支持され，現在でも愛用されている．

### 具体的な利点

1. **手首への負担軽減**：小指を大きく伸ばす動作が減り，長時間作業でも疲れにくい
2. **タイピング速度の向上**：ホームポジションから指を動かさずにCtrlキーにアクセスできる
3. **エディタ操作の効率化**：Emacs，vim，VSCodeなどのショートカットキーが使いやすくなる
4. **ターミナル操作の快適化**：Ctrl+C，Ctrl+Z，Ctrl+Dなどの操作が楽になる（詳細は[ターミナルの制御キー](terminal-control-keys.md)を参照）

Caps Lock機能は日常的にほとんど使わないため，この変更によるデメリットはほぼない．

<span style="color:red">**注意：本設定はあくまでCapsの位置にCtrlキーがあると便利であるという趣旨であり，Caps Lock機能自体が不要という意味ではない．**</span> Caps Lock機能は，大文字を連続して入力する際やShiftキーを押し続けるのが大変な場合などに有用である．Caps Lock機能を残しつつCtrlキーも使いたい場合は，後述の「Caps Lock機能を残しつつCtrlを追加」の設定を参照すること．

## Ubuntu 24.04での設定方法

### 推奨：gsettingsコマンド（最もシンプル）

最もシンプルで確実な方法は，以下のコマンド一つで設定できる：

```{code-block} console
$ gsettings set org.gnome.desktop.input-sources xkb-options "['ctrl:nocaps']"
```

このコマンドを実行するだけで，即座に設定が反映され，再起動後も保持される．

### 設定の確認

設定が正しく適用されているか確認する：

```{code-block} console
# 現在のキーボードオプションを確認
$ gsettings get org.gnome.desktop.input-sources xkb-options
['ctrl:nocaps']
```

### その他の方法（参考）

GUIで設定したい場合は，Gnome Tweaksを使用することもできる：

```{code-block} console
# Gnome Tweaksをインストール
$ sudo apt install gnome-tweaks
```

Gnome Tweaksを起動し，「キーボードとマウス」→「追加のレイアウトオプション」→「Caps Lock behavior」から「Make Caps Lock an additional Ctrl」を選択する．

## その他の選択肢

### Caps LockとCtrlを入れ替える

Caps LockをCtrlにするのではなく，両者を入れ替えたい場合：

```{code-block} console
$ setxkbmap -option ctrl:swapcaps
```

または，Gnome Tweaksで「Swap Ctrl and Caps Lock」を選択する．

### Caps Lock機能を残しつつCtrlを追加

Caps Lock機能も使いたい場合は，Shift+Caps LockでCaps Lockを有効化する設定もある：

```{code-block} console
$ setxkbmap -option ctrl:ctrl_modifier
```

## まとめ

Caps LockをCtrlに変更することは，プログラマーやエンジニアにとって非常に有用なカスタマイズである．特に，ROSのプログラミングやターミナル操作を長時間行う本演習では，この設定により作業効率が大幅に向上する．

一度この設定に慣れると，元のキーボード配置には戻れなくなるほど快適である．ぜひ試してみることをお勧めする．

## 参考情報

- [ITmedia記事: Happy Hacking Keyboard Professional HYBRID Type-S](https://www.itmedia.co.jp/news/articles/1912/10/news176.html)
- [Arch Linux Wiki: Keyboard configuration](https://wiki.archlinux.org/title/Keyboard_configuration_in_Xorg)
- [Ubuntu Community Help: Keyboard Configuration](https://help.ubuntu.com/community/Custom%20keyboard%20layout%20definitions)
