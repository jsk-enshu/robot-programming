# rosemacsの利用

ターミナル上で

<div class="screen">

``` bash
$ sudo apt-get install ros-one-rosemacs
```

</div>

を実行すると，rosemacsというソフトウェアがインストールできる．
これはemacs上でROSの機能を使いやすくするためであり，
~/.emacs.d/init.elに

<div class="screen">

``` lisp
(add-to-list 'load-path "/opt/ros/one/share/emacs/site-lisp")
(require 'rosemacs-config)
```

</div>

と追加することで利用できる．

例えば，通常のターミナル上で

<div class="screen">

``` bash
$ source /opt/ros/one/setup.bash
$ roscd eusl
```

</div>

の後にTabをうつとeuslispと補完される．上記のemacs設定を使うことにより，
emacs上のターミナルでも補完されるようになる．

また，emacs上でC-x C-r fとうつと

<div class="screen">

    Enter ros path:

</div>

と表示される．これはC-x C-fのファイルやディレクトリを開く
emacsのコマンドをROSのパッケージ管理システムに対応させたものであり，
上記表示後に

<div class="screen">

    Enter ros path: euslisp

</div>

と打ちEnterをおすと euslispのディレクトリが表示される．
ファイル名も同様に

<div class="screen">

    Enter ros path: euslisp/jskeus/irteus/demo/demo.l

</div>

とうつと，ファイルが開かれる． C-x
C-fコマンドで同様のことを行おうとすると，
euslispより上のディレクトリのパスをすべて指定しなければならず
(演習環境では/home/mechuser/ros_ws/src/robot-programmingなど)，
面倒である． C-x C-r fを使うこと
ROSのパッケージは自動で探してきてくれる． もちろん，Enter ros
pathの後でTab補完も使える．

jedy_bringup.launch.pyを開く場合はC-x C-r
fをemacs上で実行した後以下を実行すればよい．

<div class="screen">

    Enter ros path: jedy_bringup/launch/jedy_bringup.launch.py

</div>

