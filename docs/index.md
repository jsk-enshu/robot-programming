# 知能ロボット行動プログラミング演習

## 2025年度　機械情報工学科　冬学期演習

この資料は，知能ロボット行動プログラミング演習のためのものである．左のサイドバーから各回の資料を参照してほしい．

### 双腕移動台車ロボット（Jedy）

```{figure} fig/jedy.png
---
width: 400px
name: fig:jedy-index
---
双腕移動台車ロボットの外観
```

### システム構成

```{figure} fig/system-configuration-all.png
---
width: 600px
name: fig:system-configuration-all-index
---
双腕移動台車ロボットシステム構成
緑：ROSノード，赤：アクチュエータ，青：センサ，破線：ROS topic通信，実線：デバイスへのアクセス・その他
```

## 演習情報

### 担当教員

- 岡田慧 教授 (k-okada@jsk.imi.i.u-tokyo.ac.jp)
- 小島邦生 講師 (k-kojima@jsk.imi.i.u-tokyo.ac.jp)
- 矢野倉伊織 助教 (yanokura@jsk.imi.i.u-tokyo.ac.jp)
- 真壁佑 助教 (makabe@jsk.imi.i.u-tokyo.ac.jp)
- 山口真奈美 技術専門職員
- TA: 情報システム工学研究室

### 日程とスケジュール

冬学期演習の最後のまとめとしてこれまで学習してきた様々な内容を含んだ統合的演習を3回行う．全3回の内容は以下のような予定になっている．

1. **第1回 令和6年11月17日（月）知能ロボット行動プログラミング(1)**
   - 双腕移動台車ロボットを用いた認識操作プログラミング

2. **第2回 令和6年11月18日（火）知能ロボット行動プログラミング(2)**
   - ロボットの全身行動プログラミング

3. **第3回 令和6年11月20日（木）知能ロボット行動プログラミング(3)**
   - メカトロボットの製作とプログラミング
   - ※機械工学総合演習第二「電子回路演習」で配布した回路教材を用意しておくこと
   - これまでの内容を統合した知能ロボットシステムの構築

## 課題について

### 課題の提出方法

以下の必須課題を完了すること．課題の実行結果をスクリーンショット等で保存し，pdfファイルにまとめてUTOLに提出すること．

- **ファイル名**: `知能ロボット行動プログラミング演習_名前_x日目.pdf`
- **提出期限**: 各回の必須課題は当日中に提出すること

発展課題に関しては任意とするが，少なくとも1つ以上取り組んでみることをおすすめする．発展課題で取り組んだものについては最終レポートに含めて提出してみてほしい．

### 課題0（必須）

各チェックポイントを達成しながら，進捗報告シートへの記入を進め，全てのチェックポイントの進捗報告を完了せよ．各チェックポイントの結果をスクリーンショット等で保存しておくこと．

## リソース・リンク集

### GitHubリポジトリ

演習のコードはGitHubで公開されている：

https://github.com/jsk-enshu/robot-programming

コードやwikiの情報があり，wikiには演習のFAQを載せているのでそちらも参考にしてもよい．

### 便利リンク集

- **演習FAQ**
  - https://github.com/jsk-enshu/robot-programming/wiki/FAQ

- **EusLispのドキュメント**
  - https://github.com/euslisp/jskeus/blob/master/doc/jmanual.pdf
  - http://euslisp-docs.readthedocs.io/en/latest

- **EusLisp document for robot-programming**
  - http://jsk-enshu.github.io/robot-programming/robot_programming_manual.pdf
  - タートルボットのメソッド等，演習に役立つEusLispのドキュメント
  - (https://github.com/jsk-enshu/robot-programming のREADMEのdocumentationsからたどれる)

## 事前準備

初回資料の「移動台車関連の準備」を参考にし，ジョイスティックコントローラ・Jedyは適宜充電すること．前回・本日の付録の情報もぜひ見ながら演習を進めてほしい．
また必須ではないが左のサイドバーに[Caps LockをCtrlに変更](tips/caps-to-ctrl.md)などの便利な情報がまとまっているので適宜参照してほしい．
