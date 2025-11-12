# IRTビューワ(irtviewer)の操作方法

## 概要

IRTビューワ(irtviewer)は，EusLispで3Dモデルを表示するための対話型3Dビューワである．ロボットモデルの姿勢確認，認識結果の可視化，逆運動学の結果確認など，ロボットプログラミングにおいて視覚的なフィードバックを得るために不可欠なツールである．

:::{figure} ../fig/jedy-on-irtviewer.jpg
:align: center
:width: 500px
:name: fig:irtviewer-jedy-model

IRTビューワに表示されたJEDYロボットの3Dモデル
:::

本ページでは，irtviewerの基本的な操作方法とよく使われる機能について説明する．

## ビューワの起動

irtviewerは，EusLispでロボットモデルを読み込むと自動的に起動される．例えば，以下のようにロボットモデルを初期化すると，irtviewerウィンドウが開く．

```{code-block} lisp
irteusgl$ (load "package://jedyeus/euslisp/jedy.l")
irteusgl$ (setq *jedy* (jedy))
irteusgl$ (objects (list *jedy*))
```

## マウス操作によるビューワの制御

irtviewerでは，マウスドラッグによって視点を自由に変更できる．画面の異なる領域でドラッグすることで，回転，移動，拡大縮小を行う．

### 基本操作

| 操作 | 方法 | 説明 |
|------|------|------|
| **回転** | 画面中央付近でドラッグ | オブジェクトを中心に視点を回転させる |
| **上下移動** | 画面左部で上下にドラッグ | 視点を上下に平行移動させる（画面左部に見えないバーがあるイメージ） |
| **左右移動** | 画面下部で左右にドラッグ | 視点を左右に平行移動させる（画面下部に見えないバーがあるイメージ） |
| **拡大縮小** | 画面右部で上下にドラッグ | 視点を前後に移動させて拡大縮小する（画面右部に見えないバーがあるイメージ） |

### 操作のコツ

- **回転**：マウスカーソルを画面中央に配置し，ドラッグすることでオブジェクトを様々な角度から観察できる．
- **移動**：オブジェクトが画面外に出た場合，画面左部（上下移動）や画面下部（左右移動）でドラッグして視点を調整する．
- **拡大縮小**：細かい部分を確認したい場合は画面右部で上にドラッグ，全体を俯瞰したい場合は下にドラッグする．

## ビューワの更新

ロボットの姿勢を変更した後，ビューワに反映させるには以下のいずれかの方法を使用する．

### 方法1: ウィンドウをクリックする

irtviewerウィンドウをマウスでクリックすると，表示が更新される．最も簡単な方法である．

### 方法2: プログラムから更新する

以下のコマンドをEusLispプロンプトで実行すると，ビューワが更新される．

```{code-block} lisp
irteusgl$ (send *irtviewer* :draw-objects)
```

このコマンドは，スクリプト内でロボットの姿勢を連続的に更新する際に使用される．

### 方法3: フラッシュ制御（高度な使い方）

大量のオブジェクトを描画する場合，描画のちらつきを防ぐために`:flush`オプションを使用する．

```{code-block} lisp
;; 描画開始（フラッシュを無効化）
irteusgl$ (send *irtviewer* :draw-objects :flush nil)

;; 複数のオブジェクトを描画
;; ...

;; 描画完了（フラッシュを実行）
irteusgl$ (send *irtviewer* :viewer :viewsurface :flush)
```

## よく使われるビューワ操作のプログラム例

### 視点のリセット

ビューワの視点を初期状態に戻すには，以下のコマンドを使用する．

```{code-block} lisp
irteusgl$ (send *irtviewer* :look-all)
```

このコマンドは，表示されている全てのオブジェクトが画面内に収まるように視点を自動調整する．

### カメラ位置の設定

特定の視点からオブジェクトを見たい場合，カメラの位置と向きを直接設定できる．

```{code-block} lisp
;; カメラを特定の位置に配置（単位：mm）
irteusgl$ (send *irtviewer* :viewpoint #f(1000 1000 1000))  ;; カメラ位置 (x y z)
irteusgl$ (send *irtviewer* :look1 #f(1000 0 0))   ;; 注視点 (x y z)
```

### アニメーション表示

ロボットの動作をアニメーション表示するには，`:draw-objects`を繰り返し呼び出す．

```{code-block} lisp
;; 関節角度を徐々に変化させる例
irteusgl$ (dotimes (i 10)
            (send *jedy* :larm :shoulder-p :joint-angle (* i 10))
            (send *irtviewer* :draw-objects)
            (unix:usleep 100000))  ;; 100ms待機
```

## 複数オブジェクトの表示

irtviewerには，ロボットモデルだけでなく，認識結果のバウンディングボックスやチェッカーボードなど，複数のオブジェクトを同時に表示できる．

### オブジェクトの追加

```{code-block} lisp
;; ボックスオブジェクトの作成
irteusgl$ (setq *box* (make-cube 100 100 100))  ;; 100mm x 100mm x 100mm
irteusgl$ (send *box* :set-color :red)
irteusgl$ (send *box* :locate #f(500 0 500))    ;; 位置を設定

;; ビューワに追加
irteusgl$ (objects (list *jedy* *box*))
```

## 座標軸の表示

デバッグ時には，座標系を可視化すると便利である．

```{code-block} lisp
;; ワールド座標系の原点に座標軸を表示
irteusgl$ (send *irtviewer* :draw-origin 500)  ;; 500は軸の長さ[mm]

;; ロボットの手先座標系を表示
irteusgl$ (load "models/arrow-object.l")
irteusgl$ (setq *end-coords-arrow* (arrow))
irteusgl$ (send *end-coords-arrow* :newcoords (send (send *jedy* :rarm :end-coords) :copy-worldcoords))
irteusgl$ (send (send *jedy* :rarm :end-coords) :assoc *end-coords-arrow*) ;; arrow-objectを左手にassoc(くっつける)
irteusgl$ (objects (list *jedy* *end-coords-arrow*))
```

:::{figure} ../fig/irtviewer-arrow-object.jpg
:align: center
:width: 500px
:name: fig:irtviewer-arrow-object

右手にarrow-objectがついている様子．
:::


座標軸の色は以下のように対応している：
- **赤軸**：X軸
- **緑軸**：Y軸
- **青軸**：Z軸

:::{figure} ../fig/irtviewer-arrow-object-assoc.jpg
:align: center
:width: 500px
:name: fig:irtviewer-arrow-object-assoc

右手にarrow-objectがついている様子．`assoc`することでjedyの姿勢を変えてもarrowが手先について表示される．
:::


## トラブルシューティング

### ビューワが更新されない

**症状**：ロボットの姿勢を変更しても，ビューワに反映されない．

**解決方法**：
1. ビューワウィンドウをクリックする
2. `(send *irtviewer* :draw-objects)`を実行する
3. 別のウィンドウがirtviewerの前に重なっている場合は，irtviewerウィンドウを最前面に表示する

### ビューワが真っ白になる

**症状**：ビューワウィンドウが真っ白で何も表示されない．

**解決方法**：
1. `(objects (list *jedy*))`を実行してオブジェクトを再登録する
2. `(send *irtviewer* :look-all)`で視点をリセットする
3. X11転送を使用している場合，ネットワーク遅延が原因の可能性があるため，しばらく待つ

### オブジェクトが画面外に出てしまった

**症状**：視点操作の結果，オブジェクトが画面外に出て見えなくなった．

**解決方法**：
1. `(send *irtviewer* :look-all)`で視点をリセットする
2. または，画面左部・下部のドラッグで視点を移動させてオブジェクトを探す
