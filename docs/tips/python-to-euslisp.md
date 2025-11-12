# PythonプログラマのためのEusLisp入門

## 概要

EusLispはPythonに慣れたプログラマにとって，括弧が多く見慣れない構文に戸惑うかもしれないが，基本的な対応関係を理解すれば，比較的短時間で習得できる．

本ページでは，Pythonの知識を前提として，EusLispの構文と考え方を解説する．

## 基本的な構文対応

| 概念 | Python | EusLisp | 説明 |
|------|--------|---------|------|
| **関数呼び出し** | `jedy_init()` | `(jedy-init)` | 関数名を括弧で囲み，括弧の前に配置 |
| **変数代入** | `a = 1` | `(setq a 1)` | `setq`を使用して代入 |
| **メソッド呼び出し** | `ri.servo_off_all()` | `(send *ri* :servo-off-all)` | `send`でオブジェクトにメソッドを送信 |
| **引数付きメソッド** | `ri.go_velocity(0.1, 0, 0)` | `(send *ri* :go-velocity 0.1 0 0)` | 引数をカンマなしで列挙 |
| **リスト/配列** | `[0, 0, 0, 0, 0, 0]` | `#f(0 0 0 0 0 0)` | `#f`は浮動小数点数ベクトル |
| **入れ子の関数** | `jedy.inverse_kinematics(make_coords(pos=[370, 0, 150]))` | `(send *jedy* :inverse-kinematics (make-coords :pos #f(370 0 150)))` | 内側から外側へ評価される |


### 1. 関数呼び出し

**Python:**
```python
jedy_init()
print("Hello")
max(10, 20)
```

**EusLisp:**
```lisp
(jedy-init)
(print "Hello")
(max 10 20)
```

**ポイント:**
- EusLispでは，関数名を括弧の**内側**に書く（前置記法）
- 引数はカンマなしで列挙する
- 関数名にはハイフン（`-`）を使用する（Pythonのアンダースコア`_`に相当）

### 2. 変数代入

**Python:**
```python
a = 1
name = "robot"
position = [100, 200, 300]
```

**EusLisp:**
```lisp
(setq a 1)
(setq name "robot")
(setq position #f(100 200 300))
```

**ポイント:**
- `setq`（set quote）を使用して変数に値を代入する
- 文字列はダブルクォートで囲む（Pythonと同じ）
- ベクトルは`#f(...)`で表現する

### 3. メソッド呼び出し

**Python:**
```python
ri.servo_off_all()
ri.angle_vector([0, 0, 0, 0])
jedy.reset_pose()
```

**EusLisp:**
```lisp
(send *ri* :servo-off-all)
(send *ri* :angle-vector #f(0 0 0 0))
(send *jedy* :reset-pose)
```

**ポイント:**
- `send`関数を使用してオブジェクトにメッセージを送る
- メソッド名は`:method-name`のようにコロンで始まるキーワードシンボル
- オブジェクトが第一引数，メソッド名が第二引数，その後に引数を続ける

### 4. 引数付きメソッド

**Python:**
```python
ri.go_velocity(0.1, 0, 0)
ri.angle_vector([10, 20, 30], 3000)
jedy.move_to(x=100, y=200, z=300)
```

**EusLisp:**
```lisp
(send *ri* :go-velocity 0.1 0 0)
(send *ri* :angle-vector #f(10 20 30) 3000)
(send *jedy* :move-to :x 100 :y 200 :z 300)
```

**ポイント:**
- 位置引数はカンマなしで列挙
- キーワード引数は`:keyword value`のように記述
- Pythonの`func(a, b, c=10)`は，EusLispで`(func a b :c 10)`に相当

### 5. リスト・配列・ベクトル

**Python:**
```python
# リスト
my_list = [1, 2, 3, 4, 5]
# NumPy配列
import numpy as np
my_array = np.array([1.0, 2.0, 3.0])
```

**EusLisp:**
```lisp
;; リスト
(setq my-list (list 1 2 3 4 5))
;; または
(setq my-list '(1 2 3 4 5))

;; 浮動小数点数ベクトル
(setq my-vector #f(1.0 2.0 3.0))
;; 整数ベクトル
(setq my-int-vector #i(1 2 3))
```

**ポイント:**
- `#f(...)`は浮動小数点数ベクトル（float vector）
- `#i(...)`は整数ベクトル（integer vector）
- `'(...)`はリスト（クォートされたリスト）
- `(list ...)`もリストを作成する

### 6. 関数の入れ子（ネスト）

**Python:**
```python
jedy.inverse_kinematics(
    make_coords(pos=[370, 0, 150])
)

result = math.sqrt(abs(x - y))
```

**EusLisp:**
```lisp
(send *jedy* :inverse-kinematics
      (make-coords :pos #f(370 0 150)))

(setq result (sqrt (abs (- x y))))
```

**ポイント:**
- EusLispでは，内側の式から外側へ順に評価される
- `(- x y)`が最初に評価され，その結果に`abs`が適用され，最後に`sqrt`が適用される
- Pythonの関数合成`f(g(h(x)))`は，EusLispで`(f (g (h x)))`となる

## 演算子

### 算術演算

**Python:**
```python
a + b
a - b
a * b
a / b
a ** b  # べき乗
```

**EusLisp:**
```lisp
(+ a b)
(- a b)
(* a b)
(/ a b)
(expt a b)  ;; べき乗
```

### 比較演算

**Python:**
```python
a == b
a != b
a < b
a > b
a <= b
a >= b
```

**EusLisp:**
```lisp
(= a b)
(/= a b)
(< a b)
(> a b)
(<= a b)
(>= a b)
```

### 論理演算

**Python:**
```python
a and b
a or b
not a
```

**EusLisp:**
```lisp
(and a b)
(or a b)
(not a)
```

## 制御構造

### 条件分岐（if文）

**Python:**
```python
if x > 0:
    print("positive")
else:
    print("non-positive")
```

**EusLisp:**
```lisp
(if (> x 0)
    (print "positive")
  (print "non-positive"))
```

**多分岐（if-elif-else / cond）:**

**Python:**
```python
if x > 0:
    result = "positive"
elif x < 0:
    result = "negative"
else:
    result = "zero"
```

**EusLisp:**
```lisp
(cond
 ((> x 0) (setq result "positive"))
 ((< x 0) (setq result "negative"))
 (t (setq result "zero")))
```

### ループ（for文）

**Python:**
```python
for i in range(10):
    print(i)

for item in my_list:
    print(item)
```

**EusLisp:**
```lisp
;; カウンタループ
(dotimes (i 10)
  (print i))

;; リストの各要素に対するループ
(dolist (item my-list)
  (print item))
```

### while文

**Python:**
```python
while x < 100:
    x = x + 1
    print(x)
```

**EusLisp:**
```lisp
(while (< x 100)
  (setq x (+ x 1))
  (print x))
```

## EusLisp特有の重要な概念

### 1. グローバル変数の命名規則

EusLispでは，グローバル変数を`*variable*`のようにアスタリスクで囲む慣習がある．

```lisp
(setq *ri* (instance robot-interface :init))  ;; グローバル変数
(setq *jedy* (jedy))                           ;; グローバル変数
(setq local-var 10)                            ;; ローカル変数
```

### 2. コメント

**Python:**
```python
# これはコメント
x = 10  # 行末コメント
```

**EusLisp:**
```lisp
;; これはコメント
(setq x 10)  ;; 行末コメント
```

- セミコロン2つ（`;;`）が一般的
- セミコロン1つ（`;`）も使用可能

### 3. クォート（Quote）

EusLispでは，リストをそのまま評価せずにデータとして扱いたい場合，クォート（`'`）を使用する．

```lisp
;; クォートなし（関数として評価される）
(setq result (list 1 2 3))  ;; list関数を呼び出す

;; クォートあり（データとして扱う）
(setq result '(1 2 3))       ;; リストそのものを代入
```

### 4. float-vector（計算可能なベクトル）

`#f(...)`は読み取り専用のベクトルである．計算結果を格納したい場合は`float-vector`を使用する．

```lisp
;; 読み取り専用
(setq v1 #f(1.0 2.0 3.0))

;; 計算可能なベクトル
(setq v2 (float-vector 1.0 2.0 3.0))
(setq v3 (float-vector (+ 1.0 2.0) (* 2.0 3.0) (/ 10.0 2.0)))
;; => #f(3.0 6.0 5.0)
```

### 5. send-allとmapcar

複数のオブジェクトに同じ操作を行う場合の便利な関数．

**Python:**
```python
# リスト内包表記
names = [joint.name for joint in joints]

# map関数
lengths = list(map(len, strings))
```

**EusLisp:**
```lisp
;; send-all: 全てのオブジェクトに同じメソッドを送る
(setq names (send-all joints :name))

;; mapcar: 全ての要素に同じ関数を適用
(setq lengths (mapcar #'length strings))
```

## 実践例：ロボット制御

以下に，ロボット制御における典型的なPythonコードとEusLispコードの対比を示す．

### ロボットの初期化と姿勢制御

**Python（仮想的な例）:**
```python
# ロボット初期化
ri = RobotInterface()
jedy = JedyModel()

# リセット姿勢にセット
jedy.reset_pose()

# ビューワ更新
viewer.draw_objects()

# 実機に送信（4秒かけて移動）
ri.angle_vector(jedy.angle_vector(), 4000)
```

**EusLisp:**
```lisp
;; ロボット初期化
(jedy-init)  ;; *ri*と*jedy*を自動生成

;; リセット姿勢にセット
(send *jedy* :reset-pose)

;; ビューワ更新
(send *irtviewer* :draw-objects)

;; 実機に送信（4秒かけて移動）
(send *ri* :angle-vector (send *jedy* :angle-vector) 4000)
```

### 逆運動学を使った手先位置制御

**Python（仮想的な例）:**
```python
# 目標座標を作成
target = make_coords(pos=[370, 0, 150])

# 逆運動学を実行
jedy.inverse_kinematics(target, arm="larm")

# 実機に送信
ri.angle_vector(jedy.angle_vector(), 3000)
```

**EusLisp:**
```lisp
;; 目標座標を作成
(setq target (make-coords :pos #f(370 0 150)))

;; 逆運動学を実行
(send *jedy* :larm :inverse-kinematics target)

;; 実機に送信
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
```

### 関節名の取得とサーボON

**Python（仮想的な例）:**
```python
# 関節名のリストを取得
joint_names = [joint.name for joint in jedy.joint_list()]

# 特定の関節のみサーボON
ri.servo_on(names=["head_joint0", "head_joint1"])
```

**EusLisp:**
```lisp
;; 関節名のリストを取得
(setq joint-names (send-all (send *jedy* :joint-list) :name))

;; 特定の関節のみサーボON
(send *ri* :servo-on :names (list "head_joint0" "head_joint1"))
```

## まとめ

EusLispとPythonの主な違いをまとめると以下のようになる：

| 項目 | Python | EusLisp |
|------|--------|---------|
| **記法** | 中置記法（infix） | 前置記法（prefix） |
| **関数呼び出し** | `func(a, b)` | `(func a b)` |
| **メソッド呼び出し** | `obj.method(a)` | `(send obj :method a)` |
| **演算子** | `a + b` | `(+ a b)` |
| **代入** | `a = 1` | `(setq a 1)` |
| **配列/ベクトル** | `[1, 2, 3]` | `#f(1 2 3)` |
| **コメント** | `#` | `;;` |
| **括弧の意味** | 関数呼び出し，タプル，優先順位 | すべての式の境界 |

EusLispは一見複雑に見えるが，以下の3つのルールを理解すれば読み書きできるようになる：

1. **すべての式は括弧で囲まれる**
2. **関数名（または演算子）が最初に来る**
3. **引数はスペースで区切られる**

慣れるまでは，Pythonの構文をEusLispに変換する練習をすると良い．括弧の対応に注意しながらコードを読み書きすることで，徐々にLisp的な思考に慣れていくことができる．

## 参考資料

- [roseus効率的な作業方法](roseus-workflow.md) - roseusの実行環境とコマンド履歴の使い方
- [EusLispの逆運動学](euslisp-ik.md) - ロボットアームの逆運動学計算
- [IRTビューワ操作方法](irtviewer.md) - 3Dビューワの使い方
