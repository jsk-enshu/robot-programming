# EusLispの逆運動学メソッド

逆運動学が解けない場合には概ね例2の`:rotation-axis nil`を使うとよい．これは回転の誤差を許容するという設定である．

```{code-block} lisp
;; 実機に送らずロボットモデルだけでまずIKの動作を確認する
irteusgl$ (load "package://jedyeus/euslisp/jedy.l")
irteusgl$ (setq *jedy* (jedy))
irteusgl$ (objects (list *jedy*))
;; end-effector座標の確認
irteusgl$ (send (send *jedy* :rarm :end-coords) :draw-on :flush t)
;; 左手は
irteusgl$ (send (send *jedy* :larm :end-coords) :draw-on :flush t)
;; IKの例1
;;   エンドエフェクタの6自由度を守るIK(関節空間6次元，作業空間6次元)
irteusgl$ (send *jedy* :reset-pose) ;; 初期姿勢が大事！
irteusgl$ (send *jedy* :rarm :inverse-kinematics
             (make-coords :pos (float-vector 120 -90 -30) :rpy (list (deg2rad 30) (deg2rad 0) (deg2rad 0)))
             :debug-view :no-message) ;; debug-viewを指定するとanimationがみられる
;; IKの例2
;;   エンドエフェクタの位置3自由度を守るIK(関節空間6次元，作業空間3次元，冗長な構成)
irteusgl$ (send *jedy* :reset-pose)
irteusgl$ (send *jedy* :rarm :inverse-kinematics (make-coords :pos (float-vector 140 -50 -40))
            :rotation-axis nil) ;; エンドエフェクタ回転拘束がなし
;; IKの例3（目標エンドエフェクタ姿勢が達成できない例）
;;   エンドエフェクタの位置3自由度，回転2自由度を守るIK(関節空間6次元，作業空間5次元，冗長な構成)
irteusgl$ (send *jedy* :reset-pose)
irteusgl$ (send *jedy* :rarm :inverse-kinematics (make-coords :pos (float-vector 130 -60 -30))
            :rotation-axis :x) ;; エンドエフェクタのX軸まわり回転拘束がなし
;; IKの例4（目標エンドエフェクタ姿勢が達成できない例）
;;   例3でIKが解けなかったので収束判定を変える (ik failとでる収束の閾値を下げる)
irteusgl$ (send *jedy* :rarm :inverse-kinematics (make-coords :pos (float-vector 130 -60 -30))
            :rotation-axis :z
            :thre 25 :rthre (deg2rad 25)) ;; 位置と姿勢の閾値(thre=25mm,rthre=25deg)を変える
;; IKの例5
;;   収束判定を変える (ik failがでなくなる．誤差が収束しなくても解けたところまででreturnしてくる)
irteusgl$ (send *jedy* :rarm :inverse-kinematics (make-coords :pos (float-vector 300 -150 150))
            :revert-if-fail nil)
;; IKの例6
;;   move-end-pos，move-end-rotで位置だけ/姿勢だけ相対的に動かすことが
;;   できる(現在の手先位置からworldで-20,-10,-30mmだけ動かす)
irteusgl$ (send *jedy* :rarm :move-end-pos (float-vector -20 -10 -30) :world
           :rotation-axis nil)
```

