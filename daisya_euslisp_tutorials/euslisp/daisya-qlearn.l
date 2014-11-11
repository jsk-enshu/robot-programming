;;
;; 移動台車を使って迷路を強化学習を使って脱出するサンプル
;;    2009/12/02 written by nakanish
;;
(load "daisya.l")
(load "maze.l")
(load "qlearn.l")

;;迷路記述の別の方式 .. xが障害物、-が何もないところ、sがスタート、gがゴール
(setq *map* (list
	     "xxxxxxxxxxxxxxxx"
	     "x--------------x"
	     "x-s--x---------x"
	     "x------xxx-----x"
	     "x----x---------x"
	     "x-----xx--xxx--x"
	     "x-----x-x-x----x"
	     "x-----x--xx--g-x"
	     "x--------------x"
	     "xxxxxxxxxxxxxxxx"
	     ))
(defvar *m* nil)               ;;迷路のgraph表現
(defvar *maze-column-num* nil) ;;迷路の横数
(defvar *maze-row-num* nil)    ;;迷路の縦数
(setq *maze-length* 400)       ;;迷路の幅サイズ
(setq *maze-height* 150)       ;;迷路の高さサイズ
;;
;; *m*にセットされた地図の作成、ロボットのスタート位置(mapのs)への配置
(defvar *maze* nil)
(warn "(maze-init)~%")
(defvar *goal*)

(defun maze-init
  (&optional (map *map*))
  (let (start)
    ;;画面がなければ表示
    (when (not (boundp '*irtviewer*))
      (setq *irtviewer* (make-irtviewer)))

    ;; *m*, *maze-column-num*, *maze-row-num*をmapから生成
    (setq *m* (map2graph map))
    (setq *maze-column-num* (- (length (elt map 0)) 2))
    (setq *maze-row-num* (- (length map) 2))           

    ;;迷路モデルを生成する
    (setq *maze* (make-maze *m*
			    :row-num *maze-row-num*
			    :column-num *maze-column-num*
			    :maze-length *maze-length*
			    :maze-height *maze-height*))

    ;;台車モデルを生成する
    (when (not (boundp '*daisya*))
      (setq *daisya* (instance daisyaclass :init)))
  
    ;;腕は邪魔だから折畳む
    (send *daisya* :angle-vector #f(0 0 0 80 -20 -90 0 0))
  
    ;;mapからstartの番号を取得する
    (setq start (get-start-index map))
    ;;台車モデルをスタート位置にセットする
    (send *daisya* :locate
	  (generate-coords-from-nodenum start
					:column-num *maze-column-num*
					:maze-length *maze-length*) 
	  :world)

    ;;qlearnのmapに指示されたgoalの位置に赤い円筒をおく
    (setq *goal* (make-cylinder (* *maze-length* 0.4) *maze-height*))
    (setf (get *goal* :face-color) :red)
    (send *goal* :locate
	  (generate-coords-from-nodenum (get-goal-index map)
					:column-num *maze-column-num*
					:maze-length *maze-length*)
	  :world)

    ;;画面にモデルを表示
    (objects (list *maze* *daisya* *goal*))
  ))

;;迷路をとく学習を行う
(warn "(maze-learn)~%")
(defvar *search-env* nil)    ;;学習のための環境クラス
(defvar *search-agent* nil)  ;;学習を行うエージェントクラス
(defun maze-learn
  (&key (map *map*) (debug nil))
  (setq *search-env* (instance mdp-search-environment :init map))
  (setq *search-agent*
	;;学習方式によって、何通りかあるみたい
	;;(instance mdp-dyna-search-agent :init *search-env*)
	(instance mdp-search-agent :init *search-env*)
	;;(instance smdp-search-agent :init *search-env*)
	)
  ;;学習を行う
  (learn *search-agent* "qlearn.dat" :debug debug)
  )

;;学習結果を使って迷路をとくデモ(一応スタート位置も指定できるかな)
(warn "(maze-demo)~%")
(defun maze-demo
  ()
  ;;agentがなければ学習させる
  (when (not *search-agent*) (maze-learn))
  ;;
  ;;学習したエージェントの出力に応じて、1ステップずつ台車を移動させる
  (let* (start
	 (tgcount (*search-env* . ngoal))              ;;環境のゴール回数
	 (tcount 0) pos
	 )
    ;;初期位置へ移動
    (setq start (get-start-index *map*))
    (move-daisya 
     (generate-coords-from-nodenum start :column-num (get *maze* :column-num)
					 :maze-length (get *maze* :maze-length))
     :view nil)
    ;;
    (send *irtviewer* :draw-objects)
    (unix:usleep 500) ;;休憩
    ;;
    ;;agentの位置を変更
    ;;(send *search-env* :set-agent-pos index)

    ;;
    ;;もう一度ゴールするまで send a :policy-procを行う
    (while (= tgcount (*search-env* . ngoal))
      ;;1step進める
      (send *search-agent* :policy-proc)
      ;;現在の場所を求める
      (setq pos
	    (index2num (send *search-env* :state2xy (send *search-env* :state))
		       *maze-column-num*))
      
      ;;解けるとstartに戻ってしまうので、goalにいかせる
      (when (not (= tgcount (*search-env* . ngoal))) (setq pos (get-goal-index *map*)))
      ;;
      ;;そこまで移動する
      (move-daisya
       (generate-coords-from-nodenum pos
				     :column-num (get *maze* :column-num)
				     :maze-length (get *maze* :maze-length))
       :view t)
      )
    ))

#|*****************************************

qlearnのための迷路記述に書き換えるために必要な関数の定義

*******************************************|#
;;mapからgraphを生成する(廻りがxで囲まれていると仮定)
(defun map2graph
  (map)
  (let (ret width height path
	    index check-index-list
	)
    ;;xが両さいどに１つずつあるとする
    (setq height (- (length map) 2))
    (setq width (- (length (car map)) 2))
    ;;
    ;;x,yから順にせめていき、いける座標を入れていく
    (dotimes (y height)
      (dotimes (x width)
	(setq index (list (1+ x) (1+ y)))
	(setq check-index-list
	      (list
	       (list (1- (elt index 0)) (elt index 1)) ;;左
	       (list (1+ (elt index 0)) (elt index 1)) ;;右
	       (list (elt index 0) (1- (elt index 1))) ;;上
	       (list (elt index 0) (1+ (elt index 1))) ;;下
	       ))
	;;スタートが障害物じゃなかったら
	(when (not (= (elt (elt map (elt index 1)) (elt index 0)) #\x))
	  (dolist (check-index check-index-list)
	    ;;いき先が障害物じゃなかったら
	    (when (not (= (elt (elt map (elt check-index 1)) (elt check-index 0)) #\x))
	      (setq path (list (index2num index width) (index2num check-index width)))
	      ;;pathが追加ずみでなかったら,追加
	      (when (not (member path ret :test #'myset=))
		(push path ret))))
	  )))
    (nreverse ret)
    ))
;;indexから座標番号を返す(左上が1右下がwidth*height)
(defun index2num
  (index width)
  (+ 1 
     (1- (floor (elt index 0)))
     (* (1- (floor (elt index 1))) width)))

;;mapからstartのindexを取得する
(defun get-start-index
  (map)
  (get-symbol-index map #\s))
;;mapからgoalのindexを取得する
(defun get-goal-index
  (map)
  (get-symbol-index map #\g))
;;goalのindexを取得する
(defun get-symbol-index
  (map sym)
  (let* ((height (- (length map) 2))
	 (width (- (length (car map)) 2))
	 index
	 )
    (dotimes (y height)
      (dotimes (x width)
	(setq index (list (1+ x) (1+ y)))
	;;indexの文字がsだったら,indexを返す
	(when (= (elt (elt map (elt index 1)) (elt index 0)) sym)
	  (return-from get-symbol-index (index2num index width)))))
    ))
;; 集合として等しいかを返す(再帰を使わないで定義してみた)
;;  (myset= (list 1 2) (list 2 1))
(defun myset=
  (a b)
  (let ((flag t))
    (cond
     ((not (= (length a) (length b)))
      (setq flag nil))
     (t
      (dolist (c a)
	(when (not (member c b))
	  (setq flag nil)
	  (return-from nil)))))
    flag))
    

;; 台車を座標へ移動させる（アニメーション)
;;   回転、移動が必要な場合はそのように
;; 最初は移動にangle-vectorを使っていたのだけれど、挙動がおかしい
;;   base-x,y,zを原点においていってしまうorz
(defun move-daisya
  (c         ;;cは座標 #f(0 0 0)みたいなもの
   &key   
   (view t)               ;;描画する
   (ticktime 50)          ;;描画中の休み時間[msec]
   (interpolation-num 10) ;;補完数
   )
  (let* ((current-pos (send *daisya* :worldpos))
	 ;;現在のロボットの向き
	 (current-angle (rad2deg 
			 (atan (aref (matrix-column (send *daisya* :worldrot) 0) 1)
			       (aref (matrix-column (send *daisya* :worldrot) 0) 0))))
	 (diff-vector (v- c current-pos))
	 ;;目標のロボットの向き
	 (goal-angle (rad2deg (atan (aref diff-vector 1) (aref diff-vector 0))))
	 diff-angle
	 )
    (cond
     (view ;;current-vector -> goal-vectorまで補完して表示
      (setq diff-vector (scale (/ 1.0 interpolation-num) diff-vector))
      (setq diff-angle (/ (- goal-angle current-angle) interpolation-num))
      (dotimes (i interpolation-num)
	(send *daisya* :translate diff-vector :world)   ;;並行移動
	(send *daisya* :rotate (deg2rad diff-angle) :z) ;;回転
	(send *irtviewer* :draw-objects)                ;;描画
	(unix:usleep (* 1000 ticktime)))
      )
     (t
      ;;位置をあわせる
      (send *daisya* :translate diff-vector :world) ;;並行移動
      ;;回転をあわせる
      (send *daisya* :rotate (deg2rad (- goal-angle current-angle)) :z) ;;回転
      ))
    ))