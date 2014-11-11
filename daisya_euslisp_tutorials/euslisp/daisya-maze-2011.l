;;
;; 移動台車を使って迷路を脱出するサンプル
;;    2009/11/14 written by nakanish
;;
(load "daisya.l")
(load "maze.l")

;; (setq *m*
;;       '((1 2) (2 3) (3 4) (4 9) (9 14) (9 8)
;; 	(8 7) (7 12) (12 13) (12 11) (11 6)
;; 	(11 16) (14 15) (16 17) (17 22)
;; 	(21 22) (22 23) (23 18) (23 24)
;; 	(24 19) (19 20) (20 15) (15 10)
;; 	(10 5) (20 25)))
(setq *m*
      '((1 5) (2 3) (2 6) (3 4) (4 8) (5 6)
	(6 10) (7 11) (8 12) (9 13) (10 14)
	(11 15) (11 12) (13 14) (14 15)))

(setq *maze-column-num* 4)  ;;迷路の横数
(setq *maze-row-num* 4)     ;;迷路の縦数
(setq *maze-length* 470)    ;;迷路の幅サイズ
;;
;; *m*にセットされた地図の作成、ロボットのスタート位置(1)への配置
(defvar *maze* nil)
(warn "(maze-init)~%")
(defun maze-init
  (&optional (maze *m*) (start 1))
  (when (not (boundp '*irtviewer*))
    (setq *irtviewer* (make-irtviewer)))
  (setq *maze* (make-maze maze
                          :remove-wall-list (list '((5 4) (5 5))) ;;除去したい壁インデックスの指定
			  :row-num *maze-column-num*
			  :column-num *maze-row-num*
			  :maze-length *maze-length*
			  :maze-height 150.0))
  (when (not (boundp '*daisya*))
    (setq *daisya* (instance daisyaclass :init)))
  ;;腕は邪魔だから折畳む
  (send *daisya* :angle-vector #f(0 0 0 80 -20 -90 0 0))
  ;;
  (send *daisya* :locate
	(generate-coords-from-nodenum start 
				      :column-num *maze-column-num*
				      :maze-length *maze-length*) 
	:world)
  (objects (list *maze* *daisya*))
  )
;;
;;
;; 迷路をとくデモ (迷路のとく探索手順自体はlspのi-searchをごっそり利用)
(warn "(maze-demo 1 25 :extender #'depth-extender)~%")
(warn "(maze-demo 1 25 :extender #'breadth-extender)~%")
(warn "(maze-demo 1 25 :extender #'hill-climb-extender)~%")
(warn "(maze-demo 1 25 :extender #'best-first-extender)~%")
(warn "(maze-demo 1 25 :extender #'a*-extender)~%")
(defun maze-demo
  (start goal 
	 &key (extender #'depth-extender)) ;;探索手法の指定
  ;;初期位置へ移動
  (move-daisya 
   (generate-coords-from-nodenum start :column-num (get *maze* :column-num)
				 :maze-length (get *maze* :maze-length))
   :view nil)
  (send *irtviewer* :draw-objects)
  (unix:usleep 500) ;;休憩
  ;;
  (let ((path (i-search start goal (get *maze* :maze-list) :extender extender)))
    (warn "solved path is ~a~%" path)
    (dolist (p path)
      (move-daisya 
       (generate-coords-from-nodenum p :column-num (get *maze* :column-num)
				       :maze-length (get *maze* :maze-length)))
      ))
  )
;;
;;
;; 台車を座標へ移動させる（アニメーション)
;;   回転、移動が必要な場合はそのように
;; 最初は移動にangle-vectorを使っていたのだけれど、挙動がおかしい
;;   base-x,y,zを原点においていってしまうorz
(defun draw-viewers nil (send *irtviewer* :draw-objects))
(defun move-daisya
  (c ;;cは座標 #f(0 0 0)みたいなもの
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
        (send *daisya* :simulate (send *maze* :bodies))
        (draw-viewers) ;;描画
	(unix:usleep (* 1000 ticktime)))
      )
     (t
      ;;位置をあわせる
      (send *daisya* :translate diff-vector :world) ;;並行移動
      ;;回転をあわせる
      (send *daisya* :rotate (deg2rad (- goal-angle current-angle)) :z) ;;回転
      ))
    ))
  



  
  
	 
    
