;;;
;;; 移動台車モデルを使ってHanoiの塔をとくサンプル
;;;    2009/11/14 written by nakanish
;;;
(load "daisya.l")    ;;台車モデルを定義したファイルをload

;;ロボットを表示するviewerがなければviewerを作成する。
(if (not (boundp '*irtviewer*))
    (setq *irtviewer* (make-irtviewer)))

;;台車モデルを作る
(defun init-daisya nil
  ;;ここで台車モデルを作成, *daisya*という変数に台車モデルのinstanceがbindされる
  (setq *daisya* (instance daisyaclass :init)) 
  ;;逆運動学が解きやすい初期姿勢に変更
  (send *daisya* :reset-pose)
  ;;画面に表示
  (objects (list *daisya*)))

;;逆運動学のテスト（サンプル）を実行する
(defun ik-test (&optional (pos #f(300.204 10.0 180.618)))
  ;;逆運動学が解きやすい初期姿勢に変更
  (send *daisya* :reset-pose)
  ;;画面を更新
  (send *irtviewer* :draw-objects)
  ;;inverse-kinematicsという逆運動学をとくmethodを呼び出す。
  ;;  Jacobianを使った収束計算に基く逆運動学が実装されている。
  ;;引数だが、
  ;; posで手先の目標位置、rotation-axisで回転姿勢も含めてIKをとくか（今はnilなので回転は無視）
  ;; debug-viewをtにすると、IK中のJacobianの収束の様子を描画する
  (send *daisya* :inverse-kinematics (make-cascoords :pos #f(300.204 10.0 180.618)) 
        :rotation-axis nil :debug-view t)
  ;;
  ;;手をひらく
  (send *daisya* :open-hand)
  ;;描画を更新
  (send *irtviewer* :draw-objects))
;;
;;;
;;; hanoi sample
;;;
;;円板dをfromkらtoへロボットを使って動かすための関数
(defun move-disk (d from to) ;; re-define
  (let (pav)
    (if (eq from 'ground)
        (move-on d (table-top to))
      (progn
        (move-arm d 'pick)
        (move-arm (get (table-top to) :top-coords) 'place)))
    (setf (get from :disks) (cdr (get from :disks)))
    (setf (get to :disks) (cons d (get to :disks)))
    ))
;;toまで腕を動かして、pick/placeが'pickならつかむ動作を、'placeなら設置する動作を行う
(defun move-arm (to pick/place)
  (let ()
    (unless
	;;*robot*モデルに対して、:pick, :placeというmethodを送る
	;;移動台車のモデルファイルのdaisya.lに(:pick, (:placeが定義されているのを
	;;見てみるとわかるが、逆運動学で目標まで手を伸ばし、手を開く（閉じる）ということを
	;;している
        (send *robot* (if (eq pick/place 'pick) :pick :place)
              to :rotation-axis nil
              :obstacles
              (remove-if #'(lambda (x) (< (norm (send x :difference-position to :translation-axis :z)) 50)) *tables*)
              :use-wheel t
              :debug-view :no-message
              )
      (break))
    (send *irtviewer* :draw-objects)))

;;台車を使ってHanoiの塔をとくサンプル
(defun hanoi-test nil
  ;;robotの設定：ここでは台車モデル（*daisya*)を使う
  (setq *robot* *daisya*)  
  ;;
  ;;Hanoiの塔の高さ、位置、名前を設定
  (setq *table-a* (hanoi-table :name "table-a" :height 125 :pos #f(300 200 0)))
  (setq *table-b* (hanoi-table :name "table-b" :height 150 :pos #f(350 0 0)))
  (setq *table-c* (hanoi-table :name "table-b" :height 100 :pos #f(300 -200 0)))
  (setq *tables* (list *table-a* *table-b* *table-c*))
  ;;
  ;;Hanoiの塔の初期問題状態を設定（３個のdiskをtable-aに設置)
  (hanoi-init 3 *table-a*)
  (setq *disk-1* (car *disks*))
  (setq *disk-2* (cadr *disks*))
  (setq *disk-3* (caddr *disks*))
  ;;
  ;;IKをときやすい状態に台車姿勢をセット
  (send *daisya* :reset-pose)
  ;;台車(*robot*)、Hanoiの塔(*tables*)、円板(*disks*)を表示する
  (objects (append *tables* *disks* (list *robot*)))
  ;;
  ;;問題をとく 
  ;;
  ;;関数hanoi-programを実行すると、解決までの行動リストが生成される
  ;; それをdolistの引数に与えることで、解決手順が順次actionに格納されていく
  (dolist (action (hanoi-program (length *disks*)))
    ;;行動をshellに表示
    (print action)
    ;;行動を実際にロボット台車を使って実行
    (eval action)
    ;;お休みする
    (unix:usleep (* 300 1000))
    ))
(warn "
(hanoi-test)
")
;;(load "daisya-hanoi.l")をすると、この関数が呼ばれるので、
;;自動的に、viewerやロボット台車モデルの作成、表示が行われるという仕組
(init-daisya)
;;
;; Hanoiの塔を構築するための関数
;;
;;Hanoiの塔のテーブルモデルを生成する
(defun hanoi-table
  (&key (name) (pos (float-vector 0 0 0)) (height 200))
  ;;(setf height (* 1.5 height))
  (let ((b (make-cylinder 30 height))
        (c (make-cube 80 80 4))
        bs)
    (send c :locate (float-vector 0 0 height))
    (send c :set-color :gray90)
    (send b :set-color :gray90)
    (send b :assoc c)
    (setq bs (instance bodyset-link :init (make-cascoords :coords (send c :copy-worldcoords))
                       :bodies (list b c)))
    (setf (get bs :top-face)
          (car (send c :get-face :cube :top)))
    (setf (get bs :top-coords)
          (face-coords (get bs :top-face)))
    (send bs :assoc (get bs :top-coords))
    (send bs :locate pos)
    (send bs :name name)
    bs
    ))

(defun face-center (f)
  (cadr (send f :centroid)))
(defun face-coords
  (f &optional (c (make-cascoords)))
  (setq c (copy-object c))
  (send c :locate (face-center f) :world)
  c)

;;Hanoiの塔の円板モデルを生成する
(defun hanoi-disk (&optional r h)
  (let* ((c (make-cylinder r h)))
    (setf (get c :top-face)
          (car (send c :get-face :cylinder :top)))
    (setf (get c :top-coords)
          (face-coords (get c :top-face)))
    (send c :assoc (get c :top-coords))
    (setf (get c :grasp-width) (* r 2))
    c
    ))

;;Hanoiの塔の円板モデルを個数を引数にして複数生成する（だんだん小さく、色を変化させて生成)
(defun disks (n &optional disks)
  (dotimes (i n)
    (setq disks (cons (hanoi-disk (+ (* (/ 27 n) i) 12) 20)
                      disks))
    (send (car disks) :set-color (send (elt gl::*face-colors* (* (+ i 4) 32)) :name))
    (send (car disks) :name (format nil "disk~A" i))
    )
  (reverse disks))

;;i番目のdiskを返す
(defun disk (i) (elt *disks* (1- i)))
;;
;; define motions
;;
(defun table-top (table)
  (if (get table :disks)
      (car (get table :disks))
    table))
(defun move-on (o to)
  (send o
        :locate
        (v+ (send (get to :top-coords) :worldpos)
            #f(0 0 0.05))
        :world)
  )
;;;
;;; define planning
;;;
;;a,b以外のtablelistを返す
(defun other-table
  (a b
   &optional
   (tables '(*table-a* *table-b* *table-c*)))
  (car (remove b (remove a tables))))
;;Hanoiの塔を解決する
(defun hanoi-program
  (&optional
   (n 3)
   (from '*table-a*)
   (to '*table-b*)
   (other (other-table from to)))
  (cond
      
      ((= n 1)
       (list (list 'move-disk (list 'disk n)
                   from to)))
    (t (append
        (hanoi-program (1- n) from other)
        (list (list 'move-disk (list 'disk n) from to))
        (hanoi-program (1- n) other to))))
  )
;;
;;hanoiの塔の問題をセットする
(defun hanoi-init (n initial-table)
  (setq *disks* (disks n))
  (setq *worlds* (append *disks* *tables*))
  (dotimes (i n)
    (move-disk (disk (- n i))
               'ground initial-table))
  )

