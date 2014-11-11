;;
;; robot-serverをsimulationするクラスのサンプル
;;
(load "robot-server-simulator.l")
(load "maze.l")
(load "subsumption.l")

;;
;; serverをまず、別のeusで立ち上げておく
;;  server側では、迷路や障害物などの世界物体をこの例のように作成時に渡す
;;  この場合、*obstacle1*, *obstacle2*を渡してserverを立ち上げる
;; 
(warn "(init-server)~%")
(warn "(init-server 'simple-maze)~%")
(warn "(init-server 'maze)~%")

(defun init-server (&optional (world 'simple))
  ()
  (let (objects init-coords maze)
    (case
     world
     ('maze
      ;;make red cone for goal
      (setq goal (make-cone (float-vector 0 0 300) 100))
      (setf (get goal :face-color) :red)
      (send goal :locate (float-vector (* 470 5.5) (* 470 4.5) 0))
      ;;迷路オブジェクトを作る(maze.lを参照のこと)
      (setq maze 
	    (make-maze *m* ;;迷路リスト
		       :remove-wall-list (list '((5 4) (5 5))) ;;除去したい壁インデックスの指定
		       :maze-length 470  ;;実際の壁のregulationに準ズル
		       :maze-height 200))
      (setq objects (list maze goal))
      (setq init-coords (make-coords :pos
                                     (generate-coords-from-nodenum 1)
                                     :rpy (float-vector pi/2 0 0))))
     ('simple-maze
      ;;世界を構成するobjectを生成する
      (setq *obstacle1* (make-cube 100 100 200))
      (send *obstacle1* :locate #f(550 0 100))
      (setq *obstacle2* (make-cube 100 100 200))
      (send *obstacle2* :locate #f(-550 0 100))
      (setq *obstacle3* (make-cube 2500 10 200))
      (send *obstacle3* :locate #f(0 500 100))
      (setq *obstacle4* (make-cube 2500 10 200))
      (send *obstacle4* :locate #f(0 -500 100))
      (setq *obstacle5* (make-cube 10 1000 200))
      (send *obstacle5* :locate #f(1250 0 100))
      (setq *obstacle6* (make-cube 10 1000 200))
      (send *obstacle6* :locate #f(-1250 0 100))
      
      (setq objects (list *obstacle1*
                          *obstacle2*
                          *obstacle3*
                          *obstacle4*
                          *obstacle5*
                          *obstacle6*))
      (send-all objects :set-color :lightslateblue)
      (send *obstacle1* :set-color :blue)
      (send *obstacle2* :set-color :red)
      (setq init-coords (make-coords))
      )
     (t
      ;;世界を構成するobjectを生成する
      (setq *obstacle1* (make-cube 100 200 200))
      (send *obstacle1* :locate #f(600 0 100))
      (setq *obstacle2* (make-cube 100 200 200))
      (send *obstacle2* :locate #f(-400 0 100))
      (send *obstacle1* :set-color :red)
      (send *obstacle2* :set-color :red)
      (setq objects (list *obstacle1* *obstacle2*))
      (setq init-coords (make-coords)))
      )
    (start-robotsim-server objects :init-coords init-coords)
    ))
(defun m () (init-server 'maze))

;;
;; 上述の(init-server)で立ち上げたserverがいる状態で
;; (demo)を実行すると、
;; connect-robotsim-serverで simulation-serverと接続する
;; 以降は、connect-robotsim-serverで得られた *rs-sim*
;; を実機のロボットserver *rs* と全く同様に（一部未実装methodあるが)
;; ロボットを動かすことができる
;;
;; robot-programming-2009/client/eus-client/sample.lと同じinterfaceで
;; 記述されているのがわかると思う
;;
;; (connect-robot-server)のかわりに (connect-robotsim-server)を使うだけ
;;
(defun go-pos (x y theta)
  (unless *rs-sim*
    (warn ";; ERROR : ~%")
    (warn ";; you need to invoke~%")
    (warn ";; (connect-robotsim-server)~%")
    (warn ";; (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))~%")
    (warn ";; before this function~%")
    (return-from go-pos nil))
  (when (> (abs theta) 1)
    (send *rs-sim* :daisya-velocity-vector (float-vector 0 0 (* 0.5 theta)))
    (unix::usleep (round (* 69 (abs theta) 1000)))
    (let ((tmp y)) (setq y x) (setq x (abs tmp))))
  (send *rs-sim* :daisya-velocity-vector (float-vector x y 0))
  (unix::usleep (round (* 6.5 (max (abs x) (abs y)) 1000)))
  (send *rs-sim* :daisya-velocity-vector #f(0 0 0))
  (unix::usleep (* 100 1000))
  t)

(defun move-arm (vel &optional (tm 1000))
  (unless *rs-sim*
    (warn ";; ERROR : ~%")
    (warn ";; you need to invoke~%")
    (warn ";; (connect-robotsim-server)~%")
    (warn ";; (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))~%")
    (warn ";; before this function~%")
    (return-from move-arm))
  (when (not (boundp '*daisya*))
    (setq *daisya* (instance daisyaclass :init))
    (send *daisya* :arm-angle-vector #f(80 20 70 0 0)))
  (unless (send *daisya* :inverse-kinematics
                (make-cascoords :pos (v+ (send *daisya* :end-coords :worldpos) vel))
                )
    (warn "ERROR :: could not solve IK~%")
    (warn "      :: try smaller velocity~%")
    (return-from move-arm nil))
  (send *rs-sim* :arm-angle-vector (send *daisya* :arm-angle-vector) tm)
  (unix:usleep (* tm 1000))
  t)

(warn "(demo0) ;; plan->action~%")
(defun demo0
  (&key (start 1) (goal 25))
  (let ((path (maze-search start goal *m*))
        (prev-coords (make-coords :pos #f(200 200 0)
                                  :rpy (float-vector pi/2 0 0)))
        curr-coords pos rot c v)
    (when (not *rs-sim*) (connect-robotsim-server))
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))
    (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
    (warn "solved path is ~a~%" path)
    (dolist (p (cdr path))
      (setq c (generate-coords-from-nodenum p))
      (setq pos (send prev-coords :inverse-transform-vector
                      (float-vector (elt c 0) (elt c 1) 0)))
      (setq curr-coords (send prev-coords :copy-worldcoords))
      (send curr-coords :translate pos)
      (if (> (abs (elt pos 1)) (abs (elt pos 0)))
          (setq rot (if (> (elt pos 0) (elt pos 1)) -90 90))
        (setq rot 0))
      (send curr-coords :rotate (deg2rad rot) :z)
      (print (list p c pos rot))
      (go-pos (elt pos 0) (elt pos 1) rot)
      (unix:sleep 1)
      (setq prev-coords curr-coords)
      )
    (dotimes (i 3)
      (move-arm #f(0 100 0) 200)
      (move-arm #f(0 -100 0) 200))
    ))
;;
(warn "(demo1) ;; forward and back..~%")
(defun demo1
  (&optional (time 200))
  (let (bmp-vec)
    ;;robot-server(simulator)とつなぐ
    (when (not *rs-sim*) (connect-robotsim-server))
    (when (not *vs-sim*) (connect-visionsim-server))
    ;;
    ;;腕のservoをいれる
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))
    ;;ためしに腕を動かしてみる
    (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
    (unix:usleep (* 1000 1000))
    
    ;;とりあえず走りはじめる
    (send *rs-sim* :daisya-velocity-vector #f(100 0 0))
    ;;keyを押されるまで続ける
    (do-until-key
      (setq bmp-vec (send *rs-sim* :bumper-vector))
      ;;(print bmp-vec)
      ;;bumperのおされている位置に応じて、いったりきたりする
      (cond
       ;;前がぶつかったら
       ((or (= (aref bmp-vec 0) 1) (= (aref bmp-vec 5) 1))
	(send *rs-sim* :daisya-velocity-vector #f(-100 0 0))
	(warn "!!!~%")
        )
       ;;後がぶつかったら
       ((or (= (aref bmp-vec 2) 1) (= (aref bmp-vec 3) 1))
	(send *rs-sim* :daisya-velocity-vector #f(100 0 0))
	(warn "!!!~%")
        ))
      (unix:usleep (* time 1000))
      )
    ;;台車を止める
    (send *rs-sim* :wheel-reset)
    ;;腕に指令を送る
    (send *rs-sim* :arm-angle-vector (float-vector 0 0 0 0 0) 3000)
    (unix:usleep (* 1000 3000))
    ;;最後にarmのservoをきる
    (send *rs-sim* :arm-poweron-vector #f(0 0 0 0 0))
    ))

(warn "(demo2) ;; demo1 using subsumption~%")
(defun demo2
  (&optional (time 200))
  (let ((bmp-vec) (psd-vector)
        (red-centroid) ts)
    ;;robot-server(simulator)とつなぐ
    (when (not *rs-sim*) (connect-robotsim-server))
    ;;
    ;; subsumption の定義
    (push
     (instance
      subsumption-task :init :name "Back bumper" :priority 100
      :check-func
      #'(lambda (s)
	  (let ((bmp (cdr (assoc :bumper s))))
	    (if bmp
		(or (= (aref bmp 2) 1) (= (aref bmp 3) 1)))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(100 0 0))
          (warn "!!forward~%")))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front bumper" :priority 90
      :check-func
      #'(lambda (s)
	  (let ((bmp (cdr (assoc :bumper s))))
	    (if bmp
		(or (= (aref bmp 0) 1) (= (aref bmp 5) 1)))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-100 0 0))
          (warn "!!backward~%")))
     ts)

    (setq s (instance subsumption :init ts :debug nil))

    ;;腕のservoをいれる
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 1))
    ;;ためしに腕を動かしてみる
    (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
    (unix:usleep (* 1000 1000))
    
    ;;とりあえず走りはじめる
    (send *rs-sim* :daisya-velocity-vector #f(100 0 0))

    ;; subsumption をスタート
    (send s :start-clients)
    ;;keyを押されるまで続ける
    (do-until-key
      (setq bmp-vec (send *rs-sim* :bumper-vector))
      (format t ";; bmp ~A~%" bmp-vec)
      (send s :sensor-vector (list (cons :bumper bmp-vec)))
      (unix:usleep (* time 1000)))
    ;; subsumption を止める
    (send s :stop-clients)
    
    ;;台車を止める
    (send *rs-sim* :wheel-reset)
    ;;腕に指令を送る
    (send *rs-sim* :arm-angle-vector (float-vector 0 0 0 0 0) 3000)
    (unix:usleep (* 1000 3000))
    ;;最後にarmのservoをきる
    (send *rs-sim* :arm-poweron-vector #f(0 0 0 0 0))
    )
  )

(warn "(demo3) ;; use vision for maze~%")
(defun demo3
  (&optional (time 200))
  (let ((bmp-vec) (psd-vector)
        (red-centroid))
    ;;robot-server(simulator)とつなぐ
    (when (not *rs-sim*) (connect-robotsim-server))
    (when (not *vs-sim*) (connect-visionsim-server))
    ;;
    ;;腕のservoをいれる
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 1))
    ;;ためしに腕を動かしてみる
    (send *rs-sim* :arm-angle-vector #f(80 -20 -70 0 0) 1000)
    (unix:usleep (* 1000 1000))
    
    ;;とりあえず走りはじめる
    (send *rs-sim* :daisya-velocity-vector #f(100 0 0))
    ;;keyを押されるまで続ける
    (do-until-key
      (setq bmp-vec (send *rs-sim* :bumper-vector))
      (setq psd-vec (send *rs-sim* :psd-vector))
      (setq red-centroid (send *vs-sim* :result))
      (if red-centroid
          (setq red-centroid (cadr (assoc :centroid red-centroid))))
      (format t ";; bmp ~A, psd ~A, red ~A~%" bmp-vec psd-vec red-centroid)
      (cond
       ;;後がぶつかったら
       ((or (= (aref bmp-vec 2) 1) (= (aref bmp-vec 3) 1))
	(send *rs-sim* :daisya-velocity-vector #f(200 0 90))
	(warn "!!forward~%")
        (unix:usleep (* 300 1000))
	)
       ;;前がぶつかったら
       ((= (aref bmp-vec 0) 1)
	(send *rs-sim* :daisya-velocity-vector #f(-200 0 -90))
	(warn "!!backward (+ front right)~%")
        (unix:usleep (* 300 1000))
	)
       ((= (aref bmp-vec 5) 1)
	(send *rs-sim* :daisya-velocity-vector #f(-200 0 90))
	(warn "!!backward (+ front left)~%")
        (unix:usleep (* 300 1000))
	)
       ;;左に横がぶつかったら
       ((= (aref bmp-vec 1) 1)
        (send *rs-sim* :daisya-velocity-vector #f(0 -100 -30))
        (warn "!!move right~%")
        (unix:usleep (* 300 1000))
        )
       ;;左に壁が近づいてきたら
       ((< 10 (aref psd-vec 2) 100)
        (send *rs-sim* :daisya-velocity-vector #f(50 -50 -30))
        (warn "!!move+turn right~%")
        )
       ;;右に横がぶつかったら
       ((or (= (aref bmp-vec 4) 1) (< 10 (aref psd-vec 0) 100))
        (send *rs-sim* :daisya-velocity-vector #f(0 100 30))
        (warn "!!move left~%")
        (unix:usleep (* 300 1000))
        )
       ;;右に壁が近づいてきたら
       ((< 10 (aref psd-vec 0) 100)
        (send *rs-sim* :daisya-velocity-vector #f(50 50 30))
        (warn "!!move+turn left~%"))
       ;;
       ;;前に壁が近づいてきたら
       ((< 50 (aref psd-vec 1) 100)
        (send *rs-sim* :daisya-velocity-vector (float-vector 100 (random 20) 0))
        (warn "!!slow down~%")
        )
       ;; red-centroidに応じて追従行動をとる
       ((and red-centroid (> (aref red-centroid 0) 170))
        (send *rs-sim* :daisya-velocity-vector (float-vector 150 0 -10))
        (if (< 10 (aref psd-vec 1) 50) (move-arm #f(100 100 0) 500))
        (warn "!!turn left VISION~%")
        )
       ((and red-centroid (< (aref red-centroid 0) 150))
        (send *rs-sim* :daisya-velocity-vector (float-vector 150 -0 10))
        (if (< 10 (aref psd-vec 1) 50) (move-arm #f(100 -100 0) 500))
        (warn "!!turn right VISION~%"))
       ;; なにもないとき
       (t
        (send *rs-sim* :daisya-velocity-vector (float-vector 200 (random 100) (random 30))))
       )
      (unix:usleep (* time 1000))
      )
    ;;台車を止める
    (send *rs-sim* :wheel-reset)
    ;;腕に指令を送る
    (send *rs-sim* :arm-angle-vector (float-vector 0 0 0 0 0) 3000)
    (unix:usleep (* 1000 3000))
    ;;最後にarmのservoをきる
    (send *rs-sim* :arm-poweron-vector #f(0 0 0 0 0))
    )
  )

(warn "(demo4) ;; using subsumption~%")
(defun demo4
  (&optional (time 200))
  (let ((bmp-vec) (psd-vector)
        (red-centroid) ts)
    ;;robot-server(simulator)とつなぐ
    (when (not *rs-sim*) (connect-robotsim-server))
    (when (not *vs-sim*) (connect-visionsim-server))
    ;;
    ;; subsumption の定義

    (push
     (instance
      subsumption-task :init :name "Back bumper" :priority 100
      :check-func
      #'(lambda (s)
	  (let ((bmp (cdr (assoc :bumper s))))
	    (and bmp (= (length bmp) 6)
		 (or (= (aref bmp 2) 1) (= (aref bmp 3) 1)))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(200 0 90))
          (warn "!!forward~%")
          (unix:usleep (* 500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front psd" :priority 97
      :check-func
      #'(lambda (s)
	  (let ((psd (cdr (assoc :distance s))))
	    (and psd
		 (< 10 (aref psd 0) 120)
		 (< 10 (aref psd 1) 120)
		 (< 10 (aref psd 2) 120))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-200 0 0))
          (unix:usleep (* 500 1000))
          (send *rs-sim* :daisya-velocity-vector #f(0 0 -90))
          (warn "!!backward (from psd)~%")
          (unix:usleep (* 1500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front right bumper" :priority 95
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 0) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-200 0 -90))
          (warn "!!backward (+ front right)~%")
          (unix:usleep (* 500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front right bumper" :priority 90
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 5) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-200 0 90))
          (warn "!!backward (+ front left)~%")
          (unix:usleep (* 500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Left bumper" :priority 85
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 1) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(0 -100 -30))
          (warn "!!move right~%")
          (unix:usleep (* 600 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Left psd" :priority 75
      :check-func #'(lambda (s)
		      (let ((psd (cdr (assoc :distance  s))))
			(and psd (< 10 (aref psd 2) 100))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(50 -50 -30))
          (warn "!!move+turn right~%")
          (unix:usleep (* 400 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Right bumper" :priority 80
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 1) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(0 100 30))
          (warn "!!move left~%")
          (unix:usleep (* 600 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Right psd" :priority 70
      :check-func #'(lambda (s)
		      (let ((psd (cdr (assoc :distance  s))))
			(and psd (< 10 (aref psd 0) 100))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(50 50 30))
          (warn "!!move+turn left~%")
          (unix:usleep (* 400 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Center psd" :priority 55
      :check-func #'(lambda (s)
		      (let ((psd (cdr (assoc :distance  s))))
			(and psd (< 50 (aref psd 1) 100))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector (float-vector 100 (random 20) 0))
          (warn "!!slow down~%")))
     ts)

    (push
     (instance
      subsumption-task :init :name "Left Red object" :priority 35
      :check-func #'(lambda (s)
		      (let ((red (cdr (assoc :red  s))))
			(and red (> (aref red 0) 170))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector (float-vector 150 0 -10))
          (warn "!!turn left VISION~%")
          (unix:usleep (* 1000 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Right Red object" :priority 30
      :check-func #'(lambda (s)
		      (let ((red (cdr (assoc :red  s))))
			(and red (< (aref red 0) 150))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector (float-vector 150 0 10))
          (warn "!!turn right VISION~%")
          (unix:usleep (* 1000 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Wander" :priority 10
      :check-func #'(lambda (s) t)
      :action-func
      #'(lambda ()
          (print "wander")
          (send *rs-sim* :daisya-velocity-vector (float-vector 200 (- (random 100) 50) (- (random 30) 15)))
          (unix:usleep (* 100 1000))))
     ts)
    ;;
    (setq s (instance subsumption :init ts :debug nil))

    ;;腕のservoをいれる
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 1))
    ;;ためしに腕を動かしてみる
    (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
    (unix:usleep (* 1000 1000))
    
    ;;とりあえず走りはじめる
    (send *rs-sim* :daisya-velocity-vector #f(100 0 0))

    ;; subsumption をスタート
    (send s :start-clients)
    ;;keyを押されるまで続ける
    (do-until-key
      (setq bmp-vec (send *rs-sim* :bumper-vector))
      (setq psd-vec (send *rs-sim* :psd-vector))
      (setq red-centroid (send *vs-sim* :result))
      (if red-centroid
          (setq red-centroid (cadr (assoc :centroid red-centroid))))
      (format t ";; bmp ~A, psd ~A, red ~A~%" bmp-vec psd-vec red-centroid)
      (send s :sensor-vector (list (cons :bumper bmp-vec)
				   (cons :distane psd-vec)
				   (cons :red red-centroid)))
      (unix:usleep (* time 1000))
      )
    ;; subsumption を止める
    (send s :stop-clients)
    
    ;;台車を止める
    (send *rs-sim* :wheel-reset)
    ;;腕に指令を送る
    (send *rs-sim* :arm-angle-vector (float-vector 0 0 0 0 0) 3000)
    (unix:usleep (* 1000 3000))
    ;;最後にarmのservoをきる
    (send *rs-sim* :arm-poweron-vector #f(0 0 0 0 0))
    )
  )

