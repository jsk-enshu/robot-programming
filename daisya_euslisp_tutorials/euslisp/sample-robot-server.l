(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(load "package://daisya_euslisp_tutorials/euslisp/daisya-maze.l")
(load "subsumption.l")

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
    (dxl-armed-turtlebot-init :objects objects)
    ;;(start-robotsim-server objects :init-coords init-coords)
    ))

(defun test0 ()
  (warn ";; reset-pose~%")
  (send *dxl-armed-turtlebot* :reset-pose)
  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (warn ";; tuckarm-pose~%")
  (send *dxl-armed-turtlebot* :tuckarm-pose)
  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun test1 ()
  (warn ";; go-pos 0.3 0 0~%")
  (send *ri* :go-pos 0.3 0 0)
  (warn ";; go-pos 0 0 90~%")
  (send *ri* :go-pos 0 0 90)
  (warn ";; go-pos 0 0 0~%")
  (send *ri* :go-pos 0 0 0)
  )

(defun demo0
  (&key (start 1) (goal 25))
  (let ((path (maze-search start goal *m*))
        (prev-coords (make-coords :pos #f(200 200 0)
                                  :rpy (float-vector pi/2 0 0)))
        curr-coords pos rot c v)
    (init-server 'maze)
    (send *dxl-armed-turtlebot* :tuckarm-pose)
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *ri* :go-pos 0.2 0.2 0)
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
      (send *ri* :go-pos (* 1e-3 (elt c 0)) (* 1e-3 (elt c 1)) 0)
      (setq prev-coords curr-coords)
      )
    (send *dxl-armed-turtlebot* :arm :move-end-pos #f(150 0 -300) :world
          :rotation-axis nil)
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *dxl-armed-turtlebot* :arm :inverse-kinematics (make-coords :pos (send *dxl-armed-turtlebot* :transform-vector #f(430 0 250))) :rotation-axis nil)
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    ))

(defun demo1
  (&optional (time 200))
  (let (bmp-vec)
    ;;robot-server(simulator)とつなぐ
    (init-server 'maze)
    ;;(when (not *vs-sim*) (connect-visionsim-server))
    ;;
    ;;ためしに腕を動かしてみる
    (send *dxl-armed-turtlebot* :tuckarm-pose)
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *ri* :go-pos 0.2 0.2 0)
    ;;とりあえず走りはじめる
    (send *ri* :go-velocity 0.05 0 0)
    ;;keyを押されるまで続ける
    (block
        :loop
    (do-until-key
     (send *ri* :state)
     (setq bmp-vec (send *ri* :bumper-vector))
     (send *ri* :go-velocity 0.1 0 0 400)
      ;;bumperのおされている位置に応じて、いったりきたりする
      (cond
       ;;前がぶつかったら
       ((= (elt bmp-vec 2) 1)
	(warn ";; bumper hit!!!~%")
	(send *ri* :go-velocity -0.1 0 0)
        (let ((ang (read-char-case
                    ";; l (left), r (right), or f (finish): "
                    (#\r -90)
                    (#\l 90)
                    (#\f nil))))
          (if ang
              (send *ri* :go-velocity 0 0 ang)
            (return-from :loop nil))))
        )
      )
    )
    ;;台車を止める
    (send *ri* :go-stop)
    ;;腕に指令を送る
    (send *dxl-armed-turtlebot* :arm :move-end-pos #f(150 0 -400) :world
          :rotation-axis nil)
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    ))

(defun demo2
  (&optional (time 200))
  (let ((bmp-vec) (psd-vector)
        (red-centroid) ts)
    ;;robot-server(simulator)とつなぐ
    (init-server 'maze)
    ;;(when (not *vs-sim*) (connect-visionsim-server))
    ;;
    ;; subsumption の定義
    (send *ri* :go-pos 0.2 0.2 90)

    ;; (push
    ;;  (instance
    ;;   subsumption-task :init :name "Front psd" :priority 97
    ;;   :check-func
    ;;   #'(lambda (s)
    ;;       (let ((psd (cdr (assoc :distance s))))
    ;;         (and psd
    ;;     	 (< 10 (aref psd 0) 120)
    ;;     	 (< 10 (aref psd 1) 120)
    ;;     	 (< 10 (aref psd 2) 120))))
    ;;   :action-func
    ;;   #'(lambda ()
    ;;       (send *ri* :go-velocity -0.2 0 0)
    ;;       (unix:usleep (* 500 1000))
    ;;       (send *ri* :go-velocity 0 0 -90)
    ;;       (warn "!!backward (from psd)~%")
    ;;       (unix:usleep (* 1500 1000))))
    ;;  ts)

    (push
     (instance
      subsumption-task :init :name "Front left bumper" :priority 95
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 3)
			     (= (aref bmp 1) 1)
                             (= (aref bmp 2) 1))))
      :action-func
      #'(lambda ()
          (send *ri* :go-velocity -0.1 0 -20)
          (warning-message 1 "!!backward (+ front right)~%")
          ))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front right bumper" :priority 90
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 3)
			     (= (aref bmp 0) 1)
                             (= (aref bmp 2) 1)
                             )))
      :action-func
      #'(lambda ()
          (send *ri* :go-velocity -0.1 0 20)
          (warning-message 1 "!!backward (+ front left)~%")
          ))
     ts)

    (push
     (instance
      subsumption-task :init :name "Left bumper" :priority 85
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 3)
			     (= (aref bmp 1) 1)
                             (= (aref bmp 2) 0))))
      :action-func
      #'(lambda ()
          (send *ri* :go-velocity -0.1 0 -10)
          (warning-message 1 "!!move right~%")
          ))
     ts)

    (push
     (instance
      subsumption-task :init :name "Right bumper" :priority 80
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 3)
			     (= (aref bmp 0) 1)
                             (= (aref bmp 2) 0))))
      :action-func
      #'(lambda ()
          (send *ri* :go-velocity -0.1 0 10)
          (warning-message 1 "!!move left~%")
          ))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front bumper" :priority 100
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 3)
			     (= (aref bmp 2) 1)
                             (= (aref bmp 0) 0)
                             (= (aref bmp 1) 0))))
      :action-func
      #'(lambda ()
          (send *ri* :go-velocity -0.1 0 (- (random 90) 45))
          (warning-message 1 "!!move back and turn~%")
          ))
     ts)

    ;; (push
    ;;  (instance
    ;;   subsumption-task :init :name "Left psd" :priority 75
    ;;   :check-func #'(lambda (s)
    ;;     	      (let ((psd (cdr (assoc :distance  s))))
    ;;     		(and psd (< 10 (aref psd 2) 100))))
    ;;   :action-func
    ;;   #'(lambda ()
    ;;       (send *ri* :go-velocity 0.5 -0.5 -30)
    ;;       (warn "!!move+turn right~%")
    ;;       (unix:usleep (* 400 1000))))
    ;;  ts)

    ;; (push
    ;;  (instance
    ;;   subsumption-task :init :name "Right psd" :priority 70
    ;;   :check-func #'(lambda (s)
    ;;     	      (let ((psd (cdr (assoc :distance  s))))
    ;;     		(and psd (< 10 (aref psd 0) 100))))
    ;;   :action-func
    ;;   #'(lambda ()
    ;;       (send *ri* :go-velocity 0.5 0.5 30)
    ;;       (warn "!!move+turn left~%")
    ;;       (unix:usleep (* 400 1000))))
    ;;  ts)

    ;; (push
    ;;  (instance
    ;;   subsumption-task :init :name "Center psd" :priority 55
    ;;   :check-func #'(lambda (s)
    ;;     	      (let ((psd (cdr (assoc :distance  s))))
    ;;     		(and psd (< 50 (aref psd 1) 100))))
    ;;   :action-func
    ;;   #'(lambda ()
    ;;       (send *ri* :go-velocity 0.1 (* 1e-3 (random 20)) 0)
    ;;       (warn "!!slow down~%")))
    ;;  ts)

    ;; (push
    ;;  (instance
    ;;   subsumption-task :init :name "Left Red object" :priority 35
    ;;   :check-func #'(lambda (s)
    ;;     	      (let ((red (cdr (assoc :red  s))))
    ;;     		(and red (> (aref red 0) 170))))
    ;;   :action-func
    ;;   #'(lambda ()
    ;;       (send *ri* :go-velocity 0.15 0 -10)
    ;;       (warn "!!turn left VISION~%")
    ;;       (unix:usleep (* 1000 1000))))
    ;;  ts)

    ;; (push
    ;;  (instance
    ;;   subsumption-task :init :name "Right Red object" :priority 30
    ;;   :check-func #'(lambda (s)
    ;;     	      (let ((red (cdr (assoc :red  s))))
    ;;     		(and red (< (aref red 0) 150))))
    ;;   :action-func
    ;;   #'(lambda ()
    ;;       (send *ri* :go-velocity 0.150 0 10)
    ;;       (warn "!!turn right VISION~%")
    ;;       (unix:usleep (* 1000 1000))))
    ;;  ts)

    (push
     (instance
      subsumption-task :init :name "Wander" :priority 10
      :check-func #'(lambda (s) t)
      :action-func
      #'(lambda ()
          (print "wander")
          (send *ri* :go-velocity 0.1 0 (- (random 30) 15) 500)
          ))
     ts)
    ;;
    (setq s (instance subsumption :init ts :debug nil))

    ;;ためしに腕を動かしてみる
    (send *dxl-armed-turtlebot* :tuckarm-pose)
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    
    ;;とりあえず走りはじめる
    (send *ri* :go-velocity 0.100 0 0 100)

    ;; subsumption をスタート
    (send s :start-clients)
    ;;keyを押されるまで続ける
    (do-until-key
     (send *ri* :state)
      (setq bmp-vec (send *ri* :bumper-vector))
      ;;(setq psd-vec (send *ri* :psd-vector))
      ;;(setq red-centroid (send *vs-sim* :result))
      (if red-centroid
          (setq red-centroid (cadr (assoc :centroid red-centroid))))
      ;;(format t ";; bmp ~A, psd ~A, red ~A~%" bmp-vec psd-vec red-centroid)
      (format t ";; bmp ~A, psd ~A, red ~A~%" bmp-vec nil nil)
      (send s :sensor-vector (list (cons :bumper bmp-vec)
				   ;;(cons :distane psd-vec)
				   ;;(cons :red red-centroid)
                                   ))
      ;;(unix:usleep (* time 1000))
      )
    ;; subsumption を止める
    (send s :stop-clients)
    
    ;;台車を止める
    (send *ri* :go-stop)
    ;;腕に指令を送る
    (send *dxl-armed-turtlebot* :tuckarm-pose)
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    )
  )

(warn "
(init-server)
(init-server 'simple-maze)
(init-server 'maze)
(demo0) ;; plan->action
(demo1) ;; forward and back..
(demo2) ;; using subsumption
")

#|
(defun test3
  (&key ((:viewer vw) (send *ri* :viewer))
        (robot (send *ri* :robot)))
;;  (unless (boundp 'camview)
    (setq camview
         (view
          :x 320 :y 240
          :viewing (send (elt (send robot :cameras) 1) :viewing)
          :viewsurface
          (instance gl::glviewsurface :create  :glcon ((send vw :viewer :viewsurface) . gl::glcon)
                    :title "daisha-view" :width 320 :height 240)
          :title "daisya view"
          ))
    ;;)
    (dotimes (i 2)
  (let* ((near gl::*perspective-near*)
         (far gl::*perspective-far*)
         (cam (elt (send robot :cameras) 1)))
    (setq gl::*perspective-near* (send cam :hither))
    (setq gl::*perspective-far* (send cam :yon))
    (send camview :viewsurface :makecurrent)
    (send cam :draw-objects camview (send vw :objects))
    (setq gl::*perspective-near* near)
    (setq gl::*perspective-far* far)
    ;;
    (send vw :viewer :viewsurface :makecurrent)
    (send vw :draw-objects)
    ))
  )
|#