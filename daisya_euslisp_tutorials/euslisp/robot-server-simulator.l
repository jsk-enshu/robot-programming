;;
;; robot-serverをsimulationするクラスの定義
;;  daisyaを仮定しているので拡張性に乏しいのが欠点
;;
(load "daisya.l")
(load "virtual_interpolator.l")
(defvar *sim*)
(defun sim-update () (send *sim* :update)) ;;timer-jobに登録するupdate関数

#|********************************************
 構築したrobot-server-simulatorをeus-serverとして
 別プロセスで起動し、それに対してsocketをconnectして
 指令を送ることで、描画更新廻りの問題を解決する
**********************************************|#
(defvar *host* "127.0.0.1")
;;world-objsは世界を記述するモデル
(defun start-robotsim-server
  (world-objs &key (port 5000) (init-coords (make-cascoords)))
  (let ()
    (when (not (boundp' *daisya*)) 
      (setq *daisya* (instance daisyaclass :init)))
    (send *daisya* :newcoords init-coords)
    (cond 
     ((not *sim*)
      (setq *sim* (instance daisya-server-simulator :init
			       *daisya* world-objs))
      (send *sim* :start)
      (unix:usleep (* 500 1000))
      (warn "start eus server~%")
      ;;eus-serverを立ち上げる
      (eus-server port :host *host*)
      )
     (t
      (send *sim* :rebuild-world world-objs)
      ))
    ))

#|***********************************************
 eus-serverで別プロセスで起動したrobot-server-simulator
 に対して同じinterfaceで指令を送るためのクラス
************************************************|#
(defvar *rs-sim* nil)
(defvar *vs-sim* nil)
(defun connect-robotsim-server
  (&key (port 5000) (host *host*))
  (setq *rs-sim* 
	(instance robotsim-server-connector :init 
					    :host host 
					    :port port)))
(defun connect-visionsim-server
  ()
  (setq *vs-sim* *rs-sim*))

(defvar *sim-lock* (sys::make-mutex-lock))
(defclass robotsim-server-connector
  :super propertied-object
  :slots
  (server-strm
   ))
(defmethod robotsim-server-connector
  (:init 
   (&key (port 5000) (host *host*))
   (setq server-strm (connect-server-until-success host port))
   self)
  ;;commandを送信
  (:send
   (&rest command)
   (let ((com "(send *sim*") ret)
     (sys::mutex
      *sim-lock*
     (finish-output server-strm) ;;最初に綺麗にする？
     (cond 
      ((not command) 
       (return-from :send)
       )
      (t
       (dolist (c command)
	 (setq com (concatenate string com " " 
				(format nil "~a" c))))
       (setq com (concatenate string com ")~%"))
       ;;(print com)
       (format server-strm com)
       ;;
       ;;ちょっと休むか
       (unix:usleep 10000)
       ;;(when (select-stream (list server-strm) 5)
       ;;(setq ret (read server-strm))
       (let ((tmp-strm (select-stream (list server-strm) 0.001)))
         (if tmp-strm (setq ret (read (car tmp-strm)))))
       ))
     ret)))
  ;;何も定義されていないmethodが呼ばれた場合、これが呼ばれる.
  ;;  このときmethod名込みで、引数とともにargsに渡される
  (:nomethod
   (&rest args)
   (if (assoc (car args) (send daisya-server-simulator :methods))
       (send* self :send args)
     (warn "Can't find method ~A ,~A~%" (car args) (cdr args)))
   )
  ;; for vision-simulator
  (:result () (send self :send ":camera-vector"))
  )

#|*********************************************
   ロボットと世界を持つsimulatorクラスの定義
    これだけだと、何もおきない
**********************************************|#
(defclass robot-server-simulator
  :super propertied-object
  :slots
  (robot         ;;robotmodelを持つ
   world-objects ;;世界を定義するbodyなどのobjectのlist
   viewer        ;;viewer
   camview       ;;camera viewer
   tick-time     ;;どのくらいの粒度でsimulationするか   [msec]
   world-bodies
   ))
(defmethod robot-server-simulator
  (:init
   (rbt wld-objs &key ((:tick-time tt) 20))
   (setq robot rbt
	 world-objects wld-objs)
   (setq tick-time tt)
   (setq viewer (make-irtviewer))
   ;;
   (setq camview
         (view
          :x 320 :y 240
          :viewing (send rbt :camera :viewing)
          :viewsurface
          (instance gl::glviewsurface :create  :glcon ((send viewer :viewer :viewsurface) . gl::glcon)
                    :title "daisha-view" :width 320 :height 240)
          :title "daisya view"
          ))

   ;;世界を構成するbodiesを格納する
   (setq world-bodies nil)
   (dolist (obj world-objects)
     (cond 
      ((derivedp obj body)
       (push obj world-bodies))
      (t
       (setq world-bodies (append (send obj :bodies) world-bodies)))
      ))
   ;;objectsを表示
   (objects (cons robot world-objects))
   self)
  ;;
  ;;simulationの世界を進めるときの処理をかく(描画更新など)
  (:update
   ()
   (let ((near gl::*perspective-near*)
         (far gl::*perspective-far*)
         (cam (send robot :camera)))
     (setq gl::*perspective-near* (send cam :hither))
     (setq gl::*perspective-far* (send cam :yon))
     (send camview :viewsurface :makecurrent)
     ;;(send cam :draw-objects camview (send viewer :objects))
     (setq gl::*perspective-near* near)
     (setq gl::*perspective-far* far)
     ;;
     (send viewer :viewer :viewsurface :makecurrent)
     (send viewer :draw-objects)
     ))
  ;;simulationをスタートする
  (:start
   ()
   ;;粒度の設定
   (setq *top-selector-interval* (* tick-time 0.001))
   (pushnew 'sim-update *timer-job*)
   (warn "simulation start!~%")
   )
  ;;simulationをストップする
  (:stop
   ()
   ;;粒度を荒く戻しておくらしい (particle.lを参考)
   (setq *top-selector-interval* 2)
   (setq *timer-job* (remove 'sim-update *timer-job*))
   (warn "simulation stop!~%")
   )
  ;;accessor
  (:robot (&rest args) (forward-message-to-all robot args))
  )

#|*********************************************
   台車のためのserver-simulator
     台車のためのarm,wheel,sensor状態を更新する中身が記述される

   <実装method>
    daisya-velocity-vector

   <未実装method>
    wheelのシミュレーションは難しいため実装していない
      どのようにwheelを動かすと、台車の位置がどうなるかは計算で求めるのは困難なため

**********************************************|#
;;wheelの制御モード(予約番号)  robot-client.lと共通
(defvar *PWM_CONTROL* 0)
(defvar *VELOCITY_CONTROL* 1)
(defvar *ANGLE_CONTROL* 2)
;;servoの補間方式(予約番号)   robot-client.lと共通
(defvar *INTERPOLATE_FINISH* 0)
(defvar *INTERPOLATE_LINEAR* 1)
(defvar *INTERPOLATE_MINJERK* 2)
;;
(defclass daisya-server-simulator
  :super robot-server-simulator
  :slots
  (
   ;;robot(daisya)の内部状態を記述する変数群を定義
   ;; for wheel
   daisya-velocity-vector   ;;(xvec[mm/sec], yvec[mm/sec], rotvec[deg/sec])
   wheel-controlmode-vector ;;車輪の制御モード
   ;; for arm
   arm-linear-interpolator  ;;腕の補間器(linear)
   arm-minjerk-interpolator ;;腕の補間器(minjerk)
   arm-current-interpolator
   ;;arm-angle-vector         ;;腕の目標角度vector
   arm-interpolation-method ;;腕の補間モード
   arm-poweron-vector       ;;腕の制御モード
   ;;
   ;; for sensor(obstacle)
   collision-count
   initial-rbt-pos
   initial-rbt-rot
   ;;
   ;; for daisya 位置制御
   daisya-linear-interpolator  ;;台車の補間器(linear)
   daisya-minjerk-interpolator ;;台車の補間器(minjerk)
   daisya-current-interpolator
   daisya-prev-posvector
   daisya-controlmode
   ;;
   ;;noise  ;;robotがどのような確率モデルで動くかとかを指定するのかな..
   ))
(defmethod daisya-server-simulator
  (:init
   (rbt wld &key ((:tick-time tt) 20))
   (send-super :init rbt wld :tick-time tt)
   (send self :wheel-reset)
   ;;
   ;;ロボットの初期位置を記憶しておく
   (setq initial-rbt-pos (copy-seq (send robot :worldpos)))
   (setq initial-rbt-rot (copy-object (send robot :worldrot)))
   ;;
   ;;armのための補完器を作る
   (setq arm-linear-interpolator
	 (instance linear-interpolator :init :sec/step (* tick-time 0.001)))
   (setq arm-minjerk-interpolaor
	 (instance minjerk-interpolator :init :sec/step (* tick-time 0.001)))
   ;;daisyaのための補間器を作る
   (setq daisya-linear-interpolator
	 (instance linear-interpolator :init :sec/step (* tick-time 0.001)))
   (setq daisya-current-interpolator daisya-linear-interpolator)
   (setq daisya-prev-posvector (float-vector 0 0 0))
   ;;台車の制御モードは1(速度), 2(位置)
   (setq daisya-controlmode 1)
   ;;
   ;;制御モードはデフォルトではOFF
   (setq arm-poweron-vector #f(0 0 0 0 0))
   ;;defaultでは腕の補完モードは線型
   (setq arm-interpolation-method *INTERPOLATE_LINEAR*)
   (setq arm-current-interpolator arm-linear-interpolator)
   ;;
   (setq collision-count 0)
   ;;   
   self)
  ;;
  ;;初期状態に戻す
  (:reset
   ()
   ;;一度シミュレーションを止める
   (send self :stop)
   ;;robotを元に戻す
   (send robot :move-to (make-cascoords :pos initial-rbt-pos 
					:rot initial-rbt-rot) :world)
   (setq collision-count 0)
   (send self :start)
   )
  ;;環境を置き換える
  (:rebuild-world
   (objs)
   (send self :stop)
   ;;初期位置情報を更新
   (setq initial-rbt-pos (copy-seq (send robot :worldpos)))
   (setq initial-rbt-rot (copy-object (send robot :worldrot)))
   ;;物体を更新
   ;;世界を構成するbodiesを格納する
   (setq world-objects objs)
   (setq world-bodies nil)
   (dolist (obj world-objects)
     (cond 
      ((derivedp obj body)
       (push obj world-bodies))
      (t
       (setq world-bodies (append (send obj :bodies) world-bodies)))
      ))
   ;;objectsを表示
   (objects (cons robot world-objects))
   ;;
   (setq collision-count 0)
   (send self :start)
   )
  ;;
  ;;timer-jobで毎回呼ばれる
  (:update
   ()
   ;;台車を動かす
   (send self :update-daisya-position)
   ;;腕を動かす
   (send self :update-arm-angle)
   ;;sensorをシミュレーション(衝突判定、距離計算)する
   (send self :update-sensor-value)
   ;;描画更新
   (send-super :update)
   )
  ;;
  ;;更新関数の定義
  ;;
  ;;台車の位置をwheel-velocity-vectorに従って更新
  (:update-daisya-position
   ()
   (let* ((diff-x (* (aref wheel-velocity-vector 0) tick-time 0.001))
	  (diff-y (* (aref wheel-velocity-vector 1) tick-time 0.001))
	  (diff-rot (* (aref wheel-velocity-vector 2) tick-time 0.001))
          tmp-angle-vector
	  )
     (cond
      ;;台車位置制御の場合
      ((= daisya-controlmode 2)
       ;;現在の補間器の時間を進めて、送る角度をもとめてrobotに送る
       (setq tmp-angle-vector
             (send daisya-current-interpolator :pass-time (send daisya-current-interpolator :sec/step)))
       ;;補間が終了していたらnilが却ってくる
       (when tmp-angle-vector
         ;;wheel-velocity-vectorを更新しておかないと、ぶつかったときにとまらない
         (dotimes (i 3)
           (setf (aref wheel-velocity-vector i)
                 (/ (- (aref tmp-angle-vector i) (aref daisya-prev-posvector i)) (* tick-time 0.001)))
           )
         (send robot :move (v- tmp-angle-vector daisya-prev-posvector))
         ;;前回の指令を覚えておく
         (dotimes (i 3)
           (setf (aref daisya-prev-posvector i) (aref tmp-angle-vector i)))
         )
       )
      ;;台車速度制御の場合
      (t
       (send robot :move (float-vector diff-x diff-y diff-rot))
       ))
     ))
  ;;腕の姿勢を更新
  (:update-arm-angle
   ()
   (let (tmp-angle-vector im)
     ;;現在の補完器の時間を進めて、送る角度をもとめてrobotに送る
     (setq tmp-angle-vector
	   (send arm-current-interpolator :pass-time (send arm-current-interpolator :sec/step)))
     ;;(print tmp-angle-vector)
     ;;補完が終了していたらnilが却ってくる
     (when tmp-angle-vector
       ;;poweron-vectorが0のものにはmaskをかけて動かさない
       (dotimes (i (length tmp-angle-vector))
	 (when (= (aref arm-poweron-vector i) 0)
	   (setf (aref tmp-angle-vector i) 0)))
       (send robot :angle-vector tmp-angle-vector))
     ))
  (:update-sensor-value
   ()
   ;;衝突判定や距離測定を行う
   (when world-bodies (send robot :simulate world-bodies))
   ;;衝突したかを調べる
   (cond 
    ;;衝突していないならば
    ((= (norm (send robot :bumper-vector)) 0)
     (setq collision-count 0))
    ;;衝突している場合
    (t
     (incf collision-count)))
   ;;
   ;;2回以上ぶつかってら台車の移動を修正する（それ以上はめりこませないため)
   (when (> collision-count 1)
     (let* ((diff-x (* -2.0 (aref wheel-velocity-vector 0) tick-time 0.001))
	    (diff-y (* -2.0 (aref wheel-velocity-vector 1) tick-time 0.001))
	    (diff-rot (* -2.0 (aref wheel-velocity-vector 2) tick-time 0.001)))
       (send robot :move (float-vector diff-x diff-y diff-rot)))
     ;;(decf collision-count)
     ;;(print collision-count)
     (setq collision-count 0)
     )
   )
  ;;
  ;;台車へのinterface (これは、robot-clientと全く同じにする必要あり)
  ;; <for wheel>
  ;; 
  ;; 車輪制御モードの変更 wheel-controlmode-vector
  ;;   PWM制御:0, 速度制御:1, 長さ制御: 2, 制御OFF:-1
  ;;   Ex. (send *rs* :wheel-controlmode-vector #f(1 1 1)) 
  (:wheel-controlmode-vector
   (ctrl-vector)
   (setq wheel-controlmode-vector ctrl-vector)
   nil)
  (:wheel-angle-vector
   (angle-vector &optional (time 1000))
   (warn "Sorry, no implementation in robot-server-simulation~%"))
  (:wheel-velocity-vector
   (vel-vector)
   (warn "Sorry, no implementation in robot-server-simulation~%"))
  ;;初期状態(servo-off状態, encoder=0, 各種目標値0)に戻す
  (:wheel-reset 
   ()
   (setq wheel-controlmode-vector (float-vector *VELOCITY_CONTROL* *VELOCITY_CONTROL* *VELOCITY_CONTROL*))
   (setq wheel-velocity-vector #f(0 0 0))
   )
  ;;台車に目標xy速度、目標角速度[deg]/secを与える
  (:daisya-velocity-vector 
   (daisyavel-vector)
   (cond 
    ;;車輪が速度制御になっているときのみ反映するようにする
    ((and (= (aref wheel-controlmode-vector 0) *VELOCITY_CONTROL*)
	  (= (aref wheel-controlmode-vector 1) *VELOCITY_CONTROL*)
	  (= (aref wheel-controlmode-vector 2) *VELOCITY_CONTROL*))
     (setq wheel-velocity-vector daisyavel-vector))
    ;;それ以外の場合はエラーを返す
    (t
     (warn "daisya-velocity-vector can be done with all wheels under velocity control~%")
     )))
  ;;台車の制御モードを変更する
  (:daisya-controlmode
   (cm)
   (setq daisya-controlmode cm)
   )
  ;;台車の位置制御指令を送る
  (:daisya-position-vector
   (daisyapos-vector &optional (time 1000))
   (dotimes (i 3) (setf (aref daisya-prev-posvector i) 0))
   ;;現在の補間器に投げる
   (send daisya-current-interpolator
	 :reset
	 :angle-vector-list
	 (list (float-vector 0 0 0) daisyapos-vector)
	 :step-list
	 (list (/ time tick-time) (/ time tick-time)))
   (send daisya-current-interpolator :start-interpolation)
   nil)
  ;;
  ;; for arm(servo)
  ;;
  ;;(send *rs* :arm-poweron-vector #f(0 0 0 0 0))でOFF
  ;;(send *rs* :arm-poweron-vector #f(1 1 1 1 1))でON
  (:arm-poweron-vector
   (armpon-vector)
   (setq arm-poweron-vector armpon-vector)
   nil)
  (:arm-interpolation-method
   (im)
   (setq arm-interpolation-method im)
   (print arm-interpolation-method)
   (cond 
    ((= arm-interpolation-method *INTERPOLATE_LINEAR* )
     (warn "change linear~%")
     (setq arm-current-interpolator arm-linear-interpolator))
    ((= arm-interpolation-method *INTERPOLATE_MINJERK* )
     (warn "change minjerk~%")
     (setq arm-current-interpolator arm-minjerk-interpolator))
    (t
     (warn "no such interpolation method~%")))
   nil)
  (:arm-angle-vector
   (angle-vector &optional (time 1000))
   ;;現在の補間器に投げる
   (send arm-current-interpolator 
	 :reset
	 :angle-vector-list 
	 (list (send robot :angle-vector) angle-vector)
	 :step-list
	 (list (/ time tick-time) (/ time tick-time)))
   (send arm-current-interpolator :start-interpolation)
   nil)
  ;;
  ;; robotstateの取得 (および各種センサのaccessor)
  (:get-robot-state
   nil
   (warn "Sorry, no implementation in robot-server-simulation~%"))
  (:get-wheels-angle 
   () 
   (warn "Sorry, no implementation in robot-server-simulation~%"))
  (:get-wheels-controlmode () wheel-controlmode-vector)
  (:bumper-vector 
   () 
   (send robot :bumper-vector))
  (:psd-vector 
   () 
   (send robot :psd-vector))
  (:camera-vector
   ()
   (send robot :camera-vector))
  )
