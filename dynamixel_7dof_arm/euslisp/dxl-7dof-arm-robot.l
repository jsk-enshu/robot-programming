;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Dynamixel AX-12Aモータによる7DOF アームロボット
;;   7dof = 6dof arm + 1dof gripper
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; body生成関数
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Dynamixel AX-12A
;;   http://www.besttechnology.co.jp/modules/knowledge/?BTX030B%20Dynamixel%20AX-12A
(defun make-dynamixel-AX-12A-motor-body ()
  (let ((b1 (make-cube 26 11.5 32))
        (b2 (make-cube 32 38.5 32))
        (b3 (make-cylinder (/ 22 2.0) 5))
        (b4 (make-cylinder (/ 10 2.0) 3))
        )
    (send b1 :translate (float-vector 0 (/ 11.5 2.0) 0))
    (send b2 :translate (float-vector 0 (/ 38.5 -2.0) 0))
    (send b3 :translate (float-vector 0 0 (/ 32 2.0)))
    (send b4 :translate (float-vector 0 0 (- (+ (/ 32 2.0) 3))))
    (let ((b (body+ b1 b2 b3 b4)))
      (send b :set-color :gray10)
      (send b :put :attach-coords (make-cascoords :pos (float-vector 0 -13.5 -16)))
      (send b :assoc (send b :get :attach-coords))
      (send b :put :joint-coords (make-cascoords :pos (float-vector 0 0 (+ 16 5))))
      (send b :assoc (send b :get :joint-coords))
      b)))

;; Small frame parts for Dynamixel AX-12A
;;  F3 in http://www.besttechnology.co.jp/modules/knowledge/?BTX030B%20Dynamixel%20AX-12A
(defun make-dynamixel-AX-12A-frame1-body ()
  (let ((b1 (make-cube 25 32 4))
        (b2 (make-cube 25 3.1 9))
        (b3 (make-cube 25 3.1 9)))
    (send b1 :translate (float-vector 0 0 3.5))
    (send b2 :translate (float-vector 0 17.55 1.0))
    (send b3 :translate (float-vector 0 -17.55 1.0))
    (let ((b (body+ b1 b2 b3)))
      (send b :set-color :slategray)
      (send b :put :attach-coords (make-cascoords :pos (float-vector 0 0 5.5)))
      (send b :assoc (send b :get :attach-coords))
      b)))

;; Large frame parts for Dynamixel AX-12A
;;  F2 in http://www.besttechnology.co.jp/modules/knowledge/?BTX030B%20Dynamixel%20AX-12A
(defun make-dynamixel-AX-12A-frame2-body ()
  (let ((b1 (make-cube 24.8 41.8 7))
        (b2 (make-cube 24.8 3.2 37.5))
        (b3 (make-cube 24.8 3.2 37.5)))
    (send b1 :translate (float-vector 0 0 23))
    (send b2 :translate (float-vector 0 22.5 7.75))
    (send b3 :translate (float-vector 0 -22.5 7.75))
    (let ((b (body+ b1 b2 b3)))
      (send b :set-color :slategray)
      (send b :put :attach-coords (make-cascoords :pos (float-vector 0 0 26.5) :rpy (list 0 pi 0)))
      (send b :assoc (send b :get :attach-coords))
      b)))

;; gripper parts
(defun make-dxl-7dof-arm-gripper-body ()
  (let ((b1 (make-prism
             (list (float-vector 0 80 0)
                   (float-vector 5 80 0)
                   (float-vector 15 35 0)
                   (float-vector 15 0 0)
                   (float-vector 0 0 0)
                   )
             40))
        (b2 (make-cube 50 100 38)))
    (send b2 :translate (float-vector (+ 25 1) (- 50 1) (+ 1 (/ 38 2.0))))
    (let ((b (body- b1 b2)))
      (send b :translate-vertices (float-vector 0 -13 -20))
      (send b :set-color :silver)
      b)))

;; base parts
(defun make-dxl-7dof-arm-base-body ()
  (let ((b1 (make-cube 41 30 5))
        (b2 (make-cube 5 30 40)))
    (send b1 :translate (float-vector -7.5 0 2.5))
    (send b2 :translate (float-vector -25.5 0 20))
    (let ((b (body+ b1 b2)))
      (send b :set-color :silver)
      (send b :put :attach-coords (make-cascoords :pos (float-vector -28 0 21.5) :rpy (list 0 pi/2 0)))
      (send b :assoc (send b :get :attach-coords))
      b)))

;; motor+frame1
(defun make-dynamixel-AX-12A-motor-unit-bodyset
  (&key (use-frame1 (list :bottom :left)))
  (let* ((motor-body (make-dynamixel-AX-12A-motor-body))
        (frame1-bodies
         (mapcar #'(lambda (x)
                     (let ((b (make-dynamixel-AX-12A-frame1-body)))
                       (case x
                         (:bottom
                          (send b :name :bottom)
                          (send b :rotate pi/2 :x)
                          (send b :translate (float-vector 0 0 36)))
                         (:left
                          (send b :name :left)
                          (send b :rotate pi/2 :x)
                          (send b :rotate -pi/2 :y)
                          (send b :translate (float-vector (/ 27 -2.0) -14.5 0) :world))
                         (:right
                          (send b :name :right)
                          (send b :rotate pi/2 :x)
                          (send b :rotate pi/2 :y)
                          (send b :translate (float-vector (/ 27 2.0) -14.5 0) :world))
                         (t ))
                       (list x b)))
                 use-frame1))
        (unit-bodies
         (append (list (list :motor motor-body))
                 frame1-bodies)))
    (send motor-body :name :motor)
    (dolist (b (cdr unit-bodies))
      (send (cadr (car unit-bodies)) :assoc (cadr b)))
    (let ((b (instance bodyset :init (make-cascoords)
                       :bodies (mapcar #'cadr unit-bodies))))
      (send b :put :joint-coords (make-cascoords :coords (send (send (cadr (car unit-bodies)) :get :joint-coords) :copy-worldcoords)))
      (send b :assoc (send b :get :joint-coords))
      (dolist (bb unit-bodies)
        (send b :put (read-from-string (format nil "~A-attach-coords" (car bb)))
              (send (cadr bb) :get :attach-coords)))
      b)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; リンク生成関数
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; root-link : base+frame1-bottom+motor
(defun make-dxl-7dof-arm-root-link ()
  (let ((b1s (make-dynamixel-AX-12A-motor-unit-bodyset :use-frame1 (list :bottom)))
        (b2 (make-dxl-7dof-arm-base-body)))
    (send b2 :move-coords
          (send b1s :get :bottom-attach-coords) (send b2 :copy-worldcoords))
    (send b1s :rotate -pi/2 :y)
    (let ((bs (append (send b1s :bodies) (list b2))))
      (dolist (b (cdr bs)) (send (car bs) :assoc b))
      (send (car bs) :rotate pi/2 :y)
      (send (car bs) :rotate pi/2 :z)
      (send (car bs) :rotate pi :x)
      (send (car bs) :translate
            (v- (send (car bs) :worldpos) (send (send b2 :get :attach-coords) :worldpos))
            :world)
      (let ((bl (instance bodyset-link :init (make-cascoords)
                          :bodies bs :name :root-link)))
        (send bl :put :joint-coords
              (make-cascoords :coords (send (send (send (car bs) :copy-worldcoords) :rotate pi :y) :rotate pi/2 :z)))
        (send bl :assoc (send bl :get :joint-coords))
        bl))))

;; link1 : frame2
(defun make-dxl-7dof-arm-link1 ()
  (let ((b (make-dynamixel-AX-12A-frame2-body)))
    (send b :rotate -pi/2 :x)
    (send b :rotate -pi/2 :z :world)
    (let ((bl (instance bodyset-link :init (make-cascoords)
                        :bodies (list b) :name :arm-link1)))
      (send bl :put :joint-coords
            (make-cascoords :coords (send (send (send (send b :get :attach-coords) :copy-worldcoords) :rotate pi/2 :y) :rotate pi/2 :x :world)))
      (send bl :assoc (send bl :get :joint-coords))
      bl)))

;; link2 : motor+frame1bottom+frame1left+motor
(defun make-dxl-7dof-arm-link2 ()
  (let ((b1s (make-dynamixel-AX-12A-motor-unit-bodyset :use-frame1 (list :bottom)))
        (b2s (make-dynamixel-AX-12A-motor-unit-bodyset :use-frame1 (list :left))))
    (send b2s :move-coords
          (send (send (send (send b1s :get :bottom-attach-coords) :copy-worldcoords) :rotate pi :x) :rotate -pi/2 :z)
          (send b2s :get :left-attach-coords))
    (let ((bs (append (send b1s :bodies) (send b2s :bodies))))
      (dolist (b (cdr bs)) (send (car bs) :assoc b))
      (send (car bs) :rotate -pi/2 :y)
      (send (car bs) :rotate -pi/2 :x :world)
      (send (car bs) :translate (v- (send (car bs) :worldpos) (send (send (car bs) :get :joint-coords) :worldpos)) :world)
      (let ((bl (instance bodyset-link :init (make-cascoords)
                          :bodies bs :name :arm-link2)))
        (send bl :put :joint-coords
              (make-cascoords :coords (send (send (send (send (elt bs 2) :copy-worldcoords) :rotate -pi/2 :x) :rotate -pi/2 :y :world) :rotate pi :z :world)))
        (send bl :assoc (send bl :get :joint-coords))
        bl))))

;; link3 : frame2-0+offset+motor
(defun make-dxl-7dof-arm-link3 ()
  (let ((b1 (make-dynamixel-AX-12A-motor-body))
        (b2 (make-dynamixel-AX-12A-frame2-body)))
    (send b2 :move-coords
          (send b1 :get :attach-coords)
          (send (send (send (send b2 :get :attach-coords) :rotate pi :x) :rotate pi/2 :z)
                :translate (float-vector 0 0 8)))
    (let ((bs (list b1 b2)))
      (dolist (b (cdr bs)) (send (car bs) :assoc b))
      (send (car bs) :rotate pi/2 :y)
      (send (car bs) :rotate pi/2 :x :world)
      (send b1 :translate (v- (send b1 :worldpos) (send b2 :worldpos)) :world)
      (let ((bl (instance bodyset-link :init (make-cascoords)
                          :bodies bs :name :arm-link3)))
        (send bl :put :joint-coords
              (make-cascoords
               :coords
               (send (send (send (send (car bs) :get :joint-coords) :copy-worldcoords) :rotate -pi/2 :x) :rotate -pi/2 :z)))
        (send bl :assoc (send bl :get :joint-coords))
        bl))))

;; link4 : frame2
(defun make-dxl-7dof-arm-link4 ()
  (let ((b (make-dynamixel-AX-12A-frame2-body)))
    (send b :rotate -pi/2 :y)
    (send b :translate (v- (send b :worldpos) (send (send b :get :attach-coords) :worldpos)) :world)
    (let ((bl (instance bodyset-link :init (make-cascoords)
                        :bodies (list b) :name :arm-link4)))
      (send bl :put :joint-coords
            (make-cascoords :coords (send (send b :copy-worldcoords) :rotate pi/2 :y)))
      (send bl :assoc (send bl :get :joint-coords))
      bl)))

;; link5 : motor+frame1-bottom
(defun make-dxl-7dof-arm-link5 ()
  (let ((b1s (make-dynamixel-AX-12A-motor-unit-bodyset :use-frame1 (list :bottom))))
    (let ((bs (send b1s :bodies)))
      (dolist (b (cdr bs)) (send (car bs) :assoc b))
      (send (car bs) :rotate -pi/2 :x)
      (send (car bs) :rotate pi/2 :y :world)
      (let ((bl (instance bodyset-link :init (make-cascoords)
                          :bodies bs :name :arm-link5)))
        (send bl :put :joint-coords
              (make-cascoords :coords (send (send (send b1s :get :bottom-attach-coords) :copy-worldcoords) :rotate -pi/2 :y)))
        (send bl :assoc (send bl :get :joint-coords))
        bl))))

;; link6 : motor+offset+frame1-left+motor+frame1-bottom+gripper
(defun make-dxl-7dof-arm-link6 ()
  (let ((b1 (make-dynamixel-AX-12A-motor-body))
        (b2s (make-dynamixel-AX-12A-motor-unit-bodyset :use-frame1 (list :left :bottom)))
        (b3 (make-dxl-7dof-arm-gripper-body)))
    (send b2s :move-coords
          (send b1 :get :attach-coords)
          (send (send (send b2s :get :left-attach-coords) :copy-worldcoords) :translate (float-vector 0 0 7)))
    (send b3 :move-coords
          (send (send (send b2s :get :bottom-attach-coords) :rotate -pi/2 :y :world) :rotate -pi/2 :x :world)
          (send b3 :copy-worldcoords))
    (let ((bs (append (list b1) (send b2s :bodies) (list b3))))
      (dolist (b (cdr bs)) (send (car bs) :assoc b))
      (send (car bs) :rotate -pi/2 :y)
      (send (car bs) :rotate pi/2 :x :world)
      (send (car bs) :translate (v- (send b1 :worldpos) (send (send b1 :get :joint-coords) :worldpos))
            :world)
      (let ((bl (instance bodyset-link :init (make-cascoords)
                          :bodies bs :name :arm-link6)))
        (send bl :put :joint-coords
              (make-cascoords :coords (send (send (elt bs 1) :copy-worldcoords) :rotate 0 :x)))
        (send bl :assoc (send bl :get :joint-coords))
        bl))))

;; link7 : frame2+gripper
(defun make-dxl-7dof-arm-link7 ()
  (let ((b1 (make-dynamixel-AX-12A-frame2-body))
        (b2 (make-dxl-7dof-arm-gripper-body)))
    (send b2 :move-coords
          (send b1 :get :attach-coords)
          (send (send (send b2 :copy-worldcoords) :rotate -pi/2 :y) :rotate pi/2 :z))
    (let ((bs (list b1 b2)))
      (dolist (b (cdr bs)) (send (car bs) :assoc b))
      (send (car bs) :rotate pi :z)
      (send (car bs) :rotate -pi/2 :x :world)
      (instance bodyset-link :init (make-cascoords)
                :bodies bs :name :arm-link7)
      )))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 7DOF アームロボットモデルクラス
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass dxl-7dof-arm-robot
  :super robot-model
  :slots (jc0 jc1 jc2 jc3 jc4 jc5 jc6)) ;; 関節インスタンス

(defmethod dxl-7dof-arm-robot
  (:init
   (&rest args
    &key (name "dxl-7dof-arm-robot"))
   (send-super* :init :name name args)
   ;; 1. make links links and assoc all links
   (let ((aroot-link (send self :make-root-link)))
     (setq rarm (send self :make-arm-links))
     (send (car rarm) :move-coords (send aroot-link :get :joint-coords) (send (car rarm) :copy-worldcoords))
     ;; 2. assoc links
     ;;    Root link should be associated with "self".
     (send self :assoc aroot-link)
     (send aroot-link :assoc (car rarm))

     ;; 3. make all joints
     ;;    Before making joints, you should :assoc all links.
     (let (
           ;; http://www.besttechnology.co.jp/modules/knowledge/?BTX030B%20Dynamixel%20AX-12A
           (max-joint-velocity (/ (* 59 2pi) 60.0)) ;; [rad/s]
           (max-joint-torque 12.0)) ;; [Nm]
       (setq jc0 (instance rotational-joint :init :parent-link aroot-link :child-link (elt rarm 0) :name "arm_joint1" :axis :-z :min -90 :max 90 :max-joint-velocity max-joint-velocity :max-joint-torque max-joint-torque))
       (setq jc1 (instance rotational-joint :init :parent-link (elt rarm 0) :child-link (elt rarm 1) :name "arm_joint2" :axis :x :min -150 :max 150 :max-joint-velocity max-joint-velocity :max-joint-torque max-joint-torque))
       (setq jc2 (instance rotational-joint :init :parent-link (elt rarm 1) :child-link (elt rarm 2) :name "arm_joint3" :axis :y :min -90 :max 45 :max-joint-velocity max-joint-velocity :max-joint-torque max-joint-torque))
       (setq jc3 (instance rotational-joint :init :parent-link (elt rarm 2) :child-link (elt rarm 3) :name "arm_joint4" :axis :x :min -180 :max 180 :max-joint-velocity max-joint-velocity :max-joint-torque max-joint-torque))
       (setq jc4 (instance rotational-joint :init :parent-link (elt rarm 3) :child-link (elt rarm 4) :name "arm_joint5" :axis :y :min -90 :max 90 :max-joint-velocity max-joint-velocity :max-joint-torque max-joint-torque))
       (setq jc5 (instance rotational-joint :init :parent-link (elt rarm 4) :child-link (elt rarm 5) :name "arm_joint6" :axis :x :min -180 :max 180 :max-joint-velocity max-joint-velocity :max-joint-torque max-joint-torque))
       (setq jc6 (instance rotational-joint :init :parent-link (elt rarm 5) :child-link (elt rarm 6) :name "arm_joint7" :axis :z :min -55 :max 50 :max-joint-velocity max-joint-velocity :max-joint-torque max-joint-torque))
       )

     ;; 4. define slots for robot class
     ;;    links and joint-list for cascaded-link.
     (setq links (append (list aroot-link) rarm))
     (setq joint-list (list jc0 jc1 jc2 jc3 jc4 jc5 jc6))
     ;;    These are for robot-model.
     (setq rarm-root-link (car rarm))
     ;;    end-coords
     (setq rarm-end-coords (make-cascoords :coords
                                           (send (send (elt rarm 5) :copy-worldcoords) :translate (float-vector 100 0 0))))
     (send (elt rarm 5) :assoc rarm-end-coords)

     ;; 5. call :init-ending after defining links and joint-list and return "self"
     (send self :init-ending)
     self))
  ;; links
  (:make-root-link
   ()
   (make-dxl-7dof-arm-root-link))
  (:make-arm-links
   ()
   (let ((ln
          (list (make-dxl-7dof-arm-link1)
                (make-dxl-7dof-arm-link2)
                (make-dxl-7dof-arm-link3)
                (make-dxl-7dof-arm-link4)
                (make-dxl-7dof-arm-link5)
                (make-dxl-7dof-arm-link6)
                (make-dxl-7dof-arm-link7))))
     (dotimes (i (1- (length ln)))
       (send (elt ln (1+ i)) :move-coords
             (send (elt ln i) :get :joint-coords)
             (send (elt ln (1+ i)) :copy-worldcoords)))
     (dotimes (i (1- (length ln)))
       (send (elt ln i) :assoc (elt ln (1+ i))))
     ln))
  ;; joints
  (:arm_joint1 () jc0)
  (:arm_joint2 () jc1)
  (:arm_joint3 () jc2)
  (:arm_joint4 () jc3)
  (:arm_joint5 () jc4)
  (:arm_joint6 () jc5)
  (:arm_joint7 () jc6)
  ;; limbs
  (:arm (&rest args)
        "Accessor for arm methods."
        (unless args (setq args (list nil))) (send* self :limb :rarm args))
  ;; poses
  (:reset-pose
   ()
   "Reset pose."
   (send self :angle-vector (float-vector 0.0 0.0 -90.0 0.0 90.0 0.0 0.0)))
  (:reset-pose2
   ()
   "Reset pose2."
   (send self :angle-vector (float-vector 0.0 150.0 -90.0 0.0 90.0 0.0 0.0)))
  (:tuckarm-pose
   ()
   "Folding arm pose."
   (send self :angle-vector (float-vector 0.0 150.0 -90.0 0.0 0.0 0.0 0.0)))
  (:tuckarm-pose2
   ()
   "Folding arm pose2."
   (send self :angle-vector (float-vector 90.0 90.0 5.0 0.0 -90.0 90.0 0.0)))
  (:tuckarm-pose3
   ()
   "Folding arm pose3."
   (send self :angle-vector (float-vector 0.0 90.0 0.0 -20.0 90.0 0.0 0.0)))
  ;; inverse-kinematics
  (:inverse-kinematics
   (target-coords &rest args &key (link-list)
                  (move-target) (stop 300)
                  (use-base nil) (start-coords (send self :copy-worldcoords))
                  (thre (cond
                         ((atom target-coords) 10)
                         (t (make-list (length target-coords) :initial-element 10))))
                  (rthre (cond
                         ((atom target-coords) (deg2rad 5))
                         (t (make-list (length target-coords) :initial-element (deg2rad 5)))))
                  (base-range (list :min #f(-30 -30) :max #f( 30  30)))
                  &allow-other-keys)
   "Inverse kinemaitcs method for arm robot."
   (let (diff-pos-rot)
     (unless move-target
       (setq move-target (send self :arm :end-coords)))
     (unless link-list
       (setq link-list (send self :link-list (send move-target :parent))))

     ;; use base
     (cond
      (use-base
       (setq diff-pos-rot
             (concatenate float-vector
                          (send start-coords :difference-position self)
                          (send start-coords :difference-rotation self)))
       (send self :move-to start-coords :world)
       (with-append-root-joint
        (ll self link-list
            :joint-class wheel-joint
            :joint-args base-range)
        (send (caar ll) :joint :joint-angle
              (float-vector (elt diff-pos-rot 0)
                            (rad2deg (elt diff-pos-rot 5))))
        (send-super* :inverse-kinematics target-coords
                     :rthre rthre
                     :thre thre
                     :stop stop
                     :additional-weight-list
                     (list
                      (list (car (send self :links))
                            (if (eq use-base t) 1 use-base))
                      )
                     :link-list (car ll) ;; link-list
                     :move-target move-target
                    args)))
      (t
       (send-super* :inverse-kinematics target-coords
                    :rthre rthre
                    :thre thre
                    :stop stop
                    :link-list link-list
                    :move-target move-target
                    args))
       )))
  )

;; 7dofアームモデル生成関数
(defun dxl-7dof-arm
  ()
  "Generation function for dxl-7dof-arm-robot."
  (instance dxl-7dof-arm-robot :init))

#|
(defun init-model-viewer ()
  (setq *dxl-7dof-arm* (dxl-7dof-arm))
  (objects (list *dxl-7dof-arm*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  t)

(defun test-arm-fk (&key (div 50))
  (dolist (j (send *dxl-7dof-arm* :joint-list))
    (send *dxl-7dof-arm* :init-pose)
    (dotimes (i (1+ div))
      (let ((dja (/ (- (send j :max-angle) (send j :min-angle)) (float div))))
        (send j :joint-angle (+ (send j :min-angle) (* i dja)))
        (send *irtviewer* :draw-objects)
        (unix:usleep 5000)
        )))
  )

(defun ik-demo0
  (&key (step 10)        ;;stepは一回のループで目標を動かす距離
        (use-wheel nil)  ;;車輪を使ってIKを解くかどうか
        )
  ;;逆運動学が解きやすい初期姿勢に変更
  (send *dxl-7dof-arm* :reset-pose)
  (send *irtviewer* :draw-objects)
  (when (boundp '*ri*)
    (send *ri* :angle-vector (send *dxl-7dof-arm* :angle-vector) 5000)
    (send *ri* :wait-interpolation))
  ;;
  ;;'e'を押すまで続ける
  (warning-message 2 ";; if stop, then enter e~%")
  (warning-message 2 ";;  h:left, j:down, k:up, l:right, f:forward, b:back~%")
  (let (w goal-endcoords ll)
    ;;もし腕しか使わない場合はlinklistをあらかじめ用意しておく
    (when (not use-wheel)
      (setq ll 
            (send *dxl-7dof-arm* :link-list
                  (send *dxl-7dof-arm* :arm :end-coords :parent) ;;ここまで
                  )))
    ;;目標座標を作成する(デフォルトは、台車の手先位置と同じにする)
    (setq goal-endcoords
          (make-cascoords :pos (send *dxl-7dof-arm* :arm :end-coords :worldpos)))
    ;;ループを回す
    (while t
      (setq w (read-line)) ;;文字を取得
      ;;文字によって操作を変える
      (cond
       ((equal w "e")
        (return-from nil)) ;;loopから抜けて終了
       ((equal w "h")  ;;左へ動かす
        (send goal-endcoords :locate (float-vector 0 step 0)))
       ((equal w "j")  ;;下へ動かす
        (send goal-endcoords :locate (float-vector 0 0 (* -1 step))))
       ((equal w "k")  ;;上へ動かす
        (send goal-endcoords :locate (float-vector 0 0 step)))
       ((equal w "l")  ;;右へ動かす
        (send goal-endcoords :locate (float-vector 0 (* -1 step) 0)))
       ((equal w "f")  ;;前へ動かす
        (send goal-endcoords :locate (float-vector step 0 0)))
       ((equal w "b")  ;;後へ動かす
        (send goal-endcoords :locate (float-vector (* -1 step) 0 0)))
       ((not w)) ;;何も入れられなければ何もしない
       (t
        (warning-message 2 ";; no such command~%")
        (warning-message 2 ";; if stop, then enter e~%")
        (warning-message 2 ";;  h:left, j:down, k:up, l:right, f:forward, b:back~%")
        ))
      ;;目標値end-coordsに向かって逆運動学を解いて、動かす
      ;;  solve-ikという逆運動学をとくmethodを呼び出す。
      (cond
       (use-wheel ;;車輪を使う場合
        (send *dxl-7dof-arm* :inverse-kinematics goal-endcoords :rotation-axis nil :debug-view nil))
       (t
        (send *dxl-7dof-arm* :inverse-kinematics goal-endcoords :rotation-axis nil :debug-view nil :link-list ll :move-target (send *dxl-7dof-arm* :arm :end-coords)))
       )
      (send *irtviewer* :objects (list *dxl-7dof-arm* goal-endcoords))
      (send *irtviewer* :draw-objects)
      (when (boundp '*ri*)
        (send *ri* :angle-vector (send *dxl-7dof-arm* :angle-vector) 500)
        (send *ri* :wait-interpolation))
      ))
  (warn ";; finished~%")
  )

;; old version functions

;; version 1
|#
