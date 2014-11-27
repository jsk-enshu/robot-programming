;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; turtlebotの台車ロボット
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; turtlebotのロボットモデルクラス (urdfから自動変換して生成)
(require "package://turtleboteus/models/kobuki_hexagons_kinect.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 台車ロボットモデルクラス
;;   Euslispでセンサ等を追加
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass turtlebot-with-sensors-robot
  :super turtlebot-robot
  :slots (sensors bumper-sensors)
  )

(defmethod turtlebot-with-sensors-robot
  (:init
   (&rest args
    &key (name "turtlebot-with-sensors-robot"))
   (prog1
       (send-super* :init :name name args)
     ;; add sensor
     (let ((bumper-links
            ;; bumper is named as "cliff"
            (remove-if-not #'(lambda (x) (let ((nm (string-downcase (car x)))) (and (substringp "link" nm) (substringp "cliff" nm))))
                           (send self :slots))))
       (dolist (b bumper-links)
         (let ((bb (make-cylinder 25 50)))
           (send bb :newcoords (send (cdr b) :copy-worldcoords))
           (send bb :rotate -pi/2 :y)
           (push (instance bumper-model :init bb :name (read-from-string (format nil ":~A" (string-right-trim "_" (string-right-trim "link" (send (cdr b) :name)))))) bumper-sensors)
           )))
     (let ((acamera (send self :camera_rgb_optical_frame_lk)))
       (let ((bb (make-cube 10 10 10)))
         (send bb :newcoords (send acamera :copy-worldcoords))
         (push (instance camera-model :init bb :name :camera_rgb_optical :view-up (send bb :rotate-vector (float-vector 0 1 0))) cameras)
         ))
     (let ((acamera (send self :camera_depth_frame_lk)))
       (let ((bb (make-cube 10 10 10)))
         (send bb :newcoords (send acamera :copy-worldcoords))
         (push (instance camera-model :init bb :name :camera_depth) cameras)
         ))
     (setq sensors (append cameras bumper-sensors))
     (dolist (s sensors)
       (eval `(defmethod ,(send (class self) :name)
                (,(send s :name) (&rest args) ,s)))
       (send self :assoc s))
     t))
  (:simulate
   (objs)
   (send-all sensors :simulate
             (flatten (mapcar #'(lambda (obj) (if (find-method obj :bodies) (send obj :bodies) obj)) objs))))
  ;; sensor accessor
  (:bumper-sensors () "Returns bumper sensors." bumper-sensors)
  (:bumper-sensor (sensor-name) "Returns bumper sensor of given name" (send self :get-sensor-method :bumper-sensor sensor-name))
  )

(defun turtlebot-with-sensors
  ()
  "Generation function for turtlebot-with-sensors-robot."
  (instance turtlebot-with-sensors-robot :init))