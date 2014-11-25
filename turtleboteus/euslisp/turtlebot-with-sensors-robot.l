(require "package://turtleboteus/models/kobuki_hexagons_kinect.l")

(defclass turtlebot-with-sensors-robot
  :super turtlebot-robot
  :slots (sensors)
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
           (push (instance bumper-model :init bb :name (read-from-string (format nil ":~A" (string-right-trim "_" (string-right-trim "link" (send (cdr b) :name)))))) sensors))))
     (let ((acamera (send self :camera_rgb_optical_frame_lk)))
       (let ((bb (make-cube 10 10 10)))
         (send bb :newcoords (send acamera :copy-worldcoords))
         (push (instance camera-model :init bb :name :camera_rgb_optical) sensors)))
     (let ((acamera (send self :camera_depth_frame_lk)))
       (let ((bb (make-cube 10 10 10)))
         (send bb :newcoords (send acamera :copy-worldcoords))
         (push (instance camera-model :init bb :name :camera_depth) sensors)))
     (dolist (s sensors)
       (eval `(defmethod ,(send (class self) :name)
                (,(send s :name) (&rest args) ,s)))
       (send self :assoc s))
     (dolist (sen '(:bumper-sensor :camera-sensor))
       (send self :define-get-sensor-method sen))
     t))
  (:simulate
   (objs)
   (send-all sensors :simulate
             (flatten (mapcar #'(lambda (obj) (if (find-method obj :bodies) (send obj :bodies) obj)) objs))))
  (:bumper-sensors
   (&rest args)
   (forward-message-to-all
    (remove-if-not #'(lambda (x) (derivedp x bumper-model)) sensors)
    args))
  (:camera-sensors
   (&rest args)
   (forward-message-to-all
    (remove-if-not #'(lambda (x) (derivedp x camera-model)) sensors)
    args))
  ;; alias
  (:cameras (&rest args) (send* self :camera-sensors args))
  (:camera (&rest args) (send* self :camera-sensor args))
  )

(defun turtlebot-with-sensors-robot () (setq *turtlebot* (instance turtlebot-with-sensors-robot :init)))