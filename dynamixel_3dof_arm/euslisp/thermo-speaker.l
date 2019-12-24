(ros::load-ros-manifest "dynamixel_msgs")

(ros::roseus "thermo_speaker")
;;(load "package://pr2eus/speak.l")
(ros::set-param "thermo_thre" 50)

(defvar *arm-dof* 7)
(setq *state-list* (make-list *arm-dof*))

(defun state-cb
  (msg)
  (setf (elt *state-list* (1- (read-from-string (string-left-trim "arm_joint" (send msg :name))))) msg))

(dotimes (i *arm-dof*)
  (ros::subscribe (format nil "/arm_j~d_controller/state" (1+ i))
                  dynamixel_msgs::JointState
                  #'state-cb))

(defun main () ;; [deg]
  (ros::rate 0.1)
  (do-until-key
   (ros::spin-once)
   (let ((thre (ros::get-param "thermo_thre")))
     (if (and thre (every #'identity *state-list*))
         (let ((mot
                (remove-if-not
                 #'(lambda (x)
                     (> (elt (send x :motor_temps) 0) thre))
                 *state-list*)))
           (when mot
             ;;(speak-jp "モータが/あ'つ'いです。")
             (warn ";; motor ~A are hot (temp ~A > ~A)~%"
                   (mapcar #'(lambda (x) (elt x 0)) (send-all mot :motor_temps))
                   (mapcar #'(lambda (x) (elt x 0)) (send-all mot :motor_ids))
                   thre)
             )))
     (ros::sleep)
     )
   ))

(main)
