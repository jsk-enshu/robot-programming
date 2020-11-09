(require :robot-interface "package://pr2eus/robot-interface.l")
(load "package://cart_humanoid/euslisp/jaxon_red.l")

(defclass cart_humanoid-interface
  :super robot-interface
  :slots ())
(defmethod cart_humanoid-interface
  (:init
   (&rest args &key ((:controller-timeout ct) nil))
   (prog1
       (send-super* :init :robot *cart_humanoid-class* :controller-timeout ct args)
     ;; do something
     (setq l_gripper-action
	   (instance ros::simple-action-client
		     :init "/l_gripper_controller/follow_joint_trajectory" control_msgs::FollowJointTrajectoryAction
		     :groupname groupname))
     (setq r_gripper-action
	   (instance ros::simple-action-client
		     :init "/r_gripper_controller/follow_joint_trajectory" control_msgs::FollowJointTrajectoryAction
		     :groupname groupname))
     ))
  (:default-controller
   ()
   (list
    (list
     (cons :controller-action "fullbody_controller/follow_joint_trajectory")
     (cons :controller-state "fullbody_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n)) (send-all (send robot :joint-list) :name)))
    )
   )
   )

  (:go-velocity
   (x_m y_m z_m)
   (defun leg-ik
       (lleg-pos rleg-pos)
       (send *robot* :inverse-kinematics (list (make-coords :pos lleg-pos) (make-coords :pos rleg-pos))
	     :move-target (list
			   (send *robot* :lleg :end-coords)
			   (send *robot* :rleg :end-coords))
	     :translation-axis (list t t)
	     :rotation-axis (list t t)
	     :stop 10
	     :revert-if-fail nil :debug-view nil
	     )
     (send *ri* :angle-vector (send *robot* :angle-vector) 10)
     (send *ri* :wait-interpolation)
     )
   (let ((dx (* x_m 500)) (dy (* y_m 500)))
     (leg-ik (float-vector 0  100        -970) (float-vector 0      -100        -970))
     (leg-ik (float-vector 0  100        -870) (float-vector 0      -100        -970))
     (leg-ik (float-vector dx (+ 100 dy) -870) (float-vector (- dx) (- -100 dy) -970))
     (leg-ik (float-vector dx (+ 100 dy) -970) (float-vector (- dx) (- -100 dy) -970))
     (leg-ik (float-vector dx (+ 100 dy) -970) (float-vector (- dx) (- -100 dy) -870))
     (leg-ik (float-vector 0  100        -970) (float-vector 0      -100        -870))
     (leg-ik (float-vector 0  100        -970) (float-vector 0      -100        -970))
     )
   )

  (:start-grasp
   (&optional (limb :arms))
   (case limb
	 (:larm (send self :move-gripper-l 90))
	 (:rarm (send self :move-gripper-r 90))
	 (:arms (send self :move-gripper-l 90) (send self :move-gripper-r 90))
	 )
   )
  
  (:stop-grasp
   (&optional (limb :arms))
   (case limb
     (:larm (send self :move-gripper-l 0))
     (:rarm (send self :move-gripper-r 0))
     (:arms
      (send self :move-gripper-l 0)
      (send self :move-gripper-r 0))
     )
   )

  (:move-gripper-l
   (pos &key (tm 1000)  (wait nil))
   (let* ((goal (send l_gripper-action :make-goal-instance))
          (goal-points nil)
	  (joint-names (list "L_finger1_joint")))
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp (ros::time-now))
     (push (instance trajectory_msgs::JointTrajectoryPoint
                     :init
                     :positions (float-vector (deg2rad (- pos)))
                     :velocities (float-vector 0)
                     :time_from_start (ros::time (/ tm 1000.0)))
           goal-points)
     (send self :spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send l_gripper-action :send-goal goal)
     )
   (if wait (send l_gripper-action :wait-for-result))
   )

  (:move-gripper-r
   (pos &key (tm 1000)  (wait nil))
   (let* ((goal (send r_gripper-action :make-goal-instance))
          (goal-points nil)
	  (joint-names (list "R_finger1_joint")))
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))
     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp (ros::time-now))
     (push (instance trajectory_msgs::JointTrajectoryPoint
                     :init
                     :positions (float-vector (deg2rad (- pos)))
                     :velocities (float-vector 0)
                     :time_from_start (ros::time (/ tm 1000.0)))
           goal-points)
     (send self :spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send r_gripper-action :send-goal goal)
     )
   (if wait (send r_gripper-action :wait-for-result))
   )
  
  )



(defun cart_humanoid-init (&rest args)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* cart_humanoid-interface :init args)))
  (if (not (boundp '*robot*))
      (setq *robot* (instance *cart_humanoid-class* :init))))

;;
;;
;;
(cond
 ((and (boundp 'euscollada-robot) (class euscollada-robot))
  (let ((clst (remove-if-not #'(lambda (cl) (eq (send cl :super) euscollada-robot))
                             (system::list-all-classes))))
    (cond
     ((= (length clst) 1)
      (setq *cart_humanoid-class* (car clst)))
     ((null clst) ;; あまりないはず
      (warn ";; you should load robot-file before loading this file!")
      )
     (t
      ;; ２タイプ以上のロボットをロードしている
      ))
    ))
 (t
  (warn ";; you should load robot-file before loading this file!")
  ))
