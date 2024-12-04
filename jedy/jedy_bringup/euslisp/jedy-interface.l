#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(load "package://kxreus/euslisp/kxr-interface.l")
(load "package://jedy_bringup/euslisp/jedy.l")


(defun calc-yaw-pitch-roll (linear-acceleration)
  (let* ((x (send linear-acceleration :x))
         (y (send linear-acceleration :y))
         (z (send linear-acceleration :z))
         ;; 各角度の計算 (radian単位)
         (roll (atan (- x) (sqrt (+ (* y y) (* z z)))))
         (pitch (atan y z))
         (yaw 0))  ;; Yawは加速度だけでは計算できないので、0を仮定

    ;; 結果をdegreeにして次の順序で返す (roll, pitch, yaw)
    (list (rad2deg roll) (rad2deg pitch) (rad2deg yaw))))


(defclass jedy-interface
  :super kxr-interface
  :slots (atom-s3-button-click-count atom-s3-additional-info-topic-name imu-raw-msg))

(defmethod jedy-interface
  (:init (robot &rest args &key (namespace nil)
                (type :default-controller)
                (controller-name "fullbody_controller") &allow-other-keys)
   (send-super* :init robot :type type
                :namespace namespace
                :controller-name controller-name
                :input-joint-names (list "rarm_joint0"
                                         "rarm_joint1"
                                         "rarm_joint2"
                                         "rarm_joint3"
                                         "rarm_joint4"
                                         "rarm_joint5"
                                         "rarm_joint6"
                                         "rarm_gripper_joint"
                                         "larm_joint0"
                                         "larm_joint1"
                                         "larm_joint2"
                                         "larm_joint3"
                                         "larm_joint4"
                                         "larm_joint5"
                                         "larm_joint6"
                                         "larm_gripper_joint"
                                         "head_joint0"
                                         "head_joint1")
                args)
   (dolist (l (list
               (cons :larm-controller "larm_controller/follow_joint_trajectory")
               (cons :rarm-controller "rarm_controller/follow_joint_trajectory")
               (cons :head-controller "head_controller/follow_joint_trajectory")))
     (let ((type (car l))
           (name (cdr l))
           action)
       (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
                             controller-actions))
       (setf (gethash type controller-table) (list action))
       ))

   (setq atom-s3-button-click-count 0)
   (setq atom-s3-additional-info-topic-name (if namespace (format nil "~A/atom_s3_additional_info" namespace)
                                                                          "/atom_s3_additional_info"))
   (ros::advertise atom-s3-additional-info-topic-name std_msgs::String 1)
   (ros::subscribe (if namespace (format nil "~A/atom_s3_button_state" namespace)
                       "/atom_s3_button_state") std_msgs::Int32
                       #'send self :atom-s3-button-state-callback :groupname groupname)
   (setq imu-raw-msg (instance sensor_msgs::Imu :init))
   (ros::subscribe (if namespace (format nil "~A/imu_publisher/imu" namespace)
                       "/imu_publisher/imu") sensor_msgs::Imu
                       #'send self :atom-s3-imu-callback :groupname groupname)
   (ros::ros-info "Initializing jedy-interface"))
  (:state (&rest args) ;; overwrite robot-interface
          (case (car args)
            (:atom-s3-button
             (let ((button-value atom-s3-button-click-count))
               (setq atom-s3-button-click-count 0)
               button-value))
            (:imu
             imu-raw-msg)
            (:roll-pitch-yaw
             (calc-yaw-pitch-roll (send imu-raw-msg :linear_acceleration)))
            (t
             (send-super* :state args))
            ))
  (:atom-s3-button-state-callback (msg)
                                  (when (> (send msg :data) 0)
                                    (setq atom-s3-button-click-count (send msg :data))))
  (:atom-s3-imu-callback (msg)
                         (setq imu-raw-msg msg))
  (:publish-atom-s3-string (text)
    (ros::publish atom-s3-additional-info-topic-name
                  (instance std_msgs::String :init :data text)))
  (:stop-grasp (&key arm :arms)
               (send robot :angle-vector (send self :state :potentio-vector))
               (cond arm
                     (:rarm
                      (send robot :rarm_gripper_joint :joint-angle 30))
                     (:larm
                      (send robot :larm_gripper_joint :joint-angle 30))
                     (:arms
                      (send robot :rarm_gripper_joint :joint-angle 30)
                      (send robot :larm_gripper_joint :joint-angle 30))
                     (t (format t "[stop-grasp] Invalid arm ")))
               (send self :angle-vector (send robot :angle-vector) 1000)
               )
  (:start-grasp (&key arm :arms)
               (send robot :angle-vector (send self :state :potentio-vector))
               (cond arm
                     (:rarm
                      (send robot :rarm_gripper_joint :joint-angle 30))
                     (:larm
                      (send robot :larm_gripper_joint :joint-angle 30))
                     (:arms
                      (send robot :rarm_gripper_joint :joint-angle 30)
                      (send robot :larm_gripper_joint :joint-angle 30))
                     (t (format t "[start-grasp] Invalid arm ")))
               (send self :angle-vector (send robot :angle-vector) 1000)
               )
  (:head-controller
   ()
   (list
	  (list
	   (cons :controller-action "head_controller/follow_joint_trajectory")
	   (cons :controller-state "head_controller/state")
	   (cons :action-type control_msgs::FollowJointTrajectoryAction)
	   (cons :joint-names (list "head_joint0" "head_joint1")))))
  (:larm-controller
   ()
   (list
	  (list
	   (cons :controller-action "larm_controller/follow_joint_trajectory")
	   (cons :controller-state "larm_controller/state")
	   (cons :action-type control_msgs::FollowJointTrajectoryAction)
	   (cons :joint-names (list "larm_joint0" "larm_joint1" "larm_joint2"
                              "larm_joint3" "larm_joint4" "larm_joint5"
                              "larm_joint6" "larm_gripper_joint")))))
  (:rarm-controller
   ()
   (list
	  (list
	   (cons :controller-action "rarm_controller/follow_joint_trajectory")
	   (cons :controller-state "rarm_controller/state")
	   (cons :action-type control_msgs::FollowJointTrajectoryAction)
	   (cons :joint-names (list "rarm_joint0" "rarm_joint1" "rarm_joint2"
                              "rarm_joint3" "rarm_joint4" "rarm_joint5"
                              "rarm_joint6" "rarm_gripper_joint")))))
  (:default-controller
      ()
      (append
       (send self :larm-controller)
       (send self :rarm-controller)
       (send self :head-controller)))
  )



(defun jedy-init (&key
                   (namespace nil)
                   (without-ri nil)
                   (create-viewer t)
                   (controller-name "fullbody_controller"))
  (unless (boundp '*jedy*)
    (setq *jedy* (jedy)))
  (unless without-ri
    (unless (ros::ok) (ros::roseus "jedy_eus_interface"))
    (unless (boundp '*ri*)
      (setq *ri* (instance jedy-interface :init *jedy* :namespace namespace
                           :controller-name controller-name)))

    (ros::spin-once)
    (send *ri* :spin-once)
    (send *jedy* :angle-vector (send *ri* :state :potentio-vector)))
  (when create-viewer (objects (list *jedy*))))

(defun servo-off-sequence ()
  (send *ri* :angle-vector (send *jedy* :reset-pose) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :servo-off))
