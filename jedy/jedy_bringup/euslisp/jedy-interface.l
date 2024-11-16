#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(load "package://kxreus/euslisp/kxr-interface.l")
(load "package://jedy_bringup/euslisp/jedy.l")


(defclass jedy-interface
  :super kxr-interface
  :slots (atom-s3-button-click-count atom-s3-additional-info-topic-name))

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
   (setq atom-s3-button-click-count 0)
   (setq atom-s3-additional-info-topic-name (if namespace (format nil "~A/atom_s3_additional_info" namespace)
                                                                          "/atom_s3_additional_info"))
   (ros::advertise atom-s3-additional-info-topic-name std_msgs::String 100)
   (ros::subscribe (if namespace (format nil "~A/atom_s3_button_state" namespace)
                       "/atom_s3_button_state") std_msgs::Int32
                       #'send self :atom-s3-button-state-callback)
   (ros::ros-info "Initializing jedy-interface"))
  (:state (&rest args) ;; overwrite robot-interface
          (case (car args)
            (:atom-s3-button
             atom-s3-button-click-count)
            (t
             (send-super* :state args))
            ))
  (:atom-s3-button-state-callback (msg)
                                  (setq atom-s3-button-click-count (send msg :data)))
  (:publish-atom-s3-string (text)
    (ros::publish atom-s3-additional-info-topic-name msg))
  )


(defun jedy-init (&key
                   (namespace nil)
                   (create-viewer t)
                   (controller-name "fullbody_controller"))
  (unless (boundp '*jedy*)
    (setq *jedy* (jedy)))
  (unless (ros::ok) (ros::roseus "jedy_eus_interface"))
  (unless (boundp '*ri*)
    (setq *ri* (instance jedy-interface :init *jedy* :namespace namespace
                         :controller-name controller-name)))

  (ros::spin-once)
  (send *ri* :spin-once)
  (send *jedy* :angle-vector (send *ri* :state :potentio-vector))
  (when create-viewer (objects (list *jedy*))))


(defun servo-off-sequence ()
  (send *ri* :angle-vector (send *jedy* :reset-pose) 5000)
  (send *ri* :wait-interpolation)
  (send *ri* :servo-off))
