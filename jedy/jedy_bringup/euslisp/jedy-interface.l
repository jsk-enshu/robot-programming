#!/usr/bin/env roseus

(load "package://kxreus/euslisp/kxr-interface.l")
(load "package://jedy_bringup/euslisp/jedy.l")


(defclass jedy-interface
  :super kxr-interface
  :slots (additional-slot1 additional-slot2))

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
   (ros::ros-info "Initializing jedy-interface")))


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
