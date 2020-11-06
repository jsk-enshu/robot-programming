#!/usr/bin/env roseus
(load "package://cart_humanoid/euslisp/cart_humanoid-interface.l")

(ros::roseus "cart_humanoid_ik_node")

(cart_humanoid-init)
;; (setq *robot* *cart_humanoid*) ;; already set
(setq larm-tgt (make-coords :pos #f(500  300    0)))
(setq rarm-tgt (make-coords :pos #f(500 -300    0)))
(setq head-tgt (make-coords :pos #f(500    0    0)))
(setq lleg-tgt (make-coords :pos #f(  0  100 -970)))
(setq rleg-tgt (make-coords :pos #f(  0 -100 -970)))

(objects *robot*)

(defclass humanoid-ik-class
  :super propertied-object
  :slots ())
(defmethod humanoid-ik-class
  (:init
   ()
   (ros::subscribe "ik_larm_tgt" geometry_msgs::PoseStamped #'send self :larm-cb)
   (ros::subscribe "ik_rarm_tgt" geometry_msgs::PoseStamped #'send self :rarm-cb)
   (ros::subscribe "ik_lleg_tgt" geometry_msgs::PoseStamped #'send self :lleg-cb)
   (ros::subscribe "ik_rleg_tgt" geometry_msgs::PoseStamped #'send self :rleg-cb)
   (ros::subscribe "ik_head_tgt" geometry_msgs::PoseStamped #'send self :head-cb)
   )
  (:larm-cb
   (msg)
   (setq larm-tgt (ros::tf-pose->coords (send msg :pose))))

  (:rarm-cb
   (msg)
   (setq rarm-tgt (ros::tf-pose->coords (send msg :pose))))

  (:lleg-cb
   (msg)
   (setq lleg-tgt (ros::tf-pose->coords (send msg :pose))))

  (:rleg-cb
   (msg)
   (setq rleg-tgt (ros::tf-pose->coords (send msg :pose))))

  (:head-cb
   (msg)
   (setq head-tgt (ros::tf-pose->coords (send msg :pose))))

  )


(setq m (instance humanoid-ik-class :init))

(defun run ()
  (send *ri* :angle-vector (send *robot* :reset-floating-pose) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *robot* :reset-pose) 2000)
  (send *ri* :wait-interpolation)

  (ros::rate 10)
  (while (ros::ok)
    (send *robot* :inverse-kinematics (list larm-tgt rarm-tgt lleg-tgt rleg-tgt)
	  :move-target (list
			(send *robot* :larm :end-coords)
			(send *robot* :rarm :end-coords)
			(send *robot* :lleg :end-coords)
			(send *robot* :rleg :end-coords))
	  :translation-axis (list t t t t)
	  :rotation-axis (list t t t t)
	  :stop 2
	  :revert-if-fail nil :debug-view nil
	  )
    (send *robot* :look-at-target head-tgt)
    (send *ri* :angle-vector (send *robot* :angle-vector) 10)
    (send *irtviewer* :draw-objects :flush nil)
    (dolist (tgt (list larm-tgt rarm-tgt lleg-tgt rleg-tgt head-tgt)) (send tgt :draw-on :flush t))
    (x::window-main-one)
    (ros::spin-once)
    (ros::sleep)
    )
  )

(run)


