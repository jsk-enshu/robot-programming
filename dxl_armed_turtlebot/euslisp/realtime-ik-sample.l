#!/usr/bin/env roseus
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")

(ros::roseus "realtime_ik_node")

(dxl-armed-turtlebot-init)
(setq *robot* *dxl-armed-turtlebot*)
(setq arm-tgt (make-coords :pos #f(300 0 200)))

(objects *robot*)

(defclass dxl-armed-turtlebot-ik-class
  :super propertied-object
  :slots ())
(defmethod dxl-armed-turtlebot-ik-class
  (:init
   ()
   (ros::subscribe "ik_arm_tgt" geometry_msgs::PoseStamped #'send self :arm-cb)
   )
  (:arm-cb
   (msg)
   (setq arm-tgt (ros::tf-pose->coords (send msg :pose))))
  )

(setq m (instance dxl-armed-turtlebot-ik-class :init))

(defun run ()
  (send *ri* :angle-vector (send *robot* :reset-pose) 2000)
  (send *ri* :wait-interpolation)

  (ros::rate 10)
  (while (ros::ok)
    (send *robot* :inverse-kinematics arm-tgt
          :translation-axis t :rotation-axis nil
          :stop 2
          :revert-if-fail nil :debug-view nil
          )
    (send *ri* :angle-vector (send *robot* :angle-vector) 10)

    (send arm-tgt :draw-on :flush t :color #f(1 0 0))
    (send *irtviewer* :draw-objects :flush nil)
    (x::window-main-one)
    (ros::spin-once)
    (ros::sleep)
    )
  )

(run)


