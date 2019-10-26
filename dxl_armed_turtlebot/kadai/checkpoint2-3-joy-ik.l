#!/usr/bin/env roseus

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(load "package://roseus/test/joy.l")

(ros::roseus "checkpoint2_3_joy_ik")

(dxl-armed-turtlebot-init)
(init-ps3joy)

(ros::ros-info "send initial pose.")
(send *dxl-armed-turtlebot* :reset-pose)
(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(ros::ros-info "start !!!")
(ros::rate 10)
(do-until-key ;; Enterでループを抜ける
 (when (= (send *joy* :circle-button) 1)
   (ros::ros-info "circle button pushed.")
   (send *dxl-armed-turtlebot* :arm :move-end-pos
	 (float-vector 0 0 20) :world :rotation-axis nil) ;; 手先を20mm上に移動
   (send *irtviewer* :draw-objects)
   (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000)
   (send *ri* :wait-interpolation)
   )
 (x::window-main-one)
 (ros::sleep)
 (ros::spin-once)
 )
