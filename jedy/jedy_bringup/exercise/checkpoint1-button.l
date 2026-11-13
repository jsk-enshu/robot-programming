#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "checkpoint1_3_1_button_euslisp")

(defun button-cb
    (msg)
  (setq button-count (send msg :data))
  (ros::ros-info "subscribe msg [button-count: ~A]" button-count)
  )

(ros::subscribe "/atom_s3_button_state"
		std_msgs::Int32 #'button-cb)

(ros::ros-info "ready to subscribe !!!")
(ros::spin)
