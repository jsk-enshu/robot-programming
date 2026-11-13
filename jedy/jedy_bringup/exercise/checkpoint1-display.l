#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "checkpoint1_3_2_string_euslisp")

(ros::advertise "/atom_s3_additional_info" std_msgs::String)
(unix::sleep 1)

(setq msg (instance std_msgs::String :init))
(dotimes (i 3)
  (send msg :data (format nil "~A" (+ i 1)))
  (ros::ros-info "publish msg [data: ~A]" (send msg :data))
  (ros::publish "/atom_s3_additional_info" msg)
  (unix::sleep 1)
  )
(send msg :data (format nil "~A" "Finish!"))
(ros::ros-info "publish [data: ~A]" "Finish!")
(ros::publish "/atom_s3_additional_info" msg)

(exit)
