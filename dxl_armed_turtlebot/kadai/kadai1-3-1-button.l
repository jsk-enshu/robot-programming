#!/usr/bin/env roseus

(ros::roseus-add-msgs "kobuki_msgs")

(ros::roseus "kadai1_3_1_button_euslisp")

(defun button-cb
    (msg)
  (setq button (send msg :button))
  (setq state (send msg :state))
  (ros::ros-info "subscribe msg [button: ~A  state: ~A]" button state)
  )

(ros::subscribe "/mobile_base/events/button"
		kobuki_msgs::ButtonEvent #'button-cb)

(ros::ros-info "ready to subscribe !!!")
(ros::spin)
