#!/usr/bin/env roseus

(ros::roseus-add-msgs "kobuki_msgs")

(ros::roseus "kadai1_3_2_led_euslisp")

(ros::advertise "/mobile_base/commands/led1" kobuki_msgs::Led)
(unix::sleep 1)

(setq msg (instance kobuki_msgs::Led :init))
(dotimes (i 3)
  (send msg :value (+ i 1))
  (ros::ros-info "publish msg [value: ~A]" (send msg :value))
  (ros::publish "/mobile_base/commands/led1" msg)
  (unix::sleep 1)
  )
(send msg :value 0)
(ros::ros-info "publish [value: ~A]" (send msg :value))
(ros::publish "/mobile_base/commands/led1" msg)

(exit)
