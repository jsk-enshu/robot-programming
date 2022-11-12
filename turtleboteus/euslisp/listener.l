#!/usr/bin/env roseus

(ros::load-ros-manifest "kobuki_msgs")
(ros::roseus "listener" :anonymous t)

;; callback function
(defun imudata-cb (msg)
	(print (list 'button (send msg :button)))
    (print (list 'state (send msg :state)))
)
(ros::subscribe "/mobile_base/events/button" kobuki_msgs::ButtonEvent #'imudata-cb)



(ros::rate 10)
(do-until-key
	(ros::sleep)
	(ros::spin-once))
(exit)
