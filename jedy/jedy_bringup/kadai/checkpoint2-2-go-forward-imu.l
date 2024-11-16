#!/usr/bin/env roseus

(load "package://jedy_bringup/euslisp/jedy-interface.l")
(jedy-init)
(setq roll-pitch-yaw nil)
(setq pitch nil)
(send *ri* :spin-once)
(send *ri* :spin-once)

(ros::rate 10)
(while t
  (send *ri* :spin-once)
  (send *ri* :send-cmd-vel-raw 10 0 0)
  (setq roll-pitch-yaw (send *ri* :state :roll-pitch-yaw))
  (setq pitch (elt roll-pitch-yaw 1))
  (ros::ros-info (format nil "roll: ~A pitch: ~A yaw: ~A"
                         (elt roll-pitch-yaw 0)
                         pitch
                         (elt roll-pitch-yaw 2)))
  (when (> (abs (- 90 pitch)) 30)
    (send *ri* :send-cmd-vel-raw 0 0 0)
    (return-from nil nil))
  )
