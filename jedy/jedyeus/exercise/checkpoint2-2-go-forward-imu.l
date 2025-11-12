#!/usr/bin/env roseus

(load "package://jedyeus/euslisp/jedy-interface.l")
(jedy-init)
(setq roll-pitch-yaw nil)
(setq pitch nil)
(send *ri* :spin-once)

(ros::rate 10)
(while t
  (send *ri* :spin-once)
  (send *ri* :go-velocity 0.1 0 0 100 :stop nil :wait nil)
  (setq roll-pitch-yaw (send *ri* :state :roll-pitch-yaw))
  (setq pitch (elt roll-pitch-yaw 1))
  (ros::ros-info (format nil "roll: ~A pitch: ~A yaw: ~A"
                         (elt roll-pitch-yaw 0)
                         pitch
                         (elt roll-pitch-yaw 2)))
  (when (> (abs (- 90 pitch)) 30)
    (send *ri* :go-velocity 0 0 0 100 :stop t :wait nil)
    (return-from nil nil))
  )
