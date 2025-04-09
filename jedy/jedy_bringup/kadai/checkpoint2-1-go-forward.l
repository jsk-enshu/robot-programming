#!/usr/bin/env roseus

(load "package://jedy_bringup/euslisp/jedy-interface.l")
(jedy-init)

(setq button-state 0)
(ros::rate 10)
(while t
  (send *ri* :send-cmd-vel-raw 10 0 0)
  (setq button-state (send *ri* :state :atom-s3-button))
  (ros::ros-info (format nil "button state ~A" button-state))
  (when (not (= button-state 0))
    (send *ri* :send-cmd-vel-raw 0 0 0)
    (return-from nil nil))
  (send *ri* :spin-once))
