#!/usr/bin/env roseus

(load "package://jedyeus/euslisp/jedy-interface.l")
(jedy-init)

(setq button-state 0)
(ros::rate 10)
(while t
  (send *ri* :go-velocity -0.1 0 0 0.1 :stop nil :wait nil)
  (setq button-state (send *ri* :state :atom-s3-button))
  (ros::ros-info (format nil "button state ~A" button-state))
  (when (not (= button-state 0))
    (send *ri* :go-velocity 0 0 0 0.1 :stop t :wait t)
    (return-from nil nil))
  (send *ri* :spin-once))
