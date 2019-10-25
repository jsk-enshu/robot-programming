#!/usr/bin/env roseus

(load "package://turtleboteus/euslisp/turtlebot-interface")

(ros::roseus "checkpoint2_1_go_forward")

(turtlebot-init)

(ros::ros-info "start !!!")

(while t
  (send *ri* :go-velocity 0.1 0 0)
  (when (not (= (norm (send *ri* :state :bumper-vector)) 0)) ;; ひとつでも非ゼロなら
    (send *ri* :go-velocity 0 0 0)
    (return-from nil nil)))

(exit)
