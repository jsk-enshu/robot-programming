#!/usr/bin/env roseus

(load "package://jedy_bringup/euslisp/jedy-interface.l")
(ros::roseus "teleoperation_node")

(defun jedy-teleop-init (&key
                   (robot-a-namespace "robot_a")
                   (robot-b-namespace "robot_b")
                   (controller-name "fullbody_controller")
                   (create-viewer t))
  (setq *robot-a* (jedy))
  (unless (boundp '*robot-a-ri*)
    (setq *robot-a-ri* (instance jedy-interface :init *robot-a* :namespace robot-a-namespace
                                 :controller-name controller-name)))
  (send *robot-a* :angle-vector (send *robot-a-ri* :state :potentio-vector))

  (setq *robot-b* (jedy))
  (unless (boundp '*robot-b-ri*)
    (setq *robot-b-ri* (instance jedy-interface :init *robot-b* :namespace robot-b-namespace
                                 :controller-name controller-name)))
  (send *robot-b* :angle-vector (send *robot-b-ri* :state :potentio-vector))

  (ros::spin-once)
  (send *robot-a-ri* :spin-once)
  (send *robot-b-ri* :spin-once)
  (send *robot-a* :translate #f(0 300 0))
  (when create-viewer (objects (list *robot-a* *robot-b*))))

(jedy-teleop-init)


(ros::rate 100)
(send *robot-b-ri* :servo-on)
(send *robot-a-ri* :servo-on)
(send *robot-a* :init-pose)
(send *robot-a-ri* :angle-vector (send *robot-a* :angle-vector) 5000)
(send *robot-a-ri* :wait-interpolation)
(send *robot-b-ri* :angle-vector (send *robot-b* :angle-vector) 5000)
(send *robot-b-ri* :wait-interpolation)
(send *robot-a-ri* :servo-off)
(while (ros::ok)
  (setq robot-a-angle-vector (send *robot-a-ri* :state :potentio-vector))
  (send *robot-b* :angle-vector robot-a-angle-vector)
  (send *robot-b-ri* :angle-vector (send *robot-b* :angle-vector) 500)
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  (ros::sleep)
  )
