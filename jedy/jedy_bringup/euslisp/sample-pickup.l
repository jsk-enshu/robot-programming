#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "apriltag_ros")

(load "package://jedy_bringup/euslisp/jedy-interface.l")
(ros::roseus "sample_apriltag_pick")
(jedy-init :without-ri nil)


(setq *tag-detection* nil)

(defun tag-detection-callback (msg)
  (setq *tag-detection* msg))

(ros::subscribe "/camera/color/tag_detections"
                apriltag_ros::AprilTagDetectionArray
                #'tag-detection-callback)
(ros::rate 1)


(send *ri* :servo-on :names (list "rarm_joint0"
                                  "rarm_joint1"
                                  "rarm_joint2"
                                  "rarm_joint3"
                                  "rarm_joint4"
                                  "rarm_joint5"
                                  "rarm_joint6"
                                  "rarm_gripper_joint"
                                  "head_joint0"
                                  "head_joint1"
                                  ))
;; (send *ri* :send-stretch :value 30)
(send *jedy* :reset-pose)
(send *jedy* :rarm_joint2 :joint-angle -60)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *jedy* :angle-vector (send *ri* :state :potentio-vector))

(send *jedy* :head_joint0 :joint-angle 0)
(send *jedy* :head_joint1 :joint-angle 60)
(send *irtviewer* :draw-objects)

(send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
(send *ri* :wait-interpolation)


(do-until-key
    (ros::spin-once)
  (send *irtviewer* :draw-objects :flush nil)
  (when (and *tag-detection* (> (length (send *tag-detection* :detections)) 0))
    (dotimes (i (length (send *tag-detection* :detections)))
      (when (send *tag-detection* :detections)
        (let ((detection (car (send *tag-detection* :detections)))
              target-coords
              (bx (make-cube 45 45 45))
              pick-angle-vector)
          (when (send detection :id)
            (send *jedy* :angle-vector (send *ri* :state :potentio-vector))
            (setq target-coords
                  (ros::tf-pose->coords (send (send (send (car (send *tag-detection* :detections)) :pose) :pose) :pose)))
            (setq target-coords (send (send *jedy* :camera_color_optical_frame_lk :copy-worldcoords)
                                      :transform target-coords))
            (send target-coords :translate (float-vector 0 0 (/ -45 2)))
            (send target-coords :rotate pi/2 :y :local)
            ;; (send *target-end-coords-axis* :newcoords (send target-coords :copy-worldcoords))

            (send bx :newcoords target-coords)
            (send bx :worldcoords)
            (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画

            (send *jedy* :rarm_gripper_joint :joint-angle -30)
            (send *ri* :angle-vector (send *jedy* :angle-vector) 1000) ;; gripperをオープンする
            (send *ri* :wait-interpolation)

            (send *jedy* :rarm :inverse-kinematics
                  target-coords
                  :rotation-axis :mx
                  :revert-if-fail t)
            (setq pick-angle-vector (send *jedy* :angle-vector))

            ;; (send *jedy* :rarm :move-end-pos #f(-50 0 0) :local :revert-if-fail nil)
            ;; (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            ;; (send *jedy* :angle-vector pick-angle-vector)
            ;; (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            ;; (send *jedy* :rarm_gripper_joint :joint-angle 0)
            ;; (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            ;; (send *jedy* :rarm :move-end-pos #f(-50 0 0) :local :revert-if-fail nil)
            ;; (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            ;; (send *jedy* :reset-pose)
            ;; (send *jedy* :rarm_joint2 :joint-angle -60)
            ;; (send *jedy* :head_joint0 :joint-angle 0)
            ;; (send *jedy* :head_joint1 :joint-angle 60)
            ;; (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            )))
    )
  )
  (send *irtviewer* :viewer :viewsurface :flush)
  (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
  )

(send *ri* :servo-off)
