#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "apriltag_ros")

(load "package://jedy_bringup/euslisp/jedy-interface.l")
(ros::roseus "sample_apriltag_pick")
(jedy-init :without-ri nil)


(defun within-5-degrees-filter (vec)
  "Returns t if the angle between the input vector and (float-vector 0 0 1) is within 5 degrees, otherwise nil."
  (let* ((unit-vec (normalize-vector vec))
         (ref-vec (float-vector 0 0 1))
         (dot-product (v. unit-vec ref-vec))
         (angle (acos dot-product))) ;; Compute angle in radians
    (if (<= angle (/ (* 5 pi) 180)) ;; 5 degrees in radians
        t
        nil)))

(setq *tag-detection* nil)

(defun tag-detection-callback (msg)
  (setq *tag-detection* msg))

(ros::subscribe "/camera/color/tag_detections"
                apriltag_ros::AprilTagDetectionArray
                #'tag-detection-callback)
(ros::rate 1)


(when (boundp '*ri*)
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
                                    )))
(unix::usleep (* 1000 3000))
  ;; (send *ri* :send-stretch :value 30)
(send *jedy* :reset-pose)
(send *jedy* :rarm_joint2 :joint-angle -60)

(when (boundp '*ri*)
  (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *jedy* :angle-vector (send *ri* :state :potentio-vector)))

(send *jedy* :head_joint0 :joint-angle 0)
(send *jedy* :head_joint1 :joint-angle 60)
(send *irtviewer* :draw-objects)

(when (boundp '*ri*)
  (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
  (send *ri* :wait-interpolation))


(setq *send-ri-flag* nil)
;; 実機に指令を送りたい場合はこれをtにする


(do-until-key
    (ros::spin-once)
  (send *irtviewer* :draw-objects :flush nil)
  (when (and *tag-detection* (> (length (send *tag-detection* :detections)) 0))
    (send *jedy* :angle-vector (send *ri* :state :potentio-vector))
    (dolist (detection (send *tag-detection* :detections))
      (let (target-coords
            (bx (make-cube 45 45 45))
            pick-angle-vector)
        (when (send detection :id)
          (print (send detection :id))
          (setq target-coords
                (ros::tf-pose->coords (send (send (send detection :pose) :pose) :pose)))
          (setq target-coords (send (send *jedy* :camera_color_optical_frame_lk :copy-worldcoords)
                                    :transform target-coords))
          (when (within-5-degrees-filter (send target-coords :z-axis)) ;; 上を向いているapriltagのみ選ぶ
            (send target-coords :translate (float-vector 0 0 (/ -45 2)))
            (send target-coords :rotate pi/2 :y :local)

            ;; (send *target-end-coords-axis* :newcoords (send target-coords :copy-worldcoords))

            (send bx :newcoords target-coords)
            (send bx :worldcoords)
            (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画

            (send *jedy* :rarm_gripper_joint :joint-angle -50)
            (when *send-ri-flag*
              (send *ri* :angle-vector (send *jedy* :angle-vector) 1000) ;; gripperをオープンする
              (send *ri* :wait-interpolation))

            (setq ik-result (send *jedy* :rarm :inverse-kinematics
                                  target-coords
                                  :rotation-axis :x ;; x軸（グリッパーの前方向）への回転を許してikを解く
                                  :revert-if-fail t
                                  :warnp nil
                                  :stop 200))
            (when ik-result
              (setq pick-angle-vector (send *jedy* :angle-vector))

              (when *send-ri-flag*
                (send *jedy* :rarm :move-end-pos #f(-50 0 0) :local :revert-if-fail nil)
                (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
                (send *ri* :wait-interpolation)

                (send *jedy* :angle-vector pick-angle-vector)
                (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
                (send *ri* :wait-interpolation)

                (send *jedy* :rarm_gripper_joint :joint-angle 0)
                (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
                (send *ri* :wait-interpolation)

                (send *jedy* :rarm :move-end-pos #f(-50 0 0) :local :revert-if-fail nil)
                (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
                (send *ri* :wait-interpolation)

                (send *jedy* :reset-pose)
                (send *jedy* :rarm_joint2 :joint-angle -60)
                (send *jedy* :head_joint0 :joint-angle 0)
                (send *jedy* :head_joint1 :joint-angle 60)
                (send *ri* :angle-vector (send *jedy* :angle-vector) 3000)
                (send *ri* :wait-interpolation)))

            ))))
    )
  (send *irtviewer* :viewer :viewsurface :flush)
  (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
  )

(send *ri* :servo-off)
