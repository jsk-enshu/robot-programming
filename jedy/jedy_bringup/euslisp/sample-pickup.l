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


(send *ri* :servo-on)
(send *ri* :send-stretch :value 30)
(send *robot* :init-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *robot* :angle-vector (send *ri* :state :potentio-vector))


(do-until-key
    (ros::spin-once)
  (send *irtviewer* :draw-objects :flush nil)
  (when (and *tag-detection* (> (length (send *tag-detection* :detections)) 0))
    (dotimes (i (length (send *tag-detection* :detections)))
      (when (send *tag-detection* :detections)
        (let ((detection (car (send *tag-detection* :detections)))
              target-coords
              (bx (make-cube 40 40 40))
              pick-angle-vector)
          (when (send detection :id)
            (send *robot* :angle-vector (send *ri* :state :potentio-vector))
            (setq target-coords
                  (ros::tf-pose->coords (send (send (send (car (send *tag-detection* :detections)) :pose) :pose) :pose)))
            (setq target-coords (send (send *camera-color-optical-frame* :copy-worldcoords)
                                      :transform target-coords))
            (send (send target-coords :rotate pi :x :local) :rotate -pi/2 :y)
            (send *target-end-coords-axis* :newcoords (send target-coords :copy-worldcoords))

            (send bx :newcoords target-coords)
            (send bx :worldcoords)
            (send bx :draw-on :flush nil :color #f(1 0 0)) ;; 描画

            ;; (send *robot* :RARM_JOINT6 :joint-angle 30)
            ;; (send *ri* :angle-vector (send *robot* :angle-vector) 1000) ;; gripperをオープンする
            ;; (send *ri* :wait-interpolation)

            ;; (send *robot* :rarm :inverse-kinematics
            ;;       target-coords
            ;;       :rotation-axis :mx
            ;;       :revert-if-fail nil)
            ;; (setq pick-angle-vector (send *robot* :angle-vector))

            ;; (send *robot* :rarm :move-end-pos #f(-30 0 0) :local :revert-if-fail nil)
            ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            ;; (send *robot* :angle-vector pick-angle-vector)
            ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            ;; (send *robot* :RARM_JOINT6 :joint-angle 0)
            ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)

            ;; (send *robot* :init-pose)
            ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
            ;; (send *ri* :wait-interpolation)
            )))
    )
  )
  (send *irtviewer* :viewer :viewsurface :flush)
  (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
  )

(send *ri* :servo-off)
