;; robotの初期化
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)

(ros::load-ros-manifest "jsk_recognition_msgs")

(defvar *topic-name* "/camera/depth/boxes")
(defvar *bounding-box-list* nil)

(setq found-obj nil)
(setq obj-pos #f(0 0 0))

;; ros::initする
(ros::roseus "boundingboxarray_subscriber")

;; コールバック関数
(defun bounding-box-array-cb (msg)
  (setq *bounding-box-list* (send msg :boxes)) ;; boxesは、BoundingBoxのArray(Euslispではlist)
  (when *bounding-box-list*
    (let* ((b (elt *bounding-box-list* 0))
           (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
           (cam-coords (send (send *dxl-armed-turtlebot* :camera_depth_optical_frame_lk) :copy-worldcoords)))
      (setq obj-pos (scale 0.001 (send (send cam-coords :transform cam->obj-coords) :worldpos)))
      (setq found-obj t)
      obj-pos))
  )

(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

(setq found-obj nil)
(until found-obj
       (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
       (ros::spin-once)
       (ros::sleep)
  )

(ros::ros-warn "found target ojbect ~A, appoaching" obj-pos)

(setq 2d-pos (float-vector (elt obj-pos 0) (elt obj-pos 1)))

(send *ri* :go-pos
      (elt (scale (/ (- (norm 2d-pos) 0.6) (norm 2d-pos)) 2d-pos) 0)
      (elt (scale (/ (- (norm 2d-pos) 0.6) (norm 2d-pos)) 2d-pos) 1)
      (rad2deg (atan  (elt obj-pos 1) (elt obj-pos 0))))

;; open gripper
(send *ri* :stop-grasp)



(setq found-obj nil)
(until found-obj
       (x::window-main-one) ;; IRT viewerの視点を変えられる。見にくければ変えよう
       (ros::spin-once)
       (ros::sleep)
  )

(ros::ros-warn "re-found target object ~A" obj-pos)
(setq target-cds (make-coords :pos (scale 1000 obj-pos)))
(send target-cds :translate #f(-200 0 100)) ;;z should be 0, but the link is not rigid in gazebo, so 100 is the height offset for end effector.
(objects (list *dxl-armed-turtlebot* target-cds))

(send *dxl-armed-turtlebot* :angle-vector #f(0 0 -90 0 90 0))
(send *dxl-armed-turtlebot* :inverse-kinematics target-cds :rotation-axis :y)


(send *dxl-armed-turtlebot* :angle-vector (map float-vector #'(lambda(ang)
                                     (cond
                                      ((> ang 90) (- ang 180))
                                      ((< ang -90) (+ ang 180))
                                      (t ang)
                                      ))
                                 (send *dxl-armed-turtlebot* :angle-vector)))

(ros::ros-warn "the grap arm pose is ~A" (send *dxl-armed-turtlebot* :angle-vector))

(send *ri* :angle-vector  (send *dxl-armed-turtlebot* :angle-vector) 3000)

#|
(unix:sleep 5)
(send *ri* :go-pos 0.1 0 0)

;; grasp
;; still some bug in start-grasp, so we have to directly use :move-gripper
(send *ri* :move-gripper -30 :tm 1500 :wait t)
(unix:sleep 2)

(send *ri* :angle-vector #f(80 0 45 0 45 45)) ;;  arm tuck pose
(send *ri* :go-pos -0.6 0 0) ;;  arm tuck pose
