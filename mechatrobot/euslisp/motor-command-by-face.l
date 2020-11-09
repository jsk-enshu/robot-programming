#!/usr/bin/env roseus

;; 原点->x
;; |
;; y
;;-------------------
;;|                 |
;;|                 |
;;|                 |
;;|                 |
;;-------------------

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "opencv_apps")
(ros::roseus "motor-command-by-face")

(ros::advertise "motor1/command" std_msgs::Int64 1)
(ros::rate 10)

(setq *image-size* (float-vector 640 480)) ;;pixel
(setq *image-center* (scale 0.5 *image-size*))
(setq *face-pos* (float-vector 0.0 0.0))
(setq *motor-angle* 0) ;; [deg]

(defun face-detection-cb (msg)
  (let ((face-msg (instance opencv_apps::FaceArrayStamped :init))
        (motor-command-msg (instance std_msgs::Int64 :init))
        face)
    ;; face check
    (if (send msg :faces)
        (progn
          (setq face (send (elt (send msg :faces) 0) :face))
          (setq *face-pos* (float-vector (send face :x) (send face :y)))
	  ;;顔がカメラの左右どちらかをチェック
	  (if (<= (elt *face-pos* 0) (elt *image-center* 0))
	      (setq *motor-angle* (1- *motor-angle*))
	    (setq *motor-angle* (1+ *motor-angle*)))
	  ;;
	  (send motor-command-msg :data *motor-angle*)
	  ;; print
	  (format t "face-pos(x, y): (~a, ~a)~%"
		  (elt *face-pos* 0) (elt *face-pos* 1))
	  (format t "/motor1/command ~a~%~%"
		  (send motor-command-msg :data))
	  ;; publish
	  (ros::publish "/motor1/command" motor-command-msg)
	  )
      (progn
        (warn "no faces~%")))
    ))

(ros::subscribe "face_detection/faces" opencv_apps::FaceArrayStamped #'face-detection-cb)


(warn "(main)~%")
(defun main ()
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   ))

(main)
