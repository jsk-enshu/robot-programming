#!/usr/bin/env roseus

;; robotの初期化
(load "package://kxreus/euslisp/kxr-interface.l")
(load "package://jedyeus/euslisp/jedy-interface.l")
(jedy-init)

;;; 表示モデルなど
(load "models/chessboard-30-7x5-object.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(objects (list *target-object* *jedy*))

(ros::roseus "display_checkerboard")

;; tf-listenerの初期化
(setq *tfl* (instance ros::transform-listener :init))

(ros::rate 10)
(do-until-key
  (ros::spin-once)
  ;; camera_color_optical_frameからcheckerboardへの変換を取得
  (when (send *tfl* :wait-for-transform "camera_color_optical_frame" "checkerboard" (ros::time 0) 1)
    (let* (;; (1) tfからカメラ→checkerboardの変換を取得
           (cam->obj-coords (send *tfl* :lookup-transform "camera_color_optical_frame" "checkerboard" (ros::time 0)))
           ;; (2) *jedy*モデルがカメラの座標系をもってるので、取得する
           (cam-coords (send (send *jedy* :camera_color_optical_frame_lk) :copy-worldcoords)))
      ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
      (when cam->obj-coords
        (send *target-object* :newcoords (send cam-coords :transform cam->obj-coords))
        (ros::ros-info "Updated checkerboard position"))))
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  (ros::sleep)
  )

