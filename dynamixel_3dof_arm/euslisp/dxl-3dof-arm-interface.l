;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 3dof アームロボットのrobot-interfaceクラス
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(require :robot-interface "package://pr2eus/robot-interface.l")
(require :dxl-3dof-arm-robot "package://dynamixel_3dof_arm/euslisp/dxl-3dof-arm-robot.l")

;; loadする台車・アームのrobot-interfaceクラスのメソッド定義ファイル
(load "package://dynamixel_3dof_arm/euslisp/dxl-3dof-arm-interface-common.l")
(ros::load-ros-manifest "control_msgs")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 3dof アームロボットのrobot-interfaceクラス定義
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass dxl-3dof-arm-interface
  :super robot-interface
  :slots ())

(eval `(defmethod dxl-3dof-arm-interface
         ;; dxl-3dof-arm-interfaceのメソッドをdefmethodする
         ,@(get-method-list-for-dxl-3dof-arm-interface)
         ))
(defmethod dxl-3dof-arm-interface
  (:init
   (&rest args)
   (prog1
       (send-super :init :robot dxl-3dof-arm-robot)
     (send self :initialize-arm-robot-ros)
     )
   )
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボット用初期化関数
;;   robot-interface (*ri*) とモデル (*dxl-3dof-arm*)を生成する
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun dxl-3dof-arm-init ()
  "Initialization function for *ri* and *dxl-3dof-arm*."
  (if (not (boundp '*ri*)) ;; real robot interface
      (setq *ri* (instance dxl-3dof-arm-interface :init)))
  (if (not (boundp '*dxl-3dof-arm*)) ;; Euslisp model
      (setq *dxl-3dof-arm* (instance dxl-3dof-arm-robot :init)))
  (objects (list *dxl-3dof-arm*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects)
  )
(warn ";; (dxl-3dof-arm-init) ;; for initialize ~%")
