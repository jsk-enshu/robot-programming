;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボットのrobot-interfaceクラス
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l")
(require :robot-interface "package://pr2eus/robot-interface.l")

;; loadする台車・アームのrobot-interfaceクラスのメソッド定義ファイル
(load "package://turtleboteus/euslisp/turtlebot-interface.l")
(load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボットのrobot-interfaceクラス定義
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass dxl-armed-turtlebot-interface
  :super robot-interface
  :slots ((robot-base :forward (:def-vector-value :raw-bumper-data :raw-button-data :raw-wheel-drop-data :raw-cliff-data :raw-imu-data :imurot :update-robot-state :bumper-vector :button-vector :wheel-drop-vector :cliff-vector :cliff-bottom-vector :imucoords :power-system-vector :publish-led :publish-sound :go-stop :move-to :go-pos :go-velocity))
          gripper-action))

(eval `(defmethod dxl-armed-turtlebot-interface
         ,@(get-method-list-for-dxl-7dof-arm-interface)
         ))
(defmethod dxl-armed-turtlebot-interface
  (:init
   (&rest args)
   (prog1
       (send-super* :init :robot dxl-armed-turtlebot-robot args)

     (setq robot-base (instance turtlebot-interface :init))
     (send self :initialize-arm-robot-ros)
     )
   )
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボット用初期化関数
;;   robot-interface (*ri*) とモデル (*dxl-armed-turtlebot*)を生成する
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun dxl-armed-turtlebot-init
  (&key (objects))
  "Initialization function for *ri* and *dxl-armed-turtlebot*."
  (if (not (boundp '*ri*))
      (setq *ri* (instance dxl-armed-turtlebot-interface :init :objects objects)))
  (if (not (boundp '*dxl-armed-turtlebot*))
      (setq *dxl-armed-turtlebot* (dxl-armed-turtlebot)))
  (objects (list *dxl-armed-turtlebot*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects)
  )
(warn ";; (dxl-armed-turtlebot-init) ;; for initialize ~%")
