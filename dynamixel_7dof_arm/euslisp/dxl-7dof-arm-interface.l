;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 7dof アームロボットのrobot-interfaceクラス
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(require :robot-interface "package://pr2eus/robot-interface.l")
(require :dxl-7dof-arm-robot "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-robot.l")

;; loadする台車・アームのrobot-interfaceクラスのメソッド定義ファイル
(load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 7dof アームロボットのrobot-interfaceクラス定義
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass dxl-7dof-arm-interface
  :super robot-interface
  :slots ())

(eval `(defmethod dxl-7dof-arm-interface
         ;; dxl-7dof-arm-interfaceのメソッドをdefmethodする
         ,@(get-method-list-for-dxl-7dof-arm-interface)
         ))
(defmethod dxl-7dof-arm-interface
  (:init
   (&rest args)
   (prog1
       (send-super :init :robot dxl-7dof-arm-robot)
     (send self :initialize-arm-robot-ros)
     )
   )
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボット用初期化関数
;;   robot-interface (*ri*) とモデル (*dxl-7dof-arm*)を生成する
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun dxl-7dof-arm-init ()
  "Initialization function for *ri* and *dxl-7dof-arm*."
  (if (not (boundp '*ri*)) ;; real robot interface
      (setq *ri* (instance dxl-7dof-arm-interface :init)))
  (if (not (boundp '*dxl-7dof-arm*)) ;; Euslisp model
      (setq *dxl-7dof-arm* (instance dxl-7dof-arm-robot :init)))
  (objects (list *dxl-7dof-arm*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects)
  )
(warn ";; (dxl-7dof-arm-init) ;; for initialize ~%")
