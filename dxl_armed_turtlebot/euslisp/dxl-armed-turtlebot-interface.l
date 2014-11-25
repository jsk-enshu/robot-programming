;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボットのrobot-interfaceクラス
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l")
(load "package://pr2eus/robot-interface.l")

;; loadする台車・アームのrobot-interfaceクラスのメソッド定義ファイル
(load "package://turtleboteus/euslisp/turtlebot-interface-common.l")
(load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボットのrobot-interfaceクラス定義
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass dxl-armed-turtlebot-interface
  :super robot-interface
  :slots ())

(defmethod dxl-armed-turtlebot-interface
  (:init
   (&rest args)
   (eval `(defmethod ,(send (class self) :name)
            ;; dxl-7dof-arm-interface, turtlebot-interfaceのメソッドをそれぞれdefmethodする
            ,@(get-method-list-for-dxl-7dof-arm-interface)
            ,@(get-method-list-for-turtlebot-interface)
            ))
   (prog1
       (send-super* :init :robot dxl-armed-turtlebot-robot args)
     ;; それぞれのcallback登録など
     (send self :initialize-turtlebot-ros)
     (send self :initialize-arm-robot-ros)
     )
   )
  (:def-vector-value
   (&key (simulate-func #'(lambda () (instantiate float-vector 3)))
         (raw-data-name)
         (vector-length 3)
         (state-name :state)
         (value-name))
   (if (send self :simulation-modep)
       (funcall simulate-func)
     (let ((bp (send self raw-data-name))
           (ret (instantiate float-vector vector-length)))
       (if bp (setf (elt ret (cadr (memq value-name bp))) (cadr (memq state-name bp))))
       ret)))
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