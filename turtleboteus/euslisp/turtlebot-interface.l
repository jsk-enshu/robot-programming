;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 台車ロボットのrobot-interfaceクラス
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(require :robot-interface "package://pr2eus/robot-interface.l")
(require :turtlebot-with-sensors-robot "package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l")

;; loadする台車のrobot-interfaceクラスのメソッド定義ファイル
(require :turtlebot-interface-common "package://turtleboteus/euslisp/turtlebot-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 台車ロボットのrobot-interfaceクラス定義
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass turtlebot-interface
  :super robot-interface
  :slots ())

(eval `(defmethod turtlebot-interface
         ,@(get-method-list-for-turtlebot-interface)
         ))
(defmethod turtlebot-interface
  (:init
   (&rest args)
   (prog1
       (send-super* :init :robot turtlebot-with-sensors-robot args)
     (send self :initialize-turtlebot-ros)
     )
   )
  ;; TODO : this should be merged pr2 base controller
  ;; https://github.com/jsk-ros-pkg/jsk_common/issues/254
  (:add-controller
   (&rest args)
   (prog1
       (send-super* :add-controller args)
     (if (and (send self :simulation-modep)
              (ros::get-param "/turtlebot_laptop_battery/acpi_path")) ;; check if real robot
         (setq joint-action-enable t))
     ))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボット用初期化関数
;;   robot-interface (*ri*) とモデル (*turtlebot*)を生成する
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun turtlebot-init (&key (objects))
  "Initialization function for *ri* and *turtlebot*."
  (if (not (boundp '*ri*))
      (setq *ri* (instance turtlebot-interface :init :objects objects)))
  (if (not (boundp '*turtlebot*))
      (setq *turtlebot* (turtlebot-with-sensors)))
  (objects (list *turtlebot*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects)
  )
(warn ";; (turtlebot-init) ;; for initialize ~%")