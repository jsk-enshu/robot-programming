;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot-interface definition for turtlebot
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; loading of necessary programs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(load "package://pr2eus/robot-interface.l")
(require "package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l")
(require "package://turtleboteus/euslisp/turtlebot-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; dxl-armed-turtlebot-interface class definition
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass turtlebot-interface
  :super robot-interface
  :slots ())
(defmethod turtlebot-interface
  (:init
   (&rest args)
   (eval `(defmethod ,(send (class self) :name)
            ,@(get-method-list-for-turtlebot-interface)
            ))
   (prog1
       (send-super* :init :robot turtlebot-with-sensors-robot args)
     (send self :initialize-turtlebot-ros)
     )
   )
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; initialization function
;;   *ri* and *turtlebot* are generated
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun turtlebot-init (&key (objects))
  (if (not (boundp '*ri*))
      (setq *ri* (instance turtlebot-interface :init :objects objects)))
  (if (not (boundp '*turtlebot*))
      (setq *turtlebot* (turtlebot-with-sensors-robot)))
  (objects (list *turtlebot*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects)
  )
(warn ";; (turtlebot-init) ;; for initialize ~%")