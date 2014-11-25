;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot-interface definition for dxl-7dof-arm and turtlebot
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; !! If you want to see dxl-7dof-arm-interface's methods, please see
;; !! (load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; loading of necessary programs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(load "package://pr2eus/robot-interface.l")
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l")
(load "package://turtleboteus/euslisp/turtlebot-interface-common.l")
(load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; dxl-armed-turtlebot-interface class definition
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass dxl-armed-turtlebot-interface
  :super robot-interface
  :slots ())
(defmethod dxl-armed-turtlebot-interface
  (:init
   (&rest args)
   (eval `(defmethod ,(send (class self) :name)
            ,@(get-method-list-for-dxl-7dof-arm-interface)
            ,@(get-method-list-for-turtlebot-interface)
            ))
   (prog1
       (send-super* :init :robot dxl-armed-turtlebot-robot args)
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
;; initialization function
;;   *ri* and *dxl-armed-turtlebot* are generated
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun dxl-armed-turtlebot-init (&key (objects))
  (if (not (boundp '*ri*))
      (setq *ri* (instance dxl-armed-turtlebot-interface :init :objects objects)))
  (if (not (boundp '*dxl-armed-turtlebot*))
      (setq *dxl-armed-turtlebot* (dxl-armed-turtlebot)))
  (objects (list *dxl-armed-turtlebot*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects)
  )
(warn ";; (dxl-armed-turtlebot-init) ;; for initialize ~%")