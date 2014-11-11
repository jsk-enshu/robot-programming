;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot-interface definition to use 7dof arm robot using dynamixel AX-12A motors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; !! If you want to see dxl-7dof-arm-interface's methods, please see
;; !! (load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface-common.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; loading of necessary programs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(load "package://pr2eus/robot-interface.l")
(load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-robot.l")
(load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface-common.l")
(ros::load-ros-manifest "control_msgs")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; dxl-7dof-arm-interface class definition
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defclass dxl-7dof-arm-interface
  :super robot-interface
  :slots ())
(defmethod dxl-7dof-arm-interface
  (:init
   (&rest args)
   (eval `(defmethod ,(send (class self) :name)
            ,@(get-method-list-for-dxl-7dof-arm-interface)
            ))
   (prog1
       (send-super :init :robot dxl-7dof-arm-robot)
     (send self :initialize-arm-robot-ros)
     )
   )
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; initialization function
;;   *ri* and *dxl-7dof-arm* are generated
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun dxl-7dof-arm-init ()
  (if (not (boundp '*ri*)) ;; real robot interface
      (setq *ri* (instance dxl-7dof-arm-interface :init)))
  (if (not (boundp '*dxl-7dof-arm*)) ;; Euslisp model
      (setq *dxl-7dof-arm* (instance dxl-7dof-arm-robot :init)))
  (objects (list *dxl-7dof-arm*))
  (send *irtviewer* :change-background #f(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects)
  )
(warn ";; (dxl-7dof-arm-init) ;; for initialize ~%")