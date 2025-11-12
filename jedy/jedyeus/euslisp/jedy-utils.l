(load "package://jedyeus/euslisp/jedy.l")

(defmethod jedy-robot
  (:gripper
   (limb &rest args)
   (case limb
     (:larm (send* self :larm-gripper args))
     (:rarm (send* self :rarm-gripper args))
     (:arms (list (send* self :larm-gripper args)
                  (send* self :rarm-gripper args)))))
  )
