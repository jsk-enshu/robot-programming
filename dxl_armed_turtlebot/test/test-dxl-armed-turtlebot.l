#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(deftest test-init-model
  (load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l")
  (assert (setq *dxl-armed-turtlebot* (dxl-armed-turtlebot)))
  (assert (objects (list *dxl-armed-turtlebot*)))
  (dolist (pose (remove-if-not #'(lambda (x) (substringp "pose" (string-downcase x))) (send *dxl-armed-turtlebot* :methods)))
    (assert (send *dxl-armed-turtlebot* pose)))
  (send *dxl-armed-turtlebot* :tuckarm-pose)
  (assert (send *dxl-armed-turtlebot* :arm :move-end-pos (float-vector 20 20 -20) :world :rotation-axis nil))
  )

(deftest test-init-ri
  (load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
  (assert (dxl-armed-turtlebot-init))
  (assert (progn
            (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)
            (send *ri* :wait-interpolation)
            t))
  (assert (progn (send *ri* :go-pos 0.1 0 5) t))
  (assert (progn (send *ri* :go-velocity 0.1 0 5) t))
  )

(run-all-tests)
(exit 0)