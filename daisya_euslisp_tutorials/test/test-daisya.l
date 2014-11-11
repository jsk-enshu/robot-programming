#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(deftest test-daisya-init
  (assert (load "package://daisya_euslisp_tutorials/euslisp/daisya.l"))
  (assert (setq *daisya* (instance daisyaclass :init)))
  (assert (objects (list *daisya*)))
  (assert (send *daisya* :angle-vector #f(0.0 0.0 0.0 0.0 30.0))) ;; gripper opening
  )

(deftest test-dxl-armed-turtlebot-init
  (assert (load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l"))
  (assert (setq *dxl-armed-turtlebot* (instance dxl-armed-turtlebot-robot :init)))
  (assert (objects (list *dxl-armed-turtlebot*)))
  (assert (send *dxl-armed-turtlebot* :angle-vector #f(0.0 0.0 0.0 0.0 0.0 0.0 30.0))) ;; gripper opening
  )

(run-all-tests)
(exit 0)