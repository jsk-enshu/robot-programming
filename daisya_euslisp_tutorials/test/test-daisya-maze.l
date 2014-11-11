#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(defun init-test ()
  ;; daisya
  (load "package://daisya_euslisp_tutorials/euslisp/daisya.l")
  (setq *daisya* (instance daisyaclass :init))
  (objects (list *daisya*))
  ;; dxl-armed-turtlebot-robot
  (load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l")
  (setq *dxl-armed-turtlebot* (instance dxl-armed-turtlebot-robot :init))
  (objects (list *dxl-armed-turtlebot*))
  )
(init-test)

(deftest test-maze-init
  (assert (load "package://daisya_euslisp_tutorials/euslisp/daisya-maze.l"))
  (assert (maze-init))
  (assert (maze-demo 1 25 :extender #'depth-extender))
  (assert (maze-demo 1 25 :extender #'breadth-extender))
  (assert (maze-demo 1 25 :extender #'hill-climb-extender))
  (assert (maze-demo 1 25 :extender #'best-first-extender))
  (assert (maze-demo 1 25 :extender #'a*-extender))
  (assert (maze-init :robot *dxl-armed-turtlebot*))
  (assert (maze-demo 1 25 :extender #'depth-extender :robot *dxl-armed-turtlebot*))
  (assert (maze-demo 1 25 :extender #'breadth-extender :robot *dxl-armed-turtlebot*))
  (assert (maze-demo 1 25 :extender #'hill-climb-extender :robot *dxl-armed-turtlebot*))
  (assert (maze-demo 1 25 :extender #'best-first-extender :robot *dxl-armed-turtlebot*))
  (assert (maze-demo 1 25 :extender #'a*-extender :robot *dxl-armed-turtlebot*))
  )

(run-all-tests)
(exit 0)