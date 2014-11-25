#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(deftest test-init-model
  (load "package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l")
  (assert (setq *turtlebot* (turtlebot-with-sensors)))
  (assert (objects (list *turtlebot*)))
  )

(deftest test-init-ri
  (load "package://turtleboteus/euslisp/turtlebot-interface.l")
  (assert (turtlebot-init))
  (assert (progn (send *ri* :go-pos 0.1 0 5) t))
  (assert (progn (send *ri* :go-velocity 0.1 0 5) t))
  )

(run-all-tests)
(exit 0)