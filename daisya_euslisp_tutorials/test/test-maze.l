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
  (assert (load "package://daisya_euslisp_tutorials/euslisp/maze.l"))
  (assert (setq *maze* (make-maze *m*)))
  (assert (objects (list *maze* *daisya*)))
  (assert (send *daisya* :locate (generate-coords-from-nodenum 1) :world))
  (assert (objects (list *maze* *dxl-armed-turtlebot*)))
  (assert (send *dxl-armed-turtlebot* :locate (generate-coords-from-nodenum 1) :world))
  )

(run-all-tests)
(exit 0)