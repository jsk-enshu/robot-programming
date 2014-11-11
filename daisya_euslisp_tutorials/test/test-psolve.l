#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l")
(load "package://daisya_euslisp_tutorials/euslisp/psolve.l")

(deftest test-mase-*M*
  (assert *m*))

(deftest test-mase-search-normal
  (assert (maze-search 7 25 *m*)))

(deftest test-mase-search-breadth-extender
  (assert (maze-search 7 21 *m* :extender #'breadth-extender)))

(deftest test-mase-search-a*-extender
  (assert (maze-search 7 21 *m* :extender #'a*-extender)))

(run-all-tests)
(exit 0)