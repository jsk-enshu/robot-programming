#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(deftest test-sample-robot-server
  (assert (load "package://daisya_euslisp_tutorials/euslisp/sample-robot-server.bak.l"))
  (assert (progn (init-server) t))
  (assert (connect-robotsim-server))
  (assert (progn (send *rs-sim* :daisya-velocity-vector #f(100 0 0)) t))
  (assert (progn (send *rs-sim* :daisya-velocity-vector #f(0 0 0)) t))
  (assert (progn (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0)) t))
  (assert
   (progn
     (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
     (unix:sleep 1)
     t))
  )

(deftest test-sample-robot-server-maze
  (assert (load "package://daisya_euslisp_tutorials/euslisp/sample-robot-server.bak.l"))
  (assert (progn (init-server 'maze) t))
  (assert (connect-robotsim-server))
  (assert (progn (send *rs-sim* :daisya-velocity-vector #f(100 0 0)) t))
  (assert (progn (send *rs-sim* :daisya-velocity-vector #f(0 0 0)) t))
  (assert (progn (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0)) t))
  (assert
   (progn
     (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
     (unix:sleep 1)
     t))
  )

(run-all-tests)
(exit 0)