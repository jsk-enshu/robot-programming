#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(defmacro read-char-case
  (&rest args)
  (prog1
      (case *count*
        (0 90)
        (1 -90)
        (2 90)
        (3 -90)
        (4 nil)
        )
    (incf *count*)))

(deftest test-sample-robot-server-demo0
  (assert (load "package://daisya_euslisp_tutorials/euslisp/sample-robot-server.l"))
  (assert (progn (demo0) t))
  )

(deftest test-sample-robot-server-demo1
  (setq *count* 0)
  (assert (load "package://daisya_euslisp_tutorials/euslisp/sample-robot-server.l"))
  (assert (progn (demo1) t))
  )

(run-all-tests)
(exit 0)