#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(deftest test-subsumption
  (assert (load "package://daisya_euslisp_tutorials/euslisp/subsumption.l"))
  (assert (test-subsumption :exit-count 100)))

(run-all-tests)
(exit 0)