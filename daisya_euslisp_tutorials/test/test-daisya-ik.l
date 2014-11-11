#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(load "package://daisya_euslisp_tutorials/euslisp/daisya-ik.l")
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot.l")

;; overwrite for test
(defun read-line (&rest args)
  ;; h:left, j:down, k:up, l:right, f:forward, b:back
  (prog1
      (case *count-for-test*
        (0 "h")
        (1 "j")
        (2 "k")
        (3 "l")
        (4 "f")
        (5 "b")
        (6 "e")
        )
    (incf *count-for-test*)
    ))

(deftest test-daisya-ik-normal
  (setq *count-for-test* 0)
  (assert (ik-demo0 :use-base nil)))

(deftest test-daisya-ik-base
  (setq *count-for-test* 0)
  (assert (ik-demo0 :use-base t)))

(deftest test-dxl-armed-turtlebot-ik-normal
  (setq *count-for-test* 0)
  (assert (ik-demo0 :use-base nil :robot (instance dxl-armed-turtlebot-robot :init))))

(deftest test-dxl-armed-turtlebot-ik-base
  (setq *count-for-test* 0)
  (assert (ik-demo0 :use-base t :robot (instance dxl-armed-turtlebot-robot :init))))

(run-all-tests)
(exit 0)