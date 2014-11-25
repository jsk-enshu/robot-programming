#+:ros (require :unittest "lib/llib/unittest.l")

(unless *unit-test*
  (init-unit-test))

(deftest test-init-model
  (load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-robot.l")
  (assert (setq *dxl-7dof-arm* (dxl-7dof-arm)))
  (assert (objects (list *dxl-7dof-arm*)))
  (dolist (pose (remove-if-not #'(lambda (x) (substringp "pose" (string-downcase x))) (send *dxl-7dof-arm* :methods)))
    (assert (send *dxl-7dof-arm* pose)))
  (send *dxl-7dof-arm* :tuckarm-pose)
  (assert (send *dxl-7dof-arm* :arm :move-end-pos (float-vector 20 20 -20) :world :rotation-axis nil))
  )

(deftest test-init-ri
  (load "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-interface.l")
  (assert (dxl-7dof-arm-init))
  (assert (progn
            (send *ri* :angle-vector (send *dxl-7dof-arm* :angle-vector) 2000)
            (send *ri* :wait-interpolation)
            t))
  )

(run-all-tests)
(exit 0)