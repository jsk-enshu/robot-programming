(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")

(warn ";; (dxl-arm-sample) ;; for dxl arm sample program~%")
(warn ";; (dxl-arm-check) ;; for dxl arm direction check~%")

(defvar *arm-dof* 7)

(defun dxl-arm-sample ()
  (warn ";; press enter to stop program~%~%")
  (do-until-key
   (warn "set model's (*dxl-armed-turtlebot*) joint angle to :init-pose~%~%")
   (send *dxl-armed-turtlebot* :init-pose) ;; 関節角度を :init-pose にセット
   (send *irtviewer* :draw-objects) ;; 描画
   (unix::usleep (* 2000 1000))
   ;;
   (warn "move real robot (*ri*) with 4000 [ms]~%~%")
   (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000) ;;4000[ms] で実行
   (send *ri* :wait-interpolation) ;; 補間をまつ
   ;;
   (warn "set model's (*dxl-armed-turtlebot*) joint angle to :reset-pose~%~%")
   (send *dxl-armed-turtlebot* :reset-pose) ;; 関節角度を :reset-pose にセット
   (send *irtviewer* :draw-objects) ;; 描画
   (unix::usleep (* 2000 1000))
   ;;
   (warn "move real robot (*ri*) with 4000 [ms]~%~%")
   (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000) ;;4000[ms] で実行
   (send *ri* :wait-interpolation) ;; 補間をまつ
   ;;
   ))


(defun dxl-arm-check ()
  (let ((send-av))
    ;;arm
    (dotimes (i (- *arm-dof* 1))
      (setq send-av (instantiate float-vector *arm-dof*))
      (setf (elt send-av i) (+ (aref send-av i) 50))
      (send *dxl-armed-turtlebot* :angle-vector send-av)
      (send *irtviewer* :draw-objects)
      (warn ";;~%")
      (format t ";; move joint_~A of robot MODEL. Check it in irt viewer. Then press enter to move REAL robot.~%" i)
      (warn ";;~%")
      (do-until-key
       (x::window-main-one))
      (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)
      (warn ";;~%")
      (warn ";; Are real robot and model same? Check it, then press enter~%")
      (warn ";;~%")
      (do-until-key
       (x::window-main-one))
      (warn ";; Same?~%")
      (if (y-or-n-p)
	  (progn
	    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :init-pose) 4000)
	    (send *irtviewer* :draw-objects))
	(progn
	  (warn ";; fix arm direction!!!~%")
	  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :init-pose) 4000)
	  (send *irtviewer* :draw-objects)
	  (return-from dxl-arm-check))))
    ;;
    ;;gripper
    (send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(50))
    (send *irtviewer* :draw-objects)
    (warn ";;~%")
    (format t ";; move gripper of robot MODEL. Check it in irt viewer. Then press enter to move REAL robot.~%")
    (warn ";;~%")
    (do-until-key
     (x::window-main-one))
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000 :gripper-controller)
    (warn ";;~%")
    (warn ";; Are real robot and model same? Check it, then press enter~%")
    (warn ";;~%")
    (do-until-key
     (x::window-main-one))
    (warn ";; Same?~%")
    (if (y-or-n-p)
	(progn
	  (send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(0))
	  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000 :gripper-controller)
	  (send *irtviewer* :draw-objects))
      (progn
	(warn ";; fix gripper direction!!!~%")
	(send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(0))
	(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000 :gripper-controller)
	(send *irtviewer* :draw-objects)
	(return-from dxl-arm-check)))
    ;;
    (warn ";; finish program. back to reset pose~%")
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :reset-pose) 4000)
    (send *irtviewer* :draw-objects)
    ))
