;; robotの初期化
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)

;; arm approach  pose
(send *ri* :angle-vector #f(0 0 -90 0 90 0) 3000)
;;(send *ri* :wait-interpolation) ;; does not work?
(unix:sleep 4)

;; open gripper
(send *ri* :stop-grasp)

;; go-pos
(send *ri* :go-pos 0.56 0 1)
(unix:sleep 6)

;; grasp
;; still some bug in start-grasp, so we have to directly use :move-gripper
(send *ri* :move-gripper -30 :tm 1500 :wait t)
(unix:sleep 2)

(send *ri* :angle-vector #f(90 0 45 0 45 45)) ;;  arm tuck pose
