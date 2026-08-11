(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init)

(ros::load-ros-manifest "sensor_msgs")
(ros::roseus "joylistener" :anonymous t)

(defun joy-cb (msg)
(let ((buttons))
    (progn
        (print (list 'joy (send msg :buttons)))
        (setq buttons (send msg:buttons))
        (cond
            (= buttons aref(buttons 4))
                (send *ri* :go-velocity 0.1 0 0)
        )
    )
)                
                
