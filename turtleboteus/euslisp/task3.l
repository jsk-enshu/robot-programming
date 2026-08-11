(turtlebot-init)

(ros::load-ros-manifest "sensor_msgs")
(ros::roseus "listener" :anonymous t)

;; callback function
(defun battery-cb (msg)
    (progn
    (print (list 'laptopcharge (send msg :percentage)))
    (when (< 50 (send msg :percentage))
        (send *ri* :publish-sound :error)
)))

(ros::subscribe "/laptop_charge" sensor_msgs::BatteryState #'battery-cb)



(setq state 0)
(setq wait 0)
(ros::rate 100)
(progn
    (while t
        ;(print (send *ri* :laptop-charge))
        (ros::spin-once)
        ;(print "here")
        (cond
            ((= state 0)
                 (if (equal (send *ri* :state :bumper-vector) #f(0.0 0.0 0.0))
                    (send *ri* :go-velocity 0 0 0)
                    (setq state 1)
                 )
            )
            ((= state 1)
                 (if (> wait 100)
                    (progn
                        (setq state 0)
                        (setq wait 0)
                    )
                    (progn
                        (send *ri* :go-velocity -0.1 0 30)
                        (setq wait (+ wait 1))
                    )
                 )
             )
            (t
                (print "default")
            )
        )
     )
    (send *ri* :go-velocity 0 0 0)
)
                


        
