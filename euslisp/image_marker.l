#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-package "opencv_apps")
(ros::load-ros-package "image_view2")
(defclass msg-cb-class
    :super propertied-object
    :slots ())

(defmethod msg-cb-class
    (:init () (ros::subscribe "/camshift/track_box" opencv_apps::RotatedRectStamped #'send self :msg-cb))
    (:msg-cb (msg)
        (print (send msg :rect))
        (setq marker (instance image_view2::ImageMarker2 :init))
        (send marker :type 0)
        (send marker :position (instance geometry_msgs::Point :init :x (send msg :rect :center :x) :y (send msg :rect :center :y) :z 0))
        (ros::publish "/image_marker" marker)
        ))

(ros::roseus "client")
(ros::advertise "/image_marker" image_view2::ImageMarker2 1)
(setq m (instance msg-cb-class :init))

(do-until-key
    (ros::spin-once)
)

(ros::rosens "shutdown")
(exit)
