;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 台車ロボットのrobot-interface
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "kobuki_msgs")
(ros::load-ros-manifest "move_base_msgs")
;;(ros::load-ros-manifest "linux_hardware") linux_hardware is obsolete, use  linux_peripheral_interface

(defun get-method-list-for-turtlebot-interface ()
'(
  ;; サービスのwaitやsubscribeコールバック関数設定を行う
  (:initialize-turtlebot-ros
   ()
   ;; subscriber
   (ros::subscribe "/mobile_base/events/bumper" kobuki_msgs::BumperEvent
                   #'send self :kobuki-bumper-states-callback :groupname groupname)
   (ros::subscribe "/mobile_base/events/button" kobuki_msgs::ButtonEvent
                   #'send self :kobuki-button-states-callback :groupname groupname)
   (ros::subscribe "/mobile_base/events/power_system" kobuki_msgs::PowerSystemEvent
                   #'send self :kobuki-power-system-states-callback :groupname groupname)
   (ros::subscribe "/mobile_base/events/wheel_drop" kobuki_msgs::WheelDropEvent
                   #'send self :kobuki-wheel-drop-states-callback :groupname groupname)
   (ros::subscribe "/mobile_base/events/cliff" kobuki_msgs::CliffEvent
                   #'send self :kobuki-cliff-states-callback :groupname groupname)
   (ros::subscribe "/mobile_base/sensors/imu_data" sensor_msgs::Imu
                   #'send self :kobuki-imu-states-callback :groupname groupname)
   ;;(ros::subscribe "/laptop_charge" linux_hardware::LaptopChargeStatus #'send self :laptop-charge-callback)
   ;; publisher
   (dotimes (i 2) (ros::advertise (format nil "/mobile_base/commands/led~d" (1+ i)) kobuki_msgs::LED 1))
   (ros::advertise (format nil "/mobile_base/commands/sound") kobuki_msgs::sound 1)
   (ros::advertise "/cmd_vel_mux/input/navi" geometry_msgs::Twist 1)
   )
  (:kobuki-bumper-states-callback
   (msg)
   (send self :set-robot-state1 :bumper (list :state (send msg :state) :bumper (send msg :bumper))))
  (:kobuki-button-states-callback
   (msg)
   (send self :set-robot-state1 :button (list :state (send msg :state) :button (send msg :button))))
  (:kobuki-power-system-states-callback
   (msg)
   (send self :set-robot-state1 :power-system (list :event (send msg :event))))
  (:kobuki-wheel-drop-states-callback
   (msg)
   (send self :set-robot-state1 :wheel-drop (list :state (send msg :state) :wheel (send msg :wheel))))
  (:kobuki-cliff-states-callback
   (msg)
   (send self :set-robot-state1 :cliff (list :sensor (send msg :sensor) :state (send msg :state) :bottom (send msg :bottom))))
  (:kobuki-imu-states-callback
   (msg)
   (send self :set-robot-state1 :imu (list :orientation (send msg :orientation)
                                           :orientation_covariance (send msg :orientation_covariance)
                                           :angular_velocity_covariance (send msg :angular_velocity_covariance)
                                           :linear_acceleration (send msg :linear_acceleration)
                                           :linear_acceleration_covariance (send msg :linear_acceleration_covariance))))
;  (:laptop-charge-callback
;   (msg)
;   (send self :set-robot-state1 :laptop-charge nil)
;   )
  (:def-vector-value
   (&key (simulate-func #'(lambda () (instantiate float-vector 3)))
         (raw-data-name)
         (vector-length 3)
         (state-name :state)
         (value-name))
   (if (send self :simulation-modep)
       (funcall simulate-func)
     (let ((bp (send self raw-data-name))
           (ret (instantiate float-vector vector-length)))
       (if bp (setf (elt ret (cadr (memq value-name bp))) (cadr (memq state-name bp))))
       ret)))
  (:raw-bumper-data () (cdr (assoc :bumper robot-state)))
  (:raw-button-data () (cdr (assoc :button robot-state)))
  (:raw-wheel-drop-data () (cdr (assoc :wheel-drop robot-state)))
  (:raw-cliff-data () (cdr (assoc :cliff robot-state)))
  (:raw-imu-data () (cdr (assoc :imu robot-state)))
  (:imurot
   ()
   (let* ((qt (cadr (memq :orientation (send self :raw-imu-data)))))
     (quaternion2matrix (float-vector (send qt :x) (send qt :y) (send qt :z) (send qt :w)))))
  (:update-robot-state
   (&rest args)
   (prog1
       (send-super :update-robot-state)
     (if (send self :simulation-modep)
         (send robot :simulate objects))
     )
   )

  ;; センサ取得
  (:bumper-vector
   ()
   "Get bumper value vector."
   (send self :def-vector-value
         :simulate-func #'(lambda () (concatenate float-vector (send-all (send robot :bumper-sensors) :read)))
         :raw-data-name :raw-bumper-data
         :vector-length (length (send robot :bumper-sensors))
         :value-name :bumper))
  (:button-vector
   ()
   "Get button value vector."
   (send self :def-vector-value
         :raw-data-name :raw-button-data :value-name :button))
  (:wheel-drop-vector
   ()
   "Get wheel drop sensor vector."
   (send self :def-vector-value
         :raw-data-name :raw-wheel-drop-data :value-name :wheel :vector-length 2))
  (:cliff-vector
   ()
   "Get cliff sensor vector."
   (send self :def-vector-value
         :raw-data-name :raw-cliff-data :value-name :sensor))
  (:cliff-bottom-vector
   ()
   "Get cliff bottom vector."
   (send self :def-vector-value
         :raw-data-name :raw-cliff-data :value-name :sensor :state-name :bottom))
  (:imucoords
   ()
   "Get imucoords."
   (make-coords :rot (send self :imurot)))
  (:power-system-vector
   ()
   "Get power system vector."
   (cdr (assoc :power-system robot-state)))

  ;; publish
  (:publish-led
   (id value)
   "Publish topic to turn on/off LEG. id should be 1-2. Value should be :black, :green, :orange, and :red."
   (ros::publish
    (format nil "/mobile_base/commands/led~d" id)
    (instance kobuki_msgs::Led :init
              :value (case value ;; see kobuki_msgs/msg/Led.msg
                       (:black 0)
                       (:green 1)
                       (:orange 2)
                       (:red 3)
                       (t 0)
                       ))))
  (:publish-sound
   (value)
   "Publish topic to turn on sound. value should be :on, :off, :recharge, :button, :error, :cleaningstart, and :cleaningend."
   (ros::publish
    (format nil "/mobile_base/commands/sound")
    (instance kobuki_msgs::Sound :init
              :value (case value
                       (:on 0)
                       (:off 1)
                       (:recharge 2)
                       (:button 3)
                       (:error 4)
                       (:cleaningstart 5)
                       (:cleaningend 6)
                       (t 0)
                       ))))

  ;; overwrite go-velocity command
  (:go-velocity
   (x y d &optional (msec 1000) &key (stop t) (wait)) ;; [m/sec] [m/sec] [deg/sec]
   "Moving by desired x y translational velocity and yaw rotational velocity. x and y is [m/s] and d is [deg/s]."
   (unless joint-action-enable
     (let ((orig-coords (send robot :copy-worldcoords))
	   (coords (send robot :copy-worldcoords)))
       (send coords :translate (float-vector (* x msec) (* y msec) 0))
       (send coords :rotate (deg2rad (/ (* d msec) 1000.0)) :z)
       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	   ((> curr-tm msec))
	 (send robot :newcoords (midcoords (/ curr-tm msec) orig-coords coords))
	 (if viewer (send self :draw-objects))))
     (return-from :go-velocity t))
   (let ((lin (instance geometry_msgs::vector3 :init :x x :y y :z 0))
         (ang (instance geometry_msgs::vector3 :init :x 0 :y 0 :z (deg2rad d))))
     (ros::publish "/cmd_vel_mux/input/navi"
                   (instance geometry_msgs::Twist :init :linear lin :angular ang)))
   )
  )
)
