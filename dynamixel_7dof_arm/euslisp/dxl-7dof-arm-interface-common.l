;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 7dof アームロボットのrobot-interfaceクラスのメソッド定義部
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "dynamixel_controllers")
;;(ros::load-ros-manifest "dynamixel_msgs")
;;(ros::load-ros-manifest "control_msgs")
(ros::roseus-add-msgs "dynamixel_msgs")
(ros::roseus-add-msgs "control_msgs")

(defun get-method-list-for-dxl-7dof-arm-interface ()
'(
  ;; サービスのwaitやsubscribeコールバック関数設定を行う初期化メソッド
  (:initialize-arm-robot-ros
   ()
   ;; subscriber
   (dotimes (i (length (send robot :angle-vector)))
     (ros::subscribe
      (format nil "/arm_j~d_controller/state" (1+ i))
      dynamixel_msgs::JointState
      #'send self :dynamixel-motor-states-callback :groupname groupname))
   (unless (send self :simulation-modep)
   ;; services
     (dotimes (i (length (send robot :angle-vector)))
       (ros::wait-for-service
        (format nil "/arm_j~d_controller/set_compliance_slope" (1+ i)))
       (ros::wait-for-service
        (format nil "/arm_j~d_controller/torque_enable" (1+ i)))
       (ros::wait-for-service
        (format nil "/arm_j~d_controller/set_torque_limit" (1+ i)))
       )
     )
   ;; define actions
   (dolist (l (list
               (cons :fullbody-controller "fullbody_controller/follow_joint_trajectory")
               (cons :gripper-controller "gripper_controller/follow_joint_trajectory")
               ))
     (let ((type (car l))
           (name (cdr l))
           action)
       (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
                             controller-actions))
       (setf (gethash type controller-table)
             (if action
                 (list action)
               (list (instance ros::simple-action-client :init
                               name
                               control_msgs::followjointtrajectoryaction
                               :groupname groupname))))
       ))
   )
  ;; TODO
  ;;  This method is tempolary code.
  ;;  dynamixel_controller_manager should publish /dxl_7dof_arm/joint_states
  (:dynamixel-motor-states-callback
   (msg)
   ;; for initialize
   (dolist (key '(:position :velocity :effort :name))
     ;; neglect /joint_states from turtlebot
     (unless (and (cdr (assoc key robot-state))
                  (= (length (send robot :angle-vector)) (length (cdr (assoc key robot-state)))))
       (send self :set-robot-state1 key
             (if (eq key :name)
                 (make-list (length (send robot :angle-vector)))
               (instantiate float-vector (length (send robot :angle-vector)))))))
   ;; update values
   (dolist (key '(:position :velocity :effort :name))
     (setf (elt (cdr (assoc key robot-state)) (1- (elt (send msg :motor_ids) 0)))
           (case key
             (:position (send msg :current_pos))
             (:name (send msg :name))
             (:velocity (send msg :velocity))
             (:effort (send msg :load)))
           )
     )
   )
  (:fullbody-controller
   ()
   (list
    (list
     (cons :controller-action "fullbody_controller/follow_joint_trajectory")
     (cons :controller-state "fullbody_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                                (send-all
                                 (remove-if #'(lambda (x)
                                                (member x (send robot :gripper :arm :joint-list)))
                                            (send robot :joint-list))
                                 :name)
                                )))
    )
   )
  (:gripper-controller
   ()
   (list
    (list
     (cons :controller-action "gripper_controller/follow_joint_trajectory")
     (cons :controller-state "gripper_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (if (symbolp n) (symbol-name n) n))
                                (send-all (send robot :gripper :arm :joint-list) :name)
                                )))
    )
   )
  (:default-controller
   ()
   (send self :fullbody-controller)
   ;;(append (send self :fullbody-controller) (send self :gripper-controller))
   )
  ;; raw dynamixel command
  ;;   TODO : define these methods by considering pr2eus?
  ;; for controller parameters, please see:
  ;;  http://www.besttechnology.co.jp/modules/knowledge/?Dynamixel%E3%82%B3%E3%83%B3%E3%83%88%E3%83%AD%E3%83%BC%E3%83%AB%E3%83%86%E3%83%BC%E3%83%96%E3%83%AB%28DX%2CRX%2CAX%E3%82%B7%E3%83%AA%E3%83%BC%E3%82%BA%E7%94%A8%29#m041ac16
  (:set-compliance-slope ;; for one joint
   (id slope)
   "Set compliance slope for one joint. id should be 1-7. slope is 32 by default."
   (ros::service-call
    (format nil "/arm_j~d_controller/set_compliance_slope" id)
    (instance dynamixel_controllers::setcomplianceslopeRequest :init
              :slope (round slope)))
   )
  (:compliance-slope-vector
   (av)
   "Set compliance slope vector for all joints. #f(32 32 32 32 32 32 32) by default."
   (dotimes (i (length av))
     (send self :set-compliance-slope (1+ i) (elt av i)))
   )
  (:set-torque-limit
   (id torque-limit)
   "Set torque limit for one joint. id should be 1-7. torque-limit should be within [0, 1]."
   (ros::service-call
    (format nil "/arm_j~d_controller/set_torque_limit" id)
    (instance dynamixel_controllers::SetTorqueLimitRequest :init
              :torque_limit torque-limit)))
  (:torque-enable
   (id torque-enable)
   "Configure joint torque mode for one joint. id sohuld be 1-7. If torque-enable is t, move to torque control mode, otherwise, move to joint positoin mode."
   (ros::service-call
    (format nil "/arm_j~d_controller/torque_enable" id)
    (instance dynamixel_controllers::TorqueEnableRequest :init
              :torque_enable torque-enable)))

  ;; サーボON/OFFメソッド
  (:servo-on
   (id)
   "Servo On for one joint. id should be 1-7."
   (send self :servo-on-off id t))
  (:servo-off
   (id)
   "Servo Off for one joint. id should be 1-7."
   (send self :servo-on-off id nil))
  (:servo-on-all
   ()
   "Servo On for all joints."
   (dotimes (i (length (send robot :angle-vector)))
     (send self :servo-on-off (1+ i) t)))
  (:servo-off-all
   ()
   "Servo Off for all joints."
   (dotimes (i (length (send robot :angle-vector)))
     (send self :servo-on-off (1+ i) nil)))
  (:servo-on-off
   (id on/off) ;; id = 1-7, t -> "On", nil -> "Off"
   (format t ";; servo ~A id = ~d~%" (if on/off "On" "Off") id)
   (send self :torque-enable id on/off)
   (if on/off ;; just for servo off->on
       (send self :set-torque-limit id 1.0)))

  ;; 把持モード開始メソッド
  (:start-grasp
   (&optional (arm :arm) &key ((:gain g) 0.5) ((:objects objs) objects))
   "Start grasp mode."
   (send self :set-compliance-slope 7 1023)
   (send self :set-torque-limit 7 g)
   (send robot :gripper arm :angle-vector
         (send-all (send robot :gripper arm :joint-list) :min-angle))
   (send self :angle-vector (send robot :angle-vector) 1000 :gripper-controller)
   (send self :wait-interpolation :gripper-controller)
   (send self :state)
   (send robot :gripper arm :angle-vector
         (mapcar #'(lambda (x) (- x 5)) (send-all (send robot :gripper arm :joint-list) :joint-angle))) ;; 5[deg]
   (send self :angle-vector (send robot :angle-vector) 200 :gripper-controller)
   (send self :wait-interpolation :gripper-controller)
   )
  ;; 把持モード停止メソッド
  (:stop-grasp
   (&optional (arm :arm) &key (wait nil))
   "Stop grasp mode."
   (send robot :gripper arm :angle-vector
         (send-all (send robot :gripper arm :joint-list) :max-angle))
   (send self :angle-vector (send robot :angle-vector) 1000 :gripper-controller)
   (send self :set-compliance-slope 7 32)
   (send self :set-torque-limit 7 1.0)
   (send self :wait-interpolation :gripper-controller)
   )
  )
)
