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

   (dotimes (i (+ (length (send robot :angle-vector)) 1))
     (ros::subscribe
      (if (eq i 6)
          (format nil "/gripper_joint_controller/state")
        (format nil "/arm_j~d_controller/state" (1+ i)))
      dynamixel_msgs::JointState
      #'send self :dynamixel-motor-states-callback :groupname groupname))

   (unless (send self :simulation-modep)
   ;; services for arm joints
     (dotimes (i (length (send robot :angle-vector)))
       (ros::wait-for-service
        (format nil "/arm_j~d_controller/set_compliance_slope" (1+ i)))
       (ros::wait-for-service
        (format nil "/arm_j~d_controller/torque_enable" (1+ i)))
       (ros::wait-for-service
        (format nil "/arm_j~d_controller/set_torque_limit" (1+ i)))
       )
     ;; services for gripper joint
     (ros::wait-for-service
      (format nil "/gripper_joint_controller/set_compliance_slope"))
     (ros::wait-for-service
      (format nil "/gripper_joint_controller/torque_enable"))
     (ros::wait-for-service
      (format nil "/gripper_joint_controller/set_torque_limit"))
     )
   ;; arm controller action
   (dolist (l (list
               (cons :fullbody-controller "fullbody_controller/follow_joint_trajectory")
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

   ;; gripper controller action
   (setq gripper-action (instance ros::simple-action-client :init
                                    "/gripper_controller/follow_joint_trajectory"
                                    control_msgs::FollowJointTrajectoryAction
                                    :groupname groupname))
   ;; wait for gripper-action server
   (unless (and joint-action-enable (send gripper-action :wait-for-server 3))
     (setq joint-action-enable nil)
     (ros::ros-warn "~A is not respond, robot-interface is disabled" gripper-action))
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
                  (= (1+ (length (send robot :angle-vector))) (length (cdr (assoc key robot-state)))))
       (send self :set-robot-state1 key
             (if (eq key :name)
                 (make-list (1+ (length (send robot :angle-vector))))
               (instantiate float-vector (1+ (length (send robot :angle-vector))))))))
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
                                (send-all (send robot :joint-list) :name)
                                )))
    )
   )
  #|
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
  |#

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
   "Set compliance slope for one joint. id should be 1-6. slope is 32 by default."
   (ros::service-call
    (if (eq id 7)
        (format nil "/gripper_joint_controller/set_compliance_slope")
      (format nil "/arm_j~d_controller/set_compliance_slope" id))
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
    (if (eq id 7)
        (format nil "/gripper_joint_controller/set_torque_limit" id)
      (format nil "/arm_j~d_controller/set_torque_limit" id))
    (instance dynamixel_controllers::SetTorqueLimitRequest :init
              :torque_limit torque-limit)))
  (:torque-enable
   (id torque-enable)
   "Configure joint torque mode for one joint. id sohuld be 1-7. If torque-enable is t, move to torque control mode, otherwise, move to joint positoin mode."
   (ros::service-call
    (if (eq id 7)
        (format nil "/gripper_joint_controller/torque_enable" id)
      (format nil "/arm_j~d_controller/torque_enable" id))
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
   (dotimes (i (+ (length (send robot :angle-vector)) 1))
     (send self :servo-on-off (1+ i) t)))
  (:servo-off-all
   ()
   "Servo Off for all joints."
   (dotimes (i (+ (length (send robot :angle-vector)) 1))
     (send self :servo-on-off (1+ i) nil)))
  (:servo-on-off
   (id on/off) ;; id = 1-7, t -> "On", nil -> "Off"
   (format t ";; servo ~A id = ~d~%" (if on/off "On" "Off") id)
   (send self :torque-enable id on/off)
   (if on/off ;; just for servo off->on
       (send self :set-torque-limit id 1.0)))

  (:move-gripper
   (pos &key (tm 1000)  (wait t))
   (unless joint-action-enable
     (send (send robot :gripper_joint) :joint-angle pos )
     (send self :publish-joint-state)
     (if viewer (send self :draw-objects))
     (return-from :move-gripper nil))
   ;; for real robot
   (let* ((goal (send gripper-action :make-goal-instance))
          (goal-points nil)
          (joint-names (list (send (send robot :gripper_joint) :name))))
     (send goal :header :seq 1)
     (send goal :header :stamp (ros::time-now))

     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp (ros::time-now))

     (push (instance trajectory_msgs::JointTrajectoryPoint
                     :init
                     :positions (float-vector (deg2rad pos))
                     :velocities (float-vector 0)
                     :time_from_start (ros::time (/ tm 1000.0)))
           goal-points)

     (send self :spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send gripper-action :send-goal goal)
     )
   #|
   (send self :send-ros-controller
         gripper-action (list (send (send robot :gripper_joint) :name)) ;; action server and joint-names
         0  ;; start time
         (list
          (list (float-vector pos)                    ;; positions
                (instantiate float-vector 1) ;; velocities
                (/ tm 1000.0))))
   |#
   (if wait (send gripper-action :wait-for-result))
   )

  ;; 把持モード開始メソッド
  (:start-grasp
   (&key ((:gain g) 0.5))
   "Start grasp mode."
   (unless (send self :simulation-modep)
     (send self :set-compliance-slope 7 1023)
     (send self :set-torque-limit 7 g))
   (send self :move-gripper (send (send robot :gripper_joint) :min-angle) :tm 1000 :wait t)
   (send self :state)
   (send robot :gripper :arm :joint-angle (- (send robot :gripper :arm :joint-angle) 5)) ;; 5[deg]
   (send self :move-gripper (send robot :gripper :arm :joint-angle) :tm 200 :wait t)
   )

  ;; 把持モード停止メソッド
  (:stop-grasp
   ()
   "Stop grasp mode."
   (send self :move-gripper (send (send robot :gripper_joint) :max-angle) :tm 1000 :wait t)
   (unless (send self :simulation-modep)
     (send self :set-compliance-slope 7 32)
     (send self :set-torque-limit 7 1.0))
   )
  )
)
