#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")

(ros::roseus "checkpoint1_3_3_imu_euslisp")

;; 加速度データからyaw, pitch, rollを計算する関数
(defun calc-yaw-pitch-roll (linear-acceleration)
  (let* ((x (send linear-acceleration :x))
         (y (send linear-acceleration :y))
         (z (send linear-acceleration :z))
         ;; 各角度の計算 (radian単位)
         (roll (atan (- x) (sqrt (+ (* y y) (* z z)))))
         (pitch (atan y z))
         (yaw 0))  ;; Yawは加速度だけでは計算できないので、0を仮定

    ;; 結果をリストとして返す (roll, pitch, yaw)
    (list roll pitch yaw)))

(defun imu-cb
    (msg)
  (let ((rpy (calc-yaw-pitch-roll (send msg :linear_acceleration))))
    (ros::ros-info "roll: ~A pitch ~A yaw ~A" (elt rpy 0) (elt rpy 1) (elt rpy 2))
    ))

(ros::subscribe "/imu_publisher/imu" sensor_msgs::Imu #'imu-cb)

(ros::ros-info "ready to subscribe !!!")
(ros::spin)
