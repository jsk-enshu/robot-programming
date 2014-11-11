;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 必要なファイルのロードとROSのtopicのsubscribe/advertise設定
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "dynamixel_msgs") ;; dynamixelのメッセージを用いるためのload設定
(ros::roseus "test_arm") ;; ros initを行う

;; 7自由度分いっぺんに動かすサンプルになっているが、
;; いずれも1軸ずつ動かすことができる
(defvar *arm-dof* 7)
(setq *state-list* (make-list *arm-dof*))

;; subscribeの設定
;;   どのtopicをsubscribeするか (センサ値などの取得用)

;;   dynamixel_msgs::JointState型のメッセージをsubscribeするときに呼ばれるコールバック関数
;;     具体的には/arm_j0_controllder/stateなどをsubscribeすることになる
(defun dxl-state-cb
  (msg)
  (setf (elt *state-list* (1- (read-from-string (string-left-trim "arm_joint" (send msg :name))))) msg))

;;   subscriberの登録
(dotimes (i *arm-dof*)
  (ros::subscribe (format nil "/arm_j~d_controller/state" (1+ i)) ;; subscribeしたいtopic名
                  dynamixel_msgs::JointState ;; topicの型名
                  #'dxl-state-cb)) ;; コールバック関数の設定, #'で指定するのがEuslispのポイント

;; advertiseの設定
;;  どのtopicをpublishするか (指令値などの送信用)
(dotimes (i *arm-dof*)
  (ros::advertise (format nil "/arm_j~d_controller/command" (1+ i)) ;; topic名
                  std_msgs::Float64 ;; topicの型
                  1)) ;; キューの数


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; サンプル関数
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun test-read-state () ;; 各サーボの状態を全軸分読み出す
  (ros::spin-once)
  (pprint (send-all *state-list* :slots))
  t)
(warn ";; (test-read-state)~%")

(defun test-send-angle-vector-once () ;; 各軸、一回ずつ関節を動かす
  (ros::spin-once)
  ;; 現在関節角度からそれぞれ10度動かす
  (let ((current-av (mapcar #'rad2deg (send-all *state-list* :current_pos))))
    (dotimes (i *arm-dof*)
      (format t ";; send command position to arm ~d~%;; press enter~%" (1+ i))
      (read-line)
      (ros::publish (format nil "/arm_j~d_controller/command" (1+ i)) ;; topic名
                    (instance std_msgs::Float64 :init :data (deg2rad (+ 10 (elt current-av i))))) ;; 指令値をstd_msgs::Float64に変換して送信
      )
    t))
(warn ";; (test-send-angle-vector-once) ~%")

(defun test-send-angle-vector-loop (&key (move nil)) ;; ループで各関節を動かすサンプル
  (ros::spin-once)
  (let ((current-av '(0 0 0 0 0 0 0))
        (ii 0))
    (do-until-key
     (dotimes (i *arm-dof*)
       (format t ";; send for arm ~d~%;; press enter~%" (1+ i))
       (ros::publish (format nil "/arm_j~d_controller/command" (1+ i))
                     (if move
                         (instance std_msgs::Float64 :init :data (deg2rad (+ (* (- (mod ii 3) 1) 5) (elt current-av i))))
                       (instance std_msgs::Float64 :init :data (deg2rad (+ 0 (elt current-av i))))))
       )
     (format t ";; temps ~A~%" (send-all *state-list* :motor_temps))
     (unix:usleep 1000000) ;; 1秒まつ
     (incf ii)
     )
    t))
(warn ";; (test-send-angle-vector-loop :move nil) ~%")
(warn ";; (test-send-angle-vector-loop :move t) ~%")
