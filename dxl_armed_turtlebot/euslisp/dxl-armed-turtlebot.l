;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; turtlebotとdynamixel-7dof-armを組み合わせたアーム台車ロボット
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; loadする台車・アームのモデルファイル
(require :turtlebot-with-sensors-robot "package://turtleboteus/euslisp/turtlebot-with-sensors-robot.l")
(require :dxl-7dof-arm-robot "package://dynamixel_7dof_arm/euslisp/dxl-7dof-arm-robot.l")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; アーム台車ロボットモデルクラス
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;   turtlebotは継承、dynamixel-7dof-armはスロット変数として確保
(defclass dxl-armed-turtlebot-robot
  :super turtlebot-with-sensors-robot
  :slots (arm-robot
          arm-base-fixed-joint ;; アームのルートリンクと台車を接続するための関節
          )
  )

(defmethod dxl-armed-turtlebot-robot
  (:init
   (&rest args
    &key (name "dxl-armed-turtlebot-robot")
         (arm-origin-coords
          ;; upper arm version
          ;; (make-coords :pos (float-vector 85.725 9.525 410) ;; 9.525 = (/ 38.1 4.0), 38.1 = 2 pitch width, 85.725 = (+ (/ 152.4 2.0) (/ 38.1 4.0))
          ;;              :rpy (list 0 0 0))
          ;; lower arm version
          ;; (make-coords :pos (float-vector 85.725 9.525 402) ;; 9.525 = (/ 38.1 4.0), 38.1 = 2 pitch width, 85.725 = (+ (/ 152.4 2.0) (/ 38.1 4.0))
          ;;              :rpy (list 0 0 pi))
          ;; upper arm version + 長いアーム台座
          ;;(make-coords :pos (float-vector (+ 58.0 85.725) 9.525 402) ;; 9.525 = (/ 38.1 4.0), 38.1 = 2 pitch width, 85.725 = (+ (/ 152.4 2.0) (/ 38.1 4.0))
          ;;             :rpy (list 0 0 0))
          ;; lower arm version + 長いアーム台座
          (make-coords :pos (float-vector (+ 58.0 85.725) 9.525 402) ;; 9.525 = (/ 38.1 4.0), 38.1 = 2 pitch width, 85.725 = (+ (/ 152.4 2.0) (/ 38.1 4.0))
                       :rpy (list 0 0 pi))
          )
         ;; turtlebot CAD information is here:
         ;;   http://files.yujinrobot.com/kobuki/hardware/drawings/pdf
         ;; top plate cad:
         ;;   http://files.yujinrobot.com/kobuki/hardware/drawings/pdf/plate_top.pdf
         )
   (prog1
       (send-super* :init :name name args)
     ;; アームモデルの生成
     (setq arm-robot (instance dxl-7dof-arm-robot :init))
     (send arm-robot :newcoords arm-origin-coords)
     (send (car (send self :links)) :assoc (car (send arm-robot :links)))
     (setq arm-base-fixed-joint
           (instance rotational-joint
                     :init :min 0 :max 0
                     :name :arm-base-fixed-joint
                     :child-link (car (send arm-robot  :links))
                     :parent-link (car (send self :links))))
     (setq links (append (send self :links) (cdr (send arm-robot :links)))
           joint-list (send arm-robot :joint-list))
     (let ((bk-bodies (append bodies (send arm-robot :bodies))))
       (send self :init-ending)
       (setq bodies bk-bodies))
     (setq rarm-end-coords (arm-robot . rarm-end-coords))
     (setq rarm-root-link (car (arm-robot . rarm)))
     ;; xx-pose method copying
     (send self :method-copying "-pose")
     (send self :method-copying "_joint")
     (send self :method-copying "inverse-kinematics" t)
     t))
  (:method-copying
   (substr &optional (use-args nil))
   (let ((methods
          (remove-duplicates
           (remove-if-not
            #'(lambda (x) (substringp substr (string-downcase x)))
            (send arm-robot :methods)))))
     (dolist (me methods)
       (eval
        `(defmethod ,(send (class self) :name)
           (,me
            ,(if use-args '(&rest args) '())
            ,(if use-args `(send* arm-robot ,me args) `(send arm-robot ,me))
            )
           ))
       )))
  ;; limbs
  (:arm (&rest args)
        (unless args (setq args (list nil))) (send* self :limb :rarm args))
  )

;; アーム台車モデル生成関数
(defun dxl-armed-turtlebot
  ()
  "Generation function for dxl-armed-turtlebot-robot."
  (instance dxl-armed-turtlebot-robot :init))

#|
(defun test ()
  (setq *dxl-armed-turtlebot* (dxl-armed-turtlebot))
  (objects (list *dxl-armed-turtlebot*))
  (send *dxl-armed-turtlebot* :reset-pose)
  (send *dxl-armed-turtlebot* :inverse-kinematics
        (send (send (send *dxl-armed-turtlebot* :arm :end-coords) :copy-worldcoords) :translate (float-vector -100 0 -100) :world)
        :link-list (send *dxl-armed-turtlebot* :link-list (send *dxl-armed-turtlebot* :arm :end-coords :parent))
        :move-target (send *dxl-armed-turtlebot* :arm :end-coords)
        :revert-if-fail nil
        :rotation-axis nil :debug-view t)
  (read-line)
  (send *dxl-armed-turtlebot* :reset-pose2)
  (send *dxl-armed-turtlebot* :inverse-kinematics
        (send (send (send *dxl-armed-turtlebot* :arm :end-coords) :copy-worldcoords) :translate (float-vector 0 0 300) :world)
        :link-list (send *dxl-armed-turtlebot* :link-list (send *dxl-armed-turtlebot* :arm :end-coords :parent))
        :move-target (send *dxl-armed-turtlebot* :arm :end-coords)
        :revert-if-fail nil
        :rotation-axis nil :debug-view t)
  )
|#
