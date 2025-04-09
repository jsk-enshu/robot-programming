#!/usr/bin/env roseus

(load "package://jedy_bringup/euslisp/jedy-interface.l")
(jedy-init)

;; 利用可能なjointnameをprint
(print (send-all (send *jedy* :joint-list) :name))

;; 一度すべての関節をサーボオフにする
(send *ri* :servo-off)

;; 頭部のみ個別にservo-on
(send *ri* :servo-on :names (list "head_joint0" "head_joint1"))

;; 頭部の関節名を指定して角度を変える
(send *jedy* :head_joint0 :joint-angle 20)
(send *jedy* :head_joint1 :joint-angle 50)

(send *irtviewer* :draw-objects)

;; 関節指令を実機に反映させる (3000msかけておくる)
(send *ri* :angle-vector (send *jedy* :angle-vector) 3000)

;; キーが押されるまでうなずき続ける
(do-until-key
    (send *jedy* :head_joint0 :joint-angle 0)
  (send *jedy* :head_joint1 :joint-angle 0)
  (send *ri* :angle-vector (send *jedy* :angle-vector) 1000)
  (send *ri* :wait-interpolation)  ;; 補間がおわるのを待つ

  (send *jedy* :head_joint0 :joint-angle 0)
  (send *jedy* :head_joint1 :joint-angle -90)
  (send *ri* :angle-vector (send *jedy* :angle-vector) 1000)
  (send *ri* :wait-interpolation)  ;; 補間がおわるのを待つ
    )
