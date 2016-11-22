;; joy sample for turtlebot ensyu

(load "package://roseus/test/joy.l")
(init-ps3joy) ;; *joy* という大域変数のセット

(progn
  (ros::rate 10)
  (warn "~%press circle button~%")
  (do-until-key
   (ros::spin-once) ;; subscribe をよんで, joyというtopic を読み込む
   (ros::sleep)
   (if (= (send *joy* :circle-button) 1) ;; joyの結果から,丸ボタンの値だけ抜き出す
       (print "circle button pressed"))
   )
  )
