#!/usr/bin/env roseus

(load "package://jedy_bringup/euslisp/jedy-interface.l")


(setq *pressed-key* nil)

;; irtviewer上でキーが押されるとそのキーの値を取る
(defmethod gl::glviewsurface
    (:keyrelease (event)
     (let ((buf (make-string 1)))
       (x::lookupstring event buf 1 0 0)
       ;; (format t ";; my :keyrelease event=~A, buf=~A ch=~A~%" event buf (char buf 0))
       ))
  (:keypress (event)
             (let ((buf (make-string 1)))
	             ;; (format t ";; my :keypress event=~A~%" event)
               (x::lookupstring event buf 1 0 0)
               (setq *pressed-key* buf)
	             ;; (x::print-event event)
	           ))
  )

(jedy-init :without-ri t)
;; (jedy-init :without-ri nil)


#|*********************************************
  キーボード入力によって目標座標を少しずつ動かすサンプル
***********************************************|#
(warn "(ik-demo0)~%")
(defun ik-demo0
  (&key (robot *jedy*))
  ;;逆運動学が解きやすい初期姿勢に変更
  (warn ";; move to reset-pose~%")
  (send robot :reset-pose)
  (when (boundp '*ri*)
    (send *ri* :angle-vector (send robot :angle-vector) 5000)
    (send *ri* :wait-interpolation))
  (objects (list robot))

  ;;
  ;;'e'を押すまで続ける
  (warn ";; if stop, then enter e~%")
  (warn ";;  h:left, j:down, k:up, l:right, f:forward, b:back~%")
  (let (w goal-endcoords ll ik-result)
    ;;もし腕しか使わない場合はlinklistをあらかじめ用意しておく
    ;;目標座標を作成する(デフォルトは手先位置と同じにする)
    (setq goal-endcoords
          (make-cascoords :pos (send (send robot :rarm :end-coords :copy-worldcoords) :worldpos)))
    ;;ループを回す
    (while t
      (x::window-main-one)
      (setq w *pressed-key*) ;;文字を取得
      (setq *pressed-key* nil)
      ;;文字によって操作を変える
      (cond
       ((equal w "e")
        (return-from nil)) ;;loopから抜けて終了
       ((equal w "h")  ;;左へ動かす
        (send goal-endcoords :locate #f(0  20 0) :local))
       ((equal w "j")  ;;下へ動かす
        (send goal-endcoords :locate #f(0 0 -20) :local))
       ((equal w "k")  ;;上へ動かす
        (send goal-endcoords :locate #f(0 0  20) :local))
       ((equal w "l")  ;;右へ動かす
        (send goal-endcoords :locate #f(0 -20 0) :local))
       ((equal w "f")  ;;前へ動かす
        (send goal-endcoords :locate #f( 20 0 0) :local))
       ((equal w "b")  ;;後へ動かす
        (send goal-endcoords :locate #f(-20 0 0) :local))
       ((equal w "r")  ;;reset-poseになる
        (send *jedy* :reset-pose)
        (setq goal-endcoords
              (make-cascoords :pos (send (send robot :rarm :end-coords :copy-worldcoords) :worldpos))))
       ((not w)) ;;何も入れられなければ何もしない
       (t
        (warn ";; no such command~%")
        (warn ";; if stop, then enter e~%")
        (warn ";;  h:left, j:down, k:up, l:right, f:forward, b:back, r:reset-pose~%")
        ))

      ;;目標値goal-endcoordsに向かって逆運動学を解く．
      ;;  inverse-kinematicsという逆運動学を解くmethodを呼び出す．
      (setq ik-result (send *jedy* :rarm :inverse-kinematics goal-endcoords :rotation-axis nil
                            :warnp nil))
      (when (null ik-result)
        (warn "Failed to solve inverse kinematics~%")
        (warn ";; no such command~%")
        (warn ";; if stop, then enter e~%")
        (warn ";;  h:left, j:down, k:up, l:right, f:forward, b:back, r:reset-pose~%"))

      (send *irtviewer* :objects (list robot goal-endcoords))
      (send *irtviewer* :draw-objects)
      (unix::usleep (* 1000 10))

      ;;angle-vectorで逆運動学を解いた姿勢に移行する
      (when (boundp '*ri*)
        (send *ri* :angle-vector (send robot :angle-vector) 500)
        (send *ri* :wait-interpolation)
        )
      ))
  (warn ";; finished~%")
  t)

(ik-demo0)
