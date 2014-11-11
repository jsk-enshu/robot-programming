;;;
;;; 移動台車モデルをの逆運動学を解いて、armや台車を操縦するサンプル
;;;    
;;;
(load "daisya.l")    ;;台車モデルを定義したファイルをload
(load "irteusext.l")

#|******************************
  描画周りの初期起動処理
********************************|#
(if (not (boundp '*irtviewer*))
    (setq *irtviewer* (make-irtviewer)))
(defun init-daisya nil
  (setq *daisya* (instance daisyaclass :init)) 
  (send *daisya* :reset-pose)
  (objects (list *daisya*)))
(init-daisya)

#|*********************************************
  キーボード入力によって、目標座標を少しずつ動かすサンプル
***********************************************|#
(warn "(ik-demo0)~%")
(defun ik-demo0
  (&key (step 10)        ;;stepは一回のループで目標を動かす距離
        (use-base nil)  ;;車輪を使ってIKを解くかどうか
        (robot *daisya*)
        )
  ;;逆運動学が解きやすい初期姿勢に変更
  (send robot :reset-pose)
  (when (boundp '*ri*)
    (send *ri* :angle-vector (send robot :angle-vector) 5000)
    (send *ri* :wait-interpolation))
  (objects (list robot))

  ;;
  ;;'e'を押すまで続ける
  (warn ";; if stop, then enter e~%")
  (warn ";;  h:left, j:down, k:up, l:right, f:forward, b:back~%")
  (let (w goal-endcoords ll)
    ;;もし腕しか使わない場合はlinklistをあらかじめ用意しておく
    ;;目標座標を作成する(デフォルトは、台車の手先位置と同じにする)
    (setq goal-endcoords
          (make-cascoords :pos (send (send robot :arm :end-coords :copy-worldcoords) :worldpos)))
    ;;ループを回す
    (while t
      (setq w (read-line)) ;;文字を取得
      ;;文字によって操作を変える
      (cond
       ((equal w "e")
        (return-from nil)) ;;loopから抜けて終了
       ((equal w "h")  ;;左へ動かす
        (send goal-endcoords :locate (float-vector 0 step 0)))
       ((equal w "j")  ;;下へ動かす
        (send goal-endcoords :locate (float-vector 0 0 (* -1 step))))
       ((equal w "k")  ;;上へ動かす
        (send goal-endcoords :locate (float-vector 0 0 step)))
       ((equal w "l")  ;;右へ動かす
        (send goal-endcoords :locate (float-vector 0 (* -1 step) 0)))
       ((equal w "f")  ;;前へ動かす
        (send goal-endcoords :locate (float-vector step 0 0)))
       ((equal w "b")  ;;後へ動かす
        (send goal-endcoords :locate (float-vector (* -1 step) 0 0)))
       ((not w)) ;;何も入れられなければ何もしない
       (t
        (warn ";; no such command~%")
        (warn ";; if stop, then enter e~%")
        (warn ";;  h:left, j:down, k:up, l:right, f:forward, b:back~%")
        ))
      ;;目標値end-coordsに向かって逆運動学を解いて、動かす
      ;;  inverse-kinematicsという逆運動学をとくmethodを呼び出す。
      (cond
       (use-base ;;車輪を使う場合
        (send robot :inverse-kinematics goal-endcoords :rotation-axis nil :debug-view nil :use-base use-base))
       (t
        (send robot :inverse-kinematics goal-endcoords :rotation-axis nil :debug-view nil :link-list ll))
       )
      (send *irtviewer* :objects (list robot goal-endcoords))
      (send *irtviewer* :draw-objects)
      (when (boundp '*ri*)
        (send *ri* :angle-vector (send robot :angle-vector) 500)
        (send *ri* :wait-interpolation)
        )
      ))
  (warn ";; finished~%")
  t)
