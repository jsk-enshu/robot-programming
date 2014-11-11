#|**************************************************
  robotサーバ(simulator)をつないで、動作させてみた簡単な例
  bumperを押したら台車が動く

  2010/12/1 written by nakanish

**************************************************|#
(load "robot-server-simulator.l")

(warn "(demo)~%")
(defun demo
  ()
  (let (result) ;;局所変数の定義
    ;;robot-server(simulator)とつながってなかったらつなぐ
    (when (not *rs-sim*) (connect-robotsim-server))
    ;;
    ;;台車の制御モードを位置制御に変更する
    (send *rs-sim* :daisya-controlmode 2)

    ;;keyを押されるまで続ける
    (do-until-key
     ;;bumperの状態を取得する(6つの要素からなるベクトルが帰ってくるはず)
     (setq result (send *rs-sim* :bumper-vector))
     ;;bumperの状態を表示する(6つの要素からなるベクトルが表示されるはず)
     (warn "bumper-vector:~a~%" result)
     ;;bumperの状態に応じて台車の挙動を変更
     (cond
      ;;bumper0,あるいはbumper5が押されていた場合
      ((or (= (elt result 0) 1) (= (elt result 5) 1))
       ;;1秒かけて300[mm]バック&回転する
       (send *rs-sim* :daisya-position-vector #f(-300 0 50) 1000)
       ;;1秒休む
       (unix:sleep 1)
       )
      ;;bumper2,あるいはbumper3が押されていた場合
      ((or (= (elt result 2) 1) (= (elt result 3) 1))
       (send *rs-sim* :daisya-position-vector #f(300 0 0) 1000)
       (unix:sleep 1)
       )
      ;;bumper4が押されていた場合
      ((= (elt result 4) 1)
       (send *rs-sim* :daisya-position-vector #f(0 200 0) 1000)
       (unix:sleep 1)
       )
      ;;bumper1が押されていた場合
      ((= (elt result 1) 1)
       (send *rs-sim* :daisya-position-vector #f(0 -200 0) 1000)
       (unix:sleep 1)
       )
      ;;それ以外の場合
      (t
       ;;とりあえず直進, usleepはマイクロスリープの意味
       (send *rs-sim* :daisya-position-vector #f(300 0 0) 1000)
       (unix:usleep (* 1000 1000))
       ))
     )
    ;;台車制御を速度制御に戻しておく(ついでにspeedも0にしておく)
    (send *rs-sim* :daisya-controlmode 1)
    (send *rs-sim* :daisya-velocity-vector #f(0 0 0))
    ))
