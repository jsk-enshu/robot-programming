#|**************************************************
  robotサーバをつないで、反応動作させてみた簡単な例
  bumperを押したら台車が動く

  つなぎ先をsimulator, 実機、切り替えるだけで
  同じ行動プログラムが動くようになっている例

  2010/12/1 written by nakanish

**************************************************|#
(load "robot-server-simulator.l")            ;;simulatorとつなぐための関数のロード
(load "../client/eus-client/robot-client.l") ;;実機とつなぐための関数のロード

(warn "(demo)         ;;シミュレータと接続して反応行動~%")
(warn "(demo :real t) ;;実機と接続して反応行動~%")
(defun demo
  (&key (real nil))
  (let (result) ;;局所変数の定義
    ;;
    ;;実機かシミュレータかを引数によって接続先を変更
    (cond
     (real
      (setq *rs* (connect-robot-server)))
     (t
      (setq *rs* (connect-robotsim-server))))
    ;; 
    #| ************************************************
      反応行動プログラム部分(ここから最後まで)
        sample.lやsample-sim.lとまったく同じ
        (ただし、sample-simの場合は*rs-sim*にバインドしていたが，*rs*に変更しているという点のみ違う

       シミュレータでも実機でもまったく同じプログラムが動いているのが味噌!!
    ***************************************************|#

    ;;台車の制御モードを位置制御に変更する
    (send *rs* :daisya-controlmode 2)
    ;;keyを押されるまで続ける
    (do-until-key
     ;;bumperの状態を取得する(6つの要素からなるベクトルが帰ってくるはず)
     (setq result (send *rs* :bumper-vector))
     ;;bumperの状態を表示する(6つの要素からなるベクトルが表示されるはず)
     (warn "bumper-vector:~a~%" result)
     ;;bumperの状態に応じて台車の挙動を変更
     (cond
      ;;bumper0,あるいはbumper5が押されていた場合
      ((or (= (elt result 0) 1) (= (elt result 5) 1))
       ;;1秒かけて300[mm]バック&回転する
       (send *rs* :daisya-position-vector #f(-300 0 50) 1000)
       ;;1秒休む
       (unix:sleep 1)
       )
      ;;bumper2,あるいはbumper3が押されていた場合
      ((or (= (elt result 2) 1) (= (elt result 3) 1))
       (send *rs* :daisya-position-vector #f(300 0 0) 1000)
       (unix:sleep 1)
       )
      ;;bumper4が押されていた場合
      ((= (elt result 4) 1)
       (send *rs* :daisya-position-vector #f(0 200 0) 1000)
       (unix:sleep 1)
       )
      ;;bumper1が押されていた場合
      ((= (elt result 1) 1)
       (send *rs* :daisya-position-vector #f(0 -200 0) 1000)
       (unix:sleep 1)
       )
      ;;それ以外の場合
      (t
       ;;とりあえず直進, usleepはマイクロスリープの意味
       (send *rs* :daisya-position-vector #f(300 0 0) 1000)
       (unix:usleep (* 1000 1000))
       ))
     )
    ;;台車制御を速度制御に戻しておく(ついでにspeedも0にしておく)
    (send *rs* :daisya-controlmode 1)
    (send *rs* :daisya-velocity-vector #f(0 0 0))
    ))
