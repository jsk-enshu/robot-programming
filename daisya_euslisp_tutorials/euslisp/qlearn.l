;;**************************************************
;;
;;  Q-Learning
;;
;;           Takashi Ogura
;;                               2005/1/17
;;
;;**************************************************

(defclass mdp-qagent
  :super propertied-object
  :slots (q actions environment time
	    policy             ; e-greedy is default
	    alpha gamma epsilon  ; learning params
	    )
  )

(defmethod mdp-qagent
  (:init (act env &key ((:alpha a) 0.1) ((:gamma g) 0.95) ((:epsilon e) 0.1))
    (setq actions act
	  environment env)
    (setq alpha a
	  gamma g
	  epsilon e)
    (setq time 0)
    (setq policy :e-greedy)
    ;;
    (setq q nil)
    (dotimes (i (length actions))
      (push (make-array (send env :state-size) :initial-element 0) q))
    self)
  (:time (&optional tm)(if tm (setq time tm) time))
  (:q (s a &optional new-value)
    (if new-value
	(setf (elt (elt q a) s) new-value))
    (elt (elt q a) s)
    )
  (:max-q (s)
    (let ((max-q -100000) ai)
      (dotimes (i (length actions))
	(when (> (send self :q s i) max-q)
	  (setq max-q (send self :q s i))
	  (setq ai (list i)))
	(when (= (send self :q s i) max-q)
	  (push i ai)
	  ))
      (cons max-q ai)))
  ;;
  (:act (state)
    (send self policy state))
  ;; policy types
  (:random (state)
    (random (length actions)))
  (:e-greedy (state)
    (let (ret)
      (cond
       ((= (random (round (/ 1.0 epsilon))) 0)
	(random (length actions)))
       (t
	(setq ret (cdr (send self :max-q state)))
	(setq ret (elt ret (random (length ret))))))))
  (:greedy (state)
    (let (ret)
      (setq ret (cdr (send self :max-q state)))
      (setq ret (elt ret (random (length ret))))))
  (:boltzmann (state)
    ) ;; not implemented yet
  ;;
  (:state ()
    (send environment :state))
  (:policy-proc ()
    (let (st at)
      (setq st (send self :state))     ; get state
      (setq at (send self :greedy st))    ; select action
      (send self (elt actions at))     ; do action
      ))
  (:proc ()
    (let (st at r st+1)
      (setq st (send self :state))     ; get state
      (setq at (send self :act st))    ; select action
      (send self (elt actions at))     ; do action
      (setq r (send environment :r))   ; get reward
      (setq st+1 (send self :state))   ; get next state
      (incf time)
      (send self :q st at
	    (+ (* (- 1 alpha) (send self :q st at))
	       (* alpha (+ r (* gamma (car (send self :max-q st+1)))))))
      ))
  )

;; Sutton (1990)
(defclass mdp-dyna-qagent
  :super mdp-qagent
  :slots (num model ex-states ex-state-actions)
  )

(defmethod mdp-dyna-qagent
  (:init (an &rest args)
    (setq num an)
    (send-super* :init args))
  (:update-model (st at st+1 r)
    ;; 最新の情報を入れる
    (let ((old-exp (assoc (list st at) model :test #'equal)))
      (when (and old-exp
		 (not (equal old-exp (cons (list st at) (list st+1 r)))))
	(format t ";; model changed!~%")
	(setq model (remove old-exp model :test #'equal)))
      (push (cons (list st at) (list st+1 r)) model)
      ;;(pushnew (cons (list st at) (list st+1 r)) model :test #'equal)
      ))
  (:simulate (st at)
    (cdr (assoc (list st at) model :test #'equal))
    )
  (:add-experiment (st at)
    (let (state-action)
      (pushnew st ex-states)
      (cond
       ((setq state-action (assoc st ex-state-actions))
	(nconc state-action (list at)))
       (t
	(push (list st at) ex-state-actions)))))
  (:proc ()
    (let (st at r st+1 st-r ex-actions)
      (setq st (send self :state))     ; get state
      (setq at (send self :act st))    ; select action
      (send self (elt actions at))     ; do action
      (setq r (send environment :r))   ; get reward
      (setq st+1 (send self :state))   ; get next state
      (incf time)
      ;; 直接的強化学習
      (send self :q st at
	    (+ (* (- 1 alpha) (send self :q st at))
	       (* alpha (+ r (* gamma (car (send self :max-q st+1)))))))
      ;; モデル学習
      (send self :update-model st at st+1 r)
      ;; プランニング
      (send self :add-experiment st at)
      (dotimes (i num)
	(setq st (elt ex-states (random (length ex-states))))
	(setq ex-actions (cdr (assoc st ex-state-actions)))
	(setq at (elt ex-actions (random (length ex-actions))))
	(setq st-r (send self :simulate st at))
	(setq st+1 (car st-r) r (cadr st-r))
	(send self :q st at
	      (+ (* (- 1 alpha) (send self :q st at))
		 (* alpha (+ r (* gamma (car (send self :max-q st+1))))))))
      ))
  )

;; Moore and Atkeson (1993) and Peng and Williams (1993)
(defclass mdp-prioritized-sweeping-agent
  :super mdp-dyna-qagent
  :slots (pqueue)
  )
(defmethod mdp-prioritized-sweeping-agent
  (:init (&rest args)
    (setq pqueue (instance queue :init))
    (send-super* :init args))
  ;; state sにつながるs_, a_を求める
  (:simulate-back (s)
    (let (s-a)
      (dolist (m model)
	(if (equal (car (cdr m)) s)
	    (push (car m) s-a)))
      s-a))
  (:proc ()
    (let (st at r p (theta 0) st+1 s-a st-r ex-actions st_ at_ r_)
      (setq st (send self :state))     ; get state
      (setq at (send self :act st))    ; select action
      (send self (elt actions at))     ; do action
      (setq r (send environment :r))   ; get reward
      (setq st+1 (send self :state))   ; get next state
      ;; モデル学習
      (send self :update-model st at st+1 r)
      (incf time)
      (setq p (- (+ r (* gamma (car (send self :max-q st+1))))
		 (send self :q st at)))
      (if (> p theta)
	  (send pqueue :enqueue-by-priority (list (cons st at)) #'(lambda(x) p)))
      (dotimes (i num)
	(if (send q :emptyp) (return))
	(setq s-a (send pqueue :remove-front))
	(setq st (car s-a) at (cdr s-a))
	(setq st-r (send self :simulate st at))
	(setq st+1 (car st-r) r (cadr st-r))
	(send self :q st at
	      (+ (* (- 1 alpha) (send self :q st at))
		 (* alpha (+ r (* gamma (car (send self :max-q st+1)))))))
	;;
	(dolist (s_-a_ (send self :simulate-back st))
	  (setq st_ (car s_-a_) at_ (cdr s_-a_))
	  (setq st-r (send self :simulate st_ at_))
	  (setq r_ (cadr st-r))
	  (setq p (- (+ r_ (* gamma (car (send self :max-q st+1))))
		     (send self :q st_ at_)))
	  (if (> p theta)
	      (send pqueue :enqueue-by-priority (list (cons st_ at_)) #'(lambda(x) p)))
	  )
	)
      ))
  )

;; Bradtke and Duff (1995)
(defclass smdp-qagent
  :super mdp-qagent
  :slots (dead-lock-num)
  )

(defmethod smdp-qagent
  (:init (&rest args)
    (setq dead-lock-num 3)
    (send-super* :init args))
  (:proc ()
    (let (event st at (r 0) (count 0) st+n)
      (setq st (send self :state))
      (setq at (send self :act st))
      (setq st+n st)
      (while (equal st st+n) ;; 状態変化
	(send self (elt actions at))
	(incf r (* (expt gamma count) (send environment :r)))
	(incf count)
	(setq st+n (send self :state))
	;;(when (not (equal st st+n)) ;; 状態変化
	;;(setq event t))
	(when (> count dead-lock-num)  ;; dead lock ?
	  (setq at (send self :random st))
	  (setq count 0 r 0)
	  )
	)
      (incf time count) ;; dead lockは無かったことにする
      (send self :q st at
	    (+ (* (- 1 alpha) (send self :q st at))
	       (* alpha (+ r (* (expt gamma count) (car (send self :max-q st+n)))))))
      ))
  )


(defclass mdp-search-agent
  :super mdp-qagent
  :slots nil)
(defmethod mdp-search-agent
  (:init (env)
    (send-super :init '(:u :d :r :l) env)
    )
  (:u ()
    (send environment :u-go))
  (:d ()
    (send environment :d-go))
  (:r ()
    (send environment :r-go))
  (:l ()
    (send environment :l-go))
  )


(defclass mdp-dyna-search-agent
  :super mdp-dyna-qagent
  :slots nil)
(defmethod mdp-dyna-search-agent
  (:init (env &optional (num 3))
    (send-super :init num '(:u :d :r :l) env)
    )
  (:u ()
    (send environment :u-go))
  (:d ()
    (send environment :d-go))
  (:r ()
    (send environment :r-go))
  (:l ()
    (send environment :l-go))
  )

(defclass smdp-search-agent
  :super smdp-qagent
  :slots nil)
(defmethod smdp-search-agent
  (:init (env)
    (send-super :init '(:u :d :r :l) env)
    )
  (:u ()
    (send environment :u-go))
  (:d ()
    (send environment :d-go))
  (:r ()
    (send environment :r-go))
  (:l ()
    (send environment :l-go))
  )

;;環境クラス
(defclass mdp-qenvironment
  :super propertied-object
  :slots (state reward)
  )
(defmethod mdp-qenvironment
  (:init ()
    (setq reward 0)
    self)
  (:state (&optional st)
    (if st (setq state st) state))
  (:r () reward)
  )


;;;
;;;恐らく地図環境の作成をしている
;;;
(defclass mdp-search-environment
  :super mdp-qenvironment
  :slots (width height sp rp ob xy step ngoal))
(defmethod mdp-search-environment
  (:init (map &key ((:step stp) 1))
    (send-super :init)
    (setq step stp)
    ;;
    (send self :read-map map) ;;rp,sp,obの座標の設定
    (send self :initialize)
    self)
  (:state-size () (* width height))
  ;;mapはx,-,x,gで表された以下のような文字列のlistを想定している
  #|
	       "xxxxxxxxxxx"
	       "x-------xgx"
	       "x--x----x-x"
	       "xs-x----x-x"
	       "x--x------x"
	       "x-----x---x"
	       "x---------x"
	       "xxxxxxxxxxx"
  |#
  ;;これを解釈して、地図環境（state)を入れているみたい
  (:read-map (map)
    ;;地図の縦横を格納
    (setq width (length (car map)))
    (setq height (length map))
    (setq ob nil)
    (dotimes (i height)
      (dotimes (j width)
	(cond
	 ;;gの座標にはrp = reward point?を設定
	 ((= (elt (elt map i) j) #\g)
	  (setq rp (float-vector j i)))
	 ;;sの座標にはsp = start point? を設定
	 ((= (elt (elt map i) j) #\s)
	  (setq sp (float-vector j i)))
	 ;;xの座標には、ob = obstacleのlist を設定
	 ((= (elt (elt map i) j) #\x)
	  (push (float-vector j i) ob))
	 )
	))
    )
  ;;stateを格納 stateは数値のようだ
  (:initialize ()
    (setq ngoal 0)  ;;goalについた回数をreset
    (setq xy (copy-object sp)) ;;現在値をスタート地点に
    (setq state (send self :xy2state xy)) ;;現在stateを現在座標から計算
    )
  ;;座標をstate（数値）に変換
  (:xy2state (axy) (+ (* width (floor (elt axy 1))) (floor (elt axy 0))))
  ;;stateを座標に戻す
  (:state2xy (astate) (float-vector (mod astate width) (floor (/ astate width))))
  ;;現在のstateを返す、あるいは設定されたstateを現在stateにセットする。例外処理つき
  (:state (&optional st)
    (if (and st
	     (>= st 0)
	     (< st (* width height)))
	(setq state st))
    state)
  ;;goalに到達したかを確認する
  (:goalp (&optional (st state))
    (equal (send self :state2xy st) rp))
  ;;障害物かを確認する
  (:obstaclep (&optional (st state))
    (member (send self :state2xy st) ob :test #'equal))
  ;;地図情報を表示する
  (:print ()
    (dotimes (i height)
      (dotimes (j width)
	(cond
	 ;;現在位置を表示するのは#
	 ((= (send self :xy2state (float-vector j i)) state)
	  (format t "#"))
	 ((send self :goalp (send self :xy2state (float-vector j i)))
	  (format t "g"))
	 ((send self :obstaclep (send self :xy2state (float-vector j i)))
	  (format t "x"))
	 (t
	  (format t "-"))))
      (format t "~%"))
    )
  ;;地図の中にいるかどうか
  (:insidep (&optional (st state))
    (and (>= st 0) (< st (* width height))))
  ;;現在よりも相対的に座標方向へいく
  ;; xyは現在の座標が入る
  (:go (dxy)
    (let (i sign nxy nstate)
      ;;x,yどっち方向に進むかのiを調べる
      ;; わかりにくいけど、xに進むならiは０、yに進ならiは１になる。
      (setq i (floor (abs (elt dxy 1))))
      ;; どっち方向に進むかを決めている
      (setq sign (if (> (elt dxy i) 0) 1 -1))
      (setq nxy (copy-object xy))
      ;;stepぶんだけ進む座標を作る
      (incf (elt nxy i) (* sign step))
      ;;すすんだ場合のstateを取得する
      (setq nstate (send self :xy2state nxy))
      ;; calc reward
      ;;  nstateがgoalだったら1を、obstacleなら0を返す、それ以外も０を
      ;;  goalだとはじめて報酬
      (setq reward
	    (cond
	     ((send self :goalp nstate) 1) ;; goal
	     ((send self :obstaclep nstate) 0) ;; obstacles
	     (t 0)))
      ;; 動くかどうかを判定する
      (cond
       ;; don't move       
       ;;  しょうがいぶつだったり外にいってたら動かない
       ((or (send self :obstaclep nstate)
	    (not (send self :insidep nstate)))
	)
       ;; goalだったらgoalについた回数ngoalを更新する
       ;; で、スタートに戻る
       ((send self :goalp nstate);; goal
	(incf ngoal)
	(setq xy (copy-object sp))) ;; return to start
       ;; それ以外は更新する
       (t
	(setq xy nxy)
	)
       )
      ;;
      ;;現在座標xyにstateを更新する
      (send self :state (send self :xy2state xy))))
  ;;ひとつ上にいく
  (:u-go ()
    (send self :go (float-vector 0 1)))
  ;;ひとつ下にいく
  (:d-go ()
    (send self :go (float-vector 0 -1)))
  ;;ひとつ左にいく
  (:l-go ()
    (send self :go (float-vector -1 0)))
  ;;ひとつ右にいく
  (:r-go ()
    (send self :go (float-vector 1 0)))
  )

;;agentの政策を調べるらしい
;; してることは、agentの環境のgoal到達回数した回数だけ
;; agentのpolicy-procを読んでいる
;; 現在の状態で、goalするまでに必要なステップ数を返す?
;;
(defun check-policy (a &optional (debug nil) (time 500))
  (let* ((env (a . environment))
	 (tgcount (env . ngoal))
	 (tcount 0))
    ;;もう一度ゴールするまで send a :policy-procを行う
    (while (= tgcount (env . ngoal))
      (send a :policy-proc)
      (when debug
	(send env :print)
	(format t ";~%")
	(unix:usleep (* time 1000))
	)
      (incf tcount)
      )
    ;;tcountはもう一度ゴールするまでに必要なステップ数
    (setq (env . ngoal) tgcount)
    tcount))

;;
;;学習遂行関数 
;; aはagentのinstanceを入れるのかな
;; agentの time, procが何をしているのかを調べる必要がある
(defun learn (a &optional (fname "qlearn.dat") &key debug)
  (let ((max-time 10000) (max-goal 100) (rcount 0) (gcount 0) (max-repeat 20)
	v (env (a . environment))
	(tcount 0) ptcount)
    ;;agentの環境を初期化する(地図のスタート地点に戻す、ゴール到達回数を0に戻す)
    (send env :initialize)
    ;;学習開始
    (with-open-file (f fname :direction :output)
      (loop
       (when (= rcount max-repeat)
	 ;;何通りに収束したか
	 ;; gcountはゴールした回数
	 (format t ";~%converged in ~A goals !~%" (- gcount rcount))
	 (return))
       ;;収束しなかった場合
       (when (or (> gcount max-goal)
		 (> (send a :time) max-time))
	 (format t ";~%NOT converged!~%")
	 (return))
       ;;
       ;;多分これが肝(おそらく１ステップ進める）
       (send a :proc)
       ;;
       ;;地図情報を表示
       (when debug
	 (send env :print)
	 (format t "~%"))
       ;;
       ;;一度goalに到達する度に行う処理
       ;; goalに到達するとenvのngoalが増えるので、learnで保持しているgcountとずれる
       ;; のでわかる
       (when (not (= (env . ngoal) gcount))
	 (setq gcount (copy-object (env . ngoal)))
	 ;;agentのtimeを返す(おそらく、goalまでに到達するのにかかったステップ数)
	 (format t ";~5d :" (setq v (send a :time)))
	 (dotimes (i (floor (/ (send a :time) 10))) ;;かかったステップ数を視覚的に表しただけ(なんだorz)
	   (format t ";*"))
	 (format t "~%")
	 ;;政策を調べる  .. tcountにはgoalまでに必要なstep数が入る(多分)
	 (setq tcount (check-policy a))
	 ;;
	 (format f "~A~%" v)
	 ;;
	 ;;agentの回数をreset
	 (send a :time 0)
	 ;;
	 ;;ptcountは前回のtcount, tcountは check-policyの返り値
	 ;; 前回と同じなら、rcountをふやす
	 ;; rcountは何回連続で同じtcountが出たかを調べる引数か
	 (if (equal tcount ptcount) (incf rcount) (setq rcount 0))
	 (setq ptcount tcount)
	 )))
    (format t "~%")
    ))

#|
;;結局必要なのは、mapの用意、環境の用意、agentの用意
;; シンボル空間で迷路を解決するsample
(defun hoge
  (&optional (debug nil))
  (setq map (list
	     "xxxxxxxxxxxxxxxx"
	     "x--------------x"
	     "x-s--x---------x"
	     "x------xxx-----x"
	     "x----x---------x"
	     "x-----xx--xxx--x"
	     "x-----x-x-x----x"
	     "x-----x--xx--g-x"
	     "x--------------x"
	     "xxxxxxxxxxxxxxxx"
	     ))
  ;;環境の構築
  (setq se (instance mdp-search-environment :init map))
  ;;エージェントの構築
  ;; (いくつか方策があるけど、収束までにかかる時間が違う)
  ;;(setq nna (instance mdp-dyna-search-agent :init se))
  ;;(setq nna (instance mdp-search-agent :init se))
  (setq nna (instance smdp-search-agent :init se))
  ;;学習
  (learn nna "qlearn-map1.dat" :debug debug)
  ;;
  ;;解いてみてよ！
  (check-policy nna t 200)
  ;;
  )
|#
