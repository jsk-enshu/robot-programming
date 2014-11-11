;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;;       Virtual-Interpolator.l                              K.Nagasaka
;;;
;;;	  $Log: virtual_interpolator.l,v $
;;;	  Revision 1.1  2009-12-14 10:33:50  nakanish
;;;	  add
;;;
;;;	  Revision 1.1  2009/12/03 01:57:49  k-okada
;;;	  add
;;;	
;;;	  Revision 1.6  2007-07-06 11:19:31  kanzaki
;;;	  fix minjerk interpolator
;;;
;;;	  Revision 1.5  2007/07/06 10:06:32  kanzaki
;;;	  add minjerk interpolator
;;;	
;;;	  Revision 1.4  2006/04/18 03:51:22  kanzaki
;;;	  fix angle-vector-gradient
;;;	
;;;	  Revision 1.3  2006/04/18 02:44:02  kanzaki
;;;	  fix angle-vector-gradient
;;;	
;;;	  Revision 1.2  2006/03/20 06:01:51  kanzaki
;;;	  add smooth mode in spline-interpolator
;;;	
;;;	  Revision 1.1.1.1  2000/03/13 10:37:02  kanehiro
;;;	  EusLisp library
;;;	
;;;	  Revision 1.2  1997/04/03 13:03:19  kanehiro
;;;	  merged change in branch rbrain3_0
;;;
;;;	  Revision 1.1.2.2  1997/02/25 04:02:56  kanehiro
;;;	  misc.
;;;
;;;
;;;       update-info
;;;         941206 add spline-completion 
;;;         950223 completion > interpolation
;;;         960807 change to give dt from other objects, change slot names
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;;  Class Interpolator （姿勢補間の抽象クラス）
;;; 
;;;  ex. (sec/step = 20msec)
;;;      
;;;       step-list        (10           15             15          (dummy))
;;;                    |----------+--------------+--------------|
;;;       time-list        (   200msec         500msec        800msec      )
;;;       time         <------------------------------->
;;;       segment-time                           <----->
;;;       segment      000000000000111111111111111222222222222222
;;;       segment-num  3
;;;
;;; usage.
;;;       :init で生成 
;;;       :reset で補間姿勢リスト、ステップ数をセット
;;;       :start-interpolation で補間開始
;;;       :pass-time で時間進行し、補間した姿勢を返す
;;; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass Interpolator
  :super propertied-object
  :slots
  (
   (angle-vector-list :type cons)    ;;; 補間を行なう姿勢のリスト
   (step-list :type cons)            ;;; 補間数のリスト
   (angle-vector :type float-vector) ;;; 現在の姿勢
   (sec/step :type float)            ;;; 1step当たりの時間[sec] (20msec)
   (time-list :type cons)            ;;; step-listを連続時間スケールにしたもの
   (time :type float)                ;;; 補間開始からの時間[sec]
   (segment-time :type float)        ;;; ステップの区切れ目からの時間[sec]
   (segment :type integer)           ;;; 何番目のステップの節を処理中か
   (segment-num :type integer)       ;;; 全セグメント数
   (interpolatingp :type symbol)     ;;; 現在、補間中か否か
   ))

(defmethod Interpolator
  
  ;;; init
  
  (:init 
   (&key ((:sec/step ss) 0.020))
   (setq sec/step ss)
   (setq time 0.0)
   (setq segment-time 0.0)
   (setq segment 0)
   (setq segment-num 0)
   (setq interpolatingp nil)
   self)
  
  ;;; reset
  
  (:reset
   (&rest
    args
    &key
    (angle-vector-list (send self :angle-vector-list))
    (step-list (send self :step-list))
    &allow-other-keys
    )
   (setq (self . angle-vector-list) angle-vector-list)
   (setq (self . step-list) step-list)
   (setq time 0.0)
   (setq segment-time 0.0)
   (setq segment 0)
   (setq segment-num (1- (length step-list)))
   (send self :time-list :update t)
   (setq interpolatingp nil))
  
  ;;; accessor to slots
  
  (:angle-vector-list () angle-vector-list)
  (:step-list () step-list)
  (:angle-vector () angle-vector)
  (:sec/step () sec/step)
  (:time-list 
   ;;; update=t の時、step-listを連続時間スケールに変換し、time-listに蓄える。
   ;;; update=nil の時、time-listを返す。
   (&key update)
   (when update
     (setq time-list (butlast step-list))
     (dotimes
	 (i (1- segment-num))
       (incf (nth (1+ i) time-list) (nth i time-list)))
     (setq time-list (mapcar #'(lambda (x) (* x sec/step)) time-list)))
   time-list)
  (:time () time)
  (:segment-time () segment-time)
  (:segment () segment)
  (:segment-num () segment-num)
  (:interpolatingp () interpolatingp)
  
  ;;; operation
  
  (:pass-time
   (dt)
   (when
    interpolatingp
    (setq angle-vector (send self :interpolation))
    (incf time dt)
    (incf segment-time dt)
    (when (> time (nth segment time-list))
	  (setq segment-time (- time (nth segment time-list)))
	  (incf segment))
    (if (>= segment segment-num) (send self :reset))
    angle-vector))
  (:start-interpolation () (setq interpolatingp t))
  (:stop-interpolation () (setq interpolatingp nil))
  (:interpolation
   ()
   ;;; サブクラスで、再定義すること。すなわち、あるsegmentで、segment-time
   ;;; の状態における補間値を返すこと。
   nil)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;;  Class Linear-Interpolator （線形姿勢補間クラス）
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass Linear-Interpolator
  :super Interpolator)

(defmethod Linear-Interpolator

  ;;; operator
  (:interpolation
   ()
   (let* ((v1 (nth segment angle-vector-list))
	  (v2 (nth (1+ segment) angle-vector-list))
	  (step (nth segment step-list))
	  (t1+t2 (* sec/step step))
	  (t1 segment-time)
	  (t2 (- t1+t2 t1)))
     (setq v1 (scale (/ t2 t1+t2) v1))
     (setq v2 (scale (/ t1 t1+t2) v2))
     (v+ v1 v2)))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;;  Class Spline-Interpolator （スプライン姿勢補間クラス）
;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass Spline-Interpolator
  :super Interpolator
  :slots
  (
   (angle-vector-gradient-list :type float-vector) ;;; 各標本点でのgradのリスト
   (smooth t)
   ))

(defmethod Spline-Interpolator

  ;;; reset
  
  (:reset
   (&rest args)
   (let (p)
     (send-super-lexpr :reset args)
     (if (setq p (position :smooth args))
	 (setq smooth (elt args (1+ p))))
     (if angle-vector-list (send self :angle-vector-gradient-list :update t))))
  
  ;;; accessor to slots
  
  (:angle-vector-gradient-list
   ;;; update=t の時、angle-vector-gradient-listを更新する。
   ;;; update=nil の時、angle-vector-gradient-listを返す。
   (&key update)
   (when update
     (setq angle-vector-gradient-list (copy-object angle-vector-list))
     (if smooth
	 (let ((len (length angle-vector-list))
	       (zv (make-array (length (car angle-vector-list)) :element-type float-vector)))
	   (setf (elt angle-vector-gradient-list 0) zv)
	   (dotimes (i (- len 2))
	     (setf (elt angle-vector-gradient-list (1+ i))
		   (scale (/ (elt (send self :step-list) i)
			     (+ (elt (send self :step-list) i)
				(elt (send self :step-list) (1+ i))))
			  (v- (elt (send self :angle-vector-list) (+ i 2)) 
			      (elt (send self :angle-vector-list) i)))))
	   (if (> len 2) (setf (elt angle-vector-gradient-list (- len 2))
			       (v- (elt (send self :angle-vector-list) (- len 1))
				   (elt (send self :angle-vector-list) (- len 2)))))
	   (setf (elt angle-vector-gradient-list (- len 1)) zv))
       (let (a b c f n w1 w2)
	   (setq n (length angle-vector-list))
	   (setq a (make-array n))
	   (setq b (make-array n))
	   (setq c (make-array n))
	   (setq f (make-array n))
	   (dotimes
	    (i ((first angle-vector-list) . length))
	    (setf (aref b 0) 1.0)
	    (setf (aref c 0) 0.0)
	    (setf (aref f 0) 0.0)
	    (dotimes
	     (j (- n 2))
	     (setq w1 (/ 1.0 (nth j step-list)))
	     (setq w2 (/ 1.0 (nth (1+ j) step-list)))
	     (setf (aref a (1+ j)) w1)
	     (setf (aref b (1+ j)) (+ (* 2.0 w1) (* 2.0 w2)))
	     (setf (aref c (1+ j)) w2)
	     (setf (aref f (1+ j))
		   (+ (* -3.0 w1 w1 (aref (nth j angle-vector-list) i))
		      (*  3.0 (- (* w1 w1) (* w2 w2))
			  (aref (nth (1+ j) angle-vector-list) i))
		      (*  3.0 w2 w2
			  (aref (nth (+ j 2) angle-vector-list) i)))))
	    (setf (aref a (1- n)) 0.0)
	    (setf (aref b (1- n)) 1.0)
	    (setf (aref f (1- n)) 0.0)
	    (setq f (send self :trid-gauss-elimination a b c f n))
	    (dotimes
	     (j n)
	     (setf (aref (nth j angle-vector-gradient-list) i) (aref f j)))))))
   angle-vector-gradient-list)
  
  ;;; operation
  
  (:interpolation
   ()
   (let* (
	  (v1 (nth segment angle-vector-list))
	  (v2 (nth (1+ segment) angle-vector-list))
	  (dv1 (nth segment angle-vector-gradient-list))
	  (dv2 (nth (1+ segment) angle-vector-gradient-list))
	  (step (nth segment step-list))
	  (t1+t2 (* sec/step step))
	  (t1 segment-time)
	  (t2 (- t1 t1+t2))
	  (w1 (/ t2 t1+t2))
	  (w2 (/ t1 t1+t2)))
     (v+
      (scale (* w1 w1)
	     (v+
	      (scale (+ 3.0 (* 2.0 w1)) v1)
	      (scale (+ t1+t2 (* w1 t1+t2)) dv1)))
      (scale (* w2 w2)
	     (v- 
	      (scale (- 3.0 (* 2.0 w2)) v2)
	      (scale (- t1+t2 (* w2 t1+t2)) dv2))))))

  ;;; auxiliary
  
  (:trid-gauss-elimination
   (a b c f n)
   (let (m)
     (dotimes (i (1- n))
	      (setq m (/ (aref a (1+ i)) (aref b i)))
	      (decf (aref b (1+ i)) (* m (aref c i)))
	      (decf (aref f (1+ i)) (* m (aref f i)))))
   f)
  )

(defclass Minjerk-Interpolator
  :super Interpolator
  :slots (pos vel acc vel-vector-list acc-vector-list)
  )

(defmethod Minjerk-Interpolator
  (:pos () pos)
  (:vel () vel)
  (:acc () acc)
  (:vel-vector-list () vel-vector-list)
  (:acc-vector-list () acc-vector-list)
  ;;
  (:reset
   (&rest args
    &key
    (vel-vector-list (send self :vel-vector-list))
    (acc-vector-list (send self :acc-vector-list))
    &allow-other-keys)
   (send-super-lexpr :reset args)
   (if vel-vector-list
       (setq (self . vel-vector-list) vel-vector-list)
     (setq (self . vel-vector-list) 
	   (make-list (length angle-vector-list) 
		      :initial-element 
		      (make-array (length (car angle-vector-list)) 
				  :element-type float-vector))))
   (if acc-vector-list
       (setq (self . acc-vector-list) acc-vector-list)
     (setq (self . acc-vector-list) 
	   (make-list (length angle-vector-list) 
		      :initial-element 
		      (make-array (length (car angle-vector-list)) 
				  :element-type float-vector))))
   (setq pos (nth segment angle-vector-list))
   (setq vel (nth segment vel-vector-list))
   (setq acc (nth segment acc-vector-list))
   )
  ;;; operator
  (:interpolation
   ()
   ;;   (send self :interpolation-sequential)
   (send self :interpolation-absolute)
   )
  (:interpolation-sequential
   ()
   (let* ((xf (nth (1+ segment) angle-vector-list))
	  (vf (nth (1+ segment) vel-vector-list))
	  (af (nth (1+ segment) acc-vector-list))
	  (step (nth segment step-list))
	  (d (- (* sec/step step) segment-time))
	  da)
     (if  (> d sec/step) 
	 (progn
	   (setq da (v+ (v+ (scale (/ -9.0 d) (v+ (scale (/ -1.0 3.0) af) acc))
			    (scale (/ -36.0 (* d d)) (v+ (scale (/ 2.0 3.0) vf) vel)))
			(scale (/ 60.0 (* d d d)) (v- xf pos))))
	   (v+ acc (scale sec/step da) acc)
	   (v+ vel (scale sec/step acc) vel)
	   (v+ pos (scale sec/step vel) pos)
	   )
       (setq acc af vel vf pos xf))
     pos))
  (:interpolation-absolute
   ()
   (let* ((xi (nth segment angle-vector-list))
	  (xf (nth (1+ segment) angle-vector-list))
	  (vi (nth segment vel-vector-list))
	  (vf (nth (1+ segment) vel-vector-list))
	  (ai (nth segment acc-vector-list))
	  (af (nth (1+ segment) acc-vector-list))
	  (step (nth segment step-list))
	  (d (* sec/step step))
	  (tau (/ segment-time d))
	  (a0 xi)
	  (a1 (scale d vi))
	  (a2 (scale (* 0.5 d d) ai))
	  (a3 (v+ (scale (* -1.5 d d) (v+ (scale (/ -1.0 3.0) af) ai)) 
		  (v+ (scale (* -6.0 d) (v+ (scale (/ 2.0 3.0) vf) vi))
		      (scale 10.0 (v- xf xi)))))
	  (a4 (v+ (scale (* 1.5 d d) (v+ (scale (/ -2.0 3.0) af) ai)) 
		  (v+ (scale (* 8.0 d) (v+ (scale (/ 7.0 8.0) vf) vi))
		      (scale -15.0 (v- xf xi)))))
	  (a5 (v+ (scale (* -0.5 d d) (v+ (scale -1.0 af) ai)) 
		  (v+ (scale (* -3.0 d) (v+ vf vi))
		      (scale 6.0 (v- xf xi)))))
	  )
     (v+ a0 (scale tau a1) pos)
     (v+ pos (scale (expt tau 2) a2) pos)
     (v+ pos (scale (expt tau 3) a3) pos)
     (v+ pos (scale (expt tau 4) a4) pos)
     (v+ pos (scale (expt tau 5) a5) pos)
     ;;      ;;
     ;;      (v+ (scale (/ 1.0 d) a1) (scale (/ (* 2.0 tau) d) a2) vel)
     ;;      (v+ vel (scale (/ (* 3.0 (expt tau 2)) d) a3) vel)
     ;;      (v+ vel (scale (/ (* 4.0 (expt tau 3)) d) a4) vel)
     ;;      (v+ vel (scale (/ (* 5.0 (expt tau 4)) d) a5) vel)
     ;;      ;;
     ;;      (v+ (scale (/ 2.0 (* d d)) a2) (scale (/ (* 6.0 tau) (* d d)) a3) acc)
     ;;      (v+ acc (scale (/ (* 12.0 (expt tau 2)) (* d d)) a4) acc)
     ;;      (v+ acc (scale (/ (* 20.0 (expt tau 3)) (* d d)) a5) acc)
     ;;      ;;
     pos))
  )

;; ;; sample of minjerk interpolator
;; (setq *ip* (instance minjerk-interpolator :init))
;; (send *ip* :reset 
;;       :angle-vector-list (list #f(50.0 50.0 50.0) #f(100.0 200.0 300.0) #f(200.0 300.0 400.0)) 
;;       :step-list (list 5000 5000 5000)
;;       :vel-vector-list (list #f(0.0 0.0 0.0) #f(0.0 0.0 0.0) #f(0.0 0.0 0.0))
;;       :acc-vector-list (list #f(0.0 0.0 0.0) #f(0.0 0.0 0.0) #f(0.0 0.0 0.0)))
;; (setq *tm* (instance mtimer :init))
;; (send *ip* :start-interpolation)
;; (send *tm* :start)
;; (while (send *ip* :interpolatingp)
;;   (send *ip* :pass-time (send *ip* :sec/step))
;;   )
;; (format t "time : ~A~%" (send *tm* :stop))

#|
(nrequire :gibbonmodel
	  (concatenate string (unix:getenv "VIRTUAL_LIB") 
		       "/robot/gibbonmodel"))
(defun initl
  ()
  (gibbon)
  (setq *l* (instance linear-interpolator :init))
  (setq *p1* (send *gibbon* :angle-vector))
  (setq *p2* (float-vector
	      (deg2rad 45)   ;;; :left-crotch
	      (deg2rad -90)  ;;; :left-knee
	      (deg2rad -45)  ;;; :left-ankle
	      (deg2rad 0)    ;;; :left-shoulder
	      (deg2rad 0)    ;;; :left-elbow
	      (deg2rad 0)    ;;; :left-gripper(x)
	      (deg2rad 30)   ;;; :tilt
	      (deg2rad 45)   ;;; :right-crotch
	      (deg2rad -90)  ;;; :right-knee
	      (deg2rad -45)  ;;; :right-ankle
	      (deg2rad -135) ;;; :right-shoulder
	      (deg2rad 90)   ;;; :right-elbow
	      (deg2rad 0)    ;;; :right-gripper
	      ))
  (send *l* :reset :angle-vector-list (list *p1* *p2*) :step-list (list 5 5))
  (send *l* :start-interpolation)
  (setq *f* (open "daa.dat" :direction :output))
  (setq *i* 0)
  (setq *v* (make-parallel-viewer))
  )
(defun testl
  ()
  (setq *p* (send *l* :pass-time 0.01))
  (send *gibbon* :angle-vector *p*)
  (format *f* "~a ~a ~%"
	  *i* (send *gibbon* :joint :right-shoulder :joint-angle))
  (incf *i*)
  (send *v* :clear)
  (send *v* :draw *gibbon*))


(defun inits
  ()
  (nrequire :gibbonmodel
	    (concatenate string (unix:getenv "GIBBON") "/gibbonmodel"))
  (gibbon)
  (setq *s* (instance spline-interpolator :init))
  (setq *p1* (send *gibbon* :angle-vector))
  (setq *p2* (float-vector
	      (deg2rad 45)   ;;; :left-crotch
	      (deg2rad -90)  ;;; :left-knee
	      (deg2rad -45)  ;;; :left-ankle
	      (deg2rad 0)    ;;; :left-shoulder
	      (deg2rad 0)    ;;; :left-elbow
	      (deg2rad 0)    ;;; :left-gripper(x)
	      (deg2rad 30)   ;;; :tilt
	      (deg2rad 45)   ;;; :right-crotch
	      (deg2rad -90)  ;;; :right-knee
	      (deg2rad -45)  ;;; :right-ankle
	      (deg2rad -135) ;;; :right-shoulder
	      (deg2rad 90)   ;;; :right-elbow
	      (deg2rad 0)    ;;; :right-gripper
	      ))
  (send *s* :reset :angle-vector-list (list *p1* *p2*) :step-list (list 5 5))
  (send *s* :start-interpolation)
  (setq *f* (open "daa.dat" :direction :output))
  (setq *i* 0)
  (setq *v* (make-parallel-viewer))
  )
(defun tests
  ()
  (setq *p* (send *s* :pass-time 0.01))
  (send *gibbon* :angle-vector *p*)
  (format *f* "~a ~a ~%"
	  *i* (send *gibbon* :joint :right-shoulder :joint-angle))
  (incf *i*)
  (send *v* :clear)
  (send *v* :draw *gibbon*))
|#



(provide :virtual-interpolator "$Id: virtual_interpolator.l,v 1.1 2009-12-14 10:33:50 nakanish Exp $")

