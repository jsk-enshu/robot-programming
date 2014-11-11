(defun hvs2rgb (h &optional i s ret)
  (when (or (listp h) (vectorp h))
    (setq ret s)
    (setq s (elt h 2) i (elt h 1) h (elt h 0)))
  (if (>= h 360) (setq h (mod (round h) 360)))
  (if (< h    0) (setq h (- 360 (mod (round (- h)) 360))))
  (setq s (* 255 s) i (* 255 i))
  (let* ((hi (floor (/ h 60.0)))
	 (f (- (/ h 60.0) hi))
	 (p1 (* i (- 1.0 (/ s 255.0))))
	 (p2 (* i (- 1.0 (* f (/ s 255.0)))))
	 (p3 (* i (- 1.0 (* (- 1 f) (/ s 255.0)))))
	 r g b)
    (case hi
     ((0 6) (setq r i  g p3 b p1))
     (1 (setq r p2 g i  b p1))
     (2 (setq r p1 g i  b p3))
     (3 (setq r p1 g p2 b i))
     (4 (setq r p3 g p1 b i))
     (5 (setq r i  g p1 b p2))
     (t (warn "hri2rgb error")))
    (cond
     (ret
      (setf (elt ret 0) r)
      (setf (elt ret 1) g)
      (setf (elt ret 2) b)
      ret)
     (t (list r g b)))))


(defclass psd-model
  :super sensor-model
  :slots (max targetp detectp))
(defmethod psd-model
  (:init
   ;;だいたい最大15cmくらいしかとってこれない
   (shape &rest args &key ((:max m) 150))
   (if m (setq max m))
   ;;距離sensorはdefault(みつからないとき)は0がかえってくる仕様
   (setq data 0)
   (setq detectp nil)
   (send-super* :init shape args)
   )
  ;;
  ;;objsには距離をもとめたい対象となるbodiesが入る
  (:simulate
   (objs)
   (let* (r (mind max)
	    (p (send self :worldpos))
	    ;;z方向（sensorの視線方向のベクトルを保存)
	    (v (matrix-column (send self :worldrot) 2)))
     (if (atom objs) (setq objs (list objs)))
     ;;
     (setq detectp nil)
     (dolist (obj objs)
       (dolist (f (send obj :faces))
	 ;;点を通りベクトル方向の線分と面の交点を求める
         (setq r (send f :intersect-point-vector (send self :worldpos) v))
         (when (eq (car r) :inside) ;;もし交点が面の中にあったら（距離sensorがぶつかるということ)
	   (when (< (distance p (cadr r)) mind)
	     (setq targetp (cadr r) mind (distance p targetp))
	     (setq detectp t)
	     ))))
     (cond
      (detectp
       (setq data mind))
      ;;maxをこえている場合や、みつからない場合は0とする
      (t
       (setq data 0)))
     ))
  (:draw (vwer) (send self :draw-sensor vwer))
  ;;
  ;;
  (:draw-sensor 
   (vwer)
   (let ((pwidth (send vwer :viewsurface :line-width))
         (pcolor (send vwer :viewsurface :color))
         (v (float-vector 0 0 0)))
     (when detectp
       (setq v (normalize-vector (hvs2rgb (/ (* data -120.0) max) 0.9 0.9 v)))
       (send vwer :viewsurface :line-width 3)
       (send vwer :viewsurface :color v)
       (send vwer :viewsurface :3d-line (send self :worldpos) targetp)
       (send vwer :viewsurface :line-width pwidth)
       (send vwer :viewsurface :color pcolor))
     (gl::draw-glbody vwer self)
     ))
  )

(defmethod camera-model
  (:simulate
   (model)
   (let (bs p w ret)
     (setq bs
           (mapcan
            #'(lambda (b)
                (if (and (vectorp (get b :face-color))
                         (eps-v= #f(1 0 0) (get b :face-color) 0.1))
                    (list b))) model))
     (dolist (b bs)
       (setq w (elt (send self :viewing :view (send b :worldpos)) 2))
       (setq p (send self :screen-point (send b :worldpos)))
       (if (and (> w 0)
                (< 0 (elt p 0) pwidth) (< 0 (elt p 1) pheight)
                (< (distance (send self :worldpos) (send b :worldpos))
                   (send self :yon)))
           (push (list :centroid p) ret)))
     (setq data ret)
     ret))
  )



