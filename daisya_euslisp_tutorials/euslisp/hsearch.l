;;;
;;; heuristic search
;;;
;;; node property
;;;
(load "sets.l")
(load "gsearch.l")
;;;
#|
(defun float-vector (&rest l) (mapcar #'float l))
(defun x (p) (car p))
(defun y (p) (cadr p))
(defun v- (p0 p1) (mapcar #'- p0 p1))
(defun scale (s v) (mapcar #'(lambda (x) (* s x)) v))
(defun v. (p0 p1) (apply #'+ (mapcar #'* p0 p1)))
(defun norm (v) (sqrt (v. v v)))
(defun distance (p0 p1) (norm (v- p0 p1)))
|#
;;;
(defun point-location (node) (get node 'coordinates))
(defun point-distance (n1 n2)
  (distance (point-location n1) (point-location n2)))
(defun point-path-length (path)
  (if (endp (rest path)) 0
    (+ (node-distance (first path) (second path))
       (path-length (rest path)))))
;;;
(defun node-distance (n1 n2) (point-distance n1 n2))
(defun path-length (n1 n2) (point-path-length n1 n2))
;;;
(defun <* (p1 p2 &key (cost-f #'length))
  (< (funcall cost-f p1) (funcall cost-f p2)))
;;;
(defun shorterp (p1 p2 &key (cost-f #'path-length))
  (<* p1 p2 :cost-f cost-f))

(defun closerp (p1 p2 target-node &key (estimate-f #'node-distance))
  (<* p1 p2
      :cost-f #'(lambda (p) (funcall estimate-f (car p) target-node))))

(defun short-closerp (p1 p2 target-node
			 &key
			 (cost-f #'path-length)
			 (estimate-f #'node-distance))
  (<* p1 p2
      :cost-f #'(lambda (p)
		  (+ (funcall cost-f p)
		     (funcall estimate-f (car p) target-node)))))
;;;
;;; generalized search
;;;
;;; extender
;;;
(defun depth-extender (pq f g &key (test #'node=))
  (append (new-path-list (car pq) g :test test) (cdr pq)))

(defun depth-first (s f g &key (test #'node=))
  (g-search s f g :test test :extender #'depth-extender))

(defun breadth-extender (pq f g &key (test #'node=))
  (append (cdr pq) (new-path-list (car pq) g :test test)))

(defun breadth-first (s f g &key (test #'node=))
  (g-search s f g :test test :extender #'breadth-extender))
;;;
;;;
(defun branch-and-bound-extender (pq f g &key (test #'node=)
				     (cost-f #'path-length))
  (sort (append (new-path-list (car pq) g :test test)
		(cdr pq))
	#'shorterp))

(defun branch-and-bound (s f g &key (test #'node=) (cost-f #'path-length))
  (g-search s f g :test test :extender #'branch-and-bound-extender)
  )
;;;
(defun hill-climb-extender (pq f g &key (test #'node=))
  (append (sort (new-path-list (car pq) g :test test)
		#'(lambda (p1 p2) (closerp p1 p2 f)))
	  (cdr pq)))
  
(defun hill-climb (s f g &key (test #'node=))
  (g-search s f g :test test :extender #'hill-climb-extender))

;;;
(defun best-first-extender (pq f g &key (test #'node=))
  (sort (append
	 (new-path-list (car pq) g :test test)
	 (cdr pq))
	#'(lambda (p1 p2) (closerp p1 p2 f)))
  )
;;;
(defun best-first (s f g &key (test #'node=))
  (g-search s f g :test test :extender #'best-first-extender))

;;;
(defun a*-extender (pq f g &key (test #'node=))
  (sort (append (new-path-list (car pq) g) (cdr pq))
	#'(lambda (p1 p2) (short-closerp p1 p2 f))))
(defun a* (s f g &key (test #'node=))
  (g-search s f g :test test :extender #'a*-extender))
;;;
(defun i-search (s f g &key (test #'node=) (extender #'depth-extender))
  (prog ((pq (list (list s))))
	loop
	(if (null pq) (return 'no-route))
	(if (funcall test f (car (car pq))) (return (reverse (car pq))))
	(setq pq (funcall extender pq f g))
	(go loop)))
;;;
(defun g-search (s f g &key (pq (list (list s)))
		   (test #'node=)
		   (extender #'depth-extender))
  (cond ((null pq) 'no-route)
    ((funcall test f (car (car pq))) (reverse (car pq)))
    (t (g-search s f g
		 :pq (funcall extender pq f g)
		 :extender extender
		 :test test))))
;;;
;;;
(unless (fboundp 'list-vector)
  (if (fboundp 'float-vector)
      (setf (symbol-function 'list-vector)
	    (symbol-function 'float-vector))
    (load "lmat.l")))
;;;
(defun init-nodes nil
  (setf (get 's 'coordinates) (list-vector  0 3))
  (setf (get 'a 'coordinates) (list-vector  4 6))
  (setf (get 'b 'coordinates) (list-vector  7 6))
  (setf (get 'c 'coordinates) (list-vector 11 6))
  (setf (get 'd 'coordinates) (list-vector  3 0))
  (setf (get 'e 'coordinates) (list-vector  6 0))
  (setf (get 'f 'coordinates) (list-vector 11 3)))

(init-nodes)


