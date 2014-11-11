;;;
;;; search program	2009.10.25	M.Inaba
;;;
;;;
;;; graph
;;;
(defun node= (a b) (equal a b))

(defun a-list (v g &key (test #'node=))
  (cond ((null g) nil)
    ((set-member v (car g) :test test)
     (set+ (set-remove v (car g) :test test)
	   (a-list v (cdr g) :test test)
	   :test test))
    (t (a-list v (cdr g) :test test))))

(defun new-nodes (path graph &key (test #'node=))
  (p-extend path
	    (a-list (first path) graph :test test)
	    :test test))

(defun p-extend (path nodes &key (test #'node=))
  (cond ((null nodes) nil)
    ((set-member (car nodes) path :test test)
     (p-extend path (cdr nodes) :test test))
    (t (cons (car nodes)
	     (p-extend path (cdr nodes) :test test)))))

(defun new-path-list (path graph &key (test #'node=))
  (mapcar #'(lambda (n) (cons n path))
	  (new-nodes path graph :test test)))
;;;
;;;
;;;
(defun depth-first (s f g &optional (p (list s)) (pq (list p)))
  (cond ((null pq) nil)
    ((node= f (first p)) (reverse p))
    (t (depth-first
	s f g
	(car pq)
	(append (new-path-list p g) (rest pq))))))

;;;
(defun depth-first (s f g &optional (p (list s)) (pq (list p)))
  (do ()
      ((or (null pq) (node= f (first p)))
       (if (null pq) nil (reverse p)))
      (setq p (car pq))
      (setq pq (append (new-path-list p g) (rest pq))))
  )
;;;
;;;
(defun breadth-first (s f g &optional (p (list s)) (pq (list p)))
  (cond ((null pq) nil)
    ((node= f (first p)) (reverse p))
    (t (breadth-first
	s f g
	(car pq)
	(append (rest pq) (new-path-list p g))))))
;;;
;;;
(setq *g* '((s a) (s d) (a b) (a d) (b c) (b e) (d e) (e f)))
;;;
