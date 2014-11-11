;;;
;;; set operation
;;;
(defun set-member (e set &key (test #'eq))
  (cond
    ((null set) nil)
    ((funcall test e (car set)) t)
    (t (set-member e (cdr set) :test test))))

(defun set-member (e set &key (test #'eq))
  (do ()
      ((or (null set) (funcall test e (car set)))
       (if (null set) nil t))
      (setq set (cdr set))))

(defun set-adjoin (a s &key (test #'eq)) ;; adjoin
  (if (set-member a s :test test) s (cons a s)))

(defun set-remove (e set &key (test #'eq))
  (cond
      ((null set) nil)
    ((funcall test e (car set))
     (set-remove e (cdr set) :test test))
    (t (cons (car set)
	     (set-remove e (cdr set) :test test)))))

(defun set-remove (e set &key (test #'eq))
  (do ((r nil))
      ((null set) r)
      (if (null (funcall test e (car set)))
	  (setq r (cons (car set) r)))
      (setq set (cdr set))))

(defun set+ (a b &key (test #'eq))
  (cond ((null a) b)
    ((set-member (car a) b :test test)
     (set+ (cdr a) b :test test))
    (t (cons (car a) (set+ (cdr a) b :test test)))))

(defun set+ (a b &key (test #'eq))
  (do ((r b))
      ((null a) r)
      (if (not (set-member (car a) b :test test))
	  (setq r (cons (car a) r)))
      (setq a (cdr a))))

(defun set- (a b &key (test #'eq))
  (cond
      ((null a) nil)
    ((set-member (car a) b :test test)
     (set- (cdr a) b :test test))
    (t (cons (car a) (set- (cdr a) b :test test)))))

(defun set* (a b &key (test #'eq))
  (cond
      ((null a) nil)
    ((set-member (car a) b :test test)
     (cons (car a) (set* (cdr a) b :test test)))
    (t (set* (cdr a) b :test test))))

(defun set^ (a b &key (test #'eq))
  (cond
      ((null a) b)
    ((set-member (car a) b :test test)
     (set^ (cdr a) (remove (car a) b :test test) :test test))
    (t (cons (car a) (set^ (cdr a) b :test test)))))

(defun set< (a b &key (test #'eq))
  (cond
      ((null a) t)
    ((set-member (car a) b :test test)
     (set< (cdr a) b :test test))
    (t nil)))

(defun set= (a b &key (test #'eq))
  (null (set^ a b :test test)))

;;;
(defun eq-classes (x &key (test #'eq))
  (cond
   ((null x) nil)
   (t (eq-classes-aux
       (car x)
       (eq-classes (cdr x) :test test)
       :test test))))

(defun eq-classes-aux (a b &key (test #'eq))
  (cond
      ((null b) (list a))
    ((set* a (car b) :test test)
     (eq-classes-aux
      (set+ a (car b) :test test)
      (cdr b)
      :test test))
    (t (cons (car b)
	     (eq-classes-aux
	      a (cdr b)
	      :test test)))))
#|
> (eq-classes '((a b) (c d) (a c)))
((a b c d))
> (eq-classes '((s a) (s d) (a b) (a d)
                  (b c) (b e) (d e) (e f)))
((s a d b c e f))
> (eq-classes '((1 2) (2 3) (4 5) (6 5)))
((4 6 5) (1 2 3))
> (eq-classes
    '(((1) (2)) ((2) (3)) ((4) (5)) ((6) (5))))
(((6) (5)) ((4) (5)) ((2) (3)) ((1) (2)))
> (eq-classes
     '(((1) (2)) ((2) (3)) ((4) (5)) ((6) (5)))
      :test #'equal)
(((4) (6) (5)) ((1) (2) (3)))
|#