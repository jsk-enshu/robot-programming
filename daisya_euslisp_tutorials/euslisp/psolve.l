;;;
;;; blocks world and maze example
;;;
(load "hsearch.l")

(defun init-blocks nil
  (setq *s1* '((a on table) (b on table) (c on table)))
  (setq *s1a* '((b on table) (a on table) (c on table)))
  (setq *s2* '((a on table) (b on c)     (c on table)))
  (setq *s3* '((a on table) (b on a)     (c on table)))
  (setq *s4* '((a on c)     (b on table) (c on table)))
  (setq *s5* '((a on b)     (b on table) (c on table)))
  (setq *s6* '((a on table) (b on table) (c on b)))
  (setq *s7* '((a on table) (b on table) (c on a)))
  (setq *s8* '((a on b)     (b on c)     (c on table)))
  (setq *s9* '((a on table) (b on a)     (c on b)))
  (setq *s10* '((a on c)    (b on a)     (c on table)))
  (setq *s11* '((a on b)    (b on table) (c on a)))
  (setq *s12* '((a on c)    (b on table) (c on b)))
  (setq *s13* '((a on table) (b on c)    (c on a)))
  (setq *b*
	(list
	 (list *s1* *s2*)
	 (list *s1* *s3*)
	 (list *s1* *s4*)
	 (list *s1* *s5*)
	 (list *s1* *s6*)
	 (list *s1* *s7*)
	 (list *s2* *s8*)
	 (list *s2* *s3*)
	 (list *s3* *s9*)
	 (list *s4* *s10*)
	 (list *s5* *s11*)
	 (list *s6* *s12*)
	 (list *s6* *s7*)
	 (list *s7* *s13*)))
  (setq *b1*
	(list
	 (list *s1a* *s2*)
	 (list *s1a* *s3*)
	 (list *s1a* *s4*)
	 (list *s1a* *s5*)
	 (list *s1a* *s6*)
	 (list *s1a* *s7*)
	 (list *s2* *s8*)
	 (list *s2* *s3*)
	 (list *s3* *s9*)
	 (list *s4* *s10*)
	 (list *s5* *s11*)
	 (list *s6* *s12*)
	 (list *s6* *s7*)
	 (list *s7* *s13*)))
  )

(init-blocks)

(setq *m*
      '((1 2) (2 3) (3 4) (4 9) (9 14) (9 8)
	(8 7) (7 12) (12 13) (12 11) (11 6)
	(11 16) (14 15) (16 17) (17 22)
	(21 22) (22 23) (23 18) (23 24)
	(24 19) (19 20) (20 15) (15 10)
	(10 5) (20 25)))
;;;
(defun node= (a b)
  (if (atom a) (eq a b) (set= a b :test #'equal)))

(defun path-length (path)
  (cond
      ((symbolp (car path)) (point-path-length path))
    (t (length path))))

(defun node-distance (n1 n2)
  (cond
      ((numberp n1) (- n2 n1))
    ((atom n1) (point-distance n1 n2))
    (t (length (set- n1 n2 :test #'equal)))))

(defun blocks-state= (a b) (set= a b :test #'equal))

(defun block-search (s f g &key (extender #'depth-extender))
  (g-search s f g :extender extender :test #'blocks-state=))

(defun maze-search (s f m &key (extender #'depth-extender))
  (g-search s f m :extender extender))

#|

> (i-search 's 'f *g*)
(S A B E F)
> (i-search 's 'f *g* :extender #'breadth-extender)
(S D E F)
> (i-search 's 'f *g* :extender #'branch-and-bound-extender)
(S D E F)
> (i-search 's 'f *g* :extender #'hill-climb-extender)
(S A B E F)
> (i-search 's 'f *g* :extender #'best-first-extender)
(S A B E F)
> (i-search 's 'f *g* :extender #'a*-extender)
(S D E F)

> (block-search *s1* *s13* *b*)
(((A ON TABLE) (B ON TABLE) (C ON TABLE))
 ((A ON TABLE) (B ON TABLE) (C ON B))
 ((A ON TABLE) (B ON TABLE) (C ON A))
 ((A ON TABLE) (B ON C) (C ON A)))

> (block-search *s1* *s13* *b* :extender #'breadth-extender)
(((A ON TABLE) (B ON TABLE) (C ON TABLE))
 ((A ON TABLE) (B ON TABLE) (C ON A))
 ((A ON TABLE) (B ON C) (C ON A)))

> (block-search *s1* *s13* *b* :extender #'hill-climb-extender)
(((A ON TABLE) (B ON TABLE) (C ON TABLE))
 ((A ON TABLE) (B ON TABLE) (C ON A))
 ((A ON TABLE) (B ON C) (C ON A)))

> (block-search *s1* *s13* *b* :extender #'branch-and-bound-extender)
(((A ON TABLE) (B ON TABLE) (C ON TABLE))
 ((A ON TABLE) (B ON TABLE) (C ON A))
 ((A ON TABLE) (B ON C) (C ON A)))

> (block-search *s1* *s13* *b* :extender #'best-first-extender)
(((A ON TABLE) (B ON TABLE) (C ON TABLE))
 ((A ON TABLE) (B ON TABLE) (C ON A))
 ((A ON TABLE) (B ON C) (C ON A)))

> (i-search 7 21 *m*)
(7 8 9 14 15 20 19 24 23 22 21)
> (i-search 7 21 *m* :extender #'breadth-extender)
(7 12 11 16 17 22 21)
> (i-search 7 21 *m* :extender #'branch-and-bound-extender)
(7 12 11 16 17 22 21)
> (i-search 7 21 *m* :extender #'hill-climb-extender)
(7 12 11 16 17 22 21)
> (i-search 7 21 *m* :extender #'best-first-extender)
(7 12 11 16 17 22 21)
> (i-search 7 21 *m* :extender #'a*-extender)
(7 12 11 16 17 22 21)
|#
