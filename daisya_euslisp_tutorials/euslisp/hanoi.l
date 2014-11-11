;;; hanoi program
;;; http://www.jsk.t.u-tokyo.ac.jp/~inaba/soft3/soft3-l3-euslisp/node67.html

;;;
;;; define objects
;;;

;; (defun hanoi-table
;;   (&key (name) (pos (float-vector 0 0 0)) (height 200))
;;   (setf height (* 1.5 height))
;;   (let ((b (make-cylinder 30 height))
;;         (c (make-cube 80 80 4)))
;;     (send c :locate (float-vector 0 0 height))
;;     (let ((bs (instance bodyset-link :init (send c :copy-worlcoords)
;;                         :bodies (list b c))))
;;     ;;(setq c (body+ c b))
;;       (setf (get c :top-face)
;;             (car (send c :get-face :cube :top)))
;;       (setf (get c :top-coords)
;;             (face-coords (get c :top-face)))
;;       (send c :assoc (get c :top-coords))
;;       (send c :locate pos)
;;       (send c :name name)
;;       (send c :set-color :gray90)
;;       c
;;       )))

(defun hanoi-table
  (&key (name) (pos (float-vector 0 0 0)) (height 200))
  ;;(setf height (* 1.5 height))
  (let ((b (make-cylinder 30 height))
        (c (make-cube 80 80 4))
        bs)
    (send c :locate (float-vector 0 0 height))
    (send c :set-color :gray90)
    (send b :set-color :gray90)
    (send b :assoc c)
    (setq bs (instance bodyset-link :init (make-cascoords :coords (send c :copy-worldcoords))
                       :bodies (list b c)))
    (setf (get bs :top-face)
          (car (send c :get-face :cube :top)))
    (setf (get bs :top-coords)
          (face-coords (get bs :top-face)))
    (send bs :assoc (get bs :top-coords))
    (send bs :locate pos)
    (send bs :name name)
    bs
    ))

(defun face-center (f)
  (cadr (send f :centroid)))
(defun face-coords
  (f &optional (c (make-cascoords)))
  (setq c (copy-object c))
  (send c :locate (face-center f) :world)
  c)

(defun hanoi-disk (&optional r h)
  (let* ((c (make-cylinder r h)))
    (setf (get c :top-face)
          (car (send c :get-face :cylinder :top)))
    (setf (get c :top-coords)
          (face-coords (get c :top-face)))
    (send c :assoc (get c :top-coords))
    (setf (get c :grasp-width) (* r 2))
    c
    ))

(defun disks (n &optional disks)
  (dotimes (i n)
    (setq disks (cons (hanoi-disk (+ (* (/ 27 n) i) 12) 20)
                      disks))
    (send (car disks) :set-color (send (elt gl::*face-colors* (* (+ i 4) 32)) :name))
    (send (car disks) :name (format nil "disk~A" i))
    )
  (reverse disks))

(defun disk (i) (elt *disks* (1- i)))

;;
;; define motions
;;

(defun table-top (table)
  (if (get table :disks)
      (car (get table :disks))
    table))

(defun move-disk (d from to &optional kdraw)
  (move-on d (table-top to))
  (setf (get from :disks) (cdr (get from :disks)))
  (setf (get to :disks) (cons d (get to :disks)))
  )

(defun move-on (o to)
  (send o
        :locate
        (v+ (send (get to :top-coords) :worldpos)
            #f(0 0 0.05))
        :world)
  )

;;;
;;; define planning
;;;

(defun other-table
  (a b
   &optional
   (tables '(*table-a* *table-b* *table-c*)))
  (car (remove b (remove a tables))))

(defun hanoi-program
  (&optional
   (n 3)
   (from '*table-a*)
   (to '*table-b*)
   (other (other-table from to)))
  (cond
      
      ((= n 1)
       (list (list 'move-disk (list 'disk n)
                   from to)))
    (t (append
        (hanoi-program (1- n) from other)
        (list (list 'move-disk (list 'disk n) from to))
        (hanoi-program (1- n) other to))))
  )

(defun hanoi-execute (n &optional (init *table-a*))
  (hanoi-init n init)
  (dolist (code (hanoi-program n '*table-a* '*table-b*))
    (eval code))
  )

;;;
;;; utils
;;;

(defun hanoi-init (n initial-table)
  (setq *disks* (disks n))
  (setq *worlds* (append *disks* *tables*))
  (dotimes (i n)
    (move-disk (disk (- n i))
               'ground initial-table))
  )

(defun hanoi-draw ()
  (send *pickview* :draw-objects))

;;;
;;;
;;;

(defun hanoi-setup ()
  (let ()
    (setq *table-a* (hanoi-table :name "table-a" :height 525 :pos #f(400 200 0)))
    (setq *table-b* (hanoi-table :name "table-b" :height 675 :pos #f(550 0 0)))
    (setq *table-c* (hanoi-table :name "table-c" :height 450  :pos #f(400 -200 0)))
    (setq *tables* (list *table-a* *table-b* *table-c*))
    (hanoi-init 3 *table-a*)
    (setq *disk-1* (car *disks*))
    (setq *disk-2* (cadr *disks*))
    (setq *disk-3* (caddr *disks*))
    ))

