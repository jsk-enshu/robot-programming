#!/usr/bin/env roseus

(defun make-root nil
    (let (r)
        (setq r (make-cylinder 10 30))
        (send r :locate #f(0 0 0))
        (send r :set-color :gray20)
        r
    )
)

(defun make-hand nil
    (let ((h (make-cylinder 5 30)))
        (send h :locate #f(0 0 0))
        (send h :set-color :gray80)
        h
    )
)

(defclass slider-model
    :super cascaded-link
    :slots (r h j)
)

(defmethod slider-model
    (:init ()
        (let ()
            (send-super :init)
            (setq r (instance bodyset-link :init (make-cascoords) :bodies (list (make-root))))
            (setq h (instance bodyset-link :init (make-cascoords) :bodies (list (make-hand))))
            (setq j (instance linear-joint :init :parent-link r :child-link h :axis :z :min 0 :max 30))

            (setq links (list r h))
            (setq joint-list (list j))

            (send self :assoc r)
            (send r :assoc h)

            (send self :init-ending)
            (setq slider_pos 5)
            self
        )
    )
    (:j (&rest args) (forward-message-to j args))
    (:slider_pos (value)
        (setq slider_pos value)
        (send h :locate (float-vector slider_pos 0 0))
    )
)

(setq s (instance slider-model :init))
(objects (list s))
(dotimes (i 1000)
;    (send r :angle-vector (float-vector (* 90 (sin (/ i 100.0)))))
    (send (s . j) :joint-angle (+ 15 (* 15 (sin (/ i 100.0)))))
    (send *irtviewer* :draw-objects)
)
