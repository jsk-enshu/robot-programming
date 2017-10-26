#!/usr/bin/env roseus

(defun make-ball (size)
    (setq q0 (make-icosahedron size))
    (setq q1 (make-gdome q0))
    (setq q2 (make-gdome q1))
    (setq ball (make-gdome q2))
    ball
)

(setq sun (make-ball 10))
(send sun :set-color :red)

(setq earth (make-ball 5))
(send earth :set-color :blue)
(send earth :locate #f(100 0 0))
(send sun :assoc earth)

(setq moon (make-ball 2))
(send moon :set-color :white)
(send moon :locate #f(110 0 0))
(send earth :assoc moon)

(objects (list sun earth moon))

(do-until-key
    (send sun :rotate (deg2rad 1) :z)
    (send earth :rotate (deg2rad 10) :z)
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
)
