#!/usr/bin/env roseus

(setq c1 (make-cube 100 100 100))
(setq c2 (make-cube 50 50 50))
(send c1 :set-color :red)
(send c2 :set-color :blue)
(send c2 :locate #f(300 0 0))
(send c1 :assoc c2)
(objects (list c1 c2))
(do-until-key
    (send c1 :rotate (deg2rad 5) :z)
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
)
