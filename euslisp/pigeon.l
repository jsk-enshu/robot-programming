#!/usr/bin/env roseus

(defun make-torso nil
    (setq torso (make-cylinder 10 90))
    (send torso :locate #f(0 40 10))
    (send torso :rotate (/ pi 4) :x)
    torso
)
(defun make-head nil
    (setq head (make-cylinder 7 20))
    (send head :locate #f(0 -30 70.7))
    (send head :rotate (* 3 (/ pi 4)) :x)
    head
)
(defun make-leg-l nil
    (setq leg-l (make-cylinder 3 40))
    (send leg-l :locate #f(10 0 0))
    leg-l
)
(defun make-leg-r nil
    (setq leg-r (make-cylinder 3 40))
    (send leg-r :locate #f(-10 0 0))
    leg-r
)

(defclass pigeon-model
    :super cascaded-link
    :slots (torso head leg_l leg_r joint_leg_l joint_leg_r joint_head)
)

(defmethod pigeon-model
    (:init ()
        (let ()
            (send-super :init)
            (setq torso (instance bodyset-link :init (make-cascoords) :bodies (list (make-torso))))
            (setq head (instance bodyset-link :init (make-cascoords) :bodies (list (make-head))))
            (setq leg_l (instance bodyset-link :init (make-cascoords) :bodies (list (make-leg-l))))
            (setq leg_r (instance bodyset-link :init (make-cascoords) :bodies (list (make-leg-r))))

            (setq joint_head (instance rotational-joint :init :parent-link torso :child-link head :axis :x))
            (setq joint_leg_l (instance rotational-joint :init :parent-link torso :child-link leg_l :axis :x))
            (setq joint_leg_r (instance rotational-joint :init :parent-link torso :child-link leg_r :axis :x))

            (setq links (list torso head leg_l leg_r))
            (setq joint-list (list joint_head joint_leg_l joint_leg_r))

            (send self :assoc torso)
            (send torso :assoc head)
            (send torso :assoc leg_l)
            (send torso :assoc leg_r)

            (send self :init-ending)
            self
        )
    )
;    (:walk
;        (setq i 0)
;        (do-until-key
;            (send )
;        )
;    )
)

(setq pigeon-m (instance pigeon-model :init))
(objects (list pigeon-m))
