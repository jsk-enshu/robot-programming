#!/usr/bin/env roseus

(defun make-torso nil
    (setq torso (make-cylinder 10 50))
    (send torso :locate #f(0 0 25))
    (send torso :rotate (/ pi 4) :x)
    torso
)
(defun make-head nil
    (setq head (make-cylinder 7 20))
    (send head :locate #f(0 -45.7 70.7))
    (send head :rotate (* 3 (/ pi 4)) :x)
    head
)

(defclass Pigeon
    :super cascaded-link
    :slots (torso head leg_l leg_r joint_leg_l joint_leg_r joint_head)
)

(defmethod Pigeon
    (:init ()
        (let ()
            (send-super :init)
            (setq torso (instance bodyset-link :init (make-cascoords) :bodies (list (make-torso))))
            (setq head (instance bodyset-link :init (make-cascoords) :bodies (list (make-head))))
            (setq leg_l (instance bodyset-link :init (make-cascoords) :bodies (list (make-leg))))
            (setq leg_r (instance bodyset-link :init (make-cascoords) :bodies (list (make-leg))))

            (setq joint_head rotational-joint :init :parent-link torso :child-link head :axis :y)
            (setq joint_leg_l rotational-joint :init :parent-link torso :child-link head :axis :y)
            (setq joint_leg_r rotational-joint :init :parent-link torso :child-link head :axis :y)

            (setq links (list torso head leg_l leg_r))
            (setq joint-list (list joint_head joint_leg_l joint_leg_r))

            (send self :assoc torso)
            (send torso :assoc head)
            (send torso :assoc joint_leg_l)
            (send torso :assoc joint_leg_r)

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

(setq torso (instance bodyset-link :init (make-cascoords) :bodies (list (make-torso))))
(setq head (instance bodyset-link :init (make-cascoords) :bodies (list (make-head))))
(objects (list torso head))
