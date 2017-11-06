#!/usr/bin/env roseus

(defun make-link nil
    (setq link (make-cylinder 10 90))
    torso
)

(defclass six-dof-arm
    :super cascaded-link
    :slots (torso head leg_l leg_r joint_leg_l joint_leg_r joint_head)
)

(defmethod six-dof-arm
    (:init ()
        (let (b)
            (send-super :init)

            (setq b (make-cube 10 10 20))
            (send b :locate #f(0 0 10))
            (send b :set-color :red)
            (setq l7 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l7))
            (setq end-coords (make-cascoords :pos #f(0 0 20)))
            (send l7 :assoc end-coords)
            (send l7 :locate #f(0 0 100))

            (setq b (make-cube 10 10 100))
            (send b :locate #f(0 0 50))
            (send b :set-color :orange)
            (setq l6 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l6))
            (send l6 :assoc l7)
            (send l6 :locate #f(0 0 100))

            (setq b (make-cube 10 10 100))
            (send b :locate #f(0 0 50))
            (send b :set-color :yellow)
            (setq l5 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l5))
            (send l5 :assoc l6)
            (send l5 :locate #f(0 0 100))

            (setq b (make-cube 10 10 100))
            (send b :locate #f(0 0 50))
            (send b :set-color :green)
            (setq l4 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l4))
            (send l4 :assoc l5)
            (send l4 :locate #f(0 0 100))

            (setq b (make-cube 10 10 100))
            (send b :locate #f(0 0 50))
            (send b :set-color :blue)
            (setq l3 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l3))
            (send l3 :assoc l4)
            (send l3 :locate #f(0 0 100))

            (setq b (make-cube 10 10 100))
            (send b :locate #f(0 0 50))
            (send b :set-color :purple)
            (setq l2 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l2))
            (send l2 :assoc l3)
            (send l2 :locate #f(0 0 20))

            (setq b (body+ (make-cube 10 10 20 :pos #f(0 0 10)) (make-cube 300 300 2)))
            (send b :set-color :white)
            (setq l1 (instance bodyset-link :init (make-cascoords) :bodies (list b) :name 'l1))
            (send l1 :assoc l2)

            (setq j1 (instance rotational-joint :init :name 'j1
                        :parent-link l1 :child-link l2 :axis :x :min -100 :max 100)
                  j2 (instance rotational-joint :init :name 'j2
                        :parent-link l2 :child-link l3 :axis :y :min -100 :max 100)
                  j3 (instance rotational-joint :init :name 'j3
                        :parent-link l3 :child-link l4 :axis :z :min -100 :max 100)
                  j4 (instance rotational-joint :init :name 'j3
                        :parent-link l4 :child-link l5 :axis :x :min -100 :max 100)
                  j5 (instance rotational-joint :init :name 'j3
                        :parent-link l5 :child-link l6 :axis :y :min -100 :max 100)
                  j6 (instance rotational-joint :init :name 'j3
                        :parent-link l6 :child-link l7 :axis :z :min -100 :max 100)
            )
            (setq links (list l1 l2 l3 l4 l5 l6 l7))
            (setq joint-list (list j1 j2 j3 j4 j5 j6))

            (send self :init-ending)
            self
        )
    )
    (:end-coords (&rest args) (forward-message-to end-coords args))
    (:move-hand-to (x y z)
        (send self :angle-vector
            (send self :inverse-kinematics
                (make-cascoords :pos (float-vector x y z) :rpy #f(0 0 0))
                :link-list (send self :link-list (send self :end-coords :parent))
                :move-target (send self :end-coords)
                :translation-axis t
                :rotation-axis nil
                :debug-view nil
            )
        )
    )
)

(setq robot (instance six-dof-arm :init))
(objects (list robot))

(setq i 0)
(do-until-key
    (setq radius 200)
    (send robot :move-hand-to (* radius (cos (/ i 100.0))) (* radius (sin (/ i 100.0))) 400)

    (send *irtviewer* :draw-objects)
    (setq i (+ i 1))
)

