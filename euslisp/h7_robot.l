(load "models/room73b2-scene.l")
(load "models/h7-robot.l")

(setq *room* (room73b2))
(setq *robot* (h7))

(objects (list *robot* *room*))
(send *irtviewer* :look-all
    (geo::make-bounding-box
        (flatten (send-all (send *robot* :bodies) :vertices))
    )
)
