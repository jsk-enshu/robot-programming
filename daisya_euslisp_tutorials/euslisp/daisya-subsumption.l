(load "daisya-maze.l")
(load "subsumption.l")

;;
(defun test-subsumption nil
  (let (i)
    (setq t1 (instance subsumption-task :init :name "Wander" :priority 10
                       :check-func #'(lambda (v) t)
                       :action-func #'(lambda ()
                                        (print "wander")
                                        (unix:usleep (* 100 1000)))))
    (setq t2 (instance subsumption-task :init :name "RightBumper" :priority 90
                       :check-func #'(lambda (v) (> (elt v 0) 0))
                       :action-func #'(lambda ()
                                        (print "right")
                                        (unix:sleep 2)
                                        (print "right done")
                                        )))
    (setq t3 (instance subsumption-task :init :name "LeftBumper" :priority 100
                       :check-func #'(lambda (v) (< (elt v 0) 0))
                       :action-func #'(lambda ()
                                        (print "left")
                                        (unix:sleep 3)
                                        (print "left done")
                                        )))
    (setq s (instance subsumption :init (list t1 t2 t3)
                        :sensor-vector #f(0 0 0 0 0 0)
                        :debug nil))
    (send s :start-clients)
    (setq i 0)
    (do-until-key
     (cond
      ((> i 100) (setq sensor-vector #f( 0 0 0)))
      ((> i 50) (setq sensor-vector #f( 10 0 0)))
      ((> i 25) (setq sensor-vector #f(-10 0 0)))
      (t        (setq sensor-vector #f(  0 0 0))))
     (send s :sensor-vector sensor-vector)
     (print (list i sensor-vector))
     (unix:usleep (* 100 1000))
     (incf i)
     )
    (send s :stop-clients)
    ))
  
(defun daisya-subsumption nil
  (let (i)
    (setq t1 (instance subsumption-task :init :name "Wander" :priority 10
                       :check-func #'(lambda (v) t)
                       :action-func #'(lambda ()
                                        (print "wander")
                                        (send *daisya* :daisya-velocity-vector
                                              (float-vector 10 0 0)))))
    (setq t2 (instance subsumption-task :init :name "RightBumper" :priority 80
                       :check-func #'(lambda (v) (< (elt v 0) 150))
                       :action-func #'(lambda ()
                                        (print "right")
                                        (send *daisya* :daisya-velocity-vector
                                              (float-vector 10 0 5))
                                        )))
    (setq t3 (instance subsumption-task :init :name "LeftBumper" :priority 90
                       :check-func #'(lambda (v) (< (elt v 2) 100))
                       :action-func #'(lambda ()
                                        (print "left")
                                        (send *daisya* :daisya-velocity-vector
                                              (float-vector 10 0 -15))
                                        )))
    (setq t4 (instance subsumption-task :init :name "FrontBumper" :priority 100
                       :check-func #'(lambda (v) (< (elt v 1) 150))
                       :action-func #'(lambda ()
                                        (print "front")
                                        (send *daisya* :daisya-velocity-vector
                                              (float-vector -10 0 -5))
                                        )))
    (setq s (instance subsumption :init (list t1 t2 t3 t4)
                        :sensor-vector #f(0 0 0 0 0 0)
                        :debug nil))
    (send s :start-clients)
    (setq i 0)
    (do-until-key
     (send *daisya* :simulate (send *maze* :bodies))
     (setq sensor-vector (send *daisya* :psd-vector))
     (send s :sensor-vector sensor-vector)
     (print (list i sensor-vector))
     (unix:usleep (* 100 1000))
     (draw-viewers)
     (incf i)
     )
    (send s :stop-clients)
    ))

(warn ";; type (test-subsumption) for test~%")
(warn ";; type
;; (maze-init)
;; (daisya-subsumption)
;; for daisya subsumption simulation")

