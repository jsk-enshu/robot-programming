(load "irteusext.l")

(defclass subsumption
  :super propertied-object
  :slots (clients owner debug sensor-vector))
(defmethod subsumption
  (:init
   (c &key ((:debug d) t)
      ((:sensor-vector s)))
   (setq debug d owner 0 sensor-vector s)
   (setq clients (sort c #'>= #'(lambda (x) (send x :priority))))
   (send-all clients :main self)
   (send-all clients :debug debug)
   (send self :print-clients)
   (send self :execute (car (last clients)))
   self)
  (:print-clients
   ()
   (warn "~16A (~8A) ~A~%" 'name 'priority 'state)
   (dolist (c clients)
     (warn "~16A (~8d)~%" (send c :name) (send c :priority))))
  (:execute
   (client)
   ;; check running client
   (when (memq 'start (send-all clients :state))
     (return-from :execute nil))
   ;; check priority
   (when (and owner (< (send client :priority) owner))
     (return-from :execute nil))
   
   (if debug (warn ">execute     ~12A (~3d) / owner=~3d~%"
                   (send client :name) (send client :priority) owner))

   ;; set owner
   (setq owner (send client :priority))
   
   ;; start new owner
   (send client :state 'start)
   )
  ;;
  (:sensor-vector
   (&optional s)
   (if s (setq sensor-vector (copy-object s)))
   sensor-vector)
  (:owner (&optional o) (if o (setq owner o)) owner)
  ;;
  (:start-clients 
   ()
   (send-all clients :thread-start))
  (:stop-clients 
   ()
   (send-all clients :thread-stop))
  )

(defvar *sub-lock* (sys::make-mutex-lock))
(defclass subsumption-task
  :super propertied-object
  :slots (main state loop thr priority debug check-func action-func))
(defmethod subsumption-task
  (:init 
   (&key (name "subsumption-client") ((:priority prio) 100) ((:debug d) nil)
         ((:check-func c-func)) ((:action-func a-func)))
   (send self :name name)
   (setq state 'stop check-func c-func action-func a-func
         loop nil priority prio debug d)
   self)
  (:debug (&optional d) (setq debug d))
  (:state (&optional s) (if s (setq state s)) state)
  (:priority (&optional p) (if p (setq priority  p)) priority)
  (:main (&optional m) (if m (setq main m)) main)
  (:action
   ()
   (if action-func
       (funcall action-func)
     (unix:sleep 1)))
  (:check
   (sensor-state)
   (sys::mutex
    *sub-lock*
    (if (and check-func sensor-state)
        (funcall check-func sensor-state)
      nil)))
  ;;
  (:thread-start
   ()
   (let (thread-num)
     (unless loop
       (need-thread 1)
       (setq loop t)
       (setq thr (sys:thread #'send self :thread-loop)))
     thr))
  (:thread-loop
   ()
   (let (check owner)
     (while loop
       (setq check
             (send self :check (send main :sensor-vector)))
       (setq owner (send main :owner))
       (when check (send main :execute self))
       (if debug (warn ">thread-loop ~12A (~3d) / state = ~6A / sensor ~A~%"
                       (send self :name) priority state check))
       (when (eq state 'start)
         (send self :action)
         (setq state 'stop)
         (send main :owner owner))
       (unix:usleep (* 100 1000))
       )))
  (:thread-stop
   ()
   (setq loop nil)
   (when thr
     (if debug (warn ";; wait thread ~A ... " (send self :name)))
     (sys::wait-thread thr)
     (if debug (warn "done~%" (send self :name)))))
  )

;;;
;;;
  

(defun test-subsumption
  (&key (exit-count 150))
  (let (i)
    (setq t1 (instance subsumption-task :init :name "Wander" :priority 10
                       :check-func #'(lambda (v) t)
                       :action-func #'(lambda ()
                                        (warning-message 1 "wander~%")
                                        (unix:usleep (* 100 1000)))))
    (setq t2 (instance subsumption-task :init :name "RightBumper" :priority 90
                       :check-func #'(lambda (v) (> (elt v 0) 0))
                       :action-func #'(lambda ()
                                        (warning-message 1 "right~%")
                                        (unix:sleep 2)
                                        (warning-message 1 "right done~%")
                                        )))
    (setq t3 (instance subsumption-task :init :name "LeftBumper" :priority 100
                       :check-func #'(lambda (v) (< (elt v 0) 0))
                       :action-func #'(lambda ()
                                        (warning-message 1 "left~%")
                                        (unix:sleep 3)
                                        (warning-message 1 "left done~%")
                                        )))
    (setq s (instance subsumption :init (list t1 t2 t3)
                        :sensor-vector #f(0 0 0)
                        :debug nil))
    (send s :start-clients)
    (setq i 0)
    (block
        :loop
      (do-until-key
       (cond
        ((or (< i 25) (< 100 i)) (setq sensor-vector #f( 0 0 0)))
        ((<  i 50) (setq sensor-vector #f(-10 0 0)))
        ((>= i 50) (setq sensor-vector #f( 10 0 0)))
        )
       (send s :sensor-vector sensor-vector)
       (print (list i sensor-vector))
       (unix:usleep (* 100 1000))
       (incf i)
       (if (> i exit-count) (return-from :loop nil))
       )
      )
    (send s :stop-clients)
    ))
  
