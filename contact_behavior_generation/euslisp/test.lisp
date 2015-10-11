#!/usr/bin/env roseus

(defvar *robot-type* :hrp2jsknts-collada)
(require "motion-sequencer.lisp")
(require "dynamic-connector.lisp")

(defun test-proc
  (key)
  (case key
    (:kirin-ladder
     (cond
      ((let* ((init
	       (progn
		 (demo-climb-setup :kirin-ladder)
		 (init-pose)
		 (send *robot* :fix-leg-to-coords (make-coords :pos #F(0 0 3)))
		 (send *robot* :reset-manip-pose)
		 (send *robot* :arms :shoulder-p :joint-angle 0)))
	      (rsd (demo-motion-sequence-with-timer
		    :ik-debug-view *debug*
		    :loop-max 8
		    :tmax-hand-rate 0.6
		    :tmax-leg-rate 0.6
		    :log-file (format nil "~A/kirin-ladder" *log-root*)
		    :trajectory-check-func
		    '(lambda (next now &rest args)
		       (send next :buf :time 2.0)
		       (send now :buf :time 0.0)
		       (connect-rsd :rsd-list (list next now))
		       (send next :buf :trajectory))
		    ;;(not (assoc :status
		    ;;(send next :buf :trajectory)))
		    ;;(cdr (assoc :status
		    ;;(send next :buf :trajectory)))))
		    )))
	 (cond
	  ((listp rsd)
	   (rsd-play :file (format nil "~A/kirin-ladder.rsd" *log-root*) :auto? t)
	   (quit-graph)))
	 (atom rsd))
       (print 'kirin-ladder-error)
       (exit -1))))
    (:rock-wall
     (cond
      ((let* ((init (demo-climb-setup :rock-wall))
	      (rsd (demo-motion-sequence-with-timer
		    :ik-debug-view *debug*
		    :loop-max 8
		    :tmax-hand-rate 0.8
		    :tmax-leg-rate 0.8
		    :log-file (format nil "~A/rock-wall" *log-root*)
		    )))
	 (cond
	  ((listp rsd)
	   (rsd-play :file (format nil "~A/rock-wall.rsd" *log-root*) :auto? t)
	   (quit-graph)))
	 (atom rsd))
       (print 'rock-wall-error)
       (exit -1))))
    (:four-leg-seat
     (cond
      ((let* ((init
	       (progn (demo-climb-setup :four-leg-seat)
		      (setq *goal-contact-state* (nth 86 *contact-states*))
		      (send *robot* :reset-manip-pose)
		      (send *robot* :fix-leg-to-coords (make-coords))))
	      (rsd (demo-motion-sequence-with-timer
		    :ik-debug-view *debug*
		    :loop-max 2
		    :remove-limb :hip
		    :all-limbs '(:hip)
		    :now-rsd
		    (instance robot-state-data2 :init :contact-states
			      (cons (now-hip-contact-state)
				    (now-contact-state :limb-keys '(:rleg :lleg))))
		    :tmax-hand-rate 1.0
		    :tmax-leg-rate 1.0
		    :log-file (format nil "~A/four-leg-seat" *log-root*)
		    )))
	 (cond
	  ((listp rsd)
	   (rsd-play :file (format nil "~A/four-leg-seat.rsd" *log-root*) :auto? t)
	   (quit-graph)))
	 (atom rsd))
       (print 'four-leg-seat-error)
       (exit -1))))
    ;; ((progn
    ;;    (demo-climb-setup :param-ladder)
    ;;    (atom (demo-motion-sequence-with-timer
    ;; 	   :ik-debug-view *debug*
    ;; 	   :loop-max 8
    ;; 	   :log-file (format nil "~A/param-ladder" *log-root*))))
    ;;  (print 'param-ladder-error)
    ;;  (exit -1))
    (:simple-floor
     (cond
      ((let* ((init (progn
		      (cond
		       ((not (eq *robot-type* :staro))
			(setq *robot-type* :staro)
			(load "robot-param.lisp")))
		      (demo-climb-setup :simple-floor)
		      (send *best-facefall* :draw :friction-cone? nil)
		      (gen-init-rsd)
		      ))
	      (rsd (demo-motion-sequence-with-timer
		    :ik-debug-view *debug*
		    :now-rsd (instance robot-state-data2 :init
				       :contact-states *facefall-contact-state*)
		    :remove-limb :rleg
		    ;; :log-stream t
		    ;;:loop-max 8
		    :loop-max
		    '(lambda (cnt &rest args)
		       (let* ((ret (simple-fullbody :target (list (list (cons :target :rarm)) (list (cons :target :larm)) (list (cons :target :rleg)) (list (cons :target :lleg))) :debug-view nil :stop 30 :centroid-thre 100 :warnp nil)))
			 (if ret (send *viewer* :draw-objects)) ret))
		    :tmax-hand-rate 1.0
		    :tmax-leg-rate 1.0
		    :log-file (format nil "~A/simple-floor" *log-root*)
		    )))
	 (cond
	  ((listp rsd)
	   (rsd-play :file (format nil "~A/simple-floor.rsd" *log-root*) :auto? t)
	   (quit-graph)
	   ))
	 (atom rsd))
       (print 'simple-floor-error)
       (exit -1))))
    ))

;; start test

(defvar *log-root* (format nil "log/~A" (log-surfix)))
(defvar *debug* :no-message)
(unix:system (format nil "mkdir ~A" *log-root*))

(test-proc :kirin-ladder)
(test-proc :rock-wall)
(test-proc :four-leg-seat)
(test-proc :simple-floor)

(print 'all-ok)
(exit 0)
