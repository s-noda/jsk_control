#!/usr/bin/env roseus

(defvar *robot-type* :hrp2jsknts-collada)
(require "motion-sequencer.lisp")
(require "dynamic-connector.lisp")

(require "motion-planners/motion-planner.lisp") (defun demo-motion-sequence (&rest args) (let* ((ret (apply 'demo-motion-sequence2 (append args (list :error-thre 1.1))))) (cons (car ret) (reverse (cdr ret)))))

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
		    :ik-debug-view *ik-debug-view*
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
		    :ref-order '(:rarm :larm :rleg :lleg)
		    )))
	 (cond
	  ((listp rsd)
	   (rsd-play :file (format nil "~A/kirin-ladder.rsd" *log-root*) :auto? t)
	   (quit-graph)))
	 (atom rsd))
       (print 'kirin-ladder-error)
       t)))
    (:rock-wall
     (cond
      ((let* ((init (demo-climb-setup :rock-wall))
	      (rsd (demo-motion-sequence-with-timer
		    :ik-debug-view *ik-debug-view*
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
       t)))
    (:four-leg-seat
     (cond
      ((let* ((init
	       (progn (demo-climb-setup :four-leg-seat)
		      (setq *goal-contact-state* (nth 86 *contact-states*))
		      (send *robot* :reset-manip-pose)
		      (send *robot* :fix-leg-to-coords (make-coords))))
	      (rsd (demo-motion-sequence-with-timer
		    :ik-debug-view *ik-debug-view*
		    :loop-max
                    '(lambda (id)
                       (< (norm (send (send *goal-contact-state* :target-coords)
                                      :difference-position
                                      (send (send *goal-contact-state* :contact-coords) :worldcoords)))
                          10))
		    :remove-limb :hip
		    :all-limbs '(:hip)
		    :now-rsd
		    ;;(instance robot-state-data2 :init :contact-states (cons (now-hip-contact-state) (now-contact-state :limb-keys '(:rleg :lleg))))
		    (optimize-brli :contact-states
				   (now-contact-state :limb-keys '(:rleg :lleg))
				   :rest-contact-states
				   (list (now-hip-contact-state)))
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
       t)))
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
		    :ik-debug-view *ik-debug-view*
		    :now-rsd
		    ;;(instance robot-state-data2 :init
		    ;; :contact-states *facefall-contact-state*)
		    (optimize-brli :contact-states *facefall-contact-state*)
		    :remove-limb :rarm
		    ;; :log-stream t
		    ;;:loop-max 8
		    :loop-max
		    '(lambda (cnt &rest args)
		       (let* ((ret (simple-fullbody :target (list (list (cons :target :rarm)) (list (cons :target :larm)) (list (cons :target :rleg)) (list (cons :target :lleg))) :debug-view nil :stop 30 :centroid-thre 100 :warnp nil)))
			 (if ret (send *viewer* :draw-objects)) ret))
		    ;; :cs-filter-func '(lambda (&rest args) (car args))
		    :cs-eval-gains '(1 1)
		    :rms-loop-max 2
		    :tmax-hand-rate 1.0
		    :tmax-leg-rate 1.0
		    :log-file (format nil "~A/simple-floor" *log-root*)
		    :ref-order '(:rarm :larm :rleg :lleg)
		    ;; :error-thre 0.7
		    )))
	 (cond
	  ((and (listp rsd)
		(find-if '(lambda (rsd) (subclassp (class rsd) robot-state-data2)) rsd))
	   (rsd-play :file (format nil "~A/simple-floor.rsd" *log-root*) :auto? t)
	   (quit-graph)
	   ))
	 (atom rsd))
       (print 'simple-floor-error)
       t)))
    ))

;; start test

(setq *dcsf-max-dist-scale* 1.4)
(defvar *log-root* (format nil "log/~A" (log-surfix)))
(defvar *ik-debug-view* nil) ;;:no-message)
(unix:system (format nil "mkdir ~A" *log-root*))

(if (or
     (test-proc :kirin-ladder)
     (test-proc :rock-wall)
     (test-proc :four-leg-seat)
     (test-proc :simple-floor))
    (exit -1))

(print 'all-ok)
(exit 0)

#|

(defvar *standup*
  (cdar (rsd-deserialize :file "log/Tue_Oct_27_23:34:48_2015/simple-floor.rsd")))
(let* ((id -1))
  (dolist (rsd (cons (progn (send *best-facefall* :draw :friction-cone? nil)
			    (optimize-brli :contact-states *facefall-contact-state*))
		     *standup*))
    (cond
     ((find (send rsd :buf :mseq-mode) '(:remove :reach))
      (send rsd :draw :rest (list *climb-obj*) :torque-draw? nil)
      (objects (list *robot*))
      (send *viewer* :viewsurface :write-to-image-file
	    (format nil "buf/~A.jpg" (incf id)))
      (read-line)
      )))
  (let* ((legs (send *robot* :legs :end-coords :copy-worldcoords)))
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (car legs) :lleg)
    (send *robot* :inverse-kinematics
	  legs
	  :move-target (send *robot* :legs :end-coords)
	  :link-list (mapcar
		      '(lambda (mt) (send *robot* :link-list (send mt :parent)))
		      (send *robot* :legs :end-coords)))
    (send *irtviewer* :objects (list *robot* *climb-obj*))
    (send *viewer* :draw-objects)
    (send *viewer* :viewsurface :write-to-image-file
	  (format nil "buf/~A.jpg" (incf id)))
    ))
