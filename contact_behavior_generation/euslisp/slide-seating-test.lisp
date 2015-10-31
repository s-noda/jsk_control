#!/usr/bin/env roseus

(defvar *robot-type* :hrp2jsknts-collada)
(require "package://contact_behavior_generation/euslisp/motion-sequencer.lisp")
(require "package://contact_behavior_generation/euslisp/dynamic-connector.lisp")

(require "package://contact_behavior_generation/euslisp/motion-planners/motion-planner.lisp") (defun demo-motion-sequence (&rest args) (apply 'demo-motion-sequence2 (append (list :error-thre 1.1) args)))

(defvar *goal-contact-state*)

(defun test-proc
  nil
  (cond
   ((let* ((init
	    (progn (send *robot* :reset-manip-pose)
		   (send *robot* :fix-leg-to-coords (make-coords))))
	   (rsd (demo-motion-sequence-with-timer
		 :motion-planner-names *motion-planner-names*
		 :ik-debug-view *ik-debug-view*
		 :loop-max
		 '(lambda (cnt)
		    (cond
		     ((> cnt 5) t)
		     (t
		      (and *goal-contact-state*
			   (<
			    (norm
			     (send (send *goal-contact-state* :contact-coords)
				   :difference-position
				   (send *goal-contact-state* :target-coords)))
			    10)
			   (<
			    (norm
			     (send (send *goal-contact-state* :contact-coords)
				   :difference-rotation
				   (send *goal-contact-state* :target-coords)))
			    (deg2rad 5))))))
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
      ;; (cond
      ;;  ((listp rsd)
      ;; 	(rsd-play :file (format nil "~A/four-leg-seat.rsd" *log-root*) :auto? t)
      ;; 	(quit-graph)))
      (or (atom rsd) (not (find :loop-exeeded rsd))))
    (print 'four-leg-seat-error) nil)
   (t t)))

;; start test

(defvar *log-root* (format nil "log.slide_seating_test/~A" (log-surfix)))
(defvar *ik-debug-view* :no-message)
(unix:system (format nil "mkdir -p ~A" *log-root*))

(demo-climb-setup :four-leg-seat)
(send *robot* :reset-manip-pose)
(send *robot* :fix-leg-to-coords (make-coords))
(send *viewer* :draw-objects)
(setq *contact-states*
      (let* ((lleg-len (apply '+ (mapcar '(lambda (l1 l2) (norm (v- (send l1 :worldpos) (send l2 :worldpos)))) (cdr (send *robot* :lleg :links)) (send *robot* :lleg :links))))) (remove-if #'(lambda (cs) (> (norm (send (send cs :target-coords) :difference-position (apply 'midcoords  (cons 0.5 (send *robot* :legs :end-coords :copy-worldcoords))))) lleg-len)) *contact-states*)))

(setq *reachable-contact-state-ids* nil)
(setq *motion-planner-names*
      (list static-walk-motion-planner slide-motion-planner))
(let* ((ret nil))
  (dotimes (i (length *contact-states*))
    (setq *goal-contact-state* (nth i *contact-states*))
    (if (test-proc) (push i ret)))
  (push (print ret) *reachable-contact-state-ids*))

(setq *motion-planner-names*
      (list static-walk-motion-planner))
(let* ((ret nil))
  (dotimes (i (length *contact-states*))
    (setq *goal-contact-state* (nth i *contact-states*))
    (if (test-proc) (push i ret)))
  (push (print ret) *reachable-contact-state-ids*))

(defun draw-id
  (&rest
   args
   &key
   (ids nil)
   (csl
    (if (not ids) *contact-states*
      (let* ((buf))
	(dolist (id ids) (push (nth id *contact-states*) buf))
	buf)))
   (color (float-vector 1 0 0))
   (width 4)
   (size 30)
   (min-size width)
   v)
  ;;
  (send *viewer* :viewsurface :line-width width)
  (send *viewer* :viewsurface :color color)
  ;;
  (dolist (cs csl)
    (setq v (send (send cs :target-coords) :worldpos))
    (send *viewer* :viewsurface :3d-lines
	  (list (v+ (float-vector size size 0) v)
		(v+ (float-vector (* -1 size) size 0) v)
		(v+ (float-vector (* -1 size) (* -1 size) 0) v)
		(v+ (float-vector size (* -1 size) 0) v)
		(v+ (float-vector size size 0) v)
		)))
  (if (> size min-size)
      (apply 'draw-id :size (- size width) args)
    (send *viewer* :viewsurface :flush)))

(send *viewer* :draw-objects)
(draw-id :color (float-vector 0 1 0) :size 40)
(draw-id :ids (nth 1 *reachable-contact-state-ids*) :color (float-vector 0 0 1) :size 28)
(draw-id :ids (nth 0 *reachable-contact-state-ids*) :color (float-vector 1 0 0) :size 15)
;; (draw-id :ids (apply 'intersection *reachable-contact-state-ids*) :color (float-vector 1 1 0))
