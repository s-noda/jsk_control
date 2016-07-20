#!/usr/bin/env roseus

(defvar *robot-type* :hrp2jsk-collada)
(require "package://contact_behavior_generation/euslisp/motion-sequencer.lisp")
(require "package://contact_behavior_generation/euslisp/dynamic-connector.lisp")
(require "package://contact_behavior_generation/euslisp/motion-planners/motion-planner.lisp")

;; @Override
(defun demo-motion-sequence (&rest args) (let* ((ret (apply 'demo-motion-sequence2 (append args (list :error-thre 1.1))))) (cons (car ret) (reverse (cdr ret)))))

;; @Override for debug
(defvar *rsd-play* (function rsd-play))
(defun rsd-play (&rest args) (apply *rsd-play* :rsd-list *rsd* :graph nil args))

(defvar *last-trajectory*)
(defun gen-linear-interpolate-trajectory
  (&key
   (robot *robot*)
   (root-link (car (send *robot* :links)))
   (now (nth 3 *rsd*))
   (nxt (nth 2 *rsd*))
   (now-t (or (send now :buf :time) 0.0))
   (nxt-t (or (send nxt :buf :time) 5.0))
   (fix-coords
    (send (find-if #'(lambda (cs) (not (eq (send cs :name)
					   (send now :buf :remove-limb))))
		   (send now :contact-states))
	  :contact-coords))
   (freq 100)
   (position-list nil)
   (coords-list nil)
   (time-list nil)
   ret)
  (send now :draw :friction-cone? nil :torque-draw? nil)
  (send-all (send robot :links) :worldcoords)
  (let* ((c (send fix-coords :copy-worldcoords))
	 (total (* freq (- nxt-t now-t)))
	 (rate 0) prate)
    (dotimes (i (+ 1 total))
      (setq rate (/ (* 1.0 i) total))
      (setq prate rate)
      ;; (setq prate (- 1 (/ (+ 1 (cos (/ (* 3.141 i) total))) 2.0)))
      (push (v+ (scale prate (send now :angle-vector))
		(scale (- 1 prate) (send nxt :angle-vector)))
	    position-list)
      (send robot :angle-vector (copy-seq (car position-list)))
      (send-all (send robot :links) :worldcoords)
      ;;
      (while (> (norm (send fix-coords :difference-position c)) 1)
	(send robot :transform
	      (send (send fix-coords :copy-worldcoords)
		    :transformation
		    (send c :copy-worldcoords))
	      :local))
      ;;
      (push (send root-link :copy-worldcoords) coords-list)
      (push (+ (* rate (- nxt-t now-t)) now-t) time-list)
      ;; (send *viewer* :draw-objects)
      ;; (unix::usleep (* 100 1000))
      )
    (setq ret
	  (gen-dynamic-trajectory
	   :freq freq :position-list position-list
	   :coords-list coords-list :fix-length t))
    (mapcar #'(lambda (traj tm) (send traj :put :time tm))
            ret time-list)
    (setq *last-trajectory* (reverse ret))))

;; (setq *order-factor* 5)
(defvar *last-bspline*)
(defun slide-trajectory-check-func
  (&optional
   (now (nth 3 *rsd*))
   (nxt (nth 2 *rsd*))
   &rest args)
  (cond
   ((not (and (send now :buf :slide)
	      (send nxt :buf :slide)))
    t)
   (t
    (send now :buf :time 0)
    (send nxt :buf :time 5.0)
    (let* ((tm (send nxt :buf :time))
	   (freq 10)
	   (id-max 20)
	   (sep 3)
	   ;; freq * tm / sep * 6 equations <= id-max * 6 * 3
	   (varify
	    (if (<= (print (* id-max 6 3)) (print (+ id-max 6 (/ (* 6 freq tm) sep))))
		(progn (warning-message 1 "invalid qp status~%")
		       (throw :invalid-qp-status-exception nil))))
	   (bspline
	    (instance* partition-spline-contact-wrench-trajectory :init
		       :rsd-list (list now nxt)
		       :id-max id-max ;; * 6 * 3 dim
		       :freq freq ;;x 5 sec
		       :descrete-constraints-sep sep
		       :trajectory-elem-list
		       (gen-linear-interpolate-trajectory
			:now now :nxt nxt :freq freq)
		       ;;
		       nil
		       ))
	   ;;
	   (slide-opt (slide-friction-condition-extra-args
		       :contact-states (send now :contact-states)
		       :to (send now :contact-states
				 (send now :buf :remove-limb))
		       :from (send nxt :contact-states
				   (send nxt :buf :remove-limb)))))
      (send bspline :set-val 'extra-equality-matrix
	    (send bspline
		  :calc-coeff-matrix-for-gain-vector
		  (cadr (member :extra-equality-matrix slide-opt))
		  :tm-list (car (send bspline :contact-time-list))
		  :dim-list (send bspline :force-range)))
      (send bspline :set-val 'extra-equality-vector
	    (send bspline :constant-vector
		  (cadr (member :extra-equality-vector slide-opt))
		  :tm-list (car (send bspline :contact-time-list))))
      (cond
       ((and (send* bspline :optimize nil)
	     (send bspline :get :optimize-result)
	     (send bspline :get :optimize-success))
	;; (send now :buf :force-trajectory bspline)
	(setq *last-bspline* bspline)
	)
       (t (warning-message 1 "optimization failed~%") nil))))))

;; test-proc
;; test-proc :u-rate 2.0 :init (nth 5 (reverse *rsd*))
(setq  *kca-cog-gain* 30.0)
(defun test-proc
  (&rest args &key (anime? nil) (u-rate 1.0) (u (* u-rate 0.2))
         (init
          (progn (send *robot* :reset-manip-pose)
                 (send *robot* :reset-pose)
                 (send *robot* :arms :elbow-p :joint-angle -90)
                 (send *robot* :arms :shoulder-p :joint-angle -30)
                 (send *robot* :fix-leg-to-coords (make-coords)) ;; :pos (float-vector -10 0 0)))
                 (send *viewer* :draw-objects)
                 (optimize-brli :contact-states
                                (now-contact-state :limb-keys '(:rleg :lleg))
                                :rest-contact-states
                                (list (now-hip-contact-state)))
                 ))
         &allow-other-keys)
  (send init :draw :friction-cone? nil :torque-draw? nil)
  (demo-climb-setup :four-leg-seat)
  (send init :draw :friction-cone? nil :torque-draw? nil)
  ;;
  (setq *goal-contact-state* (nth 87 *contact-states*))
  (setf (aref (send (send *goal-contact-state* :target-coords) :worldpos) 0)
        (+ (aref (send (send *goal-contact-state* :target-coords) :worldpos) 0) 60))
  ;;
  (setq *contact-states*
        (remove-if '(lambda (d) (< (norm (send (send d :target-coords) :difference-position (send (send d :contact-coords) :worldcoords))) 20)) *contact-states*))
  (dolist (v (send-all (send-all *contact-states* :target-coords) :worldpos))
    (setf (aref v 2) (- (aref v 2) 10)))
  (dolist (c (remove-if '(lambda (cs) (not (eq (send cs :name) :hip)))
                        (append (send init :contact-states) *contact-states*)))
    (setf (aref (send c :slip-matrix) 0 2) u)
    (setf (aref (send c :slip-matrix) 1 2) u)
    (setf (aref (send c :slip-matrix) 5 2) u))
  ;;
  (unix:system (format nil "mkdir -p ~A" *log-root*))
  (cond
   ((let* ((rsd (demo-motion-sequence-with-timer
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
                 init
                 :tmax-hand-rate 1.0
                 :tmax-leg-rate 1.0
		 ;;
		 :trajectory-check-func
		 'slide-trajectory-check-func
                 :log-file (format nil "~A/four-leg-seat-u~A" *log-root* u)
                 )))
      (setq *rsd* rsd)
      (cond
       ((and anime? (listp rsd))
        (rsd-play :file (format nil "~A/four-leg-seat-u~A.rsd" *log-root* u) :auto? t)
        (quit-graph)))
      (atom rsd))
    (print 'four-leg-seat-error)
    t)))

(defun rsd-union
  (rsd-list)
  (append
   (append
    (list (car rsd-list) (cadr rsd-list))
    (flatten
     (mapcar
      '(lambda (rsd1 rsd2)
         (if (< (print (norm (v- (send rsd1 :angle-vector) (send rsd2 :angle-vector)))) 10)
             nil rsd1))
      (cddr rsd-list) (cdr rsd-list))))))


;;init

(progn (demo-climb-setup :four-leg-seat)
       ;;
       (send-all (send *robot* :links) :analysis-level :coords)
       (dolist (v (send-all (send-all *contact-states* :target-coords) :worldpos))
         (setf (aref v 2) (- (aref v 2) 10)))
       ;; (setq *goal-contact-state* (nth 86 *contact-states*))
       (setq *goal-contact-state* (nth 87 *contact-states*))
       (send *robot* :reset-manip-pose)
       (send *robot* :reset-pose)
       (send *robot* :arms :elbow-p :joint-angle -90)
       (send *robot* :fix-leg-to-coords (make-coords)))

(defvar *mu0* 1.1)
(defvar *rsd*)
(defvar *log-root* (format nil "log/~A" (log-surfix)))
(defvar *ik-debug-view* nil) ;; :no-message)
(unix:system (format nil "mkdir -p ~A" *log-root*))
(cond
 ((let* ((log (read-line (piped-fork "ls -t log/*/four-leg-seat.rsd | head -n 1") nil)))
    (and
     (and log (probe-file (format nil "~A" log)))
     (progn
       (unix::system (format nil "cp ~A ~A" log *log-root*))
       (setq *rsd* (car (rsd-deserialize :file (format nil "~A" log)))))))
  'nop)
 (t
  (if (test-proc :u (/ *mu0* 0.8)) ;;(* (/ 6.4 0.2) 0.4))
      (warning-message 1 "not solutions found~%"))))
(setq *rsd-org* *rsd*)

(mapcar
 '(lambda (name min max)
    (send *last-bspline* :gen-graph :name (format nil "~A-force" name)
	  :dim-list (subseq (send *last-bspline* :force-range) min max)))
 (send *last-bspline* :contact-name-list) '(0 6 12) '(6 12 18))

;; (send *viewer* :viewing :look (send *viewer* :viewing :viewpoint) (send *viewer* :viewing :viewtarget))
(send *viewer* :viewing :look #f(-63.1427 3991.32 715.147) #f(-134.75 -111.039 715.147))
(dolist (rsd (reverse (remove-if-not '(lambda (rsd) (send rsd :buf :slide)) (cdr *rsd*))))
  (send rsd :draw :rest (list *climb-obj*) :torque-draw? nil)
  (print (send *robot* :get :hip-end-coords))
  (send *viewer* :viewsurface :write-to-image-file
	(remove #\: (format nil "~A_mu~A.jpg" (send rsd :buf :mseq-mode)
			    (format nil "~A" (round (* 100 *mu0*)))
			    )))
  )


(let* ((root (format nil "flog.mu~A" (round (* 100 *mu0*))))
       (mkdir (unix::system (format nil "mkdir -p ~A" root)))
       (rfx (open (format nil "~A/rfx.log" root) :direction :output))
       (rfz (open (format nil "~A/rfz.log" root) :direction :output))
       (lfx (open (format nil "~A/lfx.log" root) :direction :output))
       (lfz (open (format nil "~A/lfz.log" root) :direction :output))
       (hfx (open (format nil "~A/hfx.log" root) :direction :output))
       (hfz (open (format nil "~A/hfz.log" root) :direction :output))
       tm f)
  (dotimes (i 100)
    (setq tm (* i 0.05))
    (setq f (send *last-bspline* :calc tm))
    (format rfx "~A ~A~%" tm (aref f 0))
    (format rfz "~A ~A~%" tm (aref f 2))
    (format lfx "~A ~A~%" tm (aref f 6))
    (format lfz "~A ~A~%" tm (aref f 8))
    (format hfx "~A ~A~%" tm (aref f 12))
    (format hfz "~A ~A~%" tm (aref f 14)))
  (close rfx) (close rfz) (close lfx) (close lfz) (close hfx) (close hfz))

#|

python -i ../../BagGraph.py ../bag.bag
-F rfx.log -l Rx -g _ -y _ -p r--x -r 0.3 -y Friction
-F rfz.log -l Rz -g _ -y _ -p r-x -r 0.3 -y Friction
-F lfx.log -l Lx -g _ -y _ -p g--o -r 0.3 -y Friction
-F lfz.log -l Lz -g _ -y _ -p g-o -r 0.3 -y Friction
-F hfx.log -l Hx -g _ -y _ -p b--^ -r 0.3 -y Force
-F hfz.log -l Hz -g _ -y _ -p b-^ -r 0.3 -y Force
