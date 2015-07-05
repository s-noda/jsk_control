
(defvar *robot-type* :jaxon_red)
(require "motion-sequencer.lisp")
(require "dynamic-bspline-trajectory.lisp")

(require "/home/s-noda/prog/euslib/demo/nozawa/hrpsys-test/test-st-state-estimate-data.l")

(setq *log* (instance datalogger-log-parser-controller :init "/home/s-noda/ダウンロード/datalogger/wall-rsj-400-3kaime_JAXON_20150701224314" :robot *robot*))

(convert-to-robot-state-data-for-st-debug)

(view-cop-trajectory :rs-list (subseq *rs-list* 7000 9000) :incf 50)

(send *irtviewer* :look-all)

(setq dynamic-trajectory nil)

#|

(setq tau-vector
      (calc-trajectory-torque
       :dynamic-trajectory
       (setq
	dynamic-trajectory
	(gen-dynamic-trajectory
	 :freq 500
	 :position-list (send-all (subseq *rs-list* 7000 9000) :get
				  :auto-balancer-reference-vector)
	 :coords-list (send-all (subseq *rs-list* 7000 9000) :get
				:auto-balancer-reference-root-coords)
	 :fix-length t
	 ))
       :f0-list-list
       (mapcar '(lambda (rsd)
		  (reverse (subseq (car (send rsd :wrench-list)) 0 2)))
	       (subseq *rs-list* 7000 9000))
       :m0-list-list
       (mapcar '(lambda (rsd)
		  (reverse (subseq (cadr (send rsd :wrench-list)) 0 2)))
	       (subseq *rs-list* 7000 9000))
       :draw? t))

(setq tau-vector2
      (mapcar
       #'(lambda (rsd)
	   (send *robot* :angle-vector
		 (copy-seq (send rsd :get :auto-balancer-reference-vector)))
	   (send *robot* :newcoords
		 (copy-object (send rsd :get :auto-balancer-reference-root-coords)))
	   (send *viewer* :draw-objects)
	   (send *robot* :calc-torque
		 :target-coords
		 (list (send *robot* :rleg :end-coords)
		       (send *robot* :lleg :end-coords))
		 :force-list
		 (subseq (car (send rsd :wrench-list)) 0 2)
		 :moment-list
		 (subseq (cadr (send rsd :wrench-list)) 0 2)))
       (subseq *rs-list* 7000 9000)))

(setq tau-vector3
      (mapcar
       #'(lambda (rsd traj)
	   (send *robot* :angle-vector
		 (copy-seq (send rsd :get :auto-balancer-reference-vector)))
	   (send *robot* :newcoords
		 (copy-object (send rsd :get :auto-balancer-reference-root-coords)))
	   ;; (send *viewer* :draw-objects)
	   (send *robot* :torque-vector2
		 :contact-states
		 (mapcar
		  #'(lambda (mt f m)
		      (list
		       (cons :link (send  mt :parent))
		       (cons :worldcoords (send mt :copy-worldcoords))
		       (cons :ext-force f)
		       (cons :ext-moment m)))
		  (list (send *robot* :rleg :end-coords)
			(send *robot* :lleg :end-coords))
		  (subseq (car (send rsd :wrench-list)) 0 2)
		  (subseq (cadr (send rsd :wrench-list)) 0 2))
		 :root-angular-velocity (send traj :w)
		 :root-spacial-velocity (scale 1e-3 (send traj :dx))
		 :root-angular-acceleration (send traj :dw)
		 :root-spacial-acceleration (scale 1e-3 (send traj :ddx))
		 :jvv (map float-vector #'deg2rad (send traj :velocity))
		 :jav (map float-vector #'deg2rad (send traj :acceleration))
		 ))
       (subseq *rs-list* 7000 9000)
       dynamic-trajectory
       ))

#|

(let (tm-list)
  (graph-view
   (list
    (map float-vector '(lambda (d) (map cons '(lambda (j tau) (send j :put :tau tau)) (send *robot* :joint-list) (car d)) (* -1 (send *robot* :rleg :knee-p :get :tau)) (aref (cadr d) 2)) tau-vector)
    (map float-vector '(lambda (d) (map cons '(lambda (j tau) (send j :put :tau tau)) (send *robot* :joint-list) (car d)) (* -1 (send *robot* :lleg :knee-p :get :tau)) (aref (caddr d) 2)) tau-vector))
   (setq tm-list (send-all (subseq *rs-list* 7000 9000) :time))
   :dump-graph t
   :keylist (list "rleg" "lleg")
   :xlabel "Time [s]"
   :ylabel "Torque [Nm]"
   :xrange (list (car tm-list) (car (last tm-list)))
   :title "torque in steer climbing1"
   :dump-graph "torque1.eps"))

(let (tm-list)
  (graph-view
   (list
    (map float-vector '(lambda (d) (map cons '(lambda (j tau) (send j :put :tau tau)) (send *robot* :joint-list) d) (send *robot* :rleg :knee-p :get :tau)) tau-vector2)
    (map float-vector '(lambda (d) (map cons '(lambda (j tau) (send j :put :tau tau)) (send *robot* :joint-list) d) (send *robot* :lleg :knee-p :get :tau)) tau-vector2))
   (setq tm-list (send-all (subseq *rs-list* 7000 9000) :time))
   :dump-graph t
   :keylist (list "rleg" "lleg")
   :xlabel "Time [s]"
   :ylabel "Torque [Nm]"
   :xrange (list (car tm-list) (car (last tm-list)))
   :title "torque in steer climbing2"
   :dump-graph "torque2.eps"
   ))

(let (tm-list)
  (graph-view
   (list
    (map float-vector '(lambda (d) (map cons '(lambda (j tau) (send j :put :tau tau)) (send *robot* :joint-list) d) (send *robot* :rleg :knee-p :get :tau)) tau-vector3)
    (map float-vector '(lambda (d) (map cons '(lambda (j tau) (send j :put :tau tau)) (send *robot* :joint-list) d) (send *robot* :lleg :knee-p :get :tau)) tau-vector3))
   (setq tm-list (send-all (subseq *rs-list* 7000 9000) :time))
   :dump-graph t
   :keylist (list "rleg" "lleg")
   :xlabel "Time [s]"
   :ylabel "Torque [Nm]"
   :xrange (list (car tm-list) (car (last tm-list)))
   :title "torque in steer climbing3"
   :dump-graph "torque3.eps"
   ))

(let (tm-list
      (key (list :crotch-y :crotch-p :crotch-r :knee-p :ankle-p :ankle-r))
      (val)
      (tau-vector tau-vector2))
  (graph-view
   (setq
    val
    (mapcar
     #'(lambda (k)
	 (map float-vector #'(lambda (d) (map cons '(lambda (j tau) (send j :put :tau tau)) (send *robot* :joint-list) d) (send *robot* :rleg k :get :tau)) tau-vector))
     key))
   (setq tm-list (send-all (subseq *rs-list* 7000 9000) :time))
   :dump-graph t
   :keylist (send-all key :pname)
   :xlabel "Time [s]"
   :ylabel "Torque [Nm]"
   :xrange (list (car tm-list) (car (last tm-list)))
   :title "right leg torque in steer climbing"
   :dump-graph "torque3.eps"
   )
  (map cons
       #'(lambda (key val)
	   (format t "~A ~A~%" key
		   (apply 'max (map cons 'abs val))))
       key val)
  )


;; model based force line

(setq key-rs-list
      (append
       (list (car (subseq *rs-list* 7000 9400)))
       (flatten
	(mapcar
	 #'(lambda (a b)
	     (if (> (norm (v- (send a :get :auto-balancer-contact-states)
			      (send b :get :auto-balancer-contact-states)))
		    1e-3)
		 a))
	 ;;(if (> (apply '+ (coerce (send b :get :auto-balancer-contact-states) cons))
	 ;; 1.5)
	 ;; a
	 ;; (list b a))
	 (cdr (subseq *rs-list* 7000 9400)) (subseq *rs-list* 7000 9400)))
       (list (nth 9400 *rs-list*))
       ))

(setq rsd-list
      (let* ((prev-mseq :suspend))
	(mapcar
	 #'(lambda (rs)
	     (send *robot* :angle-vector
		   (copy-seq (send rs :get :auto-balancer-reference-vector)))
	     (send *robot* :newcoords
		   (copy-object (send rs :get :auto-balancer-reference-root-coords)))
	     (send *viewer* :draw-objects)
	     (let* ((cs (send rs :get :auto-balancer-contact-states))
		    (rl (cond
			 ((< (aref cs 0) 0.5) :rleg)
			 ((< (aref cs 1) 0.5) :lleg)
			 (t nil)))
		    (ret (instance robot-state-data2 :init
				   :contact-states
				   (now-contact-state
				    :limb-keys
				    (remove rl '(:rleg :lleg))
				    :force0
				    (make-list (length (remove rl '(:rleg :lleg)))
					       :initial-element
					       (float-vector 1000 1000 1000 1000 1000 1000))
				    )))
		    )
	       (send ret :buf :remove-limb rl)
	       (send ret :buf :mseq-mode
		     (setq prev-mseq
			   (cond ((null rl) :suspend)
				 ((eq prev-mseq :remove) :reach)
				 (t :remove))))
	       (send ret :buf :time (send rs :time))
	       ret))
	 key-rs-list)))

(setq
 dynamic-trajectory
 (gen-dynamic-trajectory
  :freq 500
  :position-list (send-all (subseq *rs-list* 7000 9400) :get
			   :auto-balancer-reference-vector)
  :coords-list (send-all (subseq *rs-list* 7000 9400) :get
			 :auto-balancer-reference-root-coords)
  :fix-length t
  ))
(mapcar
 #'(lambda (elm rsd)
     (send elm :put :time (send rsd :time))
     elm)
 dynamic-trajectory
 (subseq *rs-list* 7000 9400))

(defun decrese-list
  (l size)
  (let* ((buf) (step (/ (* 1.0 (length l)) size)))
    ;; (push (car l) buf)
    (dotimes (i size)
      (push (nth (round (* i step)) l) buf))
    (push (car (last l)) buf)
    (reverse buf)))

(load "dynamic-bspline-trajectory.lisp")
(send-all (send *robot* :joint-list) :max-joint-torque 1e+3)
(setq *order-factor* 20)
(setq bspline
      (instance partition-spline-contact-wrench-trajectory :init
		:rsd-list rsd-list
		:trajectory-elem-list dynamic-trajectory ;;(decrese-list dynamic-trajectory 40)
		;; :id-max (* 7 (length rsd-list))
		;; :descrete-constraints-sep 21
		))
(send bspline :optimize)
(if (boundp 'a) (send a :quit))
(setq a (send bspline :gen-graph :name "all-force" :dim-list (send bspline :force-range) :plot-step 0.03))

(setq
 b
 (let* ((key '(:crotch-r :crotch-p :crotch-y :knee-p :ankle-p :ankle-r))
	(pos (mapcar
	      #'(lambda (k)
		  (position (send *robot* :rleg k) (send *robot* :joint-list)))
	      key))
	(data-org (send bspline :calc-descrete-dynamics-value :sep 10))
	data-list
	(graph
	 (create-graph "torque-graph"
		       :name-list (send-all key :pname)
		       :data-list
		       (mapcar
			#'(lambda (pos)
			    (mapcar
			     #'(lambda (data)
				 (float-vector (cdr (assoc :time data))
					       (aref (cdr (assoc :torque data)) pos)))
			     data-org))
			pos)))
	)
   (send graph :fit-draw)
   graph))

(setq *time-force-vector* (send-all (send a :data) :data))
(setq *time-torque-vector* (send-all (send b :data) :data))
(dump-loadable-structure "time-force-vector.l" *time-force-vector*)
(dump-loadable-structure "time-torque-vector.l" *time-torque-vector*)

(send a :set-data
      (list (nth 2 (send a :data))
	    (nth 8 (send a :data))))
(send (car (send a :data)) :name "right leg")
(send (cadr (send a :data)) :name "left leg")
(send a :set-range
      (float-vector (send bspline :x-min) -100)
      (float-vector (send bspline :x-max) 1400))
(send a :simple-draw-with-line)

(send b :set-range
      (float-vector (send bspline :x-min) -250)
      (float-vector (send bspline :x-max) 400))
(send b :simple-draw-with-line)

(require "util/gp-util.lisp")
(graph-panel2gp-graph a :xlabel "Time [sec]" :ylabel "Wrench" :ratio 0.4 :save? "contact_wrench.eps")
(graph-panel2gp-graph b :xlabel "Time [sec]" :ylabel "Torque [Nm]" :ratio 0.7 :save? "rleg_torque.eps")

(let* ((key '(:crotch-r :crotch-p :crotch-y :knee-p :ankle-p :ankle-r)))
  (mapcar
   #'(lambda (k dl)
       (format t "~A ~A~%"
	       k
	       (apply 'max (mapcar 'abs (mapcar #'(lambda (d) (aref d 1)) dl)))))
   key *time-torque-vector*))

;; (setq b (send a :data))
;; (dump-loadable-structure "f.dat" b)
;; (send bspline :gen-graph :name "force-graph" :dim-list (send bspline :force-range))
;; (send bspline :gen-graph :name "all-force" :dim-list (send bspline :force-range))