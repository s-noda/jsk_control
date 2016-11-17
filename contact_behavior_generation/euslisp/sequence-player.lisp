(defvar *real-flag* nil)
(defvar *sim-flag* nil)

(require "connect.lisp")
(require "my-util.lisp")
(require "util/spline.lisp")
(require "motion-sequencer.lisp")
(require "robot-state-data2.lisp")
(require "contact-state.lisp")

(defun calc-distance
  (pos)
  (sort
   (mapcar
    #'(lambda (env)
        (cons
         (cons
          :distance
          (let ((d (* -1
                      (v. (cdr (assoc :n env))
                          (v- pos (cdr (assoc :a0 env)))))))
            (if (< d 0)
                (format
                 *log-stream*
                 "[calc-distance] collision ~A detected bet ~A & ~A~%"
                 d
                 (cdr (assoc :name env))
                 pos))
            d)
          )
         env))
    *env-collision-check-list*)
   #'(lambda (a b) (< (abs (cdr (assoc :distance a)))
                      (abs (cdr (assoc :distance b)))))))

(defun reaching-trajectory
  (remove
   reach
   &key
   (all-limbs
    (send-all (send reach :contact-states) :name))
   (remove-limb (send reach :buf :remove-limb))
   (fix-limb (remove remove-limb all-limbs))
   ;;
   (robot *robot*)
   (root-link-virtual-joint-weight #F(0.1 0.1 0.01 0.01 0.01 0.01))
   dist
   (time-list '(0.0 0.7 0.8 1.0))
   (how-many 15)
   (col-dist 1) ;; this is adjust for climbing ladder
   (away-dist 80)
   ;;
   (debug-view nil)
   (pv (if (and (boundp '*irtviewer*) *irtviewer*) *irtviewer*))
   )
  (let* ((c1
	  (or (send remove :contact-states remove-limb)
	      (find-if #'(lambda (cs) (eq remove-limb (send cs :name)))
		       (now-contact-state))))
	 (c2 (send reach :contact-states remove-limb))
	 (end
	  (progn
	    (send reach :draw :robot robot :pv nil :torque-draw? nil)
	    (copy-object
	     (send (send c2 :contact-coords) :worldpos))))
	 (start
	  (progn
	    (send remove :draw :robot robot :pv nil :torque-draw? nil)
	    (copy-object
	     (send (send c2 :contact-coords) :worldpos))))
	 (mid (scale 0.5 (v+ start end)))
	 (dist (or dist (calc-distance mid)))
	 (n (transform
	     (send (send c2 :target-coords) :worldrot)
	     (send c2 :contact-n)))
	 (orbit (solve-spline
					; :n 20
		 :time-list time-list
		 :p (list (list (cons :vector start))
			  (list
			   (cons
			    :matrix
			    (make-matrix 1 (length (cdr (assoc :n (car dist))))
					 (list (cdr (assoc :n (car dist))))))
			   (cons
			    :vector
			    (v+ mid
				(if (plusp (- (cdr (assoc :distance (car dist))) 200))
				    #f(0 0 0)
				    (scale (* col-dist
					      (- (cdr (assoc :distance (car dist))) 200))
					   (cdr (assoc :n (car dist))))))))
			  (list (cons :vector
				      (v+ end (scale away-dist n)))
				(cons :matrix
				      (make-matrix 1 (length n) (list n))))
			  (list (cons :vector (v+ end (scale 30 n)))))
		 :dp (list (list (cons :vector #f(0 0 0))) nil nil
			   (list (cons :vector #f(0 0 0))))
		 :ddp (list (list (cons :vector #f(0 0 0))) nil nil
			    (list (cons :vector #F(0 0 0))))))
	 (step 10)
	 (ct (copy-seq (send *robot* :centroid)))
	 (av-list0
	  (let ((index -1) buf)
	    (mapcar
	     #'(lambda (hoge)
		 (incf index)
		 (setq buf (/ (- 1 (* (cos (* pi (/ index (- how-many 1.0)))))) 2.0))
		 (v+ (scale buf (send reach :angle-vector))
		     (scale (- 1 buf) (send remove :angle-vector))))
	     (make-list how-many))))
	 ret)
    ;; orbit
    (cond
     (debug-view
      (dotimes (i 10)
	(send (calc-spline orbit (/ i 9.0))
	      :draw-on :color #f(1 1 0) :flush nil))
      (if pv (send pv :viewer :viewsurface :flush))
      (read-line)))
    (dotimes (i how-many)
      (let* ((fix-leg
	      (if (eq remove-limb :rleg) :lleg :rleg))
	     (fix-coords (send robot fix-leg :end-coords :copy-worldcoords)))
	(send robot :angle-vector (nth i av-list0))
	(send robot :fix-leg-to-coords fix-coords fix-leg))
      (simple-fullbody
       :robot robot
       :target
       (append
	(mapcar
	 #'(lambda
	     (k)
	     (list
	      (cons :target (send (send reach :contact-states k) :contact-coords))
	      (cons :coords
		    (if (eq k remove-limb)
			(make-coords
			 :pos (calc-spline orbit (/ i (* 1.0 (- how-many 1))))
			 :rot (copy-object
			       (send (send c1 :target-coords) :worldrot)))
		      (copy-object
		       (send (send remove :contact-states k) :target-coords))))))
	 all-limbs)
	(collision-resolve-move))
       :debug-view debug-view
       :flush debug-view
       :balance-leg nil
       :target-centroid-pos ct
       :centroid-thre 100
       :stop 30
       :revert-if-fail nil
       :collision-avoidance-link-pair nil
					;       :max #f(100 100 500 100 100 100)
       :root-link-virtual-joint-weight root-link-virtual-joint-weight)
      (send pv :draw-objects :flush t)
      ;; (send
      ;;  (calc-zmp
      ;; 	:robot robot
      ;; 	:hand-keys '(:rarm :larm)
      ;; 	:hand-force
      ;; 	(list (subseq (or (send remove :contact-forces :rarm) #F(0 0 0 0 0 0)) 0 3)
      ;; 	      (subseq (or (send remove :contact-forces :larm) #F(0 0 0 0 0 0)) 0 3))
      ;; 	:hand-moment
      ;; 	(list (subseq (or (send remove :contact-forces :rarm) #F(0 0 0 0 0 0)) 3 6)
      ;; 	      (subseq (or (send remove :contact-forces :larm) #F(0 0 0 0 0 0)) 3 6)))
      ;;  :draw-on
      ;;  :flush nil
      ;;  :color #F(0 1 0)
      ;;  :width 10)
      (cond ((and debug-view pv)
	     ;(send pv :draw-objects :flush nil)
	     (send pv :viewer :viewsurface :flush)
	     (unix:usleep (round (* 1e+3 100)))))
      (push
       (list (cons :angle-vector (copy-object (send robot :angle-vector)))
	     (cons :hand-angle-vector
		   (copy-object (send robot :arms :hand :angle-vector))))
       ret)
      )
    (send reach :buf :trajectory ret)
    (list remove reach)))

(defun connect-rsd
  (&key
   (file "log/tmp.rsd")
   (rsd-list (flatten (rsd-deserialize :file file)))
   (debug-view nil))
  (mapcar
   #'(lambda (rsd1 rsd2)
       (if (and (eq (send rsd1 :buf :mseq-mode) :reach)
		(find (send rsd1 :buf :remove-limb) '(:rleg :lleg))
		(eq (send rsd2 :buf :mseq-mode) :remove))
	   (reaching-trajectory
	    rsd2 rsd1
	    :dist
	    (cond
	     ((eq (send rsd1 :buf :remove-limb) :rarm)
	      (list (list (cons :n #F(0 -1 0))
			  (cons :distance 300))))
	     ((eq (send rsd1 :buf :remove-limb) :larm)
	      (list (list (cons :n #F(0 -1 0))
			  (cons :distance 300)))))
	    :how-many 30
	    :away-dist 50
	    :col-dist 0.8
	    :time-list '(0 0.6 0.8 1.0)
	    :debug-view debug-view))
       rsd1)
   rsd-list (append (cdr rsd-list) (list *sample-rsd*))))

(defun sequence-play
  (&key
   (robot *robot*)
   (file "log/tmp.rsd")
   (rsd-list (connect-rsd :file file))
   (connect-time-step 2300)
   ;;
   (reverse? nil)
   (ref-force? nil)
   (inner-log? nil)
   (force-rate 0.8)
   (real-flag (and (boundp '*ci*) *ci*))
   (suspend-start 2)
   (suspend-buf)
   )
  (setq rsd-list (reverse rsd-list))
  (cond
   (suspend-start
    (dotimes (i suspend-start)
      (print (length rsd-list))
      (setq
       rsd-list
       (cdr
	(member
	 (setq
	  suspend-buf
	  (find-if #'(lambda (rsd)
		       (eq (send rsd :buf :mseq-mode)
			   :suspend))
		   rsd-list))
	 rsd-list))))
    (setq rsd-list (cons suspend-buf rsd-list))
    (send suspend-buf :draw
	  :robot robot :friction-cone? nil)
    (if real-flag (model2real :sleep-time 5000))
    (format t "hook~%") (read-line)
    (send robot :arms :hand :index-avoid-hook-pose)
    (if real-flag (model2real :sleep-time 1000))
    (format t "grasp~%") (read-line)
    (send robot :arms :hand :index-avoid-grasp-pose)
    (send robot :arms :hand :index-avoid-grasp-pose)
    (if real-flag (model2real :sleep-time 1000))
    (format t "ready?~%") (read-line)
    ))
  (if (and real-flag inner-log?) (send *ci* :start-log))
  (dolist (rsd rsd-list)
    (case (send rsd :buf :mseq-mode)
	  (:remove
	   (send rsd :draw :robot robot :friction-cone? nil)
	   (if (find (send rsd :buf :remove-limb) '(:rarm :larm))
	       (send robot (send rsd :buf :remove-limb) :hand :index-avoid-hook-pose))
	   (cond
	    (real-flag
	     (cond
	      (ref-force?
	       (mapcar
		#'(lambda (k)
		    (send *ci* :set-ref-force
			  (scale force-rate
				 (or (send rsd :contact-forces k) #F(0 0 0)))
			  800 k))
		'(:rarm :larm))))
	     (model2real :sleep-time 3000)
	     (send *ci* :wait-interpolation "wor")
	     )
	    (t (unix:sleep 1)))
	   (read-line)
	   )
	  (:suspend
	   (send rsd :draw :robot robot :friction-cone? nil)
	   (if (find (send rsd :buf :remove-limb) '(:rarm :larm))
	       (send robot (send rsd :buf :remove-limb) :hand :index-avoid-grasp-pose))
	   (cond
	    (real-flag
	     (cond
	      (ref-force?
	       (mapcar
		#'(lambda (k)
		    (send *ci* :set-ref-force
			  (scale force-rate
				 (or (send rsd :contact-forces k) #F(0 0 0)))
			  800 k))
		'(:rarm :larm))))
	     (model2real :sleep-time 3000)
	     (send *ci* :wait-interpolation "wor")
	     )
	    (t (unix:sleep 1)))
	   (read-line)
	   )
	  (:reach
	   (cond
	    ((null (send rsd :buf :trajectory))
	     (send rsd :draw :robot robot :friction-cone? nil)
	     (cond
	      (real-flag
	       (cond
		(ref-force?
		 (mapcar
		  #'(lambda (k)
		      (send *ci* :set-ref-force
			    (scale force-rate
				   (or (send rsd :contact-forces k) #F(0 0 0)))
			    800 k))
		  '(:rarm :larm))))
	       (model2real :sleep-time connect-time-step)
	       (send *ci* :wait-interpolation "wor")
	       )
	      (t (unix:usleep (* 1000 connect-time-step)))
	      )
	     (read-line)
	     )
	    (t
	     (let*
		 ((lkey (car (remove (send rsd :buf :remove-limb) '(:rleg :lleg))))
		  (lleg (send robot lkey :end-coords :copy-worldcoords))
		  (sleep-time
		   (round
		    (/ connect-time-step (length (send rsd :buf :trajectory))))))
	       (format t "connect in ~Ax~A sec~%"
		       sleep-time (length (send rsd :buf :trajectory)))
	       (cond
		((and real-flag reverse?)
		 (send robot :angle-vector
		       (cdr (assoc :angle-vector (car (send rsd :buf :trajectory)))))
		 (model2real :sleep-time 500)
		 (send *ci* :wait-interpolation "wor")))
	       (mapcar
		#'(lambda (av)
		    (send robot :angle-vector
			  (cdr (assoc :angle-vector av)))
		    (send *robot* :fix-leg-to-coords lleg lkey)
		    (send *irtviewer* :draw-objects)
		    (cond
		     (real-flag (model2real :sleep-time sleep-time))
		     (t (unix:usleep (* 1000 sleep-time)))))
		(if reverse?
		    (send rsd :buf :trajectory)
		  (reverse (send rsd :buf :trajectory)))
		)
	       (cond
		((and real-flag (not reverse?))
		 (send rsd :draw :robot robot :friction-cone? nil)
		 (model2real :sleep-time 500)
		 (send *ci* :wait-interpolation "wor")))
	       (cond
		((and real-flag ref-force?)
		 (mapcar
		  #'(lambda (k)
		      (send *ci* :set-ref-force
			    (scale force-rate
				   (or (send rsd :contact-forces k) #F(0 0 0)))
			    500 k))
		  '(:rarm :larm))))
	       (read-line)
	       ))))))
  (if (and real-flag inner-log?) (send *ci* :stop-and-save-log))
  )

(defun auto-balancing
  (&key
   (robot *robot*)
   (gyro (send robot :torso :get :gyro-sensor))
   (balance-leg :both)
   (grasp-hand :both)
   (time-step 500)
   (max-speed (* 10 (* 1e-3 time-step)))
   (max-force 30)
   (m (* 1e-3 (send robot :weight)))
   (arm-key (case grasp-hand
		(:both '(:rarm :larm))
		(:rarm '(:rarm))
		(:larm '(:larm))))
   (limb-key (append
	      arm-key
	      (case balance-leg
		(:both '(:rleg :lleg))
		(:rleg '(:rleg))
		(:lleg '(:lleg)))))
   fnew
   df dm)
  (if (or (not (boundp '*ci*))
	  (null *ci*))
      (return-from auto-balancing 'non-ci))
;  (print 'start-hand-auto-balancer?)
;  (if (not (eq #\y (car (concatenate cons (read-line)))))
;      (return-from auto-balancing 'skip))
  (do-until-key
   (let ((fnow
	  (apply
	   #'concatenate
	   (flatten
	    (cons float-vector
		  (mapcar
		   #'(lambda (k)
		       (list
			(send *ci* :state :filtered-absolute-force-vector k)
			(send *ci* :state :filtered-absolute-moment-vector k)))
		   limb-key)))))
	 (c (scale 1e-3 (send robot :centroid))))
     (send robot :angle-vector
	   (send *ci* :state :potentio-vector))
     (send
      robot
      :newcoords
      (make-coords
       :pos (copy-seq (send robot :worldpos))
       :rot
       (m*
	(m* (inverse-matrix (send gyro :worldrot))
	    (send robot :worldrot))
	(send
	 (make-coords
	  :rpy
	  (reverse
	   (map float-vector
		#'*
		#F(1 1 0)
		(send *ci* :state :rpy-vector))))
	 :worldrot))))
     (setq G
	   (send robot :calc-grasp-matrix
		 (mapcar #'(lambda (k)
			     (send (send robot :force-sensor k) :worldpos))
			 limb-key)))
     (setq df
	   (scale
	    -1
	    (transform
	     (pseudo-inverse G)
	     (v+
	      (concatenate float-vector
			   (scale m (scale -1e-3 *g-vec*)) ;;#F(0 0 -9.8))
			   (v* c (scale m (scale -1e-3 *g-vec*))))
	      (transform G fnow))
	     )))
     (setq df
	   (let ((index -3) buf)
	     (mapcar #'(lambda (k)
			 (setq index (+ index 3))
			 (setq buf (subseq df index (+ index 3)))
			 (cons k
			       (scale
				(min max-speed (norm buf))
				(normalize-vector buf))))
		     limb-key)))
     (mapcar
      #'(lambda (k)
	  (let ((buf))
	    (setq buf
		  (v+
		   (send *ci* :state :filtered-absolute-force-vector k)
		   (cdr (assoc k df))))
	    (setq
	     buf
	     (scale
	      (min max-force (norm buf))
	      (normalize-vector buf)))
	    (format t "[~A] ~A~%" buf  k)
	    (send *ci* :set-ref-force buf time-step k)
	    ))
      arm-key)
     (unix:usleep (* 1000 time-step))
     (x::window-main-one)
     (send *irtviewer* :draw-objects)
     ))
  )

(defun reset-hand
  nil
  (send *ci* :set-ref-force #F(0 0 0))
  (send *robot* :arms :hand :index-avoid-hook-pose)
  (model2real :sleep-time 3000))

(defun demo-setup
  (&optional (robot-con *ci*))
  (send robot-con :set-interpolation-method :linear "wor")
  (send robot-con :set-impedance-param :axis t)
  (send robot-con :set-impedance-param :moment-gain 0.07)
  (send robot-con :set-impedance-param :force-gain 0.8)
  (send robot-con :set-impedance-param :mdk #F(0.005 0.7 0.3))
  (send robot-con :set-abc-param :check-shuffling nil)
  (send robot-con :stop-st)
  (send robot-con :stop-auto-balancer)
  (send robot-con :start-impedance))
