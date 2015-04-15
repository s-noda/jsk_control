
;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *real-flag* nil)
(defvar *sim-flag* nil)

(require "my-util.lisp")
(require "util/spline.lisp")
(require "motion-sequencer.lisp")
(require "robot-state-data2.lisp")
(require "contact-state.lisp")
;; (require "dynamic-bspline-trajectory.lisp")
(require "dynamics-motion/dynamic-optimize.lisp")
(require "util/spline-interpole-test.lisp")

;; @Override
(defun ik-wrapper
  (&rest args &key target acm &allow-other-keys)
  ;; (mapcar
  ;;  #'(lambda (l)
  ;;      (send (cdr (assoc :target l))
  ;; 	     :draw-on :flush t
  ;; 	     :color #F(1 0 0)))
  ;;  target)
  ;; (read-line)
  (and
   (apply
    #'simple-fullbody
    (append
     (list :target
	   (append target (collision-resolve-move)))
     args))
   (setq acm (collision-resolve-move))
   (setq target (append (butlast target 2)
			(list
			 (cons (cons :translation-axis nil)
			       (nth (- (length target) 2) target)))
			(last target)))
   (apply
    #'simple-fullbody
    (append
     (list :target (append target acm))
     (list :root-link-virtual-joint-weight #F(1 1 1 1 1 1))
     args))
   (setq acm (collision-resolve-move))
   (setq target (append (butlast target 2)
			(list
			 (cons (cons :rotation-axis :z)
			       (nth (- (length target) 2) target)))
			(last target)))
   (apply
    #'simple-fullbody
    (append
     (list :target (append target acm))
     (list :root-link-virtual-joint-weight #F(1 1 1 1 1 1))
     args))
   (setq acm (collision-resolve-move))
   (apply
    #'simple-fullbody
    (append
     (list :target
	   (append (butlast target 2) (last target) acm))
     (list :root-link-virtual-joint-weight #F(1 1 1 1 1 1))
     args))
   (setq acm (collision-resolve-move))
   (apply
    #'simple-fullbody
    (append
     (list :target
	   (append (butlast target 2) acm))
     (list :root-link-virtual-joint-weight #F(1 1 1 1 1 1))
     args))
   )
  (setq acm (collision-resolve-move))
  )

(defun ik-wrapper
  (&rest args &key target acm &allow-other-keys)
  (apply
   #'simple-fullbody
   (append
    (list :target
	  (append target (collision-resolve-move)))
    args))
  )

(defun calc-distance
  (pos &key (log? t))
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
                 (cond
		  ((not log?) nil)
		  ((boundp '*log-stream*) *log-stream*)
		  (t t))
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

(defun float-limb-trajectory
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
   (split-cnt 10)
   (col-dist 0.8) ;; this is adjust for climbing ladder
   (land-height 30)
   (away-dist 20)
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
	 (dist (or dist (calc-distance mid :log? nil)))
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
				      (v+ end (scale (+ land-height away-dist) n)))
				(cons :matrix
				      (make-matrix 1 (length n) (list n))))
			  (list (cons :vector (v+ end (scale land-height n)))))
		 :dp (list (list (cons :vector #f(0 0 0))) nil nil
			   (list (cons :vector #f(0 0 0))))
		 :ddp (list (list (cons :vector #f(0 0 0))) nil nil
			    (list (cons :vector #F(0 0 0))))))
	 (ct (copy-seq (send *robot* :centroid)))
	 ret)
    (dotimes (i (+ 1 split-cnt))
      (push
       (make-coords
	:pos (calc-spline orbit (/ i (* 1.0 split-cnt)))
	:rot (copy-object
	      (send (send c1 :target-coords) :worldrot)))
       ret))
    (reverse ret)
    ;ret
    ))
(require "dynamic-bspline-trajectory.lisp")

(defun connect-rsd
  (&key
   (file "log/tmp.rsd")
   (rsd-list (flatten (rsd-deserialize :file file)))
   (algorithm SLSQP)
   (debug-view nil)
   (max-time 5)
   (split-cnt 5)
   (slip-cnt split-cnt)
   ;; (freq 15)
   (start-base-vel #F(0 0 0 0 0 0))
   (start-base-acc #F(0 0 0 0 0 0))
   (end-base-vel #F(0 0 0 0 0 0))
   (end-base-acc #F(0 0 0 0 0 0))
   ;;
   (collision-avoid-trajectory-limb '(:rarm :larm))
   (contact-wrench-optimize-limb '(:rleg :lleg))
   (linear-interpole-limb nil)
   (trajectory-overwrite? t)
   )
  (mapcar
   #'(lambda (rsd2 rsd1)
       (cond
	((and (send rsd1 :buf :trajectory)
	      (not trajectory-overwrite?)) 'nop)
	((and (eq (send rsd1 :buf :mseq-mode) :reach)
	      (eq (send rsd2 :buf :mseq-mode) :remove)
	      (not (send rsd1 :buf :slide))
	      (not (send rsd2 :buf :slide)))
	 (cond
	  ((find (send rsd1 :buf :remove-limb) collision-avoid-trajectory-limb)
	   (send rsd2 :draw :friction-cone? nil) ;;:rest (list *climb-obj*))
	   (send rsd1 :buf :trajectory
		 (list
		  (cons :position
			(send-all
			 (demo-reach-ladder-spline-interpole
			  :init1 nil
			  :cnt slip-cnt
			  :remove-limb (send rsd1 :buf :remove-limb)
			  :start-av (copy-seq (send rsd2 :angle-vector))
			  :start-coords (copy-object (send rsd2 :root-coords))
			  :init2 nil
			  :end-av (copy-seq (send rsd1 :angle-vector))
			  :end-coords (copy-object (send rsd1 :root-coords))
			  :debug-view nil
			  :ik-params
			  (list
			   :target-centroid-pos
			   (let* ((fix-coords
				   (send-all
				    (remove-if
				     #'(lambda (cs)
					 (not (find (send cs :name)
						    (list :rleg :lleg))))
				     (send rsd1 :contact-states))
				    :target-coords)))
			     (scale (/ 1.0 (length fix-coords))
				    (reduce #'v+ (append (list #F(0 0 0) #F(0 0 0))
							 (send-all fix-coords :worldpos))))))
			  )
			 :position)))))
	  ((find (send rsd1 :buf :remove-limb) contact-wrench-optimize-limb)
	   (let* ((bspline (dynamic-contact-transition-demo
			    :loop-max 1 :graph? nil :bspline nil
			    :trajectory-extra-args
			    (list :freq slip-cnt
				  :rsd-list (list rsd2 rsd1))
			    :init (setq *cog-buf* nil *avc-buf* nil)))
		  (traj (if bspline (send bspline :trajectory-elem-list))))
	     ;; (send rsd1 :buf :bspline bspline)
	     (send rsd1 :buf :trajectory nil)
	     (cond
	      ((and bspline traj
		    (send bspline :get :optimize-result)
		    (send bspline :get :optimize-success))
	       (send bspline :calc-descrete-dynamics-value)
	       (send rsd1 :buf :trajectory
		     (list (cons :position (reverse (send-all traj :position)))
			   (cons :time (send-all traj :get :time))
			   (cons :force (mapcar #'(lambda (d) (cdr (assoc :force d))) (send bspline :descrete-dynamics-value)))
			   (cons :torque (mapcar #'(lambda (d) (cdr (assoc :torque d))) (send bspline :descrete-dynamics-value)))))
	       )))
	   rsd1)))
	(linear-interpole-limb
	 (setq *cog-buf* nil) (setq *avc-buf* nil)
	 (let* ((traj (apply
		       #'fix-angle-vector-root-coords-vector
		       (append
			(if (listp linear-interpole-limb)
			    linear-interpole-limb)
			(list
			 ;; :split-cnt slip-cnt
			 :print-x-step (/ 1.0 slip-cnt)
			 :rsd-list (list rsd2 rsd1)
			 :move-coords-interpole-type :linear)))
		      ))
	   (send rsd1 :buf :trajectory nil)
	   (cond
	    (traj
	     (send rsd1 :buf :trajectory
		   (list (cons :position
			       (reverse (send-all traj :position)))
			 (cons :time (send-all traj :get :time))
			 ))))))
	)
       rsd1)
   (cdr rsd-list)
   rsd-list
   ;; (append (cdr rsd-list) (list *sample-rsd*))
   ))

(defun torque-x-speed-test
  (&key
   (file "log/tmp.rsd")
   (rsd-list (flatten (rsd-deserialize :file file)))
   (debug-view nil)
   ;;
   (split-cnt 15)
   (step 1)
   (start-freq 7)
   (end-freq 30)
   ;; buf
   instant average
   torque-vector name-list data-list average-list
   id freq opt rsd1 rsd2)
  (require "util/graph-sample.lisp")
  (dolist (rsds (mapcar #'cons rsd-list (append (cdr rsd-list) (list *sample-rsd*))))
    (setq rsd1 (car rsds))
    (setq rsd2 (cdr rsds))
    (if (and (eq (send rsd1 :buf :mseq-mode) :reach)
	     (find (send rsd1 :buf :remove-limb) '(:rleg :lleg))
	     (eq (send rsd2 :buf :mseq-mode) :remove))
	(let* ((cs (send rsd1 :contact-states))
	       (all-limbs (send-all cs :name))
	       (remove-limb (send rsd1 :buf :remove-limb))
	       (fix-limb (remove remove-limb all-limbs))
	       (fix-contact
		(mapcar
		 #'(lambda (k)
		     (find-if
		      #'(lambda (cs) (eq k (send cs :name))) cs))
		 fix-limb))
	       (remove-contact
		(find-if #'(lambda (cs) (eq remove-limb (send cs :name))) cs))
	       )
	  (setq
	   opt
	   (instance
	    dynamic-optimize
	    :init
	    :algorithm 11
	    :ftol 1e-3 :xtol 1e-3 :eqthre 1e-2
	    :max-eval 30
	    :start-av (send rsd2 :angle-vector)
	    :end-av (send rsd1 :angle-vector)
	    :start-base-coords
	    (progn
	      (send *robot* :angle-vector (send rsd2 :angle-vector))
	      (send *robot* :newcoords (send rsd2 :root-coords))
	      (xyzrpy (send (car (send *robot* :links)) :worldcoords)))
	    :end-base-coords
	    (progn
	      (send *robot* :angle-vector (send rsd1 :angle-vector))
	      (send *robot* :newcoords (send rsd1 :root-coords))
	      (xyzrpy (send (car (send *robot* :links)) :worldcoords)))
	    :fix-target (send-all fix-contact :contact-coords)
	    :fix-coords (send-all fix-contact :target-coords)
	    :contact-states fix-contact
	    :start-contact-force
	    (mapcar #'(lambda (v) (subseq v 0 3)) (send rsd2 :contact-forces))
	    :end-contact-force
	    (mapcar #'(lambda (v) (subseq v 0 3)) (send rsd1 :contact-forces))
	    :start-contact-moment
	    (mapcar #'(lambda (v) (subseq v 3 6)) (send rsd2 :contact-forces))
	    :end-contact-moment
	    (mapcar #'(lambda (v) (subseq v 3 6)) (send rsd2 :contact-forces))
	    :split-cnt split-cnt
	    :freq 5
	    :move-target (list (send remove-contact :contact-coords))
	    :move-coords-list
	    (list
	     (float-limb-trajectory
	      rsd2 rsd1
	      :split-cnt split-cnt
	      :away-dist 50
	      :col-dist 0.8
	      :time-list '(0 0.6 0.8 1.0)
	      :debug-view debug-view))
	    :debug? nil))
	  (return-from nil nil)
	  )))
  (setq *log-stream* nil)
  (dotimes (i (round (/ (+ 1 (- end-freq start-freq)) step)))
    (format t "[torque-x-speed-test] ~A/~A~%"
	    i (round (/ (+ 1 (- end-freq start-freq)) step)))
    (setq freq (+ start-freq (* step i)))
    (send opt :freq freq)
    (send opt :evaluation-function
	  ;;(copy-object (send opt :start-base-coords))
	  #f(-47.8787 -96.5589 726.621 -0.085119 0.441627 -0.050446)
	  #F(0)
	  nil
	  )
    (setq torque-vector
	  (mapcar #'car (send opt :tfm-vector)))
    (setq torque-vector
	  (mapcar
	   #'(lambda (tau)
	       (map float-vector
		    #'*
		    tau
		    (send-all
		     (send-all (cdr (send *robot* :links)) :joint)
		     :max-joint-torque)))
	   torque-vector))
    ;;(setq torque-vector (mapcar #'norm torque-vector))
    (setq torque-vector
	  (mapcar
	   #'(lambda (tau)
	       (abs
		(cdr (assoc
		      (send *robot* :rleg :crotch-p)
		      (map cons
			   #'cons
			   (send-all (cdr (send *robot* :links)) :joint)
			   tau)))))
	   torque-vector))
    (setq id -1)
    (push
     (format nil "~A[Hz]" (round freq))
     name-list)
    (push
     (mapcar #'(lambda (d) (float-vector (incf id) d))
	     torque-vector)
     data-list)
    (let ((buf))
      (setq id -1)
      (setq buf
	    (append (make-list (round (- (* freq 3) 1)) :initial-element
			       (car torque-vector))
		    torque-vector))
      (push
       (mapcar #'(lambda (d)
		   (float-vector (incf id)
				 (/ (apply #'+ (subseq buf id (+ id (round (* freq 3)))))
				    (round (* freq 3)))))
	       (send opt :tfm-vector))
       average-list)
      )
    )
  (setq instant
	(create-graph
	 "INSTANTENIOUS TORQUE vs TIME"
	 :size '(600 300)
	 :range (list #F(0 60) (float-vector (- split-cnt 0) 70))
	 :name-list name-list
	 :data-list data-list))
  (setq average
	(create-graph
	 "AVERAGE TORQUE vs TIME"
	 :size '(600 300)
	 :range (list #F(0 60) (float-vector (- split-cnt 0) 70))
	 :name-list name-list
	 :data-list average-list))
  (dolist (g (list instant average))
    ;;(send g :fit-draw)
    (send g :clear)
    (send g :color #xFFFFFF)
    (send g :draw-axis :measure-range "~1,2f")
    (send g :plot-with-line-data)
    (send g :line-width 4)
    (send g :draw-color-label :step 0.5)
    (send g :color #xffffff)
    (send g :message-string "Torque/Max" :left-top)
    (send g :message-string "time[sec] / interpolation time step[sec]" :right-down)
    (send g :repaint))
  (list instant average opt)
  )

(defun demo-dynamic-connection
  nil
  ;;(setq *log-stream* nil)
   (demo-climb-setup :kirin-ladder)
  (connect-rsd)
  )

(defun model2real
  (&key (sleep-time 5000)
	(wait? t) (gain 1.0)
	(base-diff-angle))
  (cond
   (base-diff-angle
    (setq sleep-time
	  (* sleep-time
	     (/ (norm (v- (send *robot* :angle-vector)
			  (send *ri* :state :potentio-vector)))
		base-diff-angle)))
    (format t "[model2real] sleep-time=~A~%" sleep-time)))
  (cond
   ((and (boundp '*ri*) *ri*)
    (send *ri* :angle-vector
	  (scale gain (send *robot* :angle-vector))
	  sleep-time)
    (if wait? (send *ri* :wait-interpolation))))
  )

(defun call-av-sequence
  (rsd &key
       (real-time-factor 1.0)
       (land-sleep-time 300)
       (connect-time-step 3000)
       (sleep-time 3000)
       (reverse? nil)
       (base-diff-angle nil)
       (min-sleep-time 1500)
       ret)
  (cond
   (base-diff-angle
    (setq connect-time-step
	  (max
	   min-sleep-time
	   (* connect-time-step
	      (/ (norm (v- (send *robot* :angle-vector)
			   (send *ri* :state :potentio-vector)))
		 base-diff-angle))))
    (format t "[call-av-sequence] sleep-time=~A~%" connect-time-step)))
  (setq sleep-time
	(round (/ (* 1.0 connect-time-step)
		  (max 1.0
		       (length (cdr (assoc :position
					   (send rsd :buf :trajectory))))))))
  (format t "[call-av-sequence] ~A x ~A~%"
	  sleep-time
	  (length (cdr (assoc :position
			      (send rsd :buf :trajectory)))))
	  ;;(length (send rsd :buf :trajectory)))
  (setq ret
	(list
	 :angle-vector-sequence
	 (funcall
	  (if reverse? 'reverse 'identity)
	  (append
	   (list
	    (cadr (assoc :position
			 (send rsd :buf :trajectory))))
	   (cdr (assoc :position
		       (send rsd :buf :trajectory)))
	   (if reverse?
	       (last (cdr (assoc :position
				 (send rsd :buf :trajectory)))))
	   (list (send rsd :angle-vector))
	   ))
	 (funcall
	  (if reverse? 'reverse 'identity)
	  (append
	   (list (* real-time-factor land-sleep-time))
	   (make-list
	    (length (cdr (assoc
			  :position
			  (send rsd :buf :trajectory))))
	    :initial-element
	    (* real-time-factor sleep-time))
	   (if reverse?
	       (list (* real-time-factor land-sleep-time)))
	   (list (* real-time-factor land-sleep-time)))
	  )))
  ;; (eval (append (list 'send '*ri*) ret))
  (send *ri* (car ret) (cadr ret) (caddr ret))
  ret
  )

(defvar *grasp-minimus-sleep-time* 3)
(defun sequence-play
  (&key
   (robot *robot*)
   (file "log/tmp.rsd")
   (rsd-list (connect-rsd :file file))
   (connect-time-step 2100)
   ;;
   (reverse? nil)
   (ref-force? nil)
   (inner-log? nil)
   (force-rate 0.8)
   (real-flag (and (boundp '*ri*) *ri*))
   (auto-grasp? t)
   (animate? (not real-flag))
   (suspend-start 2)
   (suspend-buf)
   (sleep-time 5000)
   (land-sleep-time 500)
   (land-sleep-split 10)
   (real-time-factor 1.0)
   (wait? t)
   (base-diff-angle 90)
   ret
   )
  (if (not reverse?) (setq rsd-list (reverse rsd-list)))
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
    (if real-flag (model2real :sleep-time (* real-time-factor sleep-time)
			      :base-diff-angle base-diff-angle))
    (push (cons :angle-vector
		(list (copy-object (send robot :angle-vector))
		      (* real-time-factor sleep-time)))
	  ret)
    (format t "hook~%") (if wait? (read-line))
    (if (and real-flag auto-grasp? (find-method *ri* :hand-angle-vector))
	(send *ri* :hand-angle-vector;; (send robot :hand :arms :hook-pose)))
	      (send robot :hand-angle-vector)))
    (if real-flag (model2real :sleep-time (* real-time-factor sleep-time)
			      :base-diff-angle base-diff-angle))
    (push (cons :angle-vector
		(list (copy-object (send robot :angle-vector))
		      (* real-time-factor sleep-time)))
	  ret)
    (format t "grasp~%") (if wait? (read-line))
    (if (and real-flag auto-grasp? (find-method *ri* :hand-angle-vector))
	(send *ri* :hand-angle-vector ;;(send robot :hand :arms :grasp-pose)))
	      (send robot :hand-angle-vector)))
    (if real-flag
	(model2real :sleep-time (* real-time-factor sleep-time)
		    :base-diff-angle base-diff-angle))
    (push (cons :angle-vector
		(list (copy-object (send robot :angle-vector))
		      (* real-time-factor sleep-time)))
	  ret)
    (format t "ready?~%") (if wait? (read-line))
    ))
  (if (and real-flag inner-log?) (send *ri* :start-log))
  (dolist (rsd rsd-list)
    (case (send rsd :buf :mseq-mode)
      (:remove
       (send rsd :draw :robot robot :friction-cone? nil)
       (print :remove)
       (if wait? (read-line))
       (cond
        (real-flag
         (cond
          (ref-force?
           (mapcar
            #'(lambda (k)
                (send *ri* :set-ref-force
                      (scale force-rate
                             (or (send rsd :contact-forces k) #F(0 0 0)))
                      800 k))
            '(:rarm :larm))))
	 (if (send rsd :buf :trajectory)
	     (call-av-sequence rsd :real-time-factor real-time-factor
			       :land-sleep-time land-sleep-time
			       :connect-time-step connect-time-step
			       :sleep-time sleep-time
			       :base-diff-angle base-diff-angle
			       :reverse? reverse?))
         (send *ri* :wait-interpolation)
         (model2real :sleep-time (* real-time-factor sleep-time)
		     :base-diff-angle base-diff-angle)
         (send *ri* :wait-interpolation)
         )
        (t (unix:sleep 1)))
       (mapcar
	#'(lambda (k)
	    (push (cons :set-ref-force
			(list
			 (scale force-rate
				(or (send rsd :contact-forces k) #F(0 0 0)))
			 800 k))
		  ret)
	    )
	'(:rarm :larm))
       (push (cons :angle-vector
		   (list (copy-object (send robot :angle-vector))
			 (* real-time-factor sleep-time)))
	     ret)
       (if (find (send rsd :buf :remove-limb) '(:rarm :larm :arms))
	   (cond
	    ((and real-flag auto-grasp? (find-method *ri* :hand-angle-vector))
	     (send robot :hand (send rsd :buf :remove-limb) :hook-pose)
	     (send *ri* :hand-angle-vector ;; (send robot :hand :arms :angle-vector)))))
		   (send robot :hand-angle-vector)))))
       )
      (:suspend
       (send rsd :draw :robot robot :friction-cone? nil)
       (print :suspend)
       (if wait? (read-line))
       (if (find (send rsd :buf :remove-limb) '(:rarm :larm :arms))
	   (cond
	    ((and real-flag auto-grasp? (find-method *ri* :hand-angle-vector))
	     (send robot :hand (send rsd :buf :remove-limb) :grasp-pose)
	     (send *ri* :hand-angle-vector;; (send robot :hand :arms :angle-vector)))))
		   (send robot :hand-angle-vector)))))
       (cond
        (real-flag
         (cond
          (ref-force?
           (mapcar
            #'(lambda (k)
                (send *ri* :set-ref-force
                      (scale force-rate
                             (or (send rsd :contact-forces k) #F(0 0 0)))
                      800 k))
            '(:rarm :larm))))
	 (if (send rsd :buf :trajectory)
	     (call-av-sequence rsd :real-time-factor real-time-factor
			       :land-sleep-time land-sleep-time
			       :connect-time-step connect-time-step
			       :sleep-time sleep-time
			       :base-diff-angle base-diff-angle
			       :reverse? reverse?))
         (send *ri* :wait-interpolation)
         (model2real :sleep-time (* real-time-factor sleep-time)
		     :base-diff-angle base-diff-angle)
         (send *ri* :wait-interpolation)
         )
        (t (unix:sleep 1)))
       (mapcar
	#'(lambda (k)
	    (push
	     (cons :set-ref-force
		   (list
		    (scale force-rate
			   (or (send rsd :contact-forces k) #F(0 0 0)))
		    800 k))
	     ret))
	'(:rarm :larm))
       (push (cons :angle-vector
		   (list (copy-object (send robot :angle-vector))
			 (* real-time-factor sleep-time)))
	     ret)
       )
      (:reach
       (if (find (send rsd :buf :remove-limb) '(:rarm :larm :arms))
	   (cond
	    ((and real-flag auto-grasp? (find-method *ri* :hand-angle-vector))
	     (send robot :hand (send rsd :buf :remove-limb) :open-pose)
	     (send *ri* :hand-angle-vector;; (send robot :hand :arms :angle-vector)))))
		   (send robot :hand-angle-vector)))))
       (cond
        ((null (send rsd :buf :trajectory))
         ;; (return-from sequence-play nil)
         (send rsd :draw :robot robot :friction-cone? nil)
	 (print :reach)
         (if wait? (read-line))
         (cond
          (real-flag
           (cond
            (ref-force?
             (mapcar
              #'(lambda (k)
                  (send *ri* :set-ref-force
                        (scale force-rate
                               (or (send rsd :contact-forces k) #F(0 0 0)))
                        800 k))
              '(:rarm :larm))))
           (model2real :sleep-time (* real-time-factor connect-time-step)
		       :base-diff-angle base-diff-angle)
           (send *ri* :wait-interpolation)
           )
          (t (unix:usleep (* 1000 connect-time-step)))
          )
	 (mapcar
	  #'(lambda (k)
	      (push
	       (cons :set-ref-force
		     (list
		      (scale force-rate
			     (or (send rsd :contact-forces k) #F(0 0 0)))
		      800 k))
	       ret))
	  '(:rarm :larm))
	 (push (cons :angle-vector
		     (list (copy-object (send robot :angle-vector))
			   (* real-time-factor connect-time-step)))
	       ret)
	 (if (find (send rsd :buf :remove-limb) '(:rarm :larm :arms))
	     (cond
	      ((and real-flag auto-grasp? (find-method *ri* :hand-angle-vector))
	       (send robot :hand (send rsd :buf :remove-limb) :hook-pose)
	       (send *ri* :hand-angle-vector;; (send robot :hand :arms :angle-vector)))))
		     (send robot :hand-angle-vector))
	       (unix:sleep *grasp-minimus-sleep-time*)
	       (send *ri* :wait-interpolation)
	       (send robot :hand (send rsd :buf :remove-limb) :grasp-pose)
	       (send *ri* :hand-angle-vector;; (send robot :hand :arms :angle-vector)))))
		     (send robot :hand-angle-vector)))))
         )
        (t
         (let*
             ((lkey (car (remove (send rsd :buf :remove-limb) '(:rleg :lleg))))
              (lleg (send robot lkey :end-coords :copy-worldcoords))
	      (_sleep-time sleep-time)
              (sleep-time
               (round
                (/ connect-time-step (length (cdr (assoc :position (send rsd :buf :trajectory))))))))
           (format t "connect in ~Ax~A sec~%"
                   sleep-time
                   (length (cdr (assoc :position (send rsd :buf :trajectory)))))
           (cond
            ((and real-flag reverse?)
             (send robot :angle-vector
                   (car (cdr (assoc :position (send rsd :buf :trajectory)))))
             ))
	   (cond
	    (animate?
	     (mapcar
	      #'(lambda (av)
		  (send robot :angle-vector av)
		  (send *robot* :fix-leg-to-coords lleg lkey)
		  (send *irtviewer* :draw-objects)
		  (cond
		   ;;(real-flag (model2real :sleep-time sleep-time))
		   (t (unix:usleep (* 1000 sleep-time)))))
	      (funcall
	       (if reverse? 'reverse 'identity)
	       (cdr (assoc :position (send rsd :buf :trajectory)))))))
	   (send rsd :draw :robot robot :friction-cone? nil)
	   (print :reach)
           (if wait? (read-line))
	   (cond
            ((and real-flag reverse?)
	     (send rsd :draw :robot robot :friction-cone? nil)
	     (model2real :sleep-time (* real-time-factor _sleep-time)
			 :base-diff-angle base-diff-angle)
	     (send *ri* :wait-interpolation)
             ))
           (cond
            (real-flag
	     (push
	      (call-av-sequence rsd :real-time-factor real-time-factor
				:land-sleep-time land-sleep-time
				:connect-time-step connect-time-step
				:sleep-time _sleep-time
				:reverse? reverse?)
	      ret)
	     (send *ri* :wait-interpolation)))
	   (if (find (send rsd :buf :remove-limb) '(:rarm :larm :arms))
	       (cond
		((and real-flag auto-grasp? (find-method *ri* :hand-angle-vector))
		 (send robot :hand (send rsd :buf :remove-limb) :hook-pose)
		 (send *ri* :hand-angle-vector;; (send robot :hand :arms :angle-vector)))))
		       (send robot :hand-angle-vector))
		 (unix:sleep *grasp-minimus-sleep-time*)
		 (send *ri* :wait-interpolation)
		 (send robot :hand (send rsd :buf :remove-limb) :grasp-pose)
		 (send *ri* :hand-angle-vector;; (send robot :hand :arms :angle-vector)))))
		       (send robot :hand-angle-vector)))))
	   (mapcar
	    #'(lambda (k)
		(push
		 (cons  :set-ref-force
                        (list
			 (scale force-rate
				(or (send rsd :contact-forces k) #F(0 0 0)))
			 500 k))
		 ret))
	    '(:rarm :larm))
           (cond
            ((and real-flag ref-force?)
             (mapcar
              #'(lambda (k)
                  (send *ri* :set-ref-force
                        (scale force-rate
                               (or (send rsd :contact-forces k) #F(0 0 0)))
                        500 k))
              '(:rarm :larm))))
           ))))))
  (if (and real-flag inner-log?) (send *ri* :stop-and-save-log))
  (reverse ret)
  )

#|

(setq *log-stream* nil)
(demo-climb-setup :kirin-ladder)
(connect-rsd)

#f(-47.8787 -96.5589 726.621 -0.085119 0.441627 -0.050446)
#f(-47.8787 -96.6259 726.621 -0.08308 0.456157 -0.048069)

#|
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
	   (float-limb-trajectory
	    rsd2 rsd1
	    :split-cnt 10
	    :away-dist 50
	    :col-dist 0.8
	    :time-list '(0 0.6 0.8 1.0)
	    :debug-view debug-view))
       rsd1)
   rsd-list (append (cdr rsd-list) (list *sample-rsd*))))


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
	   (let* ((cs (send rsd1 :contact-states))
		  (all-limbs (send-all cs :name))
		  (remove-limb (send rsd1 :buf :remove-limb))
		  (fix-limb (remove remove-limb all-limbs))
		  (fix-contact
		   (mapcar
		    #'(lambda (k)
			(find-if
			 #'(lambda (cs) (eq k (send cs :name))) cs))
		    fix-limb))
		  (remove-contact
		   (find-if #'(lambda (cs) (eq remove-limb (send cs :name))) cs))
		  )
	     (setq
	      hoge (list
		    (float-limb-trajectory
		     rsd2 rsd1
		     :split-cnt 10
		     :away-dist 50
		     :col-dist 0.8
		     :time-list '(0 0.6 0.8 1.0)
		     :debug-view debug-view)))
	     ))
       rsd1)
   rsd-list (append (cdr rsd-list) (list *sample-rsd*))))


(setq g (car *graph-sample*))
(send g :clear)
(send g :color #xFFFFFF)
(send g :draw-axis :measure-range "~1,2f")
(send g :plot-with-line-data)
(send g :draw-color-label)
(send g :color #xffffff)
(send g :message-string "Torque/Max" :left-top)
(send g :message-string "time[sec] / interpolation time step[sec]" :right-down)
(send g :repaint)

(list 0.957996 0.83649 0.645828 0.643536 0.641452 0.641451 0.641435 0.641188 0.640659 0.631062 0.62726 0.626183 0.613955 0.587573 0.589027 0.578813 0.578579 0.578432 0.579227 0.57839 0.578762 0.578222 0.733923 0.578215 0.73495 0.578209 0.734429 0.578201 0.734908 0.578195)

$ (progn (demo-climb-setup :kirin-ladder) (setq *log-stream* nil) (connect-rsd :algorithm L_BFGS))

ALGORITHM: SL_VM(11)
0.957996 (err=0.015267) with #f(-47.8787 -96.5589 726.621 -0.085119 0.441627 -0.050446)
0.83649 (err=0.013931) with #f(-47.8787 -96.559 726.621 -0.07951 0.444412 -0.052255)
0.645828 (err=0.014764) with #f(-47.8787 -96.559 726.621 -0.07951 0.456384 -0.054689)
0.643536 (err=0.015163) with #f(-47.8786 -96.559 726.621 -0.07951 0.456384 -0.059386)
0.641452 (err=0.017236) with #f(-47.8784 -96.5591 726.621 -0.07951 0.456384 -0.067717)
0.641451 (err=0.017236) with #f(-47.8783 -96.5592 726.621 -0.07951 0.456384 -0.067717)
0.641435 (err=0.017237) with #f(-47.8766 -96.5603 726.622 -0.07951 0.456384 -0.067717)
0.641188 (err=0.017252) with #f(-47.8501 -96.5767 726.622 -0.07951 0.456384 -0.067717)
0.640659 (err=0.017304) with #f(-47.7918 -96.6127 726.622 -0.07951 0.456384 -0.067717)
0.631062 (err=0.015926) with #f(-46.9463 -97.136 726.627 -0.07951 0.456384 -0.067717)
0.62726 (err=0.015055) with #f(-46.6068 -97.3462 726.63 -0.07951 0.456384 -0.067717)
0.626183 (err=0.015145) with #f(-46.4782 -97.4258 726.63 -0.07951 0.456384 -0.067717)
0.613955 (err=0.016428) with #f(-44.7987 -98.4651 726.641 -0.07951 0.456384 -0.067717)
0.587573 (err=0.019055) with #f(-33.9592 -102.647 727.535 -0.07951 0.456384 -0.067717)
0.589027 (err=0.01864) with #f(-33.9592 -102.647 727.535 -0.090729 0.443595 -0.064385)
0.578813 (err=0.018508) with #f(-33.9592 -102.647 727.535 -0.085435 0.449629 -0.065957)
0.578579 (err=0.018579) with #f(-33.9592 -102.647 727.535 -0.085072 0.450656 -0.065951)
0.578432 (err=0.018436) with #f(-33.9592 -102.647 727.535 -0.085436 0.451632 -0.065549)
0.579227 (err=0.01783) with #f(-33.9591 -102.647 727.535 -0.088161 0.454907 -0.063402)
0.57839 (err=0.018369) with #f(-33.9592 -102.647 727.535 -0.085676 0.45192 -0.06536)
0.578762 (err=0.017609) with #f(-33.9591 -102.647 727.535 -0.090729 0.45614 -0.061853)
0.578222 (err=0.018161) with #f(-33.9592 -102.647 727.535 -0.08657 0.452666 -0.06474)
0.733923 (err=0.017515) with #f(-33.9591 -102.647 727.535 -0.090365 0.456384 -0.062189)
0.578215 (err=0.018153) with #f(-33.9592 -102.647 727.535 -0.086608 0.452703 -0.064714)
0.73495 (err=0.017505) with #f(-33.9591 -102.647 727.535 -0.090729 0.455242 -0.062493)
0.578209 (err=0.018145) with #f(-33.9592 -102.647 727.535 -0.086649 0.452729 -0.064692)
0.734429 (err=0.017587) with #f(-33.9591 -102.647 727.535 -0.090094 0.456384 -0.06265)
0.578201 (err=0.018137) with #f(-33.9592 -102.647 727.535 -0.086683 0.452765 -0.064672)
0.734908 (err=0.017528) with #f(-33.9591 -102.647 727.535 -0.090729 0.455324 -0.062735)
0.578195 (err=0.018129) with #f(-33.9592 -102.647 727.535 -0.086724 0.452791 -0.064652)
0.578195 (err=0.018129) with #f(-33.9592 -102.647 727.535 -0.086724 0.452791 -0.064652)
Succeed: Relative tolerance on optimization parameters was reached. 
Number of iteration.:	 30
object function:  0.578195
  | where     x:  -33.9592 -102.647 727.535 -0.0867237 0.452791 -0.0646523
  |   eq constt: 
  |  neq constt: 
TIME: 149.001 sec
ALGORITHM: SL_VM(11)

(send *robot* :hand :arms :open-pose)
(send *ri* :hand-angle-vector (send *robot* :hand-angle-vector))

(send *robot* :move-centroid-on-foot :both '(:rleg :lleg :rarm :larm) :target-centroid-pos (v+ (send *robot* :centroid) #F(50 0 0)))

(send *robot* :arms :move-end-pos #F(-50 0 0) :world)

(send *robot* :legs :knee-p :joint-angle -10 :relative t)