;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsk-collada)
(require "../robot-state-data2.l")
(require "../contact-state.l")
(require "../optimize-brli.l")
(require "dynamic-torque-util.l")
(require "dynamic-trajectory.l")

(require "package://eus_qp/euslisp/eiquadprog.l")

(defvar *robot*)

(defun dynamic-motion-filter
  (&key (robot *robot*)
	(contact-states ;;
	 (remove-if
	  #'(lambda (cs)
	      (not (find (send cs :name)
			 '(:rleg :lleg))))
	  (now-contact-state)))
	(rest-contact-states nil)
	(limbs (send-all contact-states :name))
	(limb-coords (send-all contact-states :contact-coords))
	(limb-n
	 (mapcar
	  #'(lambda (c n)
	      (transform (send c :worldrot) n))
	  limb-coords
	  (send-all contact-states :contact-n)))
	(limb-local-transform
	 (mapcar
	  #'(lambda (n)
	      (transpose
	       (matrix-exponent
		(normalize-vector (v* #F(0 0 1) n))
		(acos (v. #F(0 0 1) n)))))
	  limb-n))
	(limb-local-transform6x6
	 (mapcar
	  #'(lambda (r) (matrix-append (list r r) '(1 1)))
	  limb-local-transform))
	(limb-links-prev
	 (mapcar
	  #'(lambda (c l)
              (send robot :link-list (send c :parent)
		    (if (find-method robot l)
			(send robot l :root-link))))
	  limb-coords limbs))
	;; (6dof-joints
	;;  (add-6dof-joint :robot robot
	;; 		 :link-list limb-links-prev))
	(limb-links limb-links-prev) ;;(cdr (assoc :6dof-links 6dof-joints)))
	(rest-links
	 (mapcar
	  #'(lambda (rcs)
              (send robot :link-list
		    (send (send rcs :contact-coords) :parent)
		    (if (find-method robot (send rcs :name))
			(send robot (send rcs :name) :root-link))))
	  rest-contact-states))
	(limb-joints
	 (mapcar #'(lambda (ll) (send-all ll :joint)) limb-links))
	(limb-tmax-rate (mapcar #'(lambda (h) 0.95) limbs))
	(limb-tmax-rate-for-rsd limb-tmax-rate)
	(limb-tmax
	 (mapcar
	  #'(lambda (jl r) (mapcar #'(lambda (j) (* r (send j :max-joint-torque))) jl))
	  limb-joints limb-tmax-rate))
	(limb-tmax-for-rsd
	 (mapcar
	  #'(lambda (jl r) (mapcar #'(lambda (j) (* r (send j :max-joint-torque))) jl))
	  limb-joints limb-tmax-rate-for-rsd))
	(limb-fmax
	 (send-all contact-states :force0))
	(slip-matrix
	 (copy-object (send-all contact-states :slip-matrix)))
	(slip+
	 (mapcar
	  #'(lambda (slip r)
	      (setf (aref slip 2 2) 2)
	      (m* (m- (unit-matrix 6) slip) r))
	  slip-matrix limb-local-transform6x6))
	(slip-
	 (mapcar
	  #'(lambda (slip r)
	      (setf (aref slip 2 2) 0)
	      (m* (m+ (unit-matrix 6) slip) r))
	  slip-matrix limb-local-transform6x6))
	;;(grasp-matrix
	;; (send robot :calc-grasp-matrix
	;; (send-all limb-coords :worldpos)))
	(target-ddq (scale 0 (send *robot* :angle-vector)))
	(target-dv (instantiate float-vector 6))
	(target-tau (instantiate float-vector (length (flatten limb-joints))))
	(target-f (instantiate float-vector (* 6 (length limbs))))
	(target-vector
	 (concatenate float-vector
		      target-ddq target-dv target-tau #F(0 0 0 0 0 0) target-f))
	(weight-vector
	 (concatenate
	  float-vector
	  (make-list (length target-ddq) :initial-element 0.1)
	  (make-list (length target-dv) :initial-element 0.1)
	  (make-list (length target-tau) :initial-element 10)
	  (make-list 6 :initial-element 1e-5)
	  (make-list (length target-f) :initial-element 10)))
	(weight-matrix (diag weight-vector))
	(fm-filter
	 (matrix-append
	  (list
	   (make-matrix (+ (length target-f) 0);(length target-tau))
			(+ (length target-dv) (length target-ddq) (length target-tau) 6))
	   (unit-matrix (+ (length target-f) 0)));(length target-tau))))
	  '(0 1)))
	(q-links  ;; for ddq
	 (cond
	  ((find-method robot :actuators)
	   (send-all (send robot :actuators) :parent))
	  (t (cdr (send robot :links)))))
	(dq (scale 0 (send *robot* :angle-vector)))
	(root-angular-velocity #F(0 0 0)) ;;
	(root-spacial-velocity #F(0 0 0)) ;;
	(f0
	 (apply
	  #'concatenate
	  (cons
	   float-vector
	   (calc-torque-check-with-ddq
	    :robot robot
	    :q-links q-links
	    :all-links (flatten limb-links)
	    :dq dq
	    :root-angular-velocity root-angular-velocity
	    :root-spacial-velocity root-spacial-velocity
	    :ddq-target target-ddq
	    :dv-target target-dv
	    :contact-states
	    (mapcar
	     #'(lambda (cs)
		 (list
		  (cons :link
			(send (send cs :contact-coords) :parent))
		  (cons :worldcoords
			(copy-object
			 (send (send cs :contact-coords) :worldcoords)))))
	     contact-states)
	    ))))
	(cteq
	 (calc-torque-eq-with-ddq
	  :robot robot
	  :all-links (flatten limb-links)
	  :dq dq
	  :root-angular-velocity root-angular-velocity
	  :root-spacial-velocity root-spacial-velocity
	  :contact-states
	  (mapcar
	   #'(lambda (cs)
	       (list
		(cons :link (send (send cs :contact-coords) :parent))
		(cons :worldcoords
		      (copy-object (send (send cs :contact-coords) :worldcoords)))))
	   contact-states)
	  ))
	;;
	(ddq-filter
	 (matrix-append
	  (list
	   (unit-matrix (+ (length target-dv) (length target-ddq)))
	   (make-matrix (+ (length target-dv) (length target-ddq))
			(+ (length target-tau) 6 (length target-f))))
	  '(0 1)))
	(limb-jacobian
	 (mapcar
	  #'(lambda (cs)
	      (matrix-append
	       (list
		(transpose
		 (calc-jacobian-fill0
		  :robot robot
		  :all-links q-links
		  :move-target (send cs :contact-coords)))
		;;(unit-matrix 6)
		(make-matrix
		 6 6
		 '((1 0 0 0 0 0)
		   (0 1 0 0 0 0)
		   (0 0 1 0 0 0)
		   (0 0 0 1 0 0)
		   (0 0 0 0 1 0)
		   (0 0 0 0 0 1)))
		)
	       '(0 1)))
	  contact-states))
	(contact-states-for-torque-vector2
	 (mapcar
	  #'(lambda (cs)
	      (list
	       (cons :link (send (send cs :contact-coords) :parent))
	       (cons :worldcoords
		     (send (send cs :contact-coords) :copy-worldcoords))))
	  contact-states))
	(limb-acc
	 (progn
	   (send robot
		 :torque-vector2
		 :contact-states contact-states-for-torque-vector2
		 :root-angular-velocity root-angular-velocity
		 :root-spacial-velocity root-spacial-velocity
		 :jvv dq
		 :jav (scale 0 dq))
	   (print 'hogehoge)
	   (mapcar
	    #'(lambda (cs)
		(let ((l (send (send cs :contact-coords) :parent)))
		  (concatenate
		   float-vector
		   (print (send l :spacial-acceleration))
		   (print (send l :angular-acceleration))
		   )))
	    contact-states)))
	;;
	(non-eq-mat
	 (matrix-append
	  (list
	   (matrix-append slip+ '(1 1))
	   (scale-matrix -1 (matrix-append slip- '(1 1))))
	  '(1 0)))
	(non-eq-right-vector
	 (apply #'concatenate
		(cons float-vector
		      (append limb-fmax limb-fmax))))
	(non-eq-left-vector #F())
	(f0-max
	 (concatenate
	  float-vector
	  (make-list (length dq) :initial-element 1)
	  (make-list 6 :initial-element 1)
	  (flatten limb-tmax)
	  (make-list 6 :initial-element 1)
	  (make-list (length target-f) :initial-element 1e+3)))
	;;
	(ok? #i(0))
	(oqp
	 (or
	  (solve-eiquadprog ;;octave-qp
	   :initial-state f0
	   :eval-weight-matrix weight-matrix
	   :eval-coeff-vector
	   (scale -2 (transform weight-matrix target-vector))
	   :equality-matrix
	   (matrix-append
	    (append
	     (mapcar
	      #'(lambda (j) (m* j ddq-filter))
	      limb-jacobian)
	     (list (cdr (assoc :HBIJ/00I0 cteq))))
	    '(1 0))
	   :equality-vector
	   (apply
	    #'concatenate
	    (cons
	     float-vector
	     (append
	      (mapcar #'(lambda (v) (scale -1 v)) limb-acc)
	      (list  (cdr (assoc :tfm0 cteq))))))
	   :state-min-vector (scale -1 f0-max)
	   :state-max-vector f0-max
	   :inequality-min-vector non-eq-left-vector
	   :inequality-matrix (m* non-eq-mat fm-filter)
	   :inequality-max-vector non-eq-right-vector
	   :debug? t
	   :ok? ok?
	   )
	  f0))
	(oqp-ddq
	 (subseq oqp
		 0
		 (length target-ddq)))
	(oqp-dv
	 (subseq oqp
		 (length target-ddq)
		 (+ (length target-ddq) (length target-dv))))
	(oqp-force
	 (subseq oqp
		 (+ (length target-dv) (length target-ddq) (length target-tau) 6)
		 (+ (length target-dv) (length target-ddq)
		    (length target-tau) 6 (length target-f))))
	(oqp-torque
	 (subseq oqp
		 (+ (length target-dv) (length target-ddq))
		 (+ (length target-dv) (length target-ddq) (length target-tau))))
	(limb-force (let ((index 0))
		      (mapcar #'(lambda (c)
				  (subseq oqp-force
					  (* 6 index)
					  (* 6 (incf index))))
			      limb-coords)))
	(brli-tau-gain 1)
	(rsd (instance robot-state-data2 :init
		       :contact-states
		       (append contact-states rest-contact-states)
		       :contact-forces
		       (append
			limb-force
			(mapcar #'(lambda (hoge) #F(0 0 0 0 0 0))
				rest-contact-states))
		       :contact-torque-max
		       (append
			limb-tmax-for-rsd
			(mapcar
			 #'(lambda (ll)
			     (send-all (send-all ll :joint)
				       :max-joint-torque))
			 rest-links))
		       :brli-gain brli-tau-gain
		       :log *log-stream*
		       ))
	&allow-other-keys
	)
  ;; (format *log-stream* "[optimize brli calcuration check]~%")
  ;; (error-checker "torque" limb-torque (subseq (send rsd :contact-torque)
  ;; 					      0 (length contact-states)))
  ;; (error-checker "brli-torque" brli-torque
  ;; 		 (subseq (send rsd :torque-brli) 0 (length contact-states)))
  ;; (error-checker "brli-friction" brli-force
  ;; 		 (subseq (send rsd :friction-brli) 0 (length contact-states)))
  ;; (error-checker "brli-diff" brli
  ;; 		 (subseq (send rsd :brli) 0 (length contact-states)))
  ;; (if (not (zerop (aref ok? 0)))
  ;;     (format
  ;;      *log-stream*
  ;;      "[qp failed] dump conditions~%    ~A~%"
  ;;      (list
  ;; 	:equality-matrix
  ;; 	(matrix-append
  ;; 	 (append
  ;; 	  (mapcar
  ;; 	   #'(lambda (j) (m* j ddq-filter))
  ;; 	   limb-jacobian)
  ;; 	  (list (cdr (assoc :eq-coeff cteq))))
  ;; 	 '(1 0))
  ;; 	:equality-vector
  ;; 	(apply
  ;; 	 #'concatenate
  ;; 	 (cons
  ;; 	  float-vector
  ;; 	  (append
  ;; 	   (mapcar #'(lambda (v) (scale -1 v)) limb-acc)
  ;; 	   (list  (cdr (assoc :tfm0 cteq))))))
  ;; 	;; :state-min-vector (scale -1 f0-max)
  ;; 	;; :state-max-vector f0-max
  ;; 	;; :inequality-min-vector non-eq-left-vector
  ;; 	;; :inequality-matrix (m* non-eq-mat fm-filter)
  ;; 	;; :inequality-max-vector non-eq-right-vector
  ;; 	)))
  (send rsd :buf :ddq oqp-ddq)
  (send rsd :buf :dv oqp-dv)
  ;; (funcall (cdr (assoc :del-6dof-links 6dof-joints)))
  rsd
  )

(defun demo-motion-filter-controller
  (&rest
   args
   &key
   (time-stop 5)
   &allow-other-keys)
  (do-until-key
   (apply
    #'demo-motion-filter
    (append
     (list :timer (- time-stop 1e-10))
     args))))

(defun fix-robot-state
  (&key
   (angle-vector (scale 0 (send *robot* :angle-vector)))
   (base-coords
    (concatenate
     float-vector
     (scale 1e-3 (send *robot* :worldpos))
     (reverse (car (send *robot* :rpy-angle)))))
   (move-target
    (mapcar #'(lambda (k) (send *robot* k :end-coords)) '(:rleg :lleg)))
   (fix-coords (send-all move-target :copy-worldcoords))
   (target-centroid-pos
    (scale (/ 1.0 (length fix-coords))
	   (reduce #'v+
		   (send-all fix-coords :worldpos))))
   (prev-av (copy-object (send *robot* :angle-vector)))
   (prev-c (copy-object (send *robot* :worldcoords)))
   ret
   )
  (print target-centroid-pos)
  (send *robot* :angle-vector
	(map float-vector #'rad2deg angle-vector))
  (send *robot* :newcoords
	(make-coords :pos (scale 1e+3 (subseq base-coords 0 3))
		     :rpy (reverse (subseq base-coords 3 6))))
  (send *robot*
	:fullbody-inverse-kinematics
	fix-coords
	:move-target move-target
	:link-list
	(mapcar
	 #'(lambda (mt)
	     (send *robot* :link-list (send mt :parent)))
	 move-target)
	:target-centroid-pos target-centroid-pos
	:centroid-thre 10
	:cog-gain 1
	:debug-view :no-message
	:stop 30)
  (setq ret
	(list (cons :angle-vector
		    (map float-vector #'deg2rad (send *robot* :angle-vector)))
	      (cons :base-coords
		    (concatenate
		     float-vector
		     (scale 1e-3 (send *robot* :worldpos))
		     (reverse (car (send *robot* :rpy-angle))))
		    )))
  (send *robot* :angle-vector prev-av)
  (send *robot* :newcoords prev-c)
  (send *viewer* :draw-objects)
  (send
   (concatenate
    float-vector
    (subseq (send *robot* :centroid) 0 2)
    (float-vector 0))
   :draw-on :flush t )
  ;;(read-line)
  ret)

(defun demo-motion-filter
  (&key
   (robot *robot*)
   ;;(balanced-robot (copy-object robot))
   (init
    (progn
      (base-coords-fix (make-coords) :debug? t)
      (send robot :reset-pose)
      (cond
       ((or (not (boundp '*viewer*)) (not *viewer*))
	(pickview :no-menu t)
	(objects (list robot))))
      (send *viewer* :draw-objects)))
   (move-target
    (mapcar #'(lambda (k)
		(send robot k :end-coords))
	    '(:rleg :lleg)))
   (fix-coords
    (copy-object (send-all move-target :worldcoords)))
   (dfix)
   (time-step 0.05)
   (target-q
    (map float-vector #'deg2rad
	 (scale 0 (send robot :angle-vector))))
   (target-b
    (concatenate
     float-vector
     (scale 1e-3 (send (car (send robot :links)) :worldpos))
     (matrix-log
      (send (car (send robot :links)) :worldrot))
     ))
   (q (map float-vector #'deg2rad (send robot :angle-vector)))
   (dq (scale 0 q))
   (ddq (scale 0 dq))
   (b (concatenate
       float-vector
       (scale 1e-3 (send (car (send robot :links)) :worldpos))
       (matrix-log
	(send (car (send robot :links)) :worldrot))
       ))
   (db (scale 0 b))
   (ddb (scale 0 db))
   (q-links  ;; for ddq
    (cond
     ((find-method robot :actuators)
      (send-all (send robot :actuators) :parent))
     (t (cdr (send robot :links)))))
   (timer 0)
   (convergence-thre
    (norm2
     (coerce
      (make-list (length q) :initial-element 0.01)
      float-vector)))
   (timer-stop 30.0)
   (max-ddq 1)
   (max-ddb 1)
   buf
   (target-ddq (scale 0 ddq))
   ret
   target-ddb
   )
  (format t "[demo-motion-filter]~%    target-q=~A~%   target-b=~A~%   now-b=~A~%"
	  target-q target-b b)
  (do-until-key
   (cond
    ((not
      (and
       (not
	(and (< (norm2 (v- target-q q)) convergence-thre)
	     (< (norm2 (v- target-b b)) 0.01)))
       (> timer-stop timer)))
     (print 'convergence)
     (setq ret (reverse ret))
     (return-from
      nil
      (list (cons :rsd ret)
	    (cons :time-step time-step)))))
   (format *log-stream* "[TIME] ~A~%" timer)
   (format *log-stream*
	   "[ddq update]~%   actual=~A~%   target=~A~%   velocity=~A~%"
	   ddq target-ddq (norm dq))
   (format *log-stream*
	   "[ddb update]~%   actual=~A~%   target=~A~%   velocity=~A~%" ddb target-ddb db)
   (format *log-stream*
	   "[convergence check]~% qdiff=~A~% bdiff=~A~%"
	   (v- target-q q) (v- target-b b))
   ;; update robot state
   (setq dq (v+ dq (scale time-step ddq)))
   (setq db (v+ db (scale time-step ddb)))
   (setq q (v+ q (scale time-step dq)))
   (setq b (v+ b (scale time-step db)))
   ;;
   (send robot :angle-vector (map float-vector #'rad2deg q))
   (base-coords-fix
    (make-coords :pos (scale 1e+3 (subseq b 0 3))
		 :rpy (reverse (subseq b 3 6)))
    :debug? t
    :robot robot)
   ;;(send robot :rleg :toe-p :joint-angle 0)
   ;;(send robot :lleg :toe-p :joint-angle 0)
   ;;
   ;; (setq dfix
   ;; 	 (mapcar
   ;; 	  #'(lambda (mt c)
   ;; 	      (concatenate
   ;; 	       float-vector
   ;; 	       (scale 1e-3
   ;; 		      (transform
   ;; 		       (send c :worldrot)
   ;; 		       (send c :difference-position mt)))
   ;; 	       (transform
   ;; 		(send c :worldrot)
   ;; 		(send c :difference-rotation mt))))
   ;; 	  move-target fix-coords))
   ;; (cond
   ;;  ((> (apply #'+ (mapcar #'norm2 dfix)) 1)
   ;;   (return-from nil 'endcoords-movement))
   ;;  (t
   ;;   (setq db (v- db (scale (/ 1.0 (length dfix))
   ;; 			    (reduce #'v+ dfix))))
   ;;   (setq b (v+ buf (scale time-step db)))))
   ;;
   ;; balance posture
   ;; calc next reference acc
   (setq target-ddq
	 (v+ q
	     (scale (min 0.03 (norm (v- target-q q)))
		    (normalize-vector (v- target-q q)))))
   (setq target-ddb ;; target-b
	 (v+ b
	     (scale (min 0.01 (norm (v- target-b b)))
		    (normalize-vector (v- target-b b)))))
   ;; (setq target-ddq
   ;; 	 (fix-robot-state
   ;; 	  :angle-vector target-ddq
   ;; 	  :base-coords target-ddb
   ;; 	  :move-target move-target
   ;; 	  :fix-coords fix-coords))
   ;; (setq target-ddb
   ;; 	 (cdr (assoc :base-coords target-ddq)))
   ;; (setq target-ddq
   ;; 	 (cdr (assoc :angle-vector target-ddq)))
   ;;
   (setq target-ddq
	 (v- target-ddq q))
   (setq target-ddq
	 (scale
	  (/ 1.0 (* time-step time-step))
	  (v- target-ddq
	      (scale time-step dq))))
   (setq target-ddq
	 (scale
	  (min 10 (norm target-ddq))
	  (normalize-vector target-ddq)))
   ;;
   (setq target-ddb ;; delta b
	 (v- target-ddb b))
   (setq target-ddb ;; ddb
	 (scale
	  (/ 1.0 (* time-step time-step))
	  (v- target-ddb
	      (scale time-step db))))
   (setq target-ddb ;; normalize
	 (scale
	  (min 0.01 (norm target-ddb))
	  (normalize-vector target-ddb)))
   (send-all (send *robot* :links) :worldcoords)
   (send *viewer* :draw-objects)
   (x::window-main-one)
   ;;(read-line)
   (setq buf (dynamic-motion-filter
	      :robot robot
	      :q-links q-links
	      :dq dq
	      :root-spacial-velocity (subseq db 0 3)
	      :root-angular-velocity (subseq db 3 6)
	      :target-ddq target-ddq
	      :target-dv target-ddb
	      :contact-states
	      (remove-if
	       #'(lambda (cs) (not (find (send cs :name) '(:rleg :lleg))))
	       (now-contact-state))
	      :contact-states-for-torque-vector2
	      (if buf
		  (mapcar
		   #'(lambda (ec fm)
		       (list (cons :link (send ec :parent))
			     (cons :worldcoords
				   (send ec :copy-worldcoords))
			     (cons :ext-force (subseq fm 0 3))
			     (cons :ext-moment (subseq fm 3 6))))
		   (send buf :contact-coords)
		   (send buf :contact-forces)))
	      ))
   ;; error check
   (cond
    ((and (< (norm (send buf :buf :ddq)) 1e+7)
	  (< (norm (send buf :buf :dv)) 1e+7))
     (setq ddq (send buf :buf :ddq))
     (setq ddb (send buf :buf :dv)))
    (t (format t "too large!! ~A~%"
	       (send buf :buf))
       (return-from nil 'too-large)))
   ;; acc check
   (send robot :torque-vector2
	 :contact-states
	 (if buf
	     (mapcar
	      #'(lambda (ec fm)
		  (list (cons :link (send ec :parent))
			(cons :worldcoords
			      (send ec :copy-worldcoords))
			(cons :ext-force (subseq fm 0 3))
			(cons :ext-moment (subseq fm 3 6))))
	      (send buf :contact-coords)
	      (send buf :contact-forces)))
	 :root-angular-velocity (subseq db 3 6)
	 :root-spacial-velocity (subseq db 0 3)
	 :root-angular-acceleration (subseq ddb 3 6)
	 :root-spacial-acceleration (subseq ddb 0 3)
	 :jvv dq
	 :jav ddq)
   (format t "[limb acc]~%")
   (pprint
    (mapcar
     #'(lambda (k)
	 (concatenate
	  float-vector
	  (send (send robot k :end-coords :parent) :spacial-acceleration)
	  (send (send robot k :end-coords :parent) :angular-acceleration)))
     '(:rleg :lleg)))
   ;;
   (setq timer (+ timer time-step))
   (push buf ret)
   )
  )

(defun motion-filter-play
  (&rest
   args
   &key
   (filtered-motion (apply #'demo-motion-filter args))
   (rsd (cdr (assoc :rsd filtered-motion)))
   (angle-vector (send-all rsd :angle-vector))
   (base-coords (send-all rsd :root-coords))
   (time-step (cdr (assoc :time-step filtered-motion)))
   (callback
    #'(lambda (ts) (unix:usleep (round (* 1e+6 ts)))))
   (ri nil)
   &allow-other-keys
   )
  (mapcar
   #'(lambda (c av)
       (send *robot* :newcoords (copy-object c))
       (send *robot* :angle-vector (copy-seq av))
       (send *viewer* :draw-objects)
       (funcall callback time-step)
       )
   base-coords angle-vector)
  (cond
   (ri
    (print 'ok)
    (read-line)
    (send ri
	  :angle-vector-sequence
	  angle-vector
	  (make-list (length angle-vector)
		     :initial-element
		     (* 1e+3 time-step))))))

#|

(defun test-motion-filter
  nil
  (setq *robot-type* :hrp2jsk)
  (send *robot* :reset-pose)
  (send *robot* :newcoords (make-coords))
  (demo-motion-filter
   :target-q
   (float-vector 17.7639 18.2084 -78.5163 77.986 -23.2486 -5.06223 -17.2259 -17.04 -73.6776 66.3962 -16.3345 3.78576 -43.0 42.0 8.6 7.98004 -41.6274 19.2954 35.0469 -4.68708 2.88048 8.35046 -1.97449 -15.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.)
   :target-b
   (concatenate float-vector
		(scale 1e-3 #F(-128.389 35.513 -82.859))
		(reverse '(0.294 0.319 0.341)))))

(let* ((move-target
	(mapcar
	 #'(lambda (k)
	     (send *robot* k :end-coords))
	 '(:rarm :rleg :lleg :torso)))
       (target-coords
	(mapcar
	 #'(lambda (mt coords)
	     (make-coords
	      :pos (v+ (send mt :worldpos)
		       (send coords :worldpos))
	      :rot (m* (send coords :worldrot)
		       (send mt :worldrot))))
	 move-target
	 (list (make-coords
		:pos (scale +1 (send *robot* :rleg :end-coords :worldpos)))
	       (make-coords)
	       (make-coords)
	       (make-coords
		:pos #F(0 0 -100)))))
       (rotation-axis
	'(nil t t nil))
       )
  (send *robot*
	:fullbody-inverse-kinematics
	target-coords
	:move-target move-target
	:link-list
	(mapcar
	 #'(lambda (mt)
	     (send *robot* :link-list
		   (send mt :parent)))
	 move-target)
	:rotation-axis rotation-axis
	;:target-centroid-pos nil
	:stop 100
	:debug-view :no-message
	:collision-avoidance-link-pair nil
	))

;;hrp2jsknt

(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords))
(send *viewer* :draw-objects)

(send *robot* :torso :waist-p :joint-angle 90)
(send *robot* :rarm :inverse-kinematics
      (make-coords :pos #F(10000 0 0))
      :rotation-axis nil
      :revert-if-fail nil)
(send *robot* :larm :inverse-kinematics
      (make-coords :pos #F(0 10000 0))
      :rotation-axis nil
      :revert-if-fail nil)
(send *viewer* :draw-objects)

(send *robot* :legs :move-end-pos #F(0 0 100))
(send *robot* :legs :crotch-p :joint-angle -90)
(send *robot* :legs :knee-p :joint-angle 10)
(send *robot* :legs :ankle-p :joint-angle 30)
(send *robot* :fix-leg-to-coords (make-coords))
(send *viewer* :draw-objects)

(send *robot* :rarm :inverse-kinematics
      (make-coords :pos #F(0 0 -1000))
      :rotation-axis nil
      :revert-if-fail nil)
(send *robot* :head :look-at #F(0 10000000000 0))
(send *robot* :move-centroid-on-foot
      :both '(:rleg :lleg)
      :debug-view :no-message)

(send *robot* :angle-vector
      (float-vector 4.3623 1.90788 -118.526 68.8727 -0.384729 -2.96897 4.37937 1.92252 -118.459 69.3164 -0.896309 -2.99165 44.0 0.0 44.0 -9.99994 -34.2603 -35.1868 4.89789 0.994618 46.9578 5.79069 -44.8672 15.0 -86.8598 34.1718 17.9421 0.805567 20.8225 33.4117 -3.79171 -15.0))
(send *robot* :fix-leg-to-coords (make-coords))
;;(send *viewer* :draw-objects)
(demo-motion-filter
 :target-q
 (map float-vector #'deg2rad (send *robot* :angle-vector))
 :target-b
 (concatenate
  float-vector
  (scale 1e-3 (send (car (send *robot* :links)) :worldpos))
  (matrix-log
   (send (car (send *robot* :links)) :worldrot))))

 #f(4.3623 1.90788 -118.526 68.8727 -0.384729 -2.96897 4.37937 1.92252 -118.459 69.3164 -0.896309 -2.99165 44.0 0.0 44.0 -9.99994 -34.2603 -35.1868 4.89789 0.994618 46.9578 5.79069 -44.8672 15.0 -86.8598 34.1718 17.9421 0.805567 20.8225 33.4117 -3.79171 -15.0))
