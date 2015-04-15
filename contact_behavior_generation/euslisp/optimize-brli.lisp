;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "contact-state.l")
(require "robot-state-data2.l")
(require "robot-param.l")

(require "package://eus_qp/euslisp/eiquadprog.l")
(require "package://eus_qpoases/euslisp/eus-qpoases.l")
(require "math/matrix-util.l")

(require "util/partition-spline/partition-spline.l")

(defvar *log-stream* t)

(defun limb-jacobian
  (&key
   (robot *robot*)
   (limbs '(:rarm :larm :rleg :lleg))
   (limb-coords
    (mapcar
     #'(lambda (k) (send robot k :end-coords))
     limbs))
   (limb-links
    (mapcar
     #'(lambda (c) (send robot :link-list (send c :parent)))
     limb-coords))
   )
  (let* ((Js
	  (mapcar
	   #'(lambda (leg coords link)
	       (send robot :calc-jacobian-from-link-list
		     link
		     :move-target coords
		     :transform-coords (make-coords)
		     :translation-axis '(t)
		     :rotation-axis '(t)))
	   limbs limb-coords limb-links))
	 (ret (make-matrix
	       (apply #'+
		      (mapcar
		       #'(lambda (J)
			   (cdr (assoc 'dim1 (send J :slots))))
		       Js))
	       (* 6 (length Js))))
	 (index 0))
    (mapcar #'(lambda (J)
		(matrix-set ret
			    (transpose J)
			    (* 6 index)
			    (* index (cdr (assoc 'dim1 (send J :slots)))))
		(setq index (+ index 1)))
	    Js)
    ret))

(defun offset-torque
  (&key
   (robot *robot*)
   (limbs '(:rarm :larm :rleg :lleg))
   (limb-coords (mapcar #'(lambda (k) (send robot k :end-coords)) limbs))
   (limb-joints (mapcar #'(lambda (k) (send robot k :joint-list)) limbs)))
  (send robot :torque-vector
	:target-coords limb-coords
	:force-list (mapcar #'(lambda (hoge) #f(0 0 0)) limb-coords)
	:moment-list (mapcar #'(lambda (hoge) #f(0 0 0)) limb-coords))
  (mapcar
   #'(lambda (jl)
       (mapcar
	#'(lambda (j) (send j :joint-torque))
	jl))
   limb-joints))

(defun simple-grasp-force
  (&key
   (robot *robot*)
   (mg (scale (* 1e-3 (send robot :weight)) (scale -1e-3 *g-vec*))) ;;#f(0 0 -9.8)))
   (mgcmg
    (concatenate float-vector
		 mg
		 (v* (scale 1e-3 (send robot :centroid)) mg)))
   (limb-keys '(:rarm :larm :rleg :lleg))
   (limb-coords (mapcar #'(lambda (k) (send robot k :end-coords)) limb-keys))
   (limb-weight (mapcar #'(lambda (k) (if (find k '(:rarm :larm)) 1 3)) limb-keys))
   (weight-matrix
    (let ((W (unit-matrix (* 6 (length limb-coords)))) (index 0))
      (mapcar
       #'(lambda (weight)
	   (matrix-set
	    W
	    (scale-matrix weight (unit-matrix 6))
	    (* 6 index) (* 6 index))
	   (incf index))
       limb-weight)
      W)))
  (transform
   (m*
    weight-matrix
    (pseudo-inverse-loop
     (m*
      (send robot :calc-grasp-matrix
	    (send-all limb-coords :worldpos))
      weight-matrix)))
   (scale -1 mgcmg)))

(defun slip-matrix
  (ux uy uz lx ly)
  (make-matrix 6 6
	       (list (list 0 0 ux 0 0 0)
		     (list 0 0 uy 0 0 0)
		     (list 0 0 0  0 0 0)
		     (list 0 0 ly 0 0 0)
		     (list 0 0 lx 0 0 0)
		     (list 0 0 uz 0 0 0))))

(defun linear-brli-matrix
  (&key
   f0
   limb-local-transform
   limb-slip-matrix
   limb-jacobians
   limb-tmax
   )
  (let* (buf
	 (friction
	  (mapcar
	   #'(lambda (S R f0)
	       (setq buf (transform (m* S R) f0))
	       (setq buf
		     (map float-vector
			  #'(lambda (v) (/ 1.0 (+ v 1e-1)))
			  buf))
	       (setq buf (diag buf))
	       (setf (aref buf 2 2) 0)
	       (m* buf R))
	   limb-slip-matrix
	   limb-local-transform
	   f0))
	 (torque
	  (mapcar
	   #'(lambda (J tmax)
	       (m*
		(diag
		 (map float-vector
		      #'(lambda (v) (/ 1.0 (+ v 1e-1)))
		      tmax))
		J))
	   limb-jacobians
	   limb-tmax)))
    (list torque friction)))

(defun test-qp
  (&rest args)
  (apply #'octave-qp
	 (append args (list :debug? t)))
  (apply #'solve-eiquadprog
	 (append args (list :debug? nil)))
  )

(defun octave-qp
  (&key
   (initial-state #F(0 0 0))
   (eval-weight-matrix (scale-matrix -1 (unit-matrix (length initial-state))))
   (eval-coeff-vector (scale 0 initial-state))
   (equality-matrix #2f())
   (equality-vector #f())
   (state-min-vector (make-list (length initial-state) :initial-element -10))
   (state-max-vector (make-list (length initial-state) :initial-element +10))
   (inequality-matrix
    (make-matrix 1 (length initial-state) (list (make-list (length initial-state) :initial-element 1))))
   (inequality-min-vector #F(-3))
   (inequality-max-vector #F(3))
   (timer (instance mtimer :init))
   (timebuf #F(0))
   (debug? nil)
   &allow-other-keys
   )
  (let ((octave (if (and (boundp '*octave*) *octave*)
		    *octave* (setq *octave* (octave))))
	res)
    (setq
     res
     (send octave
	   :command
	   (apply
	    #'concatenate
	    (cons
	     string
	     (append
	      (cons
	       "qp( "
	       (butlast
		(flatten
		 (mapcar
		  #'(lambda (a)
		      (list
		       (format
			nil "~A"
			(send octave :eus->octave
			      (cond
			       ((or (listp a) (vectorp a))
				(transpose
				 (make-matrix 1 (length a)
					      (list a))))
			       (t a))))
		       ", "))
		  (list initial-state eval-weight-matrix eval-coeff-vector
			equality-matrix equality-vector
			state-min-vector state-max-vector
			inequality-min-vector inequality-matrix inequality-max-vector)))))
	      (list ")"))))))
    ;;(send octave :quit)
    (setf (aref timebuf 0) (send timer :stop))
    (if debug? (format t "TIME: ~A sec~%" (aref timebuf 0)))
    res))

(defun oases-qp
  (&rest args)
  (apply
   #'solve-qpoases
   (apply
    #'append
    (mapcar
     #'(lambda (k)
	 (if (member k args) (list k (cadr (member k args)))))
     (list :initial-state
	   :eval-weight-matrix
	   :eval-coeff-vector
	   :equality-matrix
	   :equality-vector
	   :state-min-vector
	   :state-max-vector
	   :inequality-matrix
	   :inequality-min-vector
	   :inequality-max-vector
	   :inequality-dim
	   :print-level
	   :debug)))))


(defun error-checker
  (name teacher student)
  (format *log-stream* " ~A check " name)
  (let ((buf))
    (cond
     ((and (listp teacher)
	   (listp student)
	   (eq (length teacher) (length student)))
      (setq buf
	    (apply #'+
		   (mapcar #'norm
			   (mapcar #'v- teacher student)))))
     (t (setq buf (norm (v- teacher student)))))
    (if (< buf 1)
	(format *log-stream* "ok~%")
      (format *log-stream* "ng~% o: ~A~% x:~A~%" teacher student))
    (< buf 1)))

(defun optimize-brli
  (&key (robot *robot*)
	(grasp-matrix-full-rank? t)
	(contact-states (now-contact-state))
	(rest-contact-states nil)
	(limbs (send-all contact-states :name))
	(limb-coords (send-all contact-states :contact-coords))
	(limb-n
	 (mapcar
	  #'(lambda (c n)
	      (transform (send c :worldrot) n))
	  limb-coords
	  (send-all contact-states :contact-n)))
	(limb-local-transform (mapcar #'(lambda (cs) (transpose (send cs :z-rot))) contact-states))
	;; (send-all contact-states :z-rot))
	;; (limb-local-transform
	;;  (mapcar
	;;   #'(lambda (n)
	;;       (transpose
	;;        (matrix-exponent
	;; 	(normalize-vector (v* #F(0 0 1) n)) (acos (v. #F(0 0 1) n)))))
	;;   limb-n))
	(limb-local-transform6x6
	 (mapcar #'(lambda (r) (matrix-append (list r r) '(1 1))) limb-local-transform))
	(limb-links
	 (mapcar
	  #'(lambda (c l)
              (send robot :link-list (send c :parent)
		    (if (find-method robot l) (send robot l :root-link))))
	  limb-coords limbs))
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
	(limb-fmax-scale 1.0)
	(limb-fmax
	 (mapcar
	  #'(lambda (cs s) (scale s (send cs :force0)))
	  contact-states
	  (if (listp limb-fmax-scale) limb-fmax-scale
	    (mapcar #'(lambda (cs) limb-fmax-scale) contact-states))
	  ))
	;;(send-all contact-states :force0))
	(limb-jacobians
	 (mapcar
	  #'(lambda (link coord)
	      (transpose
	       (send robot :calc-jacobian-from-link-list
		     link
		     :move-target coord
		     :transform-coords (make-coords)
		     :translation-axis '(t)
		     :rotation-axis '(t))))
	  limb-links limb-coords))
	(limb-tmax-offset (offset-torque :robot robot :limb-coords limb-coords
					 :limb-joints limb-joints))
	(mg (scale (* 1e-3 (send robot :weight)) (scale -1e-3 *g-vec*))) ;;#f(0 0 -9.8)))
	(mgcmg
	 (concatenate float-vector
		      mg
		      (v* (scale 1e-3 (send robot :centroid)) mg)))
	(limb-weight (mapcar #'(lambda (k) (if (find k '(:rarm :larm)) 2 1)) limbs))
	(weight-vector
	 (apply #'concatenate
		(cons float-vector
		      (mapcar #'(lambda (s) (make-list 6 :initial-element s)) limb-weight))))
	(weight-matrix (diag weight-vector))
	(f0
	 (simple-grasp-force :robot robot :mg mg :mgcmg mgcmg
			     :limb-keys limbs :limb-coords limb-coords))
	(limb-forces0 (let ((index 0))
			(mapcar #'(lambda (c)
				    (subseq f0 (* 6 index) (* 6 (incf index))))
				limb-coords)))
	(slip-matrix (copy-object (send-all contact-states :slip-matrix)))
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
	(linear-brli
	 (matrix-append
	  (mapcar
	   #'(lambda (ml) (matrix-append ml '(1 1)))
	   (linear-brli-matrix
	    :f0 ;limb-forces0
	    (mapcar
	     #'(lambda (k r) (transform (transpose r)
					(if (find k '(:rarm :larm))
					    #F(0 0 100 0 0 0)
					    #F(0 0 800 0 0 0))))
	     limbs limb-local-transform6x6)
	    :limb-local-transform limb-local-transform6x6
	    :limb-slip-matrix slip-matrix :limb-jacobians limb-jacobians
	    :limb-tmax limb-tmax))
	  '(1 0)))
	(grasp-matrix
	 (send robot :calc-grasp-matrix
	       (send-all limb-coords :worldpos)))
	(non-eq-mat
	 (matrix-append
	  (append
	   (if (not grasp-matrix-full-rank?)
	       (list grasp-matrix))
	   (list
	    (matrix-append limb-jacobians '(1 1))
	    (matrix-append slip+ '(1 1))
	    (matrix-append slip- '(1 1))))
	  '(1 0)))
	(non-eq-right-vector
	 (apply
	  #'concatenate
	  (cons
	   float-vector
	   (append
	    (if (not grasp-matrix-full-rank?)
		(list (map float-vector #'(lambda (val) (+ val 0.1)) (scale -1 mgcmg))))
	    (mapcar #'(lambda (a b) (mapcar #'+ a b)) limb-tmax-offset limb-tmax)
	    limb-fmax
	    (list (make-list (* 6 (length limbs)) :initial-element 1e+5))
	    ))))
	(non-eq-left-vector
	 (apply
	  #'concatenate
	  (cons
	   float-vector
	   (append
	    (if (not grasp-matrix-full-rank?)
		(list (map float-vector #'(lambda (val) (- val 0.1)) (scale -1 mgcmg))))
	    (mapcar #'(lambda (a b) (mapcar #'- a b)) limb-tmax-offset limb-tmax)
	    (list (make-list (* 6 (length limbs)) :initial-element -1e+5))
	    (mapcar #'(lambda (v) (scale -1 v)) limb-fmax)))))
	(extra-equality-matrix nil)
	(extra-equality-vector nil)
	(eiquadprog-function 'solve-eiquadprog-raw)
	(ret-buf
	 (solve-qp
	  ;; (octave-qp
	  ;; (oases-qp
	  :debug? nil :trial-cnt 10 :ineq-offset 1.0 :ineq-scale 30.0
	  :initial-state f0
	  :eval-weight-matrix (if linear-brli
				  (m* (transpose linear-brli) linear-brli)
				weight-matrix)
	  :eval-coeff-vector (instantiate float-vector (* 6 (length limbs)))
	  :equality-matrix (matrix-append
			    (flatten
			     (list (if grasp-matrix-full-rank? grasp-matrix #2f())
				   extra-equality-matrix))
			    '(1 0))
	  :equality-vector (concatenate
			    float-vector
			    (if grasp-matrix-full-rank? (scale -1 mgcmg) #f())
			    extra-equality-vector)
	  ;; :state-min-vector
	  ;; (coerce (make-list (* 6 (length limbs)) :initial-element -1e+5)
	  ;; 	  float-vector)
	  ;; :state-max-vector
	  ;; (coerce (make-list (* 6 (length limbs)) :initial-element 1e+5)
	  ;; 	  float-vector)
	  :inequality-min-vector non-eq-left-vector
	  :inequality-matrix non-eq-mat
	  :inequality-max-vector non-eq-right-vector
	  :debug? nil
	  :eiquadprog-function eiquadprog-function
	  ))
	(ret (or ret-buf f0))
	 ;; (octave-qp
	 ;;  :f0 f0
	 ;;  :eval-weight-matrix (if linear-brli
	 ;; 			  (m* (transpose linear-brli) linear-brli)
	 ;; 			weight-matrix)
	 ;;  :eval-coeff-vector (instantiate float-vector (* 6 (length limbs)))
	 ;;  :eq-matrix (if grasp-matrix-full-rank? grasp-matrix #2f())
	 ;;  :eq-vector (if grasp-matrix-full-rank? (scale -1 mgcmg) #f())
	 ;;  :f0-min
	 ;;  (coerce (make-list (* 6 (length limbs)) :initial-element -1e+5)
	 ;; 	  float-vector)
	 ;;  :f0-max
	 ;;  (coerce (make-list (* 6 (length limbs)) :initial-element 1e+5)
	 ;; 	  float-vector)
	 ;;  :non-eq-min-vector non-eq-left-vector
	 ;;  :non-eq-matrix non-eq-mat
	 ;;  :non-eq-max-vector non-eq-right-vector
	 ;;  ))
	(extra-constraints-check
	 (cond
	  ((and extra-equality-vector extra-equality-matrix)
	   (format *log-stream* "[optimize brli extra constrains check]~%")
	   (format *log-stream* "  ~A * ~A = ~A~%" extra-equality-matrix ret extra-equality-vector)
	   (let* ((_ret t)
		  (check (map cons
			      #'(lambda (val)
				  (let ((buf (< (abs val) 0.01)))
				    (setq _ret (and _ret buf))
				    (format *log-stream* " ~A(~A)" val buf) buf))
			      (v- (transform extra-equality-matrix ret) extra-equality-vector))))
	     (format *log-stream* "~%")
	     _ret))
	  (t t)))
	(limb-force (let ((index 0))
		      (mapcar #'(lambda (c)
				  (subseq ret (* 6 index) (* 6 (incf index))))
			      limb-coords)))
	(brli-tau-gain 1)
	(torque-check t)
	(rsd (instance robot-state-data2 :init
		       :robot robot
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
		       ;;:torque-vector nil
		       :brli-gain brli-tau-gain
		       :log *log-stream*
		       :torque-check torque-check
		       ))
	(force-list (mapcar #'(lambda (f) (subseq f 0 3)) limb-force))
	(moment-list (mapcar #'(lambda (f) (subseq f 3 6)) limb-force))
	(centroid-acc (v+ (transform grasp-matrix ret) mgcmg))
	(limb-torque
	 (mapcar #'(lambda (J f off)
		     (v- (coerce off float-vector)
			 (transform J f)))
		 limb-jacobians limb-force limb-tmax-offset))
	(limb-friction-cone
	 (mapcar #'(lambda (cs r slip fmax f)
		     (setf (aref slip 2 2)
			   (aref (send cs :fz-max) 2))
		     (setq f (transform r f))
		     (v+ (if (minusp (aref f 2))
			     (scale 0 f)
			   (transform slip f))
			 fmax))
		 contact-states
		 limb-local-transform6x6
		 slip-matrix
		 limb-fmax ;; (send-all contact-states :force0)
		 limb-force))
	(f/fmax (mapcar #'(lambda (f fmax r)
			    (map float-vector
				 #'(lambda (f fmax) (/ f (if (zerop fmax) 1e-5 fmax)))
				 (transform r f) fmax))
			limb-force
			limb-friction-cone
			limb-local-transform6x6))
	(t/tmax (mapcar #'(lambda (tau tmax) (map float-vector #'/ tau tmax))
			limb-torque limb-tmax-for-rsd))
	(ok? (<
	      (apply #'max
		     (mapcar #'abs
			     (flatten
			      (apply #'concatenate
				     (cons cons (append f/fmax t/tmax))))))
	      1.01))
	(brli-force
	 (mapcar #'(lambda (r f) (transform (transpose r) f))
		 limb-local-transform6x6 f/fmax))
	(brli-torque
	 (mapcar #'(lambda (J tau) (transform (pseudo-inverse-loop J) tau))
		 limb-jacobians t/tmax))
	(brli
	 (mapcar #'(lambda (f v) (v- f (scale brli-tau-gain v)))
		 brli-force brli-torque))
	&allow-other-keys
	)
  ;;
  ;;
  (format *log-stream* "[optimize brli calcuration check]~%")
  (error-checker "torque" limb-torque (subseq (send rsd :contact-torque)
					      0 (length contact-states)))
  (error-checker "brli-torque" brli-torque
		 (subseq (send rsd :torque-brli) 0 (length contact-states)))
  (error-checker "brli-friction" brli-force
		 (subseq (send rsd :friction-brli) 0 (length contact-states)))
  (error-checker "brli-diff" brli
		 (subseq (send rsd :brli) 0 (length contact-states)))
  ;;
  (cond
   ((not (eq ok? (send rsd :full-constrainted)))
    (format *log-stream* " warn ~A vs ~A, constraint inconsist~%"
	    ok? (send rsd :full-constrainted))))
  (cond
   ((not extra-constraints-check)
    (format *log-stream* " extrac-constranch-check rejected~%")
    (send rsd :set-val 'full-constrainted nil)))
  (if extra-equality-vector (send rsd :buf :extra-equality-vector extra-equality-vector))
  (if extra-equality-matrix (send rsd :buf :extra-equality-matrix extra-equality-matrix))
  (if (send rsd :full-constrainted)
      (format *log-stream* " ok rsd!!!~%")
    (format *log-stream* " ng rsd...~%"))
  ;; (send rsd :fix-coords)
  rsd
  ;; (list (cons :limb-keys limbs)
  ;; 	(cons :force-list force-list)
  ;; 	(cons :moment-list moment-list)
  ;; 	(cons :limb-force limb-force)
  ;; 	(cons :limb-friction-cone limb-friction-cone)
  ;; 	(cons :f/fmax f/fmax)
  ;; 	(cons :centroid-acc centroid-acc)
  ;; 	(cons :limb-torque limb-torque)
  ;; 	(cons :limb-tmax limb-tmax)
  ;; 	(cons :limb-joints limb-joints)
  ;; 	(cons :t/tmax t/tmax)
  ;; 	(cons :ok? ok?)
  ;; 	(cons :limb-coords limb-coords)
  ;; 	(cons :brli brli)
  ;; 	(cons :brli-force brli-force)
  ;; 	(cons :brli-torque brli-torque)
  ;; 	)
  )

;; (require "euslib/irteus_proposals/motion-lib-proposal.l")
;; (defun optimize-brli
;;   (&key (robot *robot*)
;; 	(contact-states (now-contact-state))
;; 	(rest-contact-states nil)
;; 	(eiquadprog-function 'solve-eiquadprog)
;; 	(contact-coords-list
;; 	 (send-all contact-states :z-coords)) ;; :target-coords))
;; 	(contact-constraint-matrix-list
;; 	 (mapcar #'(lambda (x sm)
;; 		     (calc-constraint-matrices
;; 		      x
;; 		      :mu-trans (aref sm 0 2)
;; 		      :mu-rot (aref sm 5 2)
;; 		      :l-min-x (* -1000 (aref sm 4 2))
;; 		      :l-min-y (* -1000 (aref sm 3 2))
;; 		      :l-max-x (* +1000 (aref sm 4 2))
;; 		      :l-max-y (* +1000 (aref sm 3 2))
;; 		      ))
;; 		 contact-coords-list
;; 		 (send-all contact-states :slip-matrix)))
;; 	(limb-force-moment
;; 	 (progn
;; 	   (send robot :calc-torque :calc-statics-p t)
;; 	   (or
;; 	    (cadr (member :wrench-list
;; 			  (wrench-distribute-from-total-wrench
;; 			   contact-coords-list
;; 			   contact-constraint-matrix-list
;; 			   :qp-solver eiquadprog-function
;; 			   :robot robot)))
;; 	    (send robot :calc-contact-wrenches-from-total-wrench
;; 		  (send-all contact-coords-list :worldpos)))
;; 	   ))
;; 	(limb-force
;; 	 (mapcar #'(lambda (f m) (concatenate float-vector f m))
;; 		 (nth 0 limb-force-moment) (nth 1 limb-force-moment)))
;; 	(ret (apply #'concatenate (cons float-vector limb-force)))
;; 	(brli-tau-gain 1)
;; 	(torque-check t)
;; 	(rsd (instance robot-state-data2 :init
;; 		       :robot robot
;; 		       :contact-states
;; 		       (append contact-states rest-contact-states)
;; 		       :contact-forces
;; 		       (append
;; 			limb-force
;; 			(mapcar #'(lambda (hoge) #F(0 0 0 0 0 0))
;; 				rest-contact-states))
;; 		       :brli-gain brli-tau-gain
;; 		       :log *log-stream*
;; 		       :torque-check torque-check
;; 		       ))
;; 	&allow-other-keys
;; 	)
;;   rsd
;;   )

(defun optimize-brli-neglect-hands
  (&rest args &key (neglect? t) (time-buf) &allow-other-keys)
  (let (buf limb-fmax limb-tmax-rate limbs mtime)
    (setq
     mtime
     (bench2
      (progn
	(setq buf (apply #'optimize-brli args))
	(cond
	 ((and neglect? (not (send buf :full-constrainted)))
	  (format *log-stream* "  vvvvv    optimization error!! neglect hand mode~%")
	  (setq limbs (or (cadr (member :limbs args))
			  '(:rarm :larm :rleg :lleg)))
	  (setq limb-tmax-rate (cadr (member :limb-tmax-rate args)))
	  (setq limb-fmax (cadr (member :limb-fmax args)))
	  (cond
	   (limb-tmax-rate
	    (delete :limb-tmax-rate args)
	    (delete limb-tmax-rate args))
	   (t (setq limb-tmax-rate (mapcar #'(lambda (h) 1.0) limbs))))
	  (cond
	   (limb-fmax
	    (delete :limb-fmax args)
	    (delete limb-fmax args))
	   (t (setq limb-fmax (mapcar #'(lambda (k) #F(1 1 1 1 1 1)) limbs))))
					;      (print args)
	  (setq buf
		(apply #'optimize-brli
		       (append
			(list
			 :limb-tmax-rate
			 (mapcar #'(lambda (k rate)
				     (if t ;(find k '(:rarm :larm))
					 (* 1e+2 rate) rate))
				 limbs limb-tmax-rate))
			(list :limb-tmax-rate-for-rsd limb-tmax-rate)
			(list
			 :limb-fmax
			 (mapcar #'(lambda (k fmax)
				     (if t ;(find k '(:rarm :larm))
					 (scale 1e+2 fmax) fmax))
				 limbs limb-fmax))
			args))))))))
    (format *log-stream* " @ optimize-brli-neglect-hands ~A sec~%" mtime)
    (if (vectorp time-buf) (setf (aref time-buf 0) (+ (aref time-buf 0) mtime)))
    buf))
