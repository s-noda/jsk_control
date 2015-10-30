(require "my-util.lisp")
(require "util/list2file.lisp")
(require "ik-without-collision-org.lisp")
(require "robot-state-data2.lisp")
(require "contact-state.lisp")
(require "optimize-brli.lisp")

(defvar *log-stream* t)

(defun calc-walk-danger-object-function
  (remove) ;; BRLV > thre ?
  (apply #'max
	 (mapcar #'abs
		 (apply #'concatenate
			(flatten (list cons
				       (or (send remove :buf :f/fmax)
					   (send remove :f/fmax))
				       (send remove :t/tmax)))))))

;; (defun calc-walk-danger-object-function
;;   (remove) ;; BRLV > thre ?
;;   (norm2 (apply #'concatenate
;; 		(flatten (list float-vector
;; 			       (or (send remove :buf :f/fmax)
;; 				   (send remove :f/fmax))
;; 			       (send remove :t/tmax))))))

(defun best-rsd
  (rsd-list
   &key (buf (list :best-rsd t)))
  (let (sorted ret)
    (setq ret
	  (or
	   (find-if
	    #'(lambda (a) (send a :full-constrainted))
	    (setq
	     sorted
	     (sort
	      rsd-list
	      #'(lambda (a b) (< (calc-walk-danger-object-function a)
				 (calc-walk-danger-object-function b))))))
	   (car sorted)
	   (car rsd-list)))
    ;;     (setq
    ;;      ret
    ;;      (or
    ;;       (find-if
    ;;        #'(lambda (a) (send a :full-constrainted))
    ;;        (setq
    ;; 	sorted
    ;; ;	(sort
    ;; 	 (remove-if
    ;; 	  #'(lambda (rsd)
    ;; 	      (or (not (eq robot-state-data2 (class rsd)))
    ;; 		  (and
    ;; 		   (send rsd :contact-forces :rarm)
    ;; 		   (> (norm (send rsd :contact-forces :rarm)) 300))
    ;; 		  (and
    ;; 		   (send rsd :contact-forces :larm)
    ;; 		   (> (norm (send rsd :contact-forces :larm)) 300))))
    ;; 	  rsd-list)
    ;; ;	 #'(lambda (a b)
    ;; ;	     (< (norm (send a :tf-error)) (norm (send b :tf-error))))
    ;; 	 ))
    ;;       (car sorted)
    ;;       (car rsd-list)))
    (cond
     ((and ret buf)
      (apply #'send ret :buf buf)))
    (cond
     (ret
      (send *robot* :angle-vector
	    (copy-object (send ret :angle-vector)))
      (send *robot* :newcoords
	    (copy-object (send ret :root-coords)))
      ;; for transaltion/rotation-axis
      ;; (send ret :fix-coords)
      ))
    ret))

(defun barrier-filter
  (vl max &key
      (split (mapcar #'(lambda (v m) (/ v m)) vl max))
      (log-barrier nil))
  (if log-barrier
      (map float-vector
	   #'(lambda (v)
	       (* (if (minusp v) 1 -1)
		  (log (max 0.01 (- 1 (abs v))))))
	   split)
    split))

(defvar *pg-use-toes* '(nil nil))
(defvar *pg-move-max* 13)
(defvar *pg-move-gain* 10000)
(defvar *pg-optional-ik-contact-states* nil)
(defvar *pg-float-cascoords*)
(defvar *pg-check-self-collision*)
(defun pose-generate-with-contact-state ;; ret rsd or nil
  (contact-state
   &key
   (rest-contact-states nil)
   (ik-contact-states
    (union  *pg-optional-ik-contact-states* contact-state))
   (c nil) ;(send *robot* :centroid))
   (f (instantiate float-vector (* 6 (length contact-state))))
   (tf-error #F(0))
   (move-target (send-all ik-contact-states :contact-coords))
   (target-coords (send-all ik-contact-states :target-coords))
   (target-keys (send-all contact-state :name))
   (fix-coords
    (send-all rest-contact-states :contact-coords))
   (fix-target-coords
    (send-all rest-contact-states :target-coords))
   (float-cascoords *pg-float-cascoords*)
   ;;    (mapcar
   ;;     #'(lambda (k) (send *robot* k :end-coords))
   ;;     (set-difference '(:rarm :larm :rleg :lleg) target-keys)))
   (optimize-brli-args)
   (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.5 0.5))
   (keep-centroid-ik-option (make-list (length target-coords)))
   (centroid-thre 1000)
   (centroid-thre-rate 0.1)
   (convergence-thre 1)
   (ik-param nil)
   (ik-debug-view :no-message)
   (depth 0)
   (tau-gain 0.1)
   (tmax-leg-rate 0.39)
   (tmax-hand-rate 0.3)
   (non-stop nil)
   (success? nil)
   (stop 150)
   (ik-time #F(0))
   (qp-time #F(0))
   (best-rsd? t)
   (buf))
  (format *log-stream* "[pose-generate(~A/~A)]~%" depth stop)
  (format *log-stream* " >> tf-error: |~A|=~A~%" tf-error (norm tf-error))
  (cond
   ((> depth stop)
    (format *log-stream* " >> depth-over: ~A~%" depth)
    (if best-rsd? (best-rsd buf) buf))
   ((progn
      (format *log-stream* " >> convergence-check: move=~A~%"
	      (if c (subseq (v- c (send *robot* :centroid)) 0 2) 0))
      (< centroid-thre convergence-thre))
    (format *log-stream* " >> convergence!~%")
    (if best-rsd? (best-rsd buf) buf))
   ((and (zerop depth)
	 (null c)
	 (or (collision-resolve-move)
	     (find-if
	      #'identity
	      (mapcar #'(lambda (cs)
			  (or
			   (> (norm (send cs :difference-position)) 10)
			   (> (norm (send cs :difference-rotation)) (deg2rad 10))))
		      (append rest-contact-states ik-contact-states))))
	 (or
	  (format *log-stream* " >> init-rsd-check: collisoin detect~%")
	  t)
	 (null
	  (ik-trial :trial 4
		    :time-buf ik-time
		    :target
		    (append
		     (mapcar #'(lambda (cs)
				 (list (cons :target (send cs :contact-coords))
				       (cons :coords (send cs :target-coords))
				       (cons :thre 20)
				       (cons :translation-axis
					     (send cs :translation-axis))
				       (cons :rotation-axis (send cs :rotation-axis))
				       ))
			     ik-contact-states)
		     (mapcar #'(lambda (cs fc ftc)
				 (list (cons :target fc)
				       (cons :coords ftc)
				       (cons :translation-axis
					     (send cs :translation-axis))
				       (cons :rotation-axis
					     (send cs :rotation-axis))
				       ))
			     rest-contact-states
			     fix-coords
			     fix-target-coords)
		     (mapcar #'(lambda (cs)
				 (list (cons :target cs)
				       (cons :coords (send cs :copy-worldcoords))
				       (cons :translation-axis nil)
				       (cons :rotation-axis nil)))
			     float-cascoords))
		    :balance-leg nil
		    :target-centroid-pos nil
		    :collision-avoidance-link-pair nil
		    :max #F(500 500 1000 300 300 300)
		    :stop 30
		    :debug-view ik-debug-view
		    :flush (not (null ik-debug-view))
		    :warnp nil)))
    (if best-rsd? (best-rsd buf) buf))
   ((or
     (eq 'lose
	 (keep-centroid-and-avoid-collision-with-time
	  :time-buf ik-time
	  :debug-view ik-debug-view
	  :move-target move-target
	  :limb-target-coords target-coords
	  :float-cascoords float-cascoords
	  :translation-axis (send-all ik-contact-states :translation-axis)
	  :rotation-axis (send-all ik-contact-states :rotation-axis)
	  :fix-coords fix-coords
	  :fix-target-coords fix-target-coords
	  :fix-translation-axis (send-all rest-contact-states :translation-axis)
	  :fix-rotation-axis (send-all rest-contact-states :rotation-axis)
	  :centroid
	  (if (eq c (send *robot* :centroid)) nil c)
	  :centroid-thre centroid-thre
	  :keep-centroid-ik-option keep-centroid-ik-option
	  :ik-param (eval ik-param)
	  :root-link-virtual-joint-weight
	  root-link-virtual-joint-weight
	  ))
     (and *pg-check-self-collision*
	  (send *robot* :self-collision-check)))
    (format *log-stream* " >> collision unresolve: ~A~%" f)
    (if best-rsd? (best-rsd buf) buf))
   (t
    (let* ((opt
	    (apply
	     #'optimize-brli-neglect-hands
	     (append
	      optimize-brli-args
	      (list
	       :time-buf qp-time
	       :neglect? nil
	       :robot *robot*
	       :contact-states contact-state
	       :rest-contact-states rest-contact-states
	       ;; :limbs (send-all contact-state :name)
	       ;; :limb-fmax (send-all contact-state :force0)
	       :brli-tau-gain tau-gain
	       ;; :linear-brli nil
	       ;; :weight-vector
	       ;; (apply #'concatenate
	       ;; 	    float-vector
	       ;; 	    (mapcar
	       ;; 	     #'(lambda (k)
	       ;; 		 (if (find k '(:rarm :larm))
	       ;; 		     '(1 1 1 8 8 8)
	       ;; 		   '(0.1 0.1 0.1 0.1 0.1 0.1)))
	       ;; 	     target-keys))
	       :limb-tmax-rate
	       (mapcar #'(lambda (k) (if (find k '(:rarm :larm))
					 (* 1 tmax-hand-rate)
				       (* 1 tmax-leg-rate)))
		       target-keys)))))
	   (flist (send opt :contact-forces)) ;(cdr (assoc :limb-force opt)))
	   (move (scale *pg-move-gain* (send opt :gbrli-dec-move))))
      (if (send opt :full-constrainted)
	  (format *log-stream* ">>>>>>>>>>>> SUCCESS <<<<<<<<<<<~%")
	(format *log-stream* "!!!!!!!!!!!! FAIL !!!!!!!!!!!!~%"))
      (cond
       ((and (send opt :full-constrainted) (not non-stop))
	(if best-rsd? opt (cons opt buf)))
       (t
	(format *log-stream* " >> move-org = ~A~%" move)
	(if (> (norm move) *pg-move-max*)
	    (setq move (scale *pg-move-max* (normalize-vector move))))
	(pose-generate-with-contact-state
	 contact-state
	 :rest-contact-states rest-contact-states
	 :ik-contact-states ik-contact-states
	 :depth (+ depth 1)
	 :stop stop
	 :move-target move-target
	 :target-coords target-coords
	 :fix-coords fix-coords
	 :fix-target-coords fix-target-coords
	 :c (v+ (or c (send *robot* :centroid))  move)
	 :centroid-thre-rate centroid-thre-rate
	 :centroid-thre (* centroid-thre-rate (norm (subseq move 0 2)))
	 :f (setq f (apply #'concatenate (cons float-vector flist)))
	 :tf-error (send opt :tf-error)
	 :convergence-thre convergence-thre
	 :keep-centroid-ik-option keep-centroid-ik-option
	 :ik-param ik-param
	 :ik-debug-view ik-debug-view
	 :tau-gain tau-gain
	 :target-keys target-keys
	 :optimize-brli-args optimize-brli-args
	 :root-link-virtual-joint-weight root-link-virtual-joint-weight
	 :tmax-hand-rate tmax-hand-rate
	 :tmax-leg-rate tmax-leg-rate
	 :non-stop non-stop
	 :ik-time ik-time
	 :qp-time qp-time
	 :best-rsd? best-rsd?
	 :buf (cons opt buf)
	 )))))))

(defun pose-generate-with-contact-state-with-time
  (&rest args)
  (let (ret mtime buf
	    (ik-time (float-vector 0))
	    (qp-time (float-vector 0)))
    (setq
     mtime
     (bench2
      (setq ret (apply #'pose-generate-with-contact-state
		       (append
			args
			(list :ik-time ik-time
			      :qp-time qp-time))))))
    (format (or *log-stream* t)
	    " @ pose-generate-with-contact-state ~A > ~A + ~A sec~%"
	    mtime (aref ik-time 0) (aref qp-time 0))
    (format (or *log-stream* t)
	    "    | :ik-time ~A%~%" (* 100 (/ (aref ik-time 0) mtime)))
    (format (or *log-stream* t)
	    "    | :qp-time ~A%~%" (* 100 (/ (aref qp-time 0) mtime)))
    ret))

(defun demo-pose-generate
  (&key
   (initial-rsd
    (find-if
     #'(lambda (rsd)
	 (and (eq :reach (send rsd :buf :mseq-mode))
	      (< (norm (send rsd :contact-forces :rleg)) 1)))
     (car (rsd-deserialize))))
   (contact-states (send initial-rsd :contact-states))
   (contact-keys (send-all contact-states :name))
   (stop 100)
   (target-cs
    (flatten
     (mapcar #'(lambda (k)
		 (find-if #'(lambda (cs) (eq k (send cs :name)))
			  contact-states))
	     contact-keys)))
   (initialize-func
    #'(lambda nil
	(simple-fullbody
	 :target
	 (cons
	  (list
	   (cons :target :torso)
	   (cons :move (float-vector (* -1 (random 200.0))
				     (- (* 2 (random 100.0)) 100.0)
				     (- (* 2 (random 25.0)) 25.0))))
	  (mapcar #'(lambda (k) (list (cons :target k)))
		  '(:rarm :larm :rleg :lleg)))
	 :balance-leg nil
	 :target-centroid-pos nil
	 :revert-if-fail nil
	 :debug-view :no-message)))
   (tau-gain 1.0)
   ;;
   graph
   ret
   buf
   )
  (send *robot* :angle-vector
	(copy-object (send initial-rsd :angle-vector)))
  (send *robot* :newcoords
	(copy-object (send initial-rsd :root-coords)))
  (funcall initialize-func)
  (setq
   ret
   (pose-generate-with-contact-state-with-time
    target-cs
    :rest-contact-states
    (set-difference contact-states target-cs)
    :non-stop t
    :stop stop
    :ik-debug-view nil;:no-message
    :centroid-thre-rate 0.5
    :tau-gain tau-gain
    :tmax-leg-rate 0.4
    :tmax-hand-rate 0.4
    :best-rsd? nil))
  ;; graph
  (setq
   graph
   (cons
    (create-graph
     (send :all-limbs :pname)
     :size '(400 400)
     :name-list (list "MAX" "F/Fmax" "TF" "T/Tmax")
     :inc-data-list
     (append
      (list (make-list (length ret) :initial-element 1))
      (let (tmp)
	(setq
	 tmp
	 (mapcar
	  #'(lambda (rsd)
	      (setq buf
		    (list (or (apply #'concatenate float-vector (send rsd :t/tmax))
			      #F(0))
			  (or (apply #'concatenate float-vector (send rsd :f/fmax))
			      #F(0))))
	      (list
	       (/ (+ (norm2 (car buf)) (norm2 (cadr buf)))
		  (+ (length (car buf)) (length (cadr buf))))
	       (/ (norm2 (car buf)) (length (car buf)))
	       (/ (norm2 (cadr buf)) (length (cadr buf)))))
	  (reverse ret)))
	(list (mapcar 'cadr tmp)
	      (mapcar 'car tmp)
	      (mapcar 'caddr tmp))))
     :range (list #F(0 0) (float-vector (length ret) 2)))
    (mapcar #'(lambda (k)
		(create-graph
		 (send k :pname)
		 :size '(400 400)
		 :name-list (list "MAX" "F/Fmax" "TF" "T/Tmax")
		 :inc-data-list
		 (append
		  (list (make-list (length ret) :initial-element 1))
		  (let (tmp)
		    (setq
		     tmp
		     (mapcar
		      #'(lambda (rsd)
			  (setq buf
				(list (or (send rsd :t/tmax k) #F(0))
				      (or (send rsd :f/fmax k) #F(0))))
			  (list
			   (/ (+ (norm2 (car buf)) (norm2 (cadr buf)))
			      (+ (length (car buf)) (length (cadr buf))))
			   (/ (norm2 (car buf)) (length (car buf)))
			   (/ (norm2 (cadr buf)) (length (cadr buf)))))
		      (reverse ret)))
		    (list (mapcar 'cadr tmp)
			  (mapcar 'car tmp)
			  (mapcar 'caddr tmp))))
		 :range (list #F(0 0) (float-vector (length ret) 2))
		 ))
	    contact-keys)))
  (place-graph-in-order)
  (send-all graph :simple-draw-with-line)
  ret)


#|

(let* ((contact-n #F(0 -1 0))
       (contact-coords (send *robot* :rarm :end-coords))
       (n (transform (send contact-coords :worldrot) contact-n))
       (llt
	(matrix-exponent
	 (normalize-vector (v* #F(0 0 1) n))
	 (acos (v. #F(0 0 1) n))))
       (fw #F(1 0 0 0 0 0))
       (fl (concatenate float-vector
			(transform (transpose llt) (subseq fw 0 3))
			(transform (transpose llt) (subseq fw 3 6))))
       (ret (concatenate float-vector
			 (transform llt (subseq fl 0 3))
			 (transform llt (subseq fl 3 6)))))
  (print fw)
  (print fl)
  (print ret))

(demo-pose-generate :contact-keys '(:rarm :larm :rleg))
(load "../../../util/gp-util.lisp")
(graph-panel2gp-graph (cadddr *graph-sample*) :ylabel "NORM" :xlabel "LOOP COUNT" :save? t :ratio 0.7)

(send-all *graph-sample* :set-range #F(0 -2) #F(14 2))
(send-all *graph-sample* :simple-draw-with-line)

(send a :set-data (remove-if #'(lambda (d) (null (find-if #'(lambda (v) (>= (abs (aref v 1)) 1.0)) (send d :data)))) (send a :data)))
(send a :simple-draw-with-line)

 @ pose-generate-with-contact-state 2.67845 > 0.357742 + 2.28113 sec
    | :ik-time 13.3563%
    | :qp-time 85.1659%
