;; (require "my-util.lisp")

(defvar *robot-collision-check-list*
  (list
   (list
    (cons :name :head)
    (cons :move-target :head)
    (cons
     :dist
     '(lambda nil (send *robot* :head :end-coords :worldpos)))
    (cons :min 100))
   (list
    (cons :name :right-shoulder)
    (cons :move-target (send *robot* :rarm :shoulder-r))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :rarm :shoulder-r) :child-link)
	 :worldpos)))
    (cons :min 100))
   (list
    (cons :name :left-shoulder)
    (cons :move-target (send *robot* :larm :shoulder-r))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :larm :shoulder-r) :child-link)
	 :worldpos)))
    (cons :min 100))
   (list
    (cons :name :right-elbow)
    (cons :move-target (send *robot* :rarm :elbow-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :rarm :elbow-p) :child-link)
	 :worldpos)))
    (cons :min 120))
   (list
    (cons :name :left-elbow)
    (cons :move-target (send *robot* :larm :elbow-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :larm :elbow-p) :child-link)
	 :worldpos)))
    (cons :min 120))
   (list
    (cons :name :right-knee)
    (cons :move-target (send *robot* :rleg :knee-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :rleg :knee-p) :child-link)
	 :worldpos)))
    (cons :min 140))
   (list
    (cons :name :left-knee)
    (cons :move-target (send *robot* :lleg :knee-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :lleg :knee-p) :child-link)
	 :worldpos)))
    (cons :min 140))
   ))

(defvar *env-collision-check-list*)

(defun 3pos-area-distance
  (x a0 a1 a2)
  (* -1 (v. (normalize-vector (v* (v- a1 a0) (v- a2 a0)))
	    (v- x a0))))

(defun check-point-collision
  (p)
  (let* ((ccl (list (cons :dist (list 'lambda '(&rest args) p))
		    (cons :min 30))))
    (flatten
     (mapcar
      #'(lambda (env)
	  (let (d)
	    (setq d (if (functionp (cdr (assoc :check-func env)))
			(funcall (cdr (assoc :check-func env)) env ccl)))
	    (setq d
		  (-
		   (if (numberp d) d
		     (* -1
			(v. (cdr (assoc :n env))
			    (v- (funcall (cdr (assoc :dist ccl)))
				(cdr (assoc :a0 env))))))
		   (cdr (assoc :min ccl))))
	    (and (minusp d) (>= (abs d) 5))))
      *env-collision-check-list*))))

(defun calc-distance-for-collision-check
  (&key (log-head "    ") ((:min m) 5))
  (apply
   #'append
   (mapcar
    #'(lambda (env)
	(mapcar
	 #'(lambda (ccl)
	     (list
	      (cons :env (cdr (assoc :name env)))
	      (cons :robot (cdr (assoc :name ccl)))
	      (cons
	       :distance
	       (let (d)
		 (setq d (if (functionp (cdr (assoc :check-func env)))
			     (funcall (cdr (assoc :check-func env)) env ccl)))
		 (setq d
		       (-
			(if (numberp d) d
			  (* -1
			     (v. (cdr (assoc :n env))
				 (v- (funcall (cdr (assoc :dist ccl)))
				     (cdr (assoc :a0 env))))))
			(cdr (assoc :min ccl))))
		 (if (and (minusp d) (>= (abs d) m))
		     (format
		      *log-stream*
		      "~A[calc-distance] collision detected ~A bet ~A & ~A~%"
		      log-head d (cdr (assoc :name env))
		      (cdr (assoc :name ccl))))
		 d))))
	 *robot-collision-check-list*))
    *env-collision-check-list*)))

(defun collision-resolve-move
  (&optional (obj (calc-distance-for-collision-check))
	     (head-min-dist *inf*)
	     (head-min-dir nil)
	     (min 5)
	     (max 30))
  (cond
   ((null obj)
    (send *robot* :head :look-at
	  (v+ (send *robot* :head :end-coords :worldpos)
	      (scale 1000 #F(1 0 0)))) ;head-min-dir)))
    nil)
   ((plusp (cdr (assoc :distance (car obj))))
    (collision-resolve-move (cdr obj)))
   ((< (abs (cdr (assoc :distance (car obj)))) min)
    (collision-resolve-move (cdr obj)))
   (t
    (let* ((robot (cdr (assoc :robot (car obj))))
	   (env (cdr (assoc :env (car obj))))
	   (mt (cdr (assoc
		     :move-target
		     (find-if #'(lambda (a) (eq robot (cdr (assoc :name a))))
			      *robot-collision-check-list*))))
	   (dist
	    (*
	     (if (minusp (cdr (assoc :distance (car obj)))) -1 1)
	     (min
	      max
	      (abs (cdr (assoc :distance (car obj)))))))
	   (n (cdr (assoc
		    :n
		    (find-if #'(lambda (a) (eq env (cdr (assoc :name a))))
			     *env-collision-check-list*))))
	   (move (scale dist n))
	   )
      (format *log-stream* "[collision resolve move] ~A move ~A~%" robot move)
      (if (and (eq mt :head) (< dist head-min-dist))
	  (progn
	    (setq head-min-dist dist)
	    (setq head-min-dir n)))
      (cons
       (list
	(cons :target
	      (cdr
	       (assoc
		:move-target
		(find-if #'(lambda (a) (eq (cdr (assoc :name a))
					   (cdr (assoc :robot (car obj)))))
			 *robot-collision-check-list*))))
	(cons :translation-axis
	      (normalize-vector
	       (map float-vector
		    #'*
		    move
		    '(1 1 2))))
	(cons :rotation-axis nil)
	(cons :thre min)
	(cons :move move))
       (collision-resolve-move
	(cdr obj)
	head-min-dist
	head-min-dir
	min
	))))))

(defvar *ik-trial-user-check-func*)
(defun ik-trial
  (&rest args &key (trial 4) (time-buf #F(0)) &allow-other-keys)
  (let (ret col mtime
	    (ik-param (cadr (member :target args))))
    (setq
     mtime
     (bench2
      (format *log-stream* "[ik trial]~%")
      (while
	  (and (not (minusp (decf trial)))
	       (or
		(null
		 (setq ret (apply #'simple-fullbody
				  (append
				   *kca-ik-param*
				   args))))
		(collision-resolve-move)))
	(format *log-stream* "~A~%" trial)
	;;      (pull-torso :torso-pull-cnt 1)
	(apply #'simple-fullbody
	       (append *kca-ik-param* (list :revert-if-fail nil) args))
	(apply
	 #'keep-centroid-and-avoid-collision
	 (append
	  (list :ik-target (cadr (member :target args))
		:centroid nil :stop 2
		:ik-param (list :revert-if-fail nil))
	  args)))))
    (setf (aref time-buf 0) (+ (aref time-buf 0) mtime))
    (if (functionp *ik-trial-user-check-func*)
	(funcall *ik-trial-user-check-func* ret) ret)))

(defvar *kca-cog-gain* 7)
(defvar *kca-ik-param*)
(defun keep-centroid-and-avoid-collision
  (&rest
   args
   &key
   (limb-target-coords
    (mapcar #'(lambda (k) (send *robot* k :end-coords :copy-worldcoords))
	    '(:rarm :larm :rleg :lleg)))
   (centroid (copy-seq (send *robot* :centroid)))
   (centroid-thre 1e+3) ; min 1
   (move-target
    (mapcar #'(lambda (k) (send *robot* k :end-coords))
	    '(:rarm :larm :rleg :lleg)))
   (translation-axis (make-list (length move-target) :initial-element t))
   (rotation-axis (make-list (length move-target) :initial-element t))
   (float-cascoords nil)
   (fix-coords nil)
   (fix-target-coords (mapcar #'(lambda (c) (copy-object (send c :worldcoords)))
			      fix-coords))
   (depth 0)
   (stop 5)
   (log-head "  ")
   (keep-centroid-ik-option (make-list (length limb-target-coords)))
   (root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.5 0.5))
   (ik-param nil)
   (debug-view :no-message)
   ;;
   (ik-target
    (append
     (mapcar #'(lambda (mt tc ta ra at)
		 (append
		  (list (cons :target mt)
			(cons :coords tc)
			(cons :translation-axis ta)
			(cons :rotation-axis ra))
		  at))
	     move-target
	     limb-target-coords
	     translation-axis
	     rotation-axis
	     keep-centroid-ik-option)
     (mapcar #'(lambda (fc ftc)
		 (list (cons :target fc)
		       (cons :coords ftc)))
	     fix-coords
	     fix-target-coords)
     (mapcar #'(lambda (cs)
		 (list (cons :target cs)
		       (cons :coords (send cs :copy-worldcoords))
		       (cons :translation-axis nil)
		       (cons :rotation-axis nil)))
	     float-cascoords)))
   &allow-other-keys
   )
  (format *log-stream* "~A[avoid-collision(~A/~A)~%" log-head depth stop)
  (let ((collision (collision-resolve-move)))
    (cond
     ((> depth stop)
      (format *log-stream* "~A~%" 'deep-depth) 'lose)
     ((and (null collision)
	   (or (null centroid)
	       (send *robot* :cog-convergence-check
		     centroid-thre centroid)))
      (format *log-stream* "~Awin (^^)/~%" log-head) 'win)
     ((null
       (progn
	 (apply
	  #'simple-fullbody
	  (append
	   *kca-ik-param*
	   (list
	    :target
	    (append ik-target collision)
	    :debug-view debug-view
	    :flush (not (null debug-view))
	    :warnp nil
	    :collision-avoidance-link-pair nil
	    :target-centroid-pos centroid
	    :centroid-thre centroid-thre
	    :use-toes (if (and (boundp '*pg-use-toe*) *pg-use-toe*)
			  *pg-use-toe* '(nil nil))
	    :cog-gain *kca-cog-gain*
	    :balance-leg nil
	    :root-link-virtual-joint-weight root-link-virtual-joint-weight
	    :stop 50
	    :max #F(500 500 1000 300 300 300)
	    :min #F(-500 -500 -1000 -300 -300 -300))
	   ik-param))))
      (format *log-stream* "~Alose orz~%" log-head) 'lose)
     (t
      (apply #'keep-centroid-and-avoid-collision :depth (+ depth 1) args)))))

(defun keep-centroid-and-avoid-collision-with-time
  (&rest args &key time-buf &allow-other-keys)
  (let (buf ret)
    (setq
     buf
     (bench2
      (setq ret (apply #'keep-centroid-and-avoid-collision args))))
    (format *log-stream* " @ keep-centroid-and-avoid-collision ~A sec~%"
	    buf)
    (if (vectorp time-buf) (setf (aref time-buf 0) (+ buf (aref time-buf 0))))
    ret))
