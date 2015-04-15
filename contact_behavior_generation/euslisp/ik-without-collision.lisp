;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

;; (require "my-util.l")
(require "util/list2file.l")
(require "ik-without-collision-org.l")
(require "robot-state-data2.l")
(require "contact-state.l")
(require "optimize-brli.l")
(require "robot-param.l")

(defvar *log-stream* t)

(defvar *robot-collision-check-link-list*
  (append
   (mapcar
    #'(lambda (l)
	(list (cons :link l)))
    (send *robot* :links))
   (if (find-method (send *robot* :rarm) :hand)
       (apply
	#'append
	(mapcar
	 #'(lambda (k)
	     (mapcar
	      #'(lambda (l)
		  (list (cons :link l)
			(cons :parent (send *robot* k :end-coords :parent))))
	      (send *robot* k :hand :links)))
	 '(:rarm :larm))))))

(defvar *sample-obj* (make-cube 10 10 10))
(defvar *env-collision-check-link-list*
  (list
   (list
    (cons :link *sample-obj*)
    (cons :centroid-dot-normal
	  (mapcar
	   #'(lambda (f)
	       (let ((buf
		      (union
		       (send f :vertices)
		       nil)))
		 (list
		  (scale (/ 1.0 (length buf))
			 (reduce #'v+ buf))
		  (normalize-vector
		   (v*
		    (v- (nth 1 buf) (nth 0 buf))
		    (v- (nth 2 buf) (nth 0 buf)))))))
	   (send *sample-obj* :faces)))
    (cons :dist-func
	  #'(lambda (rb env)
	      (let* ((rbl (cdr (assoc :link rb)))
		     (evl (cdr (assoc :link env)))
		     (dst (pqp-collision-distance rbl evl))
		     (evp (nth 2 dst))
		     ret)
		(cond
		 ((< (car dst) 1)
		  (setq ret
			(append
			 (car
			  (sort
			   (cdr (assoc :centroid-dot-normal env))
			   #'(lambda (a b)
			       (< (norm2 (v- (car a) evp))
				  (norm2 (v- (car b) evp))))))
			 dst))))
		ret))))))

(defun collision-resolve-move
  (&key
   (assoc? nil))
  (remove
   nil
   (apply
    #'append
    (mapcar
     #'(lambda (env)
	 (mapcar
	  #'(lambda (rb)
;	      (mapcar
;	       #'(lambda (d)
;		   (if (eq :collision-resolve-move (send d :name))
;		       (send (cdr (assoc :link rb)) :dissoc d)))
;	       (send (cdr (assoc :link rb)) :descendants))
	      (let (buf)
		(setq buf
		      (funcall
		       (cdr (assoc :dist-func env))
		       rb env))
		(cond
		 (buf
		  (list
		   (cons :target
			 (make-cascoords
			  :init :link-list
			  :name :collision-resolve-move
			  :parent (if assoc?
				      (or (cdr (assoc :parent rb))
					  (cdr (assoc :link rb))))
			  :coords (make-coords :pos (nth 3 buf))))
		   (cons :translation-axis (nth 1 buf))
		   (cons :rotation-axis nil)
		   (cons :thre (* 0.9 10))
		   (cons :move (scale 10 (nth 1 buf)))))
		 (t nil))))
	  *robot-collision-check-link-list*))
     *env-collision-check-link-list*))))

(defun dissoc-collisions
  (&key
   (ik-param (collision-resolve-move :assoc? t)))
  (mapcar
   #'(lambda (l)
       (let ((c (cdr (assoc :target l))))
	 (if (and (class c)
		  (subclassp (class c) cascaded-coords)
		  (send c :parent))
	     (send (send c :parent) :dissoc c))))
   ik-param))

(defvar *kca-cog-gain* 7)
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
	     fix-target-coords)))
   &allow-other-keys
   )
  (format *log-stream* "~A[avoid-collision(~A/~A)~%" log-head depth stop)
  (let ((collision (collision-resolve-move :assoc? t)))
    (cond
     ((> depth stop)
      (dissoc-collisions :ik-param collision)
      (format *log-stream* "~A~%" 'deep-depth) 'lose)
     ((and (null collision)
	   (or (null centroid)
	       (send *robot* :cog-convergence-check
		     centroid-thre centroid)))
      (dissoc-collisions :ik-param collision)
      (format *log-stream* "~Awin (^^)/~%" log-head) 'win)
     ((null
       (progn
	 (apply
	  #'simple-fullbody
	  (append
	   (list
	    :target
	    (append ik-target collision)
	    :debug-view debug-view
	    :flush (not (null debug-view))
	    :warnp nil
	    :collision-avoidance-link-pair nil
	    :target-centroid-pos centroid
	    :centroid-thre centroid-thre
	    :cog-gain *kca-cog-gain*
	    :balance-leg nil
	    :root-link-virtual-joint-weight root-link-virtual-joint-weight
	    :stop 50 ;150;500
	    :max #F(500 500 1000 300 300 300)
	    :min #F(-500 -500 -1000 -300 -300 -300))
	   ik-param))))
      (dissoc-collisions :ik-param collision)
      (format *log-stream* "~Alose orz~%" log-head) 'lose)
     (t
      (dissoc-collisions :ik-param collision)
      (apply #'keep-centroid-and-avoid-collision :depth (+ depth 1) args)))))
