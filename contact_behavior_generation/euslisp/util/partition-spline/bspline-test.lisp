(require "bspline.lisp")

(defun plot-bspline
  (&rest
   args
   &key
   ((:id-max M) 10)
   ((:recursive-order N) 4)
   ((:recursive-cnt _n) N)
   (x-min 0)
   (x-max 1.0)
   (x x-min)
   (plot-cnt 100)
   (bspline
    (instance basic-spline :init
	      :recursive-order N
	      :recursive-cnt _n
	      :id-max M
	      :x-min x-min :x-max x-max))
   (pos-func :calc-coeff-vector)
   (graph
    (progn
      (require "../../util/graph-sample.lisp")
      (setq *bspline-coeff-matrix-hash* (make-hash-table))
      (create-graph
       (send bspline :to-string)
       :size '(640 320)
       :range
       (list (float-vector x-min 0)
	     (float-vector x-max 1.))
       :name-list
       (mapcar
	#'(lambda (id) (format nil "id=~A" id))
	(send-all (send bspline :bspline-element-list) :id))
       :data-list
       (let* ((tm (instance mtimer :init))
	      (cnt x-min)
	      (id -1)
	      dlist xlist)
	 (dotimes (i plot-cnt)
	   (push (+ x-min (/ (* i 1.0 (- x-max x-min))
			     plot-cnt)) xlist)
	   (push (send bspline pos-func (car xlist))
		 dlist))
	 (format t "TIME: ~A~%" (send tm :stop))
	 (reverse
	  (mapcar
	   #'(lambda (hh)
	       (incf id)
	       (map cons
		    #'(lambda (dl x)
			(float-vector x (aref dl id)))
		    dlist xlist))
	   (make-list M))
	  ))
       )))
   &allow-other-keys)
  (if graph
      (send graph :fit-draw))
  graph)

(defun demo-bspline-interpole
  (&key
   (robot
    (cond
     ((and (boundp '*robot*) *robot*) *robot*)
     (t ;;
      (require "package://euslisp/jskeus/irteus/demo/sample-robot-model.l")
      (setq *robot* (instance sample-robot :init))
      ;;(require "package://euslisp/jskeus/irteus/demo/sample-robot-model.lisp")
      ;;(setq *robot* (instance sample-robot :init))
      (objects (list *robot*))
      *robot*)))
   (init (progn
	   (require "package://eus_qp/euslisp/eiquadprog.lisp")
	   (send robot :reset-pose)))
   (jlist (send robot :rarm :joint-list))
   ;;(send robot :joint-list))
   ;;(flatten (send-all (send robot :rarm :links) :joint)))
   (start-av (send-all jlist :joint-angle))
   (end-av (mapcar #'(lambda (j)
		       (+ (send j :min-angle)
			  (random (- (send j :max-angle) (send j :min-angle)))))
		   jlist))
   (id-max 8)
   (recursive-order 4)
   (x-min 0.0)
   (x-max 1.0)
   (bspline
    (mapcar #'(lambda (k)
		(instance basic-spline :init
			  :id-max id-max :recursive-order recursive-order
			  :x-min x-min :x-max x-max))
	    jlist))
   (initial-state
    (instantiate float-vector (* id-max (length bspline))))
   (state-min-vector
    (apply #'concatenate
	   (cons float-vector
		 (mapcar
		  #'(lambda (j) (make-list id-max :initial-element (send j :min-angle)))
		  jlist))))
   (state-max-vector
    (apply #'concatenate
	   (cons float-vector
		 (mapcar
		  #'(lambda (j) (make-list id-max :initial-element (send j :max-angle)))
		  jlist))))
   (equality-matrix-for-start/end-pos
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (send bs :calc-gain-vector-coeff-matrix-from-via-x-list (list 0.0 0.99)))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-pos
    (concatenate float-vector
		 (flatten (map cons #'list start-av end-av))))
   ;;
   (equality-matrix-for-start/end-vel
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (make-matrix
	       2 id-max
	       (mapcar
		#'(lambda (x) (send bs :calc-delta-coeff-vector x :n 1))
		(list 0.0 0.99))))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-vel
    (scale 0 (concatenate float-vector
			  (flatten (map cons #'list start-av end-av)))))
   ;;
   (equality-matrix-for-start/end-acc
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (make-matrix
	       2 id-max
	      (mapcar
	       #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 2))
	       (list 0.0 0.99))))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-acc
    (scale 0 (concatenate float-vector
			  (flatten (map cons #'list start-av end-av)))))
   ;;
   (eval-weight-matrix
    (let* ((mat
	    (matrix-append
	     (mapcar
	      #'(lambda (rate)
		  (matrix-append
		   (mapcar
		    #'(lambda (bs)
			(make-matrix
			 1 id-max
			 (list
			  (scale
			   1e-3
			   (send bs :calc-delta-coeff-vector
				 (+ x-min (* rate (- x-max x-min)))
				 :n 3)))))
		    bspline)
		   '(1 1)))
	      '(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0))
	     '(1 0))))
      (m* (transpose mat) mat)))
   (cnt 30)
   (x-step (/ (- x-max x-min) (* 1.0 cnt)))
   (x-buf x-min)
   (ret (solve-eiquadprog
	 :debug? t
	 :initial-state initial-state
	 :eval-weight-matrix eval-weight-matrix
	 :state-min-vector state-min-vector
	 :state-max-vector state-max-vector
	 :equality-vector
	 (concatenate float-vector
		      equality-coeff-for-start/end-pos
		      equality-coeff-for-start/end-vel
		      equality-coeff-for-start/end-acc)
	 :equality-matrix
	 (matrix-append
	  (list equality-matrix-for-start/end-pos
		equality-matrix-for-start/end-vel
		equality-matrix-for-start/end-acc)
	  '(1 0))
	 ))
   )
  (if (null ret) (setq ret initial-state))
  (format t "   --- ~A x ~A = ~A variables~%" id-max (length start-av) (length initial-state))
  (let* ((retl (list (cons :gain ret))) p dp ddp (id) tau)
    (setq x-buf x-min)
    (while (<= x-buf x-max)
      (setq id 0)
      (mapcar
       #'(lambda (bs js)
	   (list
	    (send js :joint-angle
		  (send bs :calc x-buf (subseq ret id (+ id id-max))))
	    (send js :put :p (send bs :calc x-buf (subseq ret id (+ id id-max))))
	    (send js :put :dp (send bs :calc-delta x-buf (subseq ret id (+ id id-max)) :n 1))
	    (send js :put :ddp (send bs :calc-delta x-buf (subseq ret id (setq id (+ id id-max))) :n 2))
	    ))
       bspline jlist)
      (push (send-all jlist :get :ddp) ddp)
      (push (send-all jlist :get :dp) dp)
      (push (send-all jlist :get :p) p)
      (send *robot* :calc-torque-from-vel-acc
	    :jvv (map float-vector
		      #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
		      (cdr (send robot :links)))
	    :jav (map float-vector
		      #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
		      (cdr (send robot :links))))
      (push (send-all jlist :joint-torque) tau)
      (setq x-buf (+ x-buf x-step))
      (send *viewer* :draw-objects)
      (x::window-main-one)
      (unix:usleep (round (* 0.01 1000 1000))))
    (push (cons :p (reverse p)) retl)
    (push (cons :dp (reverse dp)) retl)
    (push (cons :ddp (reverse ddp)) retl)
    (push (cons :tau (reverse tau)) retl)
    (format t "  [dif] |~A| = ~A~%"
	    (map float-vector #'- end-av (send-all jlist :joint-angle))
	    (norm (map float-vector #'- end-av (send-all jlist :joint-angle))))
    retl
    )
  )

;; (dolist (arm (send *robot* :rarm :links)) (dolist (b (append (send *robot* :head :links) (send *robot* :torso :links))) (print (pqp-collision-distance arm b))))
