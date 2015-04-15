
;; calc 3d spline
;; not consider calc time

(defun matrix-set
  (mom tar &optional (x 0) (y 0))
  (dotimes (dy (cdr (assoc 'dim0 (send tar :slots))))
    (dotimes (dx (cdr (assoc 'dim1 (send tar :slots))))
      (setf (aref mom (+ y dy) (+ x dx)) (aref tar dy dx))))
  mom)

(defun diagT-abst
  (time n &key
	(d 3)
	(ret (make-matrix d (* d n)))
	(depth 0)
	(depth-func '(lambda (d) 1))
	(now 1)
	(x 0))
  (cond
   ((>= x (* d n)) ret)
   (t
    (matrix-set
     ret
     (scale-matrix
      (* now (funcall depth-func depth))
      (unit-matrix d))
     x 0)
    (diagT-abst time n
		:d d
		:ret ret
		:depth (+ depth 1)
		:depth-func depth-func
		:now (* now time)
		:x (+ x d)))))

(defun diagT
  (time n &key (d 3))
  (diagT-abst time n :d d))

(defun diagDT
  (time n &key (d 3))
  (diagT-abst
   time n
   :d d
   :depth 1
   :depth-func
   '(lambda (d) d)
   :x d))

(defun diagDDT
  (time n &key (d 3))
  (diagT-abst
   time n
   :d d
   :depth 2
   :depth-func
   '(lambda (d) (* d (- d 1)))
   :x (* 2 d)))

(defun diagDDDT
  (time n &key (d 3))
  (diagT-abst
   time n
   :d d
   :depth 3
   :depth-func
   '(lambda (d) (* d (- d 1) (- d 2)))
   :x (* 3 d)))

(defun calc-spline
  (coeff time
	 &optional
	 (d 3)
	 (diag-func 'diagT))
  (transform
   (funcall diag-func
	    time
	    (/ (length coeff) d)
	    :d d)
   coeff))

(defun vector-diag
  (vec)
  (let* ((index -1)
	 (m (flatten
	     (map cons
		  #'(lambda (v)
		      (incf index)
		      (if v
			  (let ((ret (instantiate float-vector (length vec))))
			    (setf (aref ret index) v)
			    ret)
			nil))
		  vec))))
    (make-matrix (length m) (length vec) m)))

(defun solve-spline
  (&key
   (time-list '(0 0.5 1))
   (p (list (list (cons :vector #F(0 0 0)))
	    (list (cons :vector #F(100 100 0)))
	    (list (cons :vector #f(100 100 100)))))
   (dp (make-list (length p)))
   (ddp (make-list (length p)))
   (dddp (make-list (length p)))
   (d
    (length
     (find-if
      #'vectorp
      (flatten (append p dp ddp dddp)))))
;   (n (length (flatten (mapcar #'(lambda (a) (if a t nil)) (append p dp ddp)))))
   (row
    (apply
     #'+
     (mapcar #'(lambda (a)
		 (cond
		  ((cdr (assoc :matrix a))
		   (cdr (assoc 'dim0 (send (cdr (assoc :matrix a)) :slots))))
		  ((cdr (assoc :vector a)) d)
		  (t 0)))
	     (append p dp ddp dddp))))
   (n (ceiling (/ row (* 1.0 d))))
   (col (* d n))
   &allow-other-keys
   )
  (let ((A (make-matrix row col))
	(index 0))
    (mapcar
     #'(lambda (m)
	 (matrix-set A m 0 index)
	 (setq index (+ index (cdr (assoc 'dim0 (send m :slots))))))
     (flatten
      (mapcar #'(lambda (p-list func)
		  (mapcar
		   #'(lambda (a time)
		       (if (cdr (assoc :vector a))
			   (m*
			    (or (cdr (assoc :matrix a))
				(unit-matrix d))
			    (funcall func time n :d d))))
		   p-list time-list))
	      (list p dp ddp dddp)
	      (list 'diagT 'diagDT 'diagDDT 'diagDDDT)
	      )))
    (transform
     (pseudo-inverse A)
     (apply
      #'concatenate
      (cons float-vector
	    (flatten
	     (append
	      (mapcar #'(lambda (a)
			  (if (cdr (assoc :vector a))
			      (transform
			       (or (cdr (assoc :matrix a))
				   (unit-matrix (length (cdr (assoc :vector a)))))
			       (cdr (assoc :vector a))))) p)
	      (mapcar #'(lambda (a)
			  (if (cdr (assoc :vector a))
			      (transform
			       (or (cdr (assoc :matrix a))
				   (unit-matrix (length (cdr (assoc :vector a)))))
			       (cdr (assoc :vector a))))) dp)
	      (mapcar #'(lambda (a)
			  (if (cdr (assoc :vector a))
			      (transform
			       (or (cdr (assoc :matrix a))
				   (unit-matrix (length (cdr (assoc :vector a)))))
			       (cdr (assoc :vector a))))) ddp)
	      (mapcar #'(lambda (a)
			  (if (cdr (assoc :vector a))
			      (transform
			       (or (cdr (assoc :matrix a))
				   (unit-matrix (length (cdr (assoc :vector a)))))
			       (cdr (assoc :vector a))))) dddp)
	      )))))))

(defun solve-hoffarbib
  (&key
   (time-list '(0 0.5 1))
   (d 3)
   (p (list (list (cons :vector #F(0 0 0)))
	    (list (cons :vector #F(100 100 0)))
	    (list (cons :vector #f(100 100 100)))))
   (dp (make-list (length p)))
   (ddp (make-list (length p)))
					;   (n (length (flatten (mapcar #'(lambda (a) (if a t nil)) (append p dp ddp)))))
   (row
    (apply
     #'+
     (mapcar #'(lambda (a)
		 (cond
		  ((cdr (assoc :matrix a))
		   (cdr (assoc 'dim0 (send (cdr (assoc :matrix a)) :slots))))
		  ((cdr (assoc :vector a)) d)
		  (t 0)))
	     (append p dp ddp))))
   (n (max 7
	   (+ 2 (* 4 (ceiling (/ (/ row (* 1.0 d)) 4.0))))))
   (min-cnt 7);(ceiling (/ n 2.0)))
   (col (* d n))
   )
  (let ((A (make-matrix row col))
	(At)
	(DD (make-matrix (+ (* d min-cnt) (* 1 d)) col))
	(DtD)
	(ADAt)
	(b)
	(ret)
	(index))
    (setq index 0)
;; calc minimize matrix
#|
    (mapcar
     #'(lambda (m)
	 (matrix-set DD m 0 index)
	 (setq index (+ index (cdr (assoc 'dim0 (send m :slots))))))
     (let ((i -1)
	   (step (/ 1.0 (* n 2))))
       (mapcar
	#'(lambda (hoge) (diagDDDT (* step (incf i)) n :d d))
	(make-list (* n 2)))))
|#
    (matrix-set DD (unit-matrix (* 1 d)) 0 0)
    (setq index (+ index (* 1 d)))
    (mapcar
     #'(lambda (m)
	 (matrix-set DD m 0 index)
	 (setq index (+ index (cdr (assoc 'dim0 (send m :slots))))))
     (let ((i 0)
	   (step (/ 1.0 (+ min-cnt 1))))
       (flatten
	(mapcar
	 #'(lambda (hoge)
	     (list
;	      (diagT (* step (incf i)) n :d d)
;	      (diagDT (* step (incf i)) n :d d)
;	      (diagDDT (* step i) n :d d)
	      (diagDDDT (* step (incf i)) n :d d)
	      ))
	 (make-list min-cnt)))))
;;
    (setq DtD (m* (transpose DD) DD))
;    (if (null (lu-decompose DtD)) (print 'DtD-rank-error))
    (setq DtD
	  (if (lu-decompose (copy-object DtD))
	      (inverse-matrix DtD)
	    (pseudo-inverse DtD)))
;    (setq DtD (unit-matrix (cdr (assoc 'dim0 (send DtD :slots)))))
;; calc condition matrix
    (setq index 0)
    (mapcar
     #'(lambda (m)
	 (matrix-set A m 0 index)
	 (setq index (+ index (cdr (assoc 'dim0 (send m :slots))))))
     (flatten
      (mapcar #'(lambda (p-list func)
		  (mapcar
		   #'(lambda (a time)
		       (if (cdr (assoc :vector a))
			   (m*
			    (or (cdr (assoc :matrix a))
				(unit-matrix d))
			    (funcall func time n :d d))))
		   p-list time-list))
	      (list p dp ddp)
	      (list 'diagT 'diagDT 'diagDDT)
	      )))
    (setq At (transpose A))
    (setq ADAt (m* A (m* DtD At)))
    (setq hoge (list At DtD ADAt))
;    (if (null (lu-decompose ADAt)) (print 'ADAt-rank-error))
;; calc condition vector
    (setq b
     (apply
      #'concatenate
      (cons float-vector
	    (flatten
	     (append
	      (mapcar #'(lambda (a)
			  (if (cdr (assoc :vector a))
			      (transform
			       (or (cdr (assoc :matrix a))
				   (unit-matrix (length (cdr (assoc :vector a)))))
			       (cdr (assoc :vector a))))) p)
	      (mapcar #'(lambda (a)
			  (if (cdr (assoc :vector a))
			      (transform
			       (or (cdr (assoc :matrix a))
				   (unit-matrix (length (cdr (assoc :vector a)))))
			       (cdr (assoc :vector a))))) dp)
	      (mapcar #'(lambda (a)
			  (if (cdr (assoc :vector a))
			      (transform
			       (or (cdr (assoc :matrix a))
				   (unit-matrix (length (cdr (assoc :vector a)))))
			       (cdr (assoc :vector a))))) ddp)
	      )))))
    (setq ret (transform (m* (m* DtD At)
			     (if (lu-decompose (copy-object ADAt))
				 (inverse-matrix ADAt)
			       (pseudo-inverse ADAt)))
			 b))
;    (print (m* A (m* (m* DtD At) (pseudo-inverse ADAt))))
;    (print (m* A (m* (m* DtD At) (pseudo-inverse (m* A (m* DtD At))))))
    (if (> (norm (v- b (transform A ret))) 1) (print 'not-convergence))
    ret
    ))

(defun zz-spline
  (&key
   (start-f #F(0))
   (start-tm 0)
   (end-f #F(1))
   (end-tm 1))
  (solve-spline
   :time-list (list start-tm end-tm)
   :p (list (list (cons :vector start-f))
	    (list (cons :vector end-f)))
   :dp (list (list (cons :vector (scale 0 start-f)))
	     (list (cons :vector (scale 0 end-f))))
   :ddp (list (list (cons :vector (scale 0 start-f)))
	      (list (cons :vector (scale 0 end-f))))))

(defun zz-coords-spline
  (&key
   (coords-list (list (make-coords) (make-coords :pos #F(1 0 0))))
   (time-list (list 0.0 1.0))
   (cnt 30)
   (step (/ (- (nth (- (length time-list) 1) time-list)
	       (nth 0 time-list))
	    (* 1.0 cnt)))
   ;;
   (6dof-list
    (mapcar #'(lambda (c)
		(concatenate float-vector
			     (send c :worldpos)
			     (matrix-log (send c :worldrot))))
	    coords-list))
   (pos-list (mapcar #'(lambda (p) (list (cons :vector p)))
		     6dof-list))
   (vel-list (append
	      (list (list (cons :vector (instantiate float-vector 6))))
	      (make-list (- (length 6dof-list) 2) :initial-element nil)
	      (list (list (cons :vector (instantiate float-vector 6))))))
   (acc-list (append
	      (list (list (cons :vector (instantiate float-vector 6))))
	      (make-list (- (length 6dof-list) 2) :initial-element nil)
	      (list (list (cons :vector (instantiate float-vector 6))))))
   ret
   )
  (let* ((param
	  (solve-spline
	   :time-list time-list
	   :p pos-list :dp vel-list :ddp acc-list)))
    (dotimes (i (+ cnt 1))
      (push (let ((v (calc-spline param (* i step) 6)))
	      (make-coords :pos (subseq v 0 3)
			   :rot (matrix-exponent (subseq v 3 6))))
	    ret))
    (reverse ret)))


(defun demo-spline
  (&key
   ;; constrains
   (time-list (list 0 0.5 1.5 2))
   (p-list (list #F(0 0 0) #F(20 10 -100)
		 #F(10 120 100) #F(100 100 0)))
   (dp-list (mapcar #'(lambda (hoge) nil) p-list))
   (ddp-list (mapcar #'(lambda (hoge) nil) p-list))
   ;; return
   (time-step 0.01)
   (spline-coeff-func (list 'solve-spline
			    'solve-hoffarbib))
   (debug? t)
   (n nil)
   )
  (let* ((coeff
	  (mapcar
	   #'(lambda (scf)
	       (apply scf
		      (append
		       (list :time-list time-list)
		       (list :p
			     (mapcar
			      #'(lambda (v)
				  (cond
				   ((vectorp v) (list (cons :vector v)))))
			      p-list))
		       (list :dp
			     (mapcar
			      #'(lambda (v)
				  (cond
				   ((vectorp v) (list (cons :vector v)))))
			      dp-list))
		       (list :ddp
			     (mapcar
			      #'(lambda (v)
				  (cond
				   ((vectorp v) (list (cons :vector v)))))
			      ddp-list))
		       (if n (list :n n))
		       (list :d
			     (length
			      (find-if
			       #'vectorp
			       (append p-list dp-list ddp-list)))))))
	   spline-coeff-func))
	 (timer (car time-list))
	 (end-time (car (last time-list)))
	 ret
	 sphere
	 obj
	 )
    (while (<= timer end-time)
      ;(if debug? (print timer))
      (push (mapcar #'(lambda (cf) (calc-spline cf timer)) coeff) ret)
      (setq timer (+ timer time-step)))
    (cond
     (debug?
      (dolist (pos ret)
	(setq sphere (mapcar #'(lambda (hoge) (make-sphere 1)) pos))
	(mapcar
	 #'(lambda (sph col pos)
	     (send sph :set-color col)
	     (send sph :newcoords (make-coords :pos pos)))
	 sphere (list #F(1 0 0) #F(0 1 0) #F(0 0 1)) pos)
	(push sphere obj)
	)
      (objects (flatten obj))))
    (reverse ret)))

(defun demo-view-spline
  (&rest
   args
   &key
   (time-list '(0 0.2 0.4 0.6 0.8 1))
   (p (list (list (cons :vector #F(0 0)))
	    (list (cons :vector #F(100 100)))
	    (list (cons :vector #f(100 0)))
	    (list (cons :vector #f(200 100)))
	    (list (cons :vector #f(300 0)))
	    (list (cons :vector #f(400 100)))
	    ))
   (dp (make-list (length p)))
   (ddp (make-list (length p)))
   (dddp (make-list (length p)))
   (d
    (length
     (find-if
      #'vectorp
      (flatten (append p dp ddp dddp)))))
   ;;
   (time-step 0.01)
   ;;
   (name-list (list "pos" "vel" "acc" "grk"))
   (data-list)
   &allow-other-keys
   )
  (require "graph-sample.l")
  (let ((coeff
	 (apply #'solve-spline
		(append
		 (list :time-list time-list)
		 (list :p p)
		 (list :dp dp)
		 (list :ddp ddp)
		 (list :dddp dddp)
		 (list :d d)
		 args)))
	(timer (car time-list))
	(end (car (last time-list)))
	pos vel acc grk
	max
	)
    (while (< timer end)
      (push
       (calc-spline coeff timer d 'diagT)
       pos)
      (push
       (calc-spline coeff timer d 'diagDT)
       vel)
      (push
       (calc-spline coeff timer d 'diagDDT)
       acc)
      (push
       (calc-spline coeff timer d 'diagDDDT)
       grk)
      (setq timer (+ timer time-step))
      )
    (setq max (apply #'max (mapcar #'norm pos)))
    (setq pos (mapcar #'(lambda (v) (scale (/ 1.0 max) v)) pos))
    (setq max (apply #'max (mapcar #'norm vel)))
    (setq vel (mapcar #'(lambda (v) (scale (/ 1.0 max) v)) vel))
    (setq max (apply #'max (mapcar #'norm acc)))
    (setq acc (mapcar #'(lambda (v) (scale (/ 1.0 max) v)) acc))
    (setq max (apply #'max (mapcar #'norm grk)))
    (setq grk (mapcar #'(lambda (v) (scale (/ 1.0 max) v)) grk))
    (send
     (create-graph
      "spline-view"
      :size '(640 480)
      :name-list name-list
      :data-list (list pos vel acc grk))
     :fit-draw)))
