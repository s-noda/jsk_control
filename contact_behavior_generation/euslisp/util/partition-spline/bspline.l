
;; for debug
(defun solve-qp
  (&key
   (initial-state #F(0))
   (eval-weight-matrix (unit-matrix (length initial-state)))
   (eval-coeff-vector (instantiate float-vector (length initial-state)))
   (equality-matrix #2f())
   (equality-vector #f())
   (state-min-vector #F())
   (state-max-vector #F())
   (inequality-matrix #2f())
   (inequality-min-vector #F())
   (inequality-max-vector #F())
   len
   ok?
   ;;
   ret
   (trial-cnt-feedback #i(0))
   (trial-cnt 1)
   (ineq-offset 1)
   (ineq-scale 2)
   (debug? t)
   ;;
   &allow-other-keys
   )
  (cond
   (debug?
    (cond
     ((null initial-state)
      (format t "* skip initial-state check~%"))
     ((progn
	(format t "* check initial-state len=~A~%" (setq len (length initial-state)))
	(setq ok? t)
	(cond
	 ((and state-min-vector
	       (plusp (length state-min-vector))
	       (not (eq len (length state-min-vector))))
	  (format t "--- error: check state-min-vector len=~A~%" (length state-min-vector))
	  (setq ok? nil)))
	(cond
	 ((and state-max-vector
	       (plusp (length state-max-vector))
	       (not (eq len (length state-max-vector))))
	  (format t "--- error: check state-max-vector len=~A~%" (length state-max-vector))
	  (setq ok? nil)))
	(cond
	 ((and eval-coeff-vector
	       (plusp (length eval-coeff-vector))
	       (not (eq len (length eval-coeff-vector))))
	  (format t "--- error: check state-max-vector len=~A~%" (length eval-coeff-vector))
	  (setq ok? nil)))
	(cond
	 ((and eval-weight-matrix
	       (plusp (send eval-weight-matrix :get-val 'dim0))
	       (not (eq len (send eval-weight-matrix :get-val 'dim0))))
	  (format t "--- error: check eval-weight-matrix len=~A~%"
		  (send eval-weight-matrix :get-val 'dim0))
	  (setq ok? nil)))
	(cond
	 ((and equality-matrix
	       (plusp (send equality-matrix :get-val 'dim0))
	       (not (eq len (send equality-matrix :get-val 'dim1))))
	  (format t "--- error: check equality-matrix len=~A~%"
		  (send equality-matrix :get-val 'dim1))
	  (setq ok? nil)))
	(cond
	 ((and inequality-matrix
	       (plusp (send inequality-matrix :get-val 'dim0))
	       (not (eq len (send inequality-matrix :get-val 'dim1))))
	  (format t "--- error: check inequality-matrix len=~A~%"
		  (send inequality-matrix :get-val 'dim1))
	  (setq ok? nil)))
	ok?
	)
      (format t "--- ok!~%")))
    (cond
     ((null equality-matrix)
      (format t "* skip equality-matrix check~%"))
     ((progn
	(format t "* check equality-matrix ~Ax~A~%"
		(setq len (send equality-matrix :get-val 'dim0))
		(send equality-matrix :get-val 'dim1))
	(setq ok? t)
	(cond
	 ((and equality-vector
	       (plusp (length equality-vector))
	       (not (eq len (length equality-vector))))
	  (format t "--- error: check equality-matrix len=~A~%"
		  (length equality-vector))
	  (setq ok? nil)))
	ok?
	)
      (format t "--- ok!~%")))
    (cond
     ((null inequality-matrix)
      (format t "* skip inequality-matrix check~%"))
     ((progn
	(format t "* check inequality-matrix ~Ax~A~%"
		(setq len (send inequality-matrix :get-val 'dim0))
		(send inequality-matrix :get-val 'dim1))
	(setq ok? t)
	(cond
	 ((and inequality-min-vector
	       (plusp (length inequality-min-vector))
	       (not (eq len (length inequality-min-vector))))
	  (format t "--- error: check inequality-min-vector len=~A~%"
		  (length inequality-min-vector))
	  (setq ok? nil)))
	(cond
	 ((and inequality-max-vector
	       (plusp (length inequality-max-vector))
	       (not (eq len (length inequality-max-vector))))
	  (format t "--- error: check inequality-max-vector len=~A~%"
		  (length inequality-max-vector))
	  (setq ok? nil)))
	ok?
	)
      (format t "--- ok!~%")))
    ))
  (while
      (and
       (null
	(setq ret
	      (solve-eiquadprog
	       :debug? debug?
	       :initial-state initial-state
	       :eval-weight-matrix eval-weight-matrix
	       :eval-coeff-vector eval-coeff-vector
	       :state-min-vector state-min-vector
	       :state-max-vector state-max-vector
	       :equality-matrix equality-matrix
	       :equality-vector equality-vector
	       :inequality-matrix inequality-matrix
	       :inequality-min-vector inequality-min-vector
	       :inequality-max-vector inequality-max-vector
	       )))
       (plusp (decf trial-cnt)))
    (setq inequality-min-vector
	  (map float-vector
	       #'(lambda (v) (* ineq-scale (- v ineq-offset)))
	       inequality-min-vector))
    (setq inequality-max-vector
	  (map float-vector
	       #'(lambda (v) (* ineq-scale (+ v ineq-offset)))
	       inequality-max-vector))
    (format t "[solve-qp] ~A scale constrains -->~% ~A * (org + ~A)~%"
	    trial-cnt ineq-scale ineq-offset)
    )
  (setf (aref trial-cnt-feedback 0) trial-cnt)
  ret
  )

;; basic function
(defun calc-bspline
  (x
   &rest
   args
   &key
   ((:id j) 0)
   ((:recursive-order N) 4) ;; less than M
   ((:recursive-cnt _n) N)
   ((:id-max M) 8)
   (x-min 0)
   (x-max 1.0)
   ;; buf
   (h (/ (* 1.0 (- x-max x-min)) (- M N)))
   (xj (+ (* h j)
	  (/ (* 1.0 (- (* M x-min) (* N x-max)))
	     (- M N))))
   (xj+1 (+ xj h))
   (xj+n+1 (+ xj+1 (* h _n)))
   &allow-other-keys)
  (cond
   ((or (< x xj) (>= x xj+n+1)) 0)
   ((eq _n 0) 1)
   (t
    (/ (+ (* (- x xj)
	     (apply #'calc-bspline
		    (append
		     (list x
			   :id j
			   :recursive-cnt (- _n 1)
			   :h h
			   )
		     args)))
	  (* (- xj+n+1 x)
	     (apply #'calc-bspline
		    (append
		     (list x
			   :id (+ j 1)
			   :recursive-cnt (- _n 1)
			   :h h
			   )
		     args))))
       (* _n h)))))

;; utility
;; bspline = [aj,0 aj,1 ... aj,n-1]*[t^n-1 t^n-2 .. t^0]
(defun calc-bspline-coeff-vector
  (x
   &rest
   args
   &key
   ((:id j) 0)
   ((:recursive-order N) 4) ;; less than M
   ((:recursive-cnt _n) N)
   ((:id-max M) 8)
   (x-min 0)
   (x-max 1.0)
   ;; buf
   (h (/ (* 1.0 (- x-max x-min)) (- M N)))
   (xj (+ (* h j)
	  (/ (* 1.0 (- (* M x-min) (* N x-max)))
	     (- M N))))
   (xj+1 (+ xj h))
   (xj+n+1 (+ xj+1 (* h _n)))
   v1 v2
   &allow-other-keys)
  (cond
   ((or (< x xj) (>= x xj+n+1)) (instantiate float-vector (+ _n 1)))
   ((eq _n 0) #F(1))
   (t
    (setq v1
	  (apply #'calc-bspline-coeff-vector
		 (append
		  (list x
			:id j
			:recursive-cnt (- _n 1)
			:h h
			)
		  args)))
    (setq v2
	  (apply #'calc-bspline-coeff-vector
		 (append
		  (list x
			:id (+ j 1)
			:recursive-cnt (- _n 1)
			:h h
			)
		  args)))
    (scale (/ 1.0 (* _n h))
	   (v+ (concatenate float-vector (v- v1 v2) #F(0))
	       (concatenate float-vector #F(0)
			    (v- (scale xj+n+1 v2) (scale xj v1)))))
    )))

(defun calc-matrix-linear-equation-coeff-matrix
  ;; A[p0,...,pM-1]B = {a_ik*p_kl*b_lj}ixj => C[P0;...;pM-1]
  (&key
   ((:left-matrix A) (unit-matrix 3))
   ((:right-matrix B) (unit-matrix 3))
   ((:col M) (send B :get-val 'dim0))
   ((:row N) (send A :get-val 'dim1))
   (col-list
    (let ((id -1))
      (mapcar #'(lambda (hh) (incf id))
	      (make-list (if B (send B :get-val 'dim1) M)))))
   (C
    (make-matrix
     (* (send B :get-val 'dim1) (send A :get-val 'dim0))
     (* N M)))
   (x0 0) (y0 0))
  ;;(format t "~A x ~A vs ~A x ~A vs ~A x ~A~%"
  ;;(send A :get-val 'dim0) (send A :get-val 'dim1)
  ;;(if B (send B :get-val 'dim0)) (if B (send B :get-val 'dim1))
  ;;(send C :get-val 'dim0) (send C :get-val 'dim1))
  (cond
   ((and A B)
    (dotimes (i (send B :get-val 'dim1))
      (cond
       ((find i col-list)
	(dotimes (x (send A :get-val 'dim0))
	  (dotimes (j M)
	    (dotimes (y (send A :get-val 'dim1))
	      (setf (aref C (+ x0 x) (+ y0 y (* (send A :get-val 'dim1) j)))
		    (* (aref A x y) (aref B j i))))))
	(setq x0 (+ x0 (send A :get-val 'dim0)))
	))))
   (t
    (dotimes (i M)
      (cond
       ((find i col-list)
	(dotimes (x (send A :get-val 'dim0))
	  (dotimes (y (send A :get-val 'dim1))
	    (setf (aref C (+ x0 x) (+ y0 y))
		  (aref A x y))))
	(setq x0 (+ x0 (send A :get-val 'dim0)))
	))
      (setq y0 (+ y0 (send A :get-val 'dim1)))))
   )
  C)

(defun solve-matrix-linear-equation
  ;; A[p0,...,pM-1]B = C[P0;...;pM-1] = D => return P
  (&key
   ((:left-matrix A) (unit-matrix 3))
   ((:right-matrix B) (unit-matrix 3))
   ((:col M) (send B :get-val 'dim0))
   ((:row N) (send A :get-val 'dim1))
   ((:answer-matrix D) (make-matrix (send A :get-val 'dim0)
				    (send B :get-val 'dim1)))
   (d-vector (send (transpose D) :get-val 'entity))
   (col-list
    (let ((id -1))
      (mapcar #'(lambda (hh) (incf id))
	      (make-list (if B (send B :get-val 'dim1) M)))))
   (C (calc-matrix-linear-equation-coeff-matrix
       :left-matrix A :right-matrix B
       :col M :row N :col-list col-list))
   (res (transform (pseudo-inverse C) d-vector))
   (ret (make-matrix N M))
   )
  (dotimes (i (send ret :get-val 'dim0))
    (dotimes (j (send ret :get-val 'dim1))
      (setf (aref ret i j)
	    (aref res (+ i (* j (send ret :get-val 'dim0)))))))
  ret)

(defun calc-delta-coeff-matrix-for-time-vector
  ;; D of d(At^n)/dt = ADt^n
  (n &key (ret (make-matrix (+ n 1) (+ n 1))) (cnt n) (delta 0.0))
  (dotimes (i n)
    (setf (aref ret (+ i 0) (+ 1 i)) cnt)
    (decf cnt))
  (setf (aref ret 0 0) delta)
  ret)

(defun calc-xTx-matrix
  (x
   &key
   (n 2)
   (ret (make-matrix (+ n 1) (+ n 1)))
   (depth 0)
   (x^n 1)
   buf
   )
  (cond
   ((> depth (* 2 n)) ret)
   (t
    (cond
     ((> depth n)
      (dotimes (i (+ 1 (setq buf (- n (- depth n)))))
	;; (format t "[> depth n(~A)] ~A ~A~%" depth i (- buf i))
	(setf (aref ret i (- buf i)) x^n)))
     (t
      (dotimes (i (+ 1 (setq buf depth)))
	;; (format t "[<= depth n(~A)] ~A ~A~%" depth (- n i) (+ buf i))
	(setf (aref ret (- n i) (+ (- n buf) i)) x^n))))
    (calc-xTx-matrix x :n n :ret ret
		     :depth (incf depth)
		     :x^n (* x^n x)))))

(defun calc-integral-xTx-matrix
  (x
   &key
   (n 2)
   (ret (make-matrix (+ n 1) (+ n 1)))
   (depth 0)
   (x^n x)
   buf val
   )
  (cond
   ((> depth (* 2 n)) ret)
   (t
    (setq val (* (/ 1.0 (+ depth 1)) x^n))
    (cond
     ((> depth n)
      (dotimes (i (+ 1 (setq buf (- n (- depth n)))))
	;; (format t "[> depth n(~A)] ~A ~A~%" depth i (- buf i))
	(setf (aref ret i (- buf i)) val)))
     (t
      (dotimes (i (+ 1 (setq buf depth)))
	;; (format t "[<= depth n(~A)] ~A ~A~%" depth (- n i) (+ buf i))
	(setf (aref ret (- n i) (+ (- n buf) i)) val))))
    (calc-integral-xTx-matrix x :n n :ret ret
			      :depth (incf depth)
			      :x^n (* x^n x)))))

(defun calc-x-vector
  (x
   &key
   (n 2)
   (ret (instantiate float-vector (+ n 1)))
   (depth 0)
   (x^n 1)
   buf val
   )
  (cond
   ((> depth n) ret)
   (t
    (setf (aref ret (- (- (length ret) 1) depth)) x^n)
    (calc-x-vector x :ret ret :n n :depth (incf depth) :x^n (* x^n x)))))


(defun calc-integral-x-vector
  (x
   &key
   (n 2)
   (ret (instantiate float-vector (+ n 1)))
   (depth 0)
   (x^n x)
   buf val
   )
  (cond
   ((> depth n) ret)
   (t
    (setq val (* (/ 1.0 (+ depth 1)) x^n))
    (setf (aref ret (- (- (length ret) 1) depth)) val)
    (calc-integral-x-vector x :n n :ret ret
			    :depth (incf depth)
			    :x^n (* x^n x)))))

(defclass basic-spline-element
  :super object
  :slots (coeff-matrix
	  x-step
	  x-min
	  x-max
	  x-cnt
	  recursive-order
	  id
	  id-max
	  ;;
	  row-max
	  col-max
	  ))
(defmethod basic-spline-element
  (:init
   (&rest
    args
    &key
    ((:id j) 0)
    ((:recursive-order N) 4) ;; less than M
    ;; ((:recursive-cnt _n) N)
    ((:id-max M) 8)
    ((:x-min l) 0)
    ((:x-max g) 1.0)
    ;; buf
    ((:x-step h) (/ (* 1.0 (- g l)) (- M N)))
    &allow-other-keys)
   (setq id j x-step h x-min l x-max g recursive-order N id-max M)
   (setq x-cnt (+ x-min (* 0.5 x-step)))
   (setq coeff-matrix nil)
   (while (<= x-cnt x-max)
     (push (apply #'calc-bspline-coeff-vector
		  (append (list x-cnt) args))
	   coeff-matrix)
     (setq x-cnt (+ x-cnt x-step)))
   (setq coeff-matrix (make-matrix (setq row-max (length coeff-matrix))
				   (setq col-max (length (car coeff-matrix)))
				   (reverse coeff-matrix)))
   self)
  (:coeff-vector
   (x &optional id)
   (setq id (floor (/ (- x x-min) x-step)))
   (cond
    ((or (< id 0) (>= id row-max))
     (if (> (max (- x-min x) (- x x-max)) 1e-3)
	 (format t "[basic-spline-element-coeff] invalid x=~A /E [~A,~A]~%" x x-min x-max))
     (setq id (max 0 (min (- row-max 1) id)))))
   (matrix-row coeff-matrix id))
  (:calc
   (x)
   (let* ((k (send self :coeff-vector x))
	  (v (calc-x-vector x :n recursive-order))
	  (n (length k))
	  (m (length v))
	  (ret 0))
     (dotimes (i n)
       (setq ret (+ ret (* (aref k (- (- n i) 1))
			   (aref v (- (- m i) 1))))))
     ret))
   ;; (v. (send self :coeff-vector x)
   ;; (calc-x-vector x :n recursive-order)))
   ;;
   ;; (labels ((itter
   ;; 	     (x
   ;; 	      &key (coeff-vector (reverse (send self :coeff-vector x)))
   ;; 	      (x-buf 1)
   ;; 	      (ret 0)
   ;; 	      (depth 0))
   ;; 	     (cond
   ;; 	      ((>= depth (length coeff-vector)) ret)
   ;; 	      (t (itter x
   ;; 			:coeff-vector coeff-vector
   ;; 			:ret (+ ret (* x-buf (aref coeff-vector depth)))
   ;; 			:x-buf (* x-buf x)
   ;; 			:depth (+ depth 1))))))
   ;;   (itter x)))
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )

(defclass basic-spline
  :super object
  :slots (bspline-element-list
	  x-step
	  x-min
	  x-max
	  x-cnt
	  recursive-order
	  recursive-cnt
	  id-max
	  id
	  ;;
	  row-max
	  col-max
	  ;;
	  delta-basic-spline
	  delta-matrix
	  discrete-delta-matrix
	  ))
(defmethod basic-spline
  (:init
   (&rest
    args
    &key
    ((:recursive-order N) 4) ;; less than M
    ((:recursive-cnt _n) N)
    ((:id-max M) 8)
    ((:id j) M)
    ((:x-min l) 0)
    ((:x-max g) 1.0)
    ;; buf
    ((:x-step h) (/ (* 1.0 (- g l)) (- M N)))
    &allow-other-keys)
   (setq id j x-step h x-min l x-max g recursive-order N id-max M recursive-cnt _n)
   ;;(setq delta-bspline-hash (make-hash-table))
   (setq bspline-element-list nil)
   (while (>= (decf j) 0)
     (push (instance* basic-spline-element :init (append (list :id j) args))
	   bspline-element-list))
   self)
  (:to-string
   nil
   (format nil "~A(N=~A,M=~A,xE[~A,~A])"
	   (send (class self) :name)
	   recursive-order id-max x-min x-max))
  (:calc-integral-objective-coeff-matrix
   (&key
    (n 3)
    (x-min x-min)
    (x-max x-max)
    (x-step x-step)
    (xs)
    (xe)
    (D^n
     (let* ((m (calc-delta-coeff-matrix-for-time-vector recursive-order))
	    (ret m) (buf (copy-object ret)))
       (dotimes (i (- n 1)) (setq ret (m* m ret buf)))
       ret))
    (ret (make-matrix id id))
    (retv (instantiate float-vector id))
    (MjD^n)
    (Mj)
    (ttTe-ttTs (make-matrix (+ recursive-order 1) (+ recursive-order 1)))
    (te-ts (instantiate float-vector (+ recursive-order 1)))
    (debug? nil))
   (setq xs x-min)
   (setq xe (+ x-min x-step))
   (if debug? (and (print 'D^n) (format-array D^n)))
   (while (<= (/ (+ xs xe) 2.0) x-max)
     (format debug? "[~A ~A] -> ~A~%" xs xe x-step)
     (setq Mj (send self :calc-coeff-matrix-for-time-vector
		    (/ (+ xs xe) 2.0)))
     (if debug? (and (print 'Mj) (format-array Mj)))
     (setq ttTe-ttTs
	   (m-
	    (calc-integral-xTx-matrix xe :n recursive-order)
	    (calc-integral-xTx-matrix xs :n recursive-order)
	    ttTe-ttTs))
     ;;(setq te-ts
     ;;(v-
     ;;(calc-integral-x-vector xe :n recursive-order)
     ;;(calc-integral-x-vector xs :n recursive-order)
     ;;te-ts))
     (if debug? (and (print 'ttTe-ttTs) (format-array ttTe-ttTs)))
     (setq MjD^n (m* Mj D^n))
     (setq ret (m+ ret
		   (m* (m* MjD^n ttTe-ttTs) (transpose MjD^n))
		   ret))
     ;; (setq retv (v+ retv (scale 2 (transform MjD^n te-ts)) retv))
     (if debug? (and (print 'ret) (format-array ret)))
     (setq xs (+ xs x-step))
     (setq xe (+ xe x-step))
     )
   ret)
  (:calc-coeff-matrix-for-time-vector
   ;; calc M(x) of p(x)=gT b(x)=gT M(x)x
   (x)
   (make-matrix
    id (+ recursive-order 1)
    (send-all bspline-element-list :coeff-vector x)))
  (:calc-coeff-vector
   (x) (coerce (send-all bspline-element-list :calc x) float-vector))
  (:calc
   (x gain)
   (v. gain (send self :calc-coeff-vector x)))
  (:calc-gain-vector-coeff-matrix-from-via-x-list
   (via-x-list)
   (make-matrix (length via-x-list) id-max
		(map cons #'(lambda (x) (send self :calc-coeff-vector x)) via-x-list)))
  (:calc-gain-vector-from-via-f-list
   (via-f-list via-x-list)
   (transform (pseudo-inverse
	       (send self :calc-gain-vector-coeff-matrix-from-via-x-list
		     via-x-list))
	      (coerce via-f-list float-vector)))
  (:_discrete-delta-matrix
   (&key (mode :average)
	 (s (/ 1.0 x-step)))
   (cond
    ((cdr (assoc mode discrete-delta-matrix))
     (cdr (assoc mode discrete-delta-matrix)))
    ((eq mode :forward)
     (let* ((ret (unit-matrix id-max)))
       (dotimes (i (- id-max 1))
	 (setf (aref ret i (+ i 1)) -1))
       (push (cons mode (transpose (scale-matrix (* s -1) ret)))
	     discrete-delta-matrix)
       (cdr (assoc mode discrete-delta-matrix))))
    ((eq mode :backward)
     (let* ((ret (unit-matrix id-max)))
       (dotimes (i (- id-max 1))
	 (setf (aref ret (+ i 1) i) -1))
       (push (cons mode (transpose (scale-matrix (* s +1) ret)))
	     discrete-delta-matrix)
       (cdr (assoc mode discrete-delta-matrix))))
    ((eq mode :average)
     (let* ((forward (send self :_discrete-delta-matrix :mode :forward))
	    (backward (send self :_discrete-delta-matrix :mode :backward))
	    (ret (scale-matrix 0.5 (m+ forward backward)))
	    (row (send ret :get-val 'dim0))
	    (col (send ret :get-val 'dim1)))
       (setf (aref ret 0 0) (* -1 s))
       (setf (aref ret 0 1) (* +1 s))
       (setf (aref ret (- row 1) (- col 2)) (* -1 s))
       (setf (aref ret (- row 1) (- col 1)) (* +1 s))
       (push (cons mode (transpose ret)) discrete-delta-matrix)
       (cdr (assoc mode discrete-delta-matrix))))))
  (:discrete-delta-matrix
   (&key (n 1))
   (cond
    ((eq n 0) (unit-matrix id-max))
    ((eq n 1) (send self :_discrete-delta-matrix :mode :average))
    ((> n 1) (m* (send self :_discrete-delta-matrix :mode :average)
		 (send self :discrete-delta-matrix :n (- n 1))))))
  (:_calc-delta-matrix
   (&optional
    (s (/ 1.0 x-step)))
   (cond
    ((null delta-matrix)
     (setq delta-matrix (scale-matrix s (unit-matrix id-max)))
     (setq s (* -1 s))
     (dotimes (i (- id-max 1))
       (setf (aref delta-matrix i (+ i 1)) s))))
   delta-matrix)
  (:calc-delta-matrix
   (&key (n 1))
   (cond
    ((not delta-basic-spline)
     (setq delta-basic-spline
	   (instance* basic-spline :init
		      (list
		       :recursive-order recursive-order ;;(- recursive-order 1)
		       :recursive-cnt (- recursive-cnt 1)
		       :id-max id-max
		       :id id
		       :x-min x-min
		       :x-max x-max)))))
   (cond
    ((not (plusp n)) (unit-matrix id-max))
    ((eq n 1)  (send self :_calc-delta-matrix))
    (t
     (m* (send self :_calc-delta-matrix)
	 (send delta-basic-spline
	       :calc-delta-matrix
	       :n (- n 1))))))
  (:calc-delta-matrix-for-keep-recursive-order
   ;; bspline-set = A(t)[t^n-1;t^n-2;...;t^0], A(t)=MxN
   ;; bspline = pT[A(0),...,A(M-1)]
   ;; must be (N+1)(M-N) < M^2 => M=N+1 or M>N(N+1)
   (&key
    (n 1)
    (coeff-matrix (send-all bspline-element-list :coeff-matrix))
    (Dn (make-matrix (+ 1 recursive-order) (+ 1 recursive-order)))
    (row recursive-order)
    (col (- recursive-order n))
    (order (- recursive-order n))
    A AD ret (debug? nil)
    )
   (dotimes (id (- (+ 1 recursive-order) n))
     (setf (aref Dn row col) 1)
     (setq order (+ id n))
     (dotimes (_n n)
       (setf (aref Dn row col) (* order (aref Dn row col)))
       (setq order (- order 1))
       )
     (decf row)
     (decf col))
   ;; (dotimes (i id-max)
   ;;   (push (make-matrix id-max (+ recursive-order 1)
   ;; 			(mapcar
   ;; 			 #'(lambda (cm) (matrix-row cm i))
   ;; 			 coeff-matrix))
   ;; 	   A))
   (setq A (make-matrix id-max (* (+ recursive-order 1) id-max)
			(send-all coeff-matrix :get-val 'entity)))
   ;; (print A)
   (setq AD (make-matrix id-max (* (+ recursive-order 1) id-max)
			 (send-all (mapcar #'(lambda (m) (m* m Dn)) coeff-matrix)
				   :get-val 'entity)))
   ;; (setq AD (mapcar #'(lambda (_a) (m* _a Dn)) A))
   ;; (print AD)
   (setq
    ret
    (solve-matrix-linear-equation
     :left-matrix (unit-matrix id-max)
     :right-matrix A
     :col id-max :row id-max
     :answer-matrix AD))
   (if debug?
       (let ((buf (v- (send (m- AD (m* ret A)) :get-val 'entity))))
	 (format t "[:calc-delta-matrix-for-keep-recursive-order] |~A| = ~A~%"
		 buf (setq buf (norm buf)))
	 (print (if (< buf 1e-3) 'ok))))
   ret
   )
  (:calc-discrete-delta-coeff-vector
   (x &key (n 1))
   (transform (send self :discrete-delta-matrix :n n)
	      (send self :calc-coeff-vector x)))
  (:calc-delta-coeff-vector
   (x &key (n 1))
   (cond
    ((not delta-basic-spline)
     (setq delta-basic-spline
	   (instance* basic-spline :init
		      (list
		       :recursive-order recursive-order
		       :recursive-cnt (- recursive-cnt 1)
		       :id-max id-max
		       :id id
		       :x-min x-min
		       :x-max x-max)))))
   (transform (send self :calc-delta-matrix)
	      (cond
	       ((eq n 1)
		(send delta-basic-spline
		      :calc-coeff-vector x))
	       (t
		(send delta-basic-spline :calc-delta-coeff-vector
		      x :n (- n 1))))))
  (:calc-delta
   (x gain &key (n 1) (discrete? nil))
   (if (<= n 0) (send self :calc x gain)
     (v. gain
	 (cond
	  ((not (plusp n)) (send self :calc x gain))
	  (discrete?
	   (send self :calc-discrete-delta-coeff-vector x :n n))
	  (t (send self :calc-delta-coeff-vector x :n n))))))
  ;; debug
  (:varify-sum-of-bspline-vector
   (&key (gain (coerce (make-list id-max :initial-element 1.0) float-vector))
	 (x-step (/ (- x-max x-min) 100.0))
	 (x-buf x-min)
	 average
	 buf)
   (while (<= x-buf x-max)
     (format t " ~A x " x-buf)
     (push (print (send self :calc x-buf gain)) buf)
     (setq x-buf (+ x-buf x-step)))
   (setq average (/ (apply #'+ buf) (* 1.0 (length buf))))
   (format t "[:valify-sum-of-bspline-vector]~%     average = ~A, max-diff = ~A, covar=~A~%"
	   average
	   (car (sort buf #'(lambda (a b) (> (abs (- a 1.0)) (abs (- b 1.0))))))
	   (sqrt (/ (apply #'+ (mapcar #'(lambda (a) (expt (- a average) 2.0)) buf))
		    (* 1.0 (length buf))))))
  (:varify-interpole-point
   (&key (f-list '(0 1)) (x-list '(0 1))
	 (gain (send self :calc-gain-vector-from-via-f-list f-list x-list))
	 (x-step (/ (- x-max x-min) 100.0))
	 (x-buf x-min)
	 (tol 0.1)
	 buf
	 )
   (while (<= x-buf x-max)
     (format t " ~A x " x-buf)
     (push (print (send self :calc x-buf gain)) buf)
     (setq x-buf (+ x-buf x-step)))
   (mapcar
    #'(lambda (x f)
	(let ((_f (send self :calc x gain)))
	  (format t "  ~A vs ~A (dif ~A<~A=~A)~%"
		  _f f (abs (- _f f))
		  tol (< (abs (- _f f)) tol))))
    f-list x-list)
   )
  (:varify-calc-delta
   (&key
    (gain (map float-vector
	       #'(lambda (hoge) (- (* 2.0 (random 1.0)) 1.0))
	       (make-list id-max)))
    (split-cnt 1000.0)
    (x-step (/ (- x-max x-min) (* 1.0 split-cnt)))
    (x-buf x-min)
    (f (send self :calc x-buf gain))
    prev-f
    delta
    diff
    (ok? t)
    buf)
   (while (<= (setq x-buf (+ x-buf x-step)) x-max)
     (setq prev-f f)
     (setq f (send self :calc x-buf gain))
     (setq delta (send self :calc-delta x-buf gain))
     (setq diff (/ (- f prev-f) x-step))
     (format t " [x=~A] ~A - ~A = ~A(~A)~%"
	     x-buf
	     delta diff (- delta diff)
	     (if (setq ok? (and ok? (< (abs (- delta diff))
				       (/ 100.0 split-cnt))))
		 'ok 'ng))
     )
   ok?)
  ;;
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val))
       (format t "[bspline] :nomethod warn, invalid function ~A~%"
	       args)
       nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )

;; (setq a (instance basic-spline :init :id-max 8 :recursive-order 3))
;; (send a :varify-calc-delta)
;; (send a :calc-delta-matrix-for-keep-recursive-order :debug? t)

#|

(setq a (instance basic-spline :init :id-max 8 :recursive-order 3))
(let* ((tm (random 1.0))
       (n 1)
       (ref-dcv (send a :calc-delta-coeff-vector tm :n n))
       (Mj (send a :calc-coeff-matrix-for-time-vector tm))
       (D^n
	(let* ((m (calc-delta-coeff-matrix-for-time-vector
		   (send a :recursive-order)))
	       (ret m) (buf (copy-object ret)))
	  (dotimes (i (- n 1)) (setq ret (m* m ret buf)))
	  ret))
       (x (calc-x-vector tm :n (send a :recursive-order)))
       (dcv (transform (m* Mj D^n) x)))
  (print 1)
  (print 'Mj) (format-array Mj)
  (print 'D^n) (format-array D^n)
  (print (transform D^n x))
  (format t "~A - ~A = ~A(~A)~%"
	  ref-dcv dcv (v- ref-dcv dcv) (norm (v- ref-dcv dcv)))
  (norm (v- ref-dcv dcv)))
