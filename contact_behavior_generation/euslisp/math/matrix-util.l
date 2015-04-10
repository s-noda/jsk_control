(defun pseudo-inverse-loop
  (mat
   &optional (max 10) ret buf)
  (setq ret (pseudo-inverse mat))
  (while (and (null ret)
	      (plusp (decf max)))
    (if (null buf) (setq buf (copy-object mat)))
    (dotimes (x (send mat :get-val 'dim0))
      (dotimes (y (send mat :get-val 'dim1))
	(setf (aref buf x y)
	      (* (+ 1 (random 0.01))
		 (aref buf x y)))))
    (setq ret (pseudo-inverse buf)))
  ret)

(defun matrix-set
  (mom sun x y
       &optional
       (width (cdr (assoc 'dim1 (send sun :slots))))
       (height (cdr (assoc 'dim0 (send sun :slots)))))
  (dotimes (dy height)
    (dotimes (dx width)
      (setf (aref mom (+ y dy) (+ x dx)) (aref sun dy dx))))
  mom)

(defun vector-set
  (mom sun x)
  (dotimes (dx (length sun))
    (setf (aref mom (+ x dx)) (aref sun dx))))

(defun matrix-append
  (m-list
   &optional (dir '(1 1)))
  (cond
   ((null m-list) #2f())
   ((null (cdr m-list)) (car m-list))
   (t (let* ((row (reduce #'(lambda (a b) (+ a (* (car dir) b)))
			  (mapcar #'(lambda (m) (m . dim0)) m-list)))
	     (col (reduce #'(lambda (a b) (+ a (* (cadr dir) b)))
			  (mapcar #'(lambda (m) (m . dim1)) m-list)))
	     (ret (make-matrix row col))
	     (row-i 0) (col-i 0))
	(mapcar
	 #'(lambda (m)
	     (matrix-set ret m col-i row-i)
	     (setq row-i (+ row-i (* (car dir) (m . dim0))))
	     (setq col-i (+ col-i (* (cadr dir) (m . dim1)))))
	 m-list)
	ret))))

(defun diag
  (v)
  (let ((ret (make-matrix (length v) (length v))) (index 0))
    (map cons
	 #'(lambda (val)
	     (setf (aref ret index index) val)
	     (incf index))
	 v)
    ret))

(defun inner-matrix
  (mat xs ys xe ye)
  (let ((ret (make-matrix (- ye ys) (- xe xs))))
    (dotimes (i (- xe xs))
      (dotimes (j (- ye ys))
	(setf (aref ret j i)
	      (aref mat (+ j ys) (+ i xs)))))
    ret))

(defun solve-linear-equation
  (&rest
   args
   &key ;; min xTWx s.t. Jx=y
   (equality-matrix (unit-matrix 2))
   (equality-vector
    (instantiate float-vector
		 (send equality-matrix :get-val 'dim0)))
   (eval-weight-matrix
    (unit-matrix (send equality-matrix :get-val 'dim1)))
   ;;
   (J equality-matrix)
   (y equality-vector)
   (x0 (instantiate float-vector (send J :get-val 'dim1)))
   (W eval-weight-matrix)
   &allow-other-keys
   )
  (let* ((W+Wt (m+ W (transpose W)))
	 (W+Wt# (pseudo-inverse-loop W+Wt))
	 (Jt (transpose J))
	 (JW#Jt (m* (m* J W+Wt#) Jt))
	 (W#Jt (m* W+Wt# Jt)))
    (v+
     x0
     (transform
      (m* W#Jt (pseudo-inverse-loop JW#Jt))
      (v- y (transform J x0))))))
