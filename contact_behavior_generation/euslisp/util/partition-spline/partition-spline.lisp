
(require "../../math/matrix-util.l")
(require "bspline.l")

(defclass partition-spline
  :super basic-spline
  :slots (bspline-weight-matrix
	  gain-weight-matrix)
  )
(defmethod partition-spline
  (:init
   (&rest
    args
    &key
    ((:bspline-weight-matrix W) nil)
    ((:gain-weight-matrix G) nil)
    &allow-other-keys)
   (send-super* :init args)
   (setq bspline-weight-matrix W)
   (setq gain-weight-matrix G)
   self)
  (:calc-coeff-vector
   (x)
   (if bspline-weight-matrix
       (transform bspline-weight-matrix
		  (send-super :calc-coeff-vector x))
     (send-super :calc-coeff-vector x)))
  (:_calc-delta-matrix
   (&optional
    (s (/ 1.0 x-step)))
   (cond
    ((null delta-matrix)
     (setq delta-matrix (scale-matrix s (unit-matrix id-max)))
     (setq s (* -1 s))
     (dotimes (i (- id-max 1))
       (setf (aref delta-matrix i (+ i 1)) s))))
   (if bspline-weight-matrix
       (m* bspline-weight-matrix delta-matrix)
     delta-matrix))
  )

(defclass partition-spline-vector
  :super propertied-object
  :slots (partition-spline-list
	  gain-vector
	  gain-matrix
	  dimension
	  id-max
	  recursive-order
	  x-min
	  x-max
	  ;;
	  ))
(defmethod partition-spline-vector
  (:init
   (&key
    ((:dimension dim) 3)
    ((:id-max M) 8)
    ((:recursive-order N) (make-list dim :initial-element 4))
    ((:x-min min) 0.0)
    ((:x-max max) 1.0)
    ((:partition-spline-list psl)
     (mapcar
      #'(lambda (N)
	  (instance partition-spline :init
		    :id-max M :recursive-order N
		    :x-min min :x-max max)) N))
    &allow-other-keys
    )
   (setq partition-spline-list psl)
   (setq dimension dim)
   (setq id-max M)
   (setq recursive-order N)
   (setq x-min min)
   (setq x-max max))
  (:constant-vector
   (vec &key
	(tm-list
	 (let ((id -1))
	   (mapcar #'(lambda (hh) (incf id))
		   (make-list id-max))))
	(ret (instantiate float-vector (* (length vec) (length tm-list)))))
   (setq
    vec
    (cond
     ((atom vec) (mapcar #'(lambda (tm) vec) tm-list))
     (t (subseq (flatten (mapcar #'(lambda (tm) vec) tm-list))
		0 (length tm-list)))))
   (dotimes (i (length tm-list))
     (dotimes (j (length (car vec)))
       (setf (aref ret (+ j (* i (length (car vec)))))
	     (aref (nth i vec) j))))
   ret)
  (:filter-matrix
   (dim-list)
   (let* ((ret (make-matrix (length dim-list) dimension))
	  (row -1))
     (mapcar
      #'(lambda (id)
	  (setf (aref ret (incf row) id) 1))
      dim-list)
     ret))
  ;; (:convert-horizontal-coeff-matrix-for-gain-vector
  ;;  ;; [q0,...,qM-1]T = [P0;...;pM-1] & A[q0;...;qM-1] => C[P0;...;pM-1]
  ;;  (mat &key (ret (copy-object mat)) (c0 0))
  ;;  (convert-horizontal-coeff-matrix-for-gain-vector
  ;;   mat :ret ret :c0 c0 :dimension dimension))
  (:convert-coeff-matrix-for-gain-vector
   ;; A[p0,...,pM-1]B = {a_ik*p_kl*b_lj}ixj => C[P0;...;pM-1]
   (mat &key
	(right-matrix)
	(tm-list
	 (let ((id -1))
	   (mapcar #'(lambda (hh) (incf id))
		   (make-list id-max))))
	(ret
	 (make-matrix
	  (* (length tm-list) (send mat :get-val 'dim0))
	  (* id-max dimension))))
   (calc-matrix-linear-equation-coeff-matrix
    :left-matrix mat :right-matrix right-matrix
    :col id-max :row dimension
    :col-list tm-list
    :C ret))
  (:calc-coeff-matrix-for-gain-vector ;; gain-vector = [p0;p1;...;pM-1]
   (mat &key
	(right-matrix)
	(dim-list
	 (let ((id -1))
	   (mapcar #'(lambda (hh) (incf id))
		   (make-list dimension))))
	(tm-list
	 (let ((id -1))
	   (mapcar #'(lambda (hh) (incf id))
		   (make-list id-max)))))
   (send self :convert-coeff-matrix-for-gain-vector
	 (m*
	  mat
	  (send self :filter-matrix dim-list))
	 :right-matrix right-matrix
	 :tm-list tm-list))
  (:calc-pos-constraints-coeff-matrix-for-gain-vector
   (&key mat
	 (right-matrix)
	 (tm 0.0)
	 (delta 0)
	 (dim-list
	  (let ((id -1))
	    (mapcar #'(lambda (hh) (incf id))
		    (make-list dimension))))
	 (tm-list (list 0))
	 (discrete? nil)
	 (spline-matrix
	  (transpose
	   (make-matrix
	    1 id-max
	    (list
	     (cond
	      ((< delta 1)
	       (send (nth (car dim-list) partition-spline-list)
		     :calc-coeff-vector tm))
	      (discrete?
	       (send (nth (car dim-list) partition-spline-list)
		     :calc-discrete-delta-coeff-vector tm :n delta))
	      (t
	       (send (nth (car dim-list) partition-spline-list)
		     :calc-delta-coeff-vector tm :n delta)))))))
	 (filter (send self :filter-matrix dim-list))
	 )
   (send self :convert-coeff-matrix-for-gain-vector
	 (if mat (m* m filter) filter)
	 :right-matrix
	 (if right-matrix (m* right-matrix spline-matrix)
	   spline-matrix)
	 :tm-list tm-list))
  (:convert-gain-vector-to-gain-matrix
   (gv &optional buf)
   (setq gain-vector gv)
   (dotimes (i id-max)
     (push (subseq gain-vector
		   (* i dimension)
		   (* (+ i 1) dimension))
	   buf))
   (setq
    gain-matrix
    (transpose (make-matrix (length buf) (length (car buf))
			    (reverse buf))))
   gain-matrix)
  (:calc
   (tm)
   (let ((id -1))
     (map float-vector
	  #'(lambda (spl)
	    (incf id)
	    (send spl :calc tm (matrix-row gain-matrix id)))
	  partition-spline-list)))
  (:calc-delta
   (tm &key (n 1) (discrete? t))
   (let ((id -1))
     (map float-vector
	  #'(lambda (spl)
	    (incf id)
	    (send spl :calc-delta tm (matrix-row gain-matrix id) :n n :discrete? discrete?))
	  partition-spline-list)))
  (:get-descrete-points
   (&key
    (print-x-step 0.1)
    (print-tm (- x-min print-x-step))
    (output-stream t)
    buf)
   (while (< (setq print-tm (+ print-tm print-x-step))
	     (+ x-max (/ print-x-step 2)))
     (push (list print-tm (send self :calc print-tm)) buf)
     (if output-stream (format output-stream "~A: ~A~%" (caar buf) (cadar buf))))
   (send self :put :data buf))
  (:gen-graph
   (&key
    (name ":gen-graph")
    (delta 0)
    (discrete? t)
    (dim-list
     (let ((id -1))
       (mapcar #'(lambda (hh) (incf id))
	       (make-list dimension))))
    (plot-step (/ (- x-max x-min) 30.0))
    (name-list
     (mapcar #'(lambda (id) (format nil "id=~A" id)) dim-list))
    (data-list
     (let* ((buf (make-list (length dim-list))) (x x-min) tmp)
       (while (<= x x-max)
	 (setq tmp (if (plusp delta) (send self :calc-delta x :n delta :discrete? discrete?)
		     (send self :calc x)))
	 (setq tmp
	       (mapcar #'(lambda (id) (aref tmp id)) dim-list))
	 (setq buf
	       (map cons
		    #'(lambda (buf vel)
			(cons (float-vector x vel) buf))
		    buf tmp))
	 (setq x (+ x plot-step)))
       buf))
    (graph
     (progn
       (require "../graph-sample.l")
       (create-graph
	name
	:name-list name-list
	:data-list data-list)))
    )
   (send graph :fit-draw)
   graph)
  ;;
  (:varify-convert-coeff-matrix
   (&key
    (left (make-matrix dimension dimension))
    (right (make-matrix id-max id-max))
    (P (make-matrix dimension id-max))
    (C) p-vector APB APB-vector ret)
   (dotimes (i (send left :get-val 'dim0))
     (dotimes (j (send left :get-val 'dim1))
       (setf (aref left i j) (- (* 2.0 (random 1.0)) 1.0))))
   (if right
       (dotimes (i (send right :get-val 'dim0))
	 (dotimes (j (send right :get-val 'dim1))
	   (setf (aref right i j)
		 ;;(if (eq i j) 1 0)
		 (- (* 2.0 (random 1.0)) 1.0)
		 ))))
   (dotimes (i (send P :get-val 'dim0))
     (dotimes (j (send P :get-val 'dim1))
       (setf (aref P i j) (- (* 2.0 (random 1.0)) 1.0))))
   (setq p-vector (send (transpose P) :get-val 'entity))
   (setq C (send self :calc-coeff-matrix-for-gain-vector
		 left :right-matrix right))
   (setq APB (m* left  P))
   (if right (setq APB (m* APB right)))
   ;;(print APB)
   (setq APB-vector (transform C p-vector))
   ;;(print APB-vector)
   (setq ret (v- APB-vector (send (transpose APB) :get-val 'entity)))
   (format t "[:varify-convert-coeff-matrix] |~A| = ~A~%" ret (norm ret))
   (if (< (norm ret) 1e-3) 'ok 'error)
   )
  (:varify-pos-constraints
   (&key
    (dim-list (list 0))
    (delta 0)
    (tm 0.0)
    (pos (map float-vector #'(lambda (a) (- (* 2 (random 1.0)) 1)) dim-list))
    (mat (send self :calc-pos-constraints-coeff-matrix-for-gain-vector
	       :dim-list dim-list :delta delta :tm tm))
    (gain (transform (pseudo-inverse mat) pos))
    (gain-matrix (send self :convert-gain-vector-to-gain-matrix gain))
    (pos2 (map float-vector
	       #'(lambda (id)
		   (send (nth id (send self :partition-spline-list))
			 :calc-delta tm (matrix-row gain-matrix id) :n delta))
	       dim-list))
    (diff (v- pos pos2)))
   ;;(print gain)
   (format t "[:varify-pos-constraints] |~A-~A=~A| = ~A~%"
	   pos pos2
	   diff (norm diff))
   (if (< (norm diff) 1e-3) 'ok 'ng))
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
       (format t "[partition-spline-vector] :nomethod warn, invalid function ~A~%"
	       args)
       nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )

(defun convert-horizontal-coeff-matrix-for-gain-vector
  ;; [q0,...,qM-1]T = [P0;...;pM-1] & A[q0;...;qM-1] => C[P0;...;pM-1]
  (mat &key (dimension 2) (ret (copy-object mat)) (c0 0))
  (dotimes (c (send mat :get-val 'dim1))
    ;; (print c0)
    (dotimes (r (send mat :get-val 'dim0))
      (setf (aref ret r c0) (aref mat r c)))
    (setq c0 (+ c0 dimension))
    (if (>= c0 (send mat :get-val 'dim1))
	(setq c0 (mod c0 (- (send mat :get-val 'dim1) 1))))
    )
  ret)

(defun convert-vertical-coeff-matrix-for-gain-vector
  ;; [q0,...,qM-1]T = [P0;...;pM-1] & A[q0;...;qM-1] => C[P0;...;pM-1]
  (mat &key (dimension 2) (ret (copy-object mat)) (r0 0))
  (dotimes (r (send mat :get-val 'dim0))
    (dotimes (c (send mat :get-val 'dim1))
      (setf (aref ret r0 c) (aref mat r c)))
    (setq r0 (+ r0 dimension))
    (if (>= r0 (send mat :get-val 'dim0))
	(setq r0 (mod r0 (- (send mat :get-val 'dim0) 1))))
    )
  ret)

(defun minjerk-interpole-partition-spline-vector
  (&rest
   args
   &key
   (dimension 1)
   (id-max 8)
   (recursive-order (make-list dimension :initial-element 3))
   (dim-list
    (let ((id -1))
      (mapcar #'(lambda (a) (incf id)) (make-list dimension))))
   (x-min 0.0)
   (x-max 1.0)
   (bspline
    (instance* partition-spline-vector
	       :init
	       :dimension dimension :id-max id-max
	       :x-min x-min :x-max x-max :recursive-order recursive-order
	       args))
   (start-pos (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   (end-pos   (map float-vector #'(lambda (a) 1.0) (make-list dimension)))
   (start-vel (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   (end-vel   (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   (start-acc (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   (end-acc   (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   ;;
   (mid-pos nil)
   (mid-vel nil)
   (mid-acc nil)
   (mid-pos-x nil)
   (mid-vel-x nil)
   (mid-acc-x nil)
   ;;
   (pos-coeff-list
    (mapcar
     #'(lambda (pos tm)
	 (send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
	       :dim-list dim-list :tm tm))
     (flatten (list mid-pos start-pos end-pos))
     (append mid-pos-x (list x-min  x-max))))
   (vel-coeff-list
    (mapcar
     #'(lambda (pos tm)
	 (send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
	       :dim-list dim-list :tm tm :delta 1))
     (flatten (list mid-vel start-vel end-vel))
     (append mid-vel-x (list x-min x-max))))
   (acc-coeff-list
    (mapcar
     #'(lambda (pos tm)
	 (send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
	       :dim-list dim-list :tm tm :delta 2))
     (flatten (list mid-acc start-acc end-acc))
     (append mid-acc-x (list x-min x-max))))
   (pos-coeff-matrix
    (matrix-append (append pos-coeff-list vel-coeff-list acc-coeff-list)
		   '(1 0)))
   (pos-vector
    (apply #'concatenate
	   (cons float-vector
		 (flatten
		  (list mid-pos
			start-pos end-pos
			mid-vel
			start-vel end-vel
			mid-acc
			start-acc end-acc)))))
   (n 1)
   (optimize? t)
   (objective-vector-buf
    (mapcar
     #'(lambda (bs) (instantiate float-vector (+ (send bs :recursive-order) 1)))
     (send bspline :partition-spline-list)))
   (objective-matrix
    (if optimize?
	(matrix-append
	 (mapcar
	  #'(lambda (bs buf)
	      (send bs :calc-integral-objective-coeff-matrix
		    :n n :retv buf))
	  (send bspline :partition-spline-list)
	  objective-vector-buf)
	 '(1 1))))
   (objective-vector
    (if optimize?
	(apply #'concatenate (cons float-vector objective-vector-buf))))
   (delta 1e-3)
   (delta-input-objective
    (if optimize? (scale-matrix delta (unit-matrix (* id-max dimension)))))
   (solve-qp-func 'solve-eiquadprog) ;;'solve-linear-equation)
   (gain-vector
    (if optimize?
	(let ((ret (instantiate float-vector (* id-max dimension))))
	  (require "package://eus_qp/euslisp/eiquadprog.l")
	  (format t "[minjerk-interpole]~% D:~A, N:~A, M:~A, x E [~A ~A]~% pos: ~A/~A, vel: ~A/~A, acc: ~A/~A~%"
		  dimension recursive-order id-max x-min x-max
		  (length pos-coeff-list) (length (flatten (list mid-pos start-pos end-pos)))
		  (length vel-coeff-list) (length (flatten (list mid-vel start-vel end-vel)))
		  (length acc-coeff-list) (length (flatten (list mid-acc start-acc end-acc))))
	  (or
	   (bench
	    (funcall
	     solve-qp-func
	     :initial-state ret
	     :eval-weight-matrix
	     (m+ delta-input-objective
		 (convert-vertical-coeff-matrix-for-gain-vector
		  (convert-horizontal-coeff-matrix-for-gain-vector
		   objective-matrix :dimension dimension)
		  :dimension dimension))
	     :equality-matrix pos-coeff-matrix
	     :equality-vector pos-vector))
	   ret))
      (transform (pseudo-inverse-loop pos-coeff-matrix) pos-vector)))
   (gain-matrix (send bspline :convert-gain-vector-to-gain-matrix gain-vector))
   ;;
   (debug? t)
   buf
   (split-cnt 10)
   (print-x-step (/ 1.0 split-cnt))
   (print-tm (- x-min print-x-step))
   (output-stream debug?)
   &allow-other-keys
   )
  (if debug? (send bspline :get-descrete-points
		   :print-x-step print-x-step
		   :print-tm print-tm
		   :output-stream output-stream))
  bspline
  )

;; (defun minjerk-interpole-partition-spline-vector-separate
;;   (&rest
;;    args
;;    &key
;;    (dimension 1)
;;    (id-max 8)
;;    (recursive-order (make-list dimension :initial-element 3))
;;    (dim-list
;;     (let ((id -1))
;;       (mapcar #'(lambda (a) (incf id)) (make-list 1))))
;;    (x-min 0.0)
;;    (x-max 1.0)
;;    (bspline
;;     (instance* partition-spline-vector
;; 	       :init
;; 	       :dimension 1 :id-max id-max
;; 	       :x-min x-min :x-max x-max :recursive-order recursive-order
;; 	       args))
;;    (start-pos (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
;;    (end-pos   (map float-vector #'(lambda (a) 1.0) (make-list dimension)))
;;    (start-vel (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
;;    (end-vel (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
;;    (start-acc (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
;;    (end-acc (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
;;    ;;
;;    (start-pos-sep (map cons #'(lambda (val) (float-vector val)) start-pos))
;;    (end-pos-sep (map cons #'(lambda (val) (float-vector val)) end-pos))
;;    (start-vel-sep (map cons #'(lambda (val) (float-vector val)) start-vel))
;;    (end-vel-sep (map cons #'(lambda (val) (float-vector val)) end-vel))
;;    (start-acc-sep (map cons #'(lambda (val) (float-vector val)) start-acc))
;;    (end-acc-sep (map cons #'(lambda (val) (float-vector val)) end-acc))
;;    ;;
;;    (mid-pos nil)
;;    (mid-vel nil)
;;    (mid-acc nil)
;;    ;;
;;    (mid-pos-seq (mapcar #'(lambda (mp) (map cons #'(lambda (val) (float-vector val)) mp)) mid-pos))
;;    (mid-vel-seq (mapcar #'(lambda (mp) (map cons #'(lambda (val) (float-vector val)) mp)) mid-vel))
;;    (mid-acc-seq (mapcar #'(lambda (mp) (map cons #'(lambda (val) (float-vector val)) mp)) mid-acc))
;;    ;;
;;    (pos-coeff-list
;;     (mapcar
;;      #'(lambda (pos tm)
;; 	 (send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
;; 	       :dim-list dim-list :tm tm))
;;      (flatten (list mid-pos start-pos end-pos))
;;      (append mid-pos-x (list x-min  x-max))))
;;    (vel-coeff-list
;;     (mapcar
;;      #'(lambda (pos tm)
;; 	 (send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
;; 	       :dim-list dim-list :tm tm :delta 1))
;;      (flatten (list mid-vel start-vel end-vel))
;;      (append mid-vel-x (list x-min x-max))))
;;    (acc-coeff-list
;;     (mapcar
;;      #'(lambda (pos tm)
;; 	 (send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
;; 	       :dim-list dim-list :tm tm :delta 2))
;;      (flatten (list mid-acc start-acc end-acc))
;;      (append mid-acc-x (list x-min x-max))))
;;    (pos-coeff-matrix
;;     (matrix-append (append pos-coeff-list vel-coeff-list acc-coeff-list)
;; 		   '(1 0)))
;;    (pos-vector
;;     (mapcar
;;      #'(lambda (mid-pos start-pos end-pos mid-vel start-vel end-vel mid-acc start-acc end-acc)
;; 	 (apply #'concatenate
;; 		(cons float-vector
;; 		      (flatten
;; 		       (list mid-pos
;; 			     start-pos end-pos
;; 			     mid-vel
;; 			     start-vel end-vel
;; 			     mid-acc
;; 			     start-acc end-acc)))))
;;      mid-pos-seq start-pos-sep end-pos-sep mid-vel-seq start-vel-sep end-vel-sep mid-acc-seq start-acc-sep end-acc-sep))
;;    (n 1)
;;    (optimize? t)
;;    (objective-vector-buf
;;     (instantiate float-vector
;; 		 (+ (send (car (send bspline :partition-spline-list)) :recursive-order) 1)))
;;    (objective-matrix
;;     (if optimize?
;; 	(send (car (send bspline :partition-spline-list))
;; 	      :calc-integral-objective-coeff-matrix
;; 	      :n n :retv objective-vector-buf)))
;;    (objective-vector objective-vector-buf)
;;    (delta 1e-3)
;;    (delta-input-objective
;;     (if optimize? (scale-matrix delta (unit-matrix (* id-max 1)))))
;;    (solve-qp-func 'solve-eiquadprog) ;;'solve-linear-equation)
;;    ;;
;;    (debug? t)
;;    buf
;;    (print-x-step 0.1)
;;    (print-tm (- x-min print-x-step))
;;    (output-stream debug?)
;;    ;;
;;    (ret
;;     (mapcar
;;      #'(lambda (mid-pos start-pos end-pos mid-vel start-vel end-vel mid-acc start-acc end-acc)
;; 	 (minjerk-interpole-partition-spline-vector
;; 	  :dimension dimension :id-max id-max  :recursive-order recursive-order
;; 	  :dim-list dim-list :x-min x-min :x-max x-max :bspline bspline
;; 	  :start-pos start-pos :end-pos end-pos :start-vel start-vel :end-vel end-vel
;; 	  :start-acc start-acc :end-acc end-acc
;; 	  :mid-pos mid-pos :mid-vel mid-vel :mid-acc mid-acc
;; 	  :mid-pos-x mid-pos-x :mid-vel-x mid-vel-x :mid-acc-x mid-acc-x
;; 	  :pos-coeff-list pos-coeff-list :vel-coeff-list vel-coeff-list
;; 	  :acc-coeff-list acc-coeff-list :pos-coeff-matrix pos-coeff-matrix
;; 	  :pos-vector pos-vector :n n :optimize? optimize?
;; 	  :objective-vector-buf objective-vector-buf
;; 	  :objective-matrix objective-matrix
;; 	  :objective-vector objective-vector
;; 	  ))
;;      mid-pos-seq start-pos-sep end-pos-sep
;;      mid-vel-seq start-vel-sep end-vel-sep
;;      mid-acc-seq start-acc-sep end-acc-sep))
;;    )
;;   (cond
;;    (debug?
;;     (setq buf
;; 	  (mapcar
;; 	   #'(lambda (&rest args)
;; 	       (list (caar args)
;; 		     (apply #'concatenate (cons float-vector (mapcar #'cadr args)))))
;; 	   (send-all ret :get :data)
;;     (while (< (setq print-tm (+ print-tm print-x-step)) (+ x-max (/ print-x-step 2)))
;;       (push (list print-tm (send bspline :calc print-tm)) buf)
;;       (format output-stream "~A: ~A~%" (caar buf) (cadar buf)))
;;     (send bspline :put :data buf)))
;;   bspline
;;   )

;; (defun linear-interpole-partition-spline-vector
;;   (&rest
;;    args
;;    &key
;;    (dimension 3)
;;    (id-max 8)
;;    (dim-list
;;     (let ((id -1))
;;       (mapcar #'(lambda (a) (incf id)) (make-list dimension))))
;;    (x-min 0.0)
;;    (x-max 1.0)
;;    (bspline
;;     (instance* partition-spline-vector
;; 	       :init
;; 	       :dimension dimension :id-max id-max
;; 	       :x-min x-min :x-max x-max
;; 	       args))
;;    (start-pos (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
;;    (end-pos   (map float-vector #'(lambda (a) 1.0) (make-list dimension)))
;;    (pos-list
;;     (let* ((buf) tm)
;;       (dotimes (i id-max 8)
;; 	(setq tm (/ (* i 1.0) (- id-max 1.0)))
;; 	(push (list (cons :tm (+ (* (- x-max x-min) tm) x-min))
;; 		    (cons :pos
;; 			  (v+ (scale tm end-pos)
;; 			      (scale (- 1.0 tm) start-pos))))
;; 	      buf))
;;       buf))
;;    (pos-coeff-list
;;     (mapcar
;;      #'(lambda (pos)
;; 	 (send bspline :calc-pos-constraints-coeff-matrix-for-gain-vector
;; 	       :dim-list dim-list :tm (cdr (assoc :tm pos))))
;;      pos-list))
;;    (pos-coeff-matrix
;;     (matrix-append pos-coeff-list '(1 0)))
;;    (pos-vector
;;     (apply #'concatenate
;; 	   (cons float-vector
;; 		 (mapcar
;; 		  #'(lambda (pos) (cdr (assoc :pos pos)))
;; 		  pos-list))))
;;    (gain-vector
;;     (transform (pseudo-inverse pos-coeff-matrix) pos-vector))
;;    (gain-matrix (send bspline :convert-gain-vector-to-gain-matrix gain-vector))
;;    ;;
;;    (debug? t)
;;    (print-x-step 0.1)
;;    (print-tm (- x-min print-x-step))
;;    )
;;   (cond
;;    (debug?
;;     (while (< (setq print-tm (+ print-tm print-x-step)) x-max)
;;       (print (send bspline :calc print-tm)))))
;;   bspline
;;   )

;; (defun sgn-interpole-partition-vector
;;   (&rest
;;    args
;;    &key
;;    (dimension 3)
;;    (id-max 10)
;;    (recursive-order (make-list dimension :initial-element 3))
;;    (dim-list
;;     (let ((id -1))
;;       (mapcar #'(lambda (a) (incf id)) (make-list dimension))))
;;    (x-min 0.0)
;;    (x-max 1.0)
;;    (bspline
;;     (instance* partition-spline-vector
;; 	       :init
;; 	       :dimension dimension :id-max id-max
;; 	       :x-min x-min :x-max x-max :recursive-order recursive-order
;; 	       args))
;;    (start-pos (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
;;    (end-pos   (map float-vector #'(lambda (a) 1.0) (make-list dimension)))
;;    (gain-vector
;;     (let ((v (instantiate float-vector (* id-max dimension)))
;; 	  (id 0))
;;       (dotimes (i (/ id-max 2))
;; 	(dotimes (j dimension)
;; 	  (setf (aref v id) (aref start-pos j))
;; 	  (incf id)))
;;       (dotimes (i (- id-max (/ id-max 2)))
;; 	(dotimes (j dimension)
;; 	  (setf (aref v id) (aref end-pos j))
;; 	  (incf id)))
;;       v))
;;    (gain-matrix (send bspline :convert-gain-vector-to-gain-matrix gain-vector))
;;    ;;
;;    (debug? t)
;;    (print-x-step 0.1)
;;    (print-tm (- x-min print-x-step))
;;    )
;;   (cond
;;    (debug?
;;     (while (< (setq print-tm (+ print-tm print-x-step)) x-max)
;;       (format t "~A: ~A~%" print-tm  (send bspline :calc print-tm)))))
;;   bspline
;;   )

;; @Override
(defun pos-list-interpolate-spline-minjerk
  (vector-list
   time-list
   step
   &key
   ;; :interpolator-class linear-interpolator
   (vel-vector-list)
   (acc-vector-list)
   ret
   )
  (if (eq (length time-list) 1)
      (setq time-list (list 0 (car time-list))))
  (setq
   ret
   (minjerk-interpole-partition-spline-vector
    :dimension (length (car vector-list))
    :id-max (* 2
	       (apply #'+
		      (mapcar
		       #'length
		       (list vector-list vel-vector-list acc-vector-list))))
    :x-min (car time-list)
    :x-max (car (last time-list))
    :mid-pos-x (if (> (length time-list) 2) (cdr (butlast time-list)))
    :mid-vel-x (if (> (length vel-vector-list) 2) (cdr (butlast time-list)))
    :mid-acc-x (if (> (length acc-vector-list) 2) (cdr (butlast time-list)))
    :start-pos (car vector-list)
    :end-pos (car (last vector-list))
    :mid-pos (if (> (length vector-list) 2) (cdr (butlast vector-list)))
    :start-vel (car vel-vector-list)
    :end-vel (car (last vel-vector-list))
    :mid-vel (if (> (length vel-vector-list) 2)
		 (cdr (butlast vel-vector-list)))
    :start-acc (car acc-vector-list)
    :end-acc (car (last acc-vector-list))
    :mid-acc (if (> (length acc-vector-list) 2)
		 (cdr (butlast acc-vector-list)))
    :debug? t
    :output-stream t
    :print-x-step step
    ))
  (list :data (reverse (mapcar #'cadr (send ret :get :data)))
	:time (reverse (mapcar #'car (send ret :get :data)))
	:bspline ret)
  )

;; (pos-list-interpolate-spline-minjerk (list #F(0) #F(1)) (list 0.0 1.0) 0.1 :vel-vector-list (list #F(0) #F(0)) :acc-vector-list (list #F(0) #F(0)))
