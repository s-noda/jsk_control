
(defun delta-function
  (func &optional (x 'x))
  (if (functionp func)
      (case (car func)
	('lambda
	  (delta-function (caddr func) (caadr func)))
	('lambda-closure
	 (delta-function (caddr (cdddr func)) (caadr (cdddr func)))))
    (cond
     ((listp func)
      (case (car func)
	('+ (cons '+
		  (mapcar
		   #'(lambda (f) (delta-function f x))
		   (cdr func))))
	('- (cons '-
		  (mapcar
		   #'(lambda (f) (delta-function f x))
		   (cdr func))))
	('* (labels ((cross
		      (pref nowf x)
		      (cond
		       ((null nowf) nil)
		       (t
			(cons
			 (cons '*
			       (append pref
				       (list (delta-function (car nowf) x))
				       (cdr nowf)))
			 (cross (append pref (list (car nowf)))
				(cdr nowf) x))))))
	      (cons '+ (cross nil (cdr func) x))))
	('/
	 (list '/
	       (list '-
		     (list '*
			   (delta-function (cadr func) x)
			   (caddr func))
		     (list '*
			   (delta-function (caddr func) x)
			   (cadr func)))
	       (list '* (caddr func) (caddr func))))
	('sin (list '* (delta-function (cadr func) x)
		    (list 'cos (cadr func))))
	('cos (list '* -1.0 (delta-function (cadr func) x)
		    (list 'sin (cadr func))))
	('tan (list '/
		    (list '* -1.0 (delta-function (cadr func) x))
		    (list 'expt (list 'cos (cadr func)) 2)))
	('sinh (list '*
		     (delta-function (cadr func) x)
		     'cosh (cadr func)))
	('cosh (list '* -1.0 (delta-function (cadr func) x)
		     (list 'sinh (cadr func))))
	('exp (list '* (delta-function (cadr func) x)
		    (list 'exp (cadr func))))
	('log (list '* (delta-function (cadr func) x)
		    (list '/ 1.0 (cadr func))))
	('expt (list '* (delta-function (list '* (cadr func)
					      (list 'log (caddr func))) x)
		     (list 'expt (cadr func) (caddr func))))
	('sqrt (list '/ (delta-function (cadr func) x)
		     (list '* 2.0 (list 'sqrt (cadr func)))))
	(t (delta-function (eval func) x))))
     ((eq func x) 1)
     (t 0))))

(defclass symbol-matrix
  :super object
  :slots (data)
  )
(defmethod symbol-matrix
  (:init
   (&key ((:data d) '((1 0 0) (0 1 0) (0 0 1)))
	 (atom nil) (vector nil)
	 )
   (setq data
	 (cond
	  (atom (list (list d)))
	  (vector (map cons #'list d))
	  ((vectorp d)
	   (map cons #'(lambda (v) (list v)) d))
	  ((matrixp d)
	   (let ((buf nil))
	     (dotimes (i (length (matrix-column d 0)))
	       (push (coerce (matrix-row d i) cons) buf))
	     (reverse buf)))
	  ((and (listp d) (null (find-if #'atom d))) d)
	  ((listp d) (mapcar #'list d))
	  (t (list (list d))))))
  (:symbol-matrix-linear-operation
   (sm op &key (as-symbol-matrix t))
   (let* ((sm (if (and (class sm) (subclassp (class sm) symbol-matrix))
		  sm (instance symbol-matrix :init :data sm)))
	  (ret (if (send self :dimension-check sm)
		   (mapcar #'(lambda (rl1 rl2)
			       (mapcar #'(lambda (v1 v2)
					   (send self :symbol-binary-operation
						 v1 v2 :operation op))
				       rl1 rl2))
			   data
			   (send sm :data)))))
     (if as-symbol-matrix (instance symbol-matrix :init :data ret) ret)))
  (:+ (sm &rest args &key (as-symbol-matrix t))
      (send* self :symbol-matrix-linear-operation sm '+ args))
  (:- (sm &rest args &key (as-symbol-matrix t))
      (send* self :symbol-matrix-linear-operation sm '- args))
  (:transpose
   (&key (as-symbol-matrix t))
   (let* (ret buf)
     (dotimes (i (length (car data)))
       (push (mapcar #'(lambda (l) (nth i l)) data) buf))
     (setq ret (reverse buf))
     (if as-symbol-matrix (instance symbol-matrix :init :data ret) ret)))
  (:atom nil
	 (if (and (eq (length data) 1)
		  (eq (length (car data)) 1))
	     (caar data)))
  (:symbol-eps=
   (sym tar &key (number? (and (numberp tar) (numberp sym))))
   (if (and number? (< (abs (- sym tar)) 1e-3)) t nil))
  (:symbol-binary-operation
   (d1 d2 &key (operation '+))
   (let ((d1? (numberp d1))
	 (d2? (numberp d2))
	 (ret (list operation d1 d2)))
     (cond
      ((and d1? d2?) (apply operation (list d1 d2)))
      ((not (or d1? d2?)) ret)
      (t
       (case operation
	     ('* (setq d1? (find-if #'numberp (list d1 d2)))
		 (setq d2? (find-if-not #'numberp (list d1 d2)))
		 (cond
		  ((send self :symbol-eps= d1? 0 :number? t) 0)
		  ((send self :symbol-eps= d1? 1 :number? t) d2?)
		  (t ret)))
	     ('/ (cond
		  ((send self :symbol-eps= d1 0) 0)
		  ((send self :symbol-eps= d2 1) d1)
		  (t ret)))
	     ('+ (setq d1? (find-if #'numberp (list d1 d2)))
		 (setq d2? (find-if-not #'numberp (list d1 d2)))
		 (cond
		  ((send self :symbol-eps= d1? 0 :number? t) d2?)
		  (t ret)))
	     ('- (cond
		  ((send self :symbol-eps= d2 0) d1)
		  (t ret))))))))
  (:scale
   (sym &key (as-symbol-matrix t))
   (let ((sym
	  (if (and (class sym) (subclassp (class sym) symbol-matrix))
	      (caar (send sym :data)) sym))
	 (ret
	  (mapcar
	   #'(lambda (dl)
	       (mapcar
		#'(lambda (d)
		    (send self :symbol-binary-operation sym d :operation '*))
		dl))
	   data)))
     (if as-symbol-matrix (instance symbol-matrix :init :data ret) ret)))
  (:dimension-check
   (sm &key (operation :+))
   (let* ((sm (if (and (class sm) (subclassp (class sm) symbol-matrix))
		     sm (instance symbol-matrix :init :data sm))))
     (cond
      ((and (eq operation :*)
	    (eq (length (car data)) (length (send sm :data))))
       t)
      ((and
	(eq (length data)
	    (length (send sm :data)))
	(eq (length (car data))
	    (length (car (send sm :data)))))
       t)
      (t (format t "[symbol matrix dimension check] fail!!~% |~A| != |~A|~%"
		 data (send sm :data))
	 nil))))
  (:* (sm &key (as-symbol-matrix t))
      (let* ((sm (if (and (class sm) (subclassp (class sm) symbol-matrix))
		     sm (instance symbol-matrix :init :data sm)))
	     (ret
	      (cond
	       ((send self :atom)
		(send sm :scale self))
	       ((send sm :atom)
		(send self :scale sm))
	       ((send self :dimension-check sm :operation :*)
		(mapcar
		 #'(lambda (r)
		     (mapcar
		      #'(lambda (l)
			  (cons '+
				(mapcar
				 #'(lambda (r l)
				     (send self :symbol-binary-operation
					   r l :operation '*))
				 r l)))
		      (send (send sm :transpose) :data)))
		 data))
	       (t nil)
	       )))
	(if as-symbol-matrix (instance symbol-matrix :init :data ret) ret)))
  (:cross ; vector expected '((x) (y) (z))
   (sm &key (as-symbol-matrix t))
   (let* ((sm (if (and (class sm) (subclassp (class sm) symbol-matrix))
		     sm (instance symbol-matrix :init :data sm)))
	  (ret
	   (mapcar
	    #'(lambda (index a0 a1 b0 b1)
		(list (send self :symbol-binary-operation
			    (send self :symbol-binary-operation
				  (car a0) (car b1) :operation '*)
			    (send self :symbol-binary-operation
				  (car a1) (car b0) :operation '*)
			    :operation '-)))
	    data
	    (append (cdr data) data)
	    (cdr (append (cdr data) data))
	    (append (cdr (send sm :data)) (send sm :data))
	    (cdr (append (cdr (send sm :data)) (send sm :data))))))
     (if as-symbol-matrix (instance symbol-matrix :init :data ret) ret)))
  (:cross-matrix ; 3d vector expected
   (&key (m data) (as-symbol-matrix t))
   (let ((a (caar m)) (b (caadr m)) (c (caaddr m)))
     (instance symbol-matrix :init :data (list (list 0 (list '* -1 c) b)
					       (list c 0 (list '* -1 a))
					       (list (list '* -1 b) a 0)))))
  (:simplify
   (&key (m data) (as-symbol-matrix t))
   (let ((ret (mapcar #'(lambda (r)
			  (mapcar #'(lambda (v) (symbol-simplify v)) r)) m)))
     (if as-symbol-matrix
	 (instance symbol-matrix :init :data ret) ret)))
  (:eval
   (arg-symbols arg-values &key (as-symbol-matrix t))
   (let ((ret (mapcar
	       #'(lambda (r)
		   (mapcar
		    #'(lambda (v)
			(eval-symbol-func v arg-symbols arg-values)) r))
	       data)))
     (if as-symbol-matrix (instance symbol-matrix :init :data ret) ret)))
  (:data nil data)
  (:gradient
   (sm)
   (let ((buf))
     (setq
      buf
      (mapcar
       #'(lambda (dl)
	   (mapcar #'(lambda (d) (delta-function d sm)) dl))
       data))
     (instance symbol-matrix :init :data buf)))
  (:gradient-vector
   (sm) ;; sm must be symbol vector such as '(r p y)
   (let* ((sm (if (and (class sm) (subclassp (class sm) symbol-matrix))
		  sm (instance symbol-matrix :init :data sm)))
	  buf)
     (cond
      ((send sm :atom)
       )
      (t
       (setq
	buf
	(mapcar
	 #'(lambda (dy)
	     (mapcar
	      #'(lambda (dx)
		  (cond
		   ((or (> (length dy) 1)
			(> (length dx) 1))
		    (format t "[symbol-matrix] :gradient only support for vector~%")))
		  (delta-function (nth 0 dy) (nth 0 dx)))
	      (send sm :data)))
	 data))
       (instance symbol-matrix :init :data buf)))))
  (:column
   (id)
   (instance symbol-matrix :init :data
	     (mapcar #'(lambda (d) (list (nth id d))) data)))
  (:row
   (id)
   (instance symbol-matrix :init :data
	     (mapcar #'list (nth id data))))
  )

(defun symbol-simplify
  (s
   &key (max 10) (eqthre 1e-3))
  (cond
   ((atom s) s)
   ((minusp max) s)
   (t
    (labels ((filter
	      (list check)
	      (cond
	       ((null list) nil)
	       ((funcall check (car list))
		(filter (cdr list) check))
	       (t (cons (car list) (filter (cdr list) check)))))
	     (numbers (list) (filter list #'(lambda (n) (not (numberp n)))))
	     (symbols (list) (filter list #'(lambda (n) (numberp n)))))
	    (let* ((func (car s))
		   (arg (mapcar #'(lambda (s) (symbol-simplify s :max (- max 1))) (cdr s)))
		   (num (numbers arg))
		   (sym (symbols arg))
		   buf)
					;	 (format t "~A = ~A + ~A = ~A~%" s num sym
					;	  (setq buf
	      (cond
	       ((null sym) (eval (cons func arg)))
	       ((null num) (cons func arg))
	       (t (case func
			('+ (if (< (abs (apply #'+ num)) eqthre)
				(if (= 1 (length sym)) (car sym) (cons func sym))
			      (cons func (append sym (list (apply #'+ num))))))
			('- (setq buf (car arg))
			    (setq num (numbers (cdr arg)))
			    (setq sym (symbols (cdr arg)))
			    (if (< (abs (apply #'+ num)) eqthre)
				(append (list func buf) sym)
			      (append (list func buf) sym (list (apply #'+ num)))))
			('* (cond
			     ((< (abs (apply #'* num)) eqthre) 0)
			     ((< (abs (- 1.0 (apply #'* num))) eqthre)
			      (if (= 1 (length sym)) (car sym) (cons func sym)))
			     (t (cons func (append sym
						   (if num (list (apply #'* num))))))))
			('/ (setq buf (car arg))
			    (setq num (numbers (cdr arg)))
			    (setq sym (symbols (cdr arg)))
			    (cond
			     ((and (numberp buf) (< (abs buf) eqthre)) 0)
			     ((and (null sym) num
				   (< (abs (- 1.0 (apply #'* num))) eqthre))
			      buf)
			     (t (append (list func buf) sym num))))
			(t (cons func arg))))))))))

(defun eval-symbol-func
  (func arg-symbol arg)
  (apply
   (eval (list 'function
	       (list 'lambda arg-symbol
		     (list 'eval func))))
   arg))

(defun symbol-rodrigues
  (n w
     &key (sym-n (instance symbol-matrix :init :data n :vector t)))
  (let* ((I (instance symbol-matrix :init :data (unit-matrix (length n))))
	 (a (send sym-n :cross-matrix))
	 (a^2 (send a :* a)))
    (send
     (send
      (send
       I
       :+
       (send a^2 :scale (list '- 1 (list 'cos w))))
      :+
      (send a :scale (list 'sin w)))
     :simplify)))

;; (defun test-symbol-rodrigues
;;   (&key (axis (normalize-vector
;; 	       (float-vector 0 (- (random 2.0) 1.0) (- (random 2.0) 1.0))))
;; 	(rot (deg2rad (- (random 180.0) 90))))
;;   (let* ((sr (symbol-rodrigues '(x y z) 'q))
;; 	 (evaled-sr (send (send sr :simplify)
;; 			  :eval
;; 			  '(x y z q)
;; 			  (concatenate cons
;; 				       axis (list rot))))
;; 	 (matrix-sr (make-matrix
;; 		     3 3
;; 		     (send
;; 		      (send evaled-sr :simplify)
;; 		      :data)))
;; 	 (norm #F(1 0 0))
;; 	 (rotated (transform matrix-sr norm))
;; 	 (dif-rot (acos (/ (v. norm rotated) (norm rotated))))
;; 	 )
;;     (format t "[rodrigues-test] axis=~A/rot=~A~%" axis rot)
;;     (format t "  symbol-rodrigues>>>>~%")
;;     (pprint (send (send sr :simplify) :data))
;;     (format t "  evaled-rodrigues>>>>~%")
;;     (print matrix-sr)
;;     (format t "  dif-rot ~A vs ~A~%" dif-rot (abs rot))
;;     matrix-sr))

(defun test-symbol-rodrigues
  (&key (rpy (random-vector 3.14))
	(xyz-symbol '(x y z)))
  (let* ((l^2 (reduce #'(lambda (a b) (list '+ a b))
		      (mapcar #'(lambda (s) (list '* s s)) xyz-symbol)))
	 (l (list 'sqrt l^2))
	 (xyz/l (mapcar #'(lambda (x) (list '/ x l)) xyz-symbol))
	 (exp (symbol-rodrigues xyz/l l))
	 (evaled-exp (send (send exp :simplify)
			   :eval
			   xyz-symbol
			   (coerce rpy cons)))
	 (matrix-exp (make-matrix
		      3 3
		      (send
		       (send evaled-exp :simplify)
		       :data)))
	 (eus-exp (matrix-exponent rpy))
	 )
    (format t "[rodrigues-test] rpy=~A~%" rpy)
    (format t "  symbol-rodrigues>>>>~%")
    (pprint (send (send exp :simplify) :data))
    (format t "  evaled-rodrigues>>>>~%")
    (print (send matrix-exp :get-val 'entity))
    (format t "  vs eus |~A| = ~A~%"
	    (send (m- eus-exp matrix-exp) :get-val 'entity)
	    (norm2 (send (m- eus-exp matrix-exp) :get-val 'entity)))
    matrix-exp))

(defun symbol-rodrigues-gradient
  (key-list
   &key (xyz-symbol '(x y z)))
  (let* ((l^2 (reduce #'(lambda (a b) (list '+ a b))
		      (mapcar #'(lambda (s) (list '* s s)) xyz-symbol)))
	 (l (list 'sqrt l^2))
	 (xyz/l (mapcar #'(lambda (x) (list '/ x l)) xyz-symbol))
	 (exp (symbol-rodrigues xyz/l l)))
    (labels
     ((gradient-itterator
       (key-list)
       (cond
	((null key-list) exp)
	(t
	 (send (gradient-itterator (cdr key-list))
	       :gradient
	       (car key-list))))))
     (send
      (gradient-itterator key-list)
      :simplify)
     )))

(defun test-symbol-rodrigues-gradient
  (&key
   (xyz-symbol '(x y z))
   (key 0)
   (rpy (random-vector 3.14))
   (dt 1e-8)
   (drpy dt)
   rpy+drpy)
  (let* ((R (symbol-rodrigues-gradient
	     (list (nth key xyz-symbol))
	     :xyz-symbol xyz-symbol))
	 refR
	 refR2)
    (setq rpy+drpy (copy-object rpy))
    (setf (aref rpy+drpy key)
	  (+ (aref rpy+drpy key) drpy))
    (setq R
	  (make-matrix
	   3 3
	   (send
	    (send R :eval xyz-symbol (coerce rpy cons))
	    :data)))
    (setq refR
	  (scale-matrix
	   (/ 1.0 dt)
	   (m- (matrix-exponent rpy+drpy)
	       (matrix-exponent rpy))))
    (setq refR2
	  (m*
	   (outer-product-matrix
	    (map float-vector
		 #'(lambda (id) (if (eq id key) 1 0))
		 '(0 1 2)))
	   (matrix-exponent rpy)))
    (print R)
    (print refR)
    (print refR2)
    (print (m* (transpose (matrix-exponent rpy)) refR2))
    (print (send (m- R refR) :get-val 'entity))
    (print (norm2 (send (m- R refR) :get-val 'entity)))
    (print (send (m- refR2 refR) :get-val 'entity))
    (print (norm2 (send (m- refR2 refR) :get-val 'entity)))
    ))

(defun symbol-rotate-matrix
  (rot axis)
  (let ((sin (list 'sin rot))
	(cos (list 'cos rot)))
    (instance symbol-matrix :init
	      :data
	      (case axis
		    (:x
		     (list (list 1 0 0)
			   (list 0 cos  (list '* -1 sin))
			   (list 0 sin cos)))
		    (:y
		     (list (list cos 0 sin)
			   (list 0 1 0)
			   (list (list '* -1 sin) 0 cos)))
		    (:z
		     (list (list cos  (list '* -1 sin) 0)
			   (list sin cos 0)
			   (list 0 0 1)))))))

(defun symbol-rpy-matrix
  (roll pitch yaw)
  (send
   (send
    (symbol-rotate-matrix yaw :z)
    :*
    (send
     (symbol-rotate-matrix pitch :y)
     :*
     (symbol-rotate-matrix roll :x)))
   :simplify))

(defun test-symbol-rpy-matrix
  (&key
   (roll (deg2rad (- (random 360) 180)))
   (pitch (deg2rad (- (random 360) 180)))
   (yaw (deg2rad (- (random 360) 180))))
  (let* ((smat (send (symbol-rpy-matrix 'r 'p 'y) :simplify))
	 (evaled-smat
	  (make-matrix
	   3 3
	   (send (send smat :eval '(r p y) (list roll pitch yaw)) :data)))
	 (answer
	  (send
	   (make-coords :rpy (list yaw pitch roll))
	   :worldrot)))
    (format t "[rpy-mat-test] rpy=~A~%" (list roll pitch yaw))
    (format t "  symbol-rpy-matrix>>>>~%")
    (pprint (send smat :data))
    (format t "  evaled-rpy-matrix>>>>~%")
    (pprint evaled-smat)
    (format t "  dif-mat ~A~%"
	    (norm (send (m- evaled-smat answer) :get-val 'entity)))))

(defun symbol-rpy-gradient
  (roll pitch yaw)
  (let* ((rpy (symbol-rpy-matrix roll pitch yaw)))
    (mapcar
     #'(lambda (id)
	 (send (send (send rpy :column id)
		     :gradient-vector (list roll pitch yaw))
	       :simplify))
     '(0 1 2))))

#|

(defun eval-symbol-func
  (func arg-symbol arg) ;; !! arg-symbol は破壊的に代入されます
  (mapcar #'(lambda (sym val) (eval (list 'setq sym val)))
	  arg-symbol arg)
  (eval func))


(let* ((buf (make-coords :rpy (random-vector 1.0)))
       (rpy (car (rpy-angle (send buf :worldrot))))
       (buf2 (make-coords :rpy rpy)))
  (print buf)
  (print buf2)
  (send buf :difference-rotation buf2))
