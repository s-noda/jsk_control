
(defun vget
  (vec key &key (but-val 0.0))
  (if (numberp key)
      (aref vec key)
    (case key
      (:x (aref vec 0))
      (:y (aref vec 1))
      (:z (aref vec 2))
      (t but-val))))

(defun axis-vector
  (key)
  (case key
    (:x #f(1 0 0))
    (:y #f(0 1 0))
    (:z #F(0 0 1))))

(defun next-axis
  (key)
  (case key
    (:x :y)
    (:y :z)
    (:z :x)))

(defun bef-axis
  (key)
  (case key
    (:x :z)
    (:y :x)
    (:z :y)))

(defun reconf-vector
  (vec key-list &key (but-val 0.0))
  (concatenate float-vector
	       (mapcar #'(lambda (k) (vget vec k :but-val but-val)) key-list)))

(defun vset
  (vec key val)
  (reconf-vector
   vec
   (subst :other key '(:x :y :z))
   :but-val val))


;; (defun matrix-from-normal
;;   (v &key (axis-key :z))
;;   (let* ( (axis-vector (axis-vector axis-key))
;; 	  (deg-between-axis (acos (v. axis-vector v)))
;; 	  (vnext (axis-vector (next-axis axis-key)))
;; 	  (deg-between-next
;; 	   (let* ( (key2d (remove-if
;; 			   #'(lambda (a) (equal a (bef-axis axis-key)))
;; 			   '(:x :y :z)))
;; 		   (axis2d (reconf-vector v key2d))
;; 		   (next2d (reconf-vector vnext key2d)) )
;; 	     (if (zerop (norm next2d)) 0 (v. axis2d next2d)))) )
;;     (describe deg-between-axis)
;;     (describe deg-between-next)
;;     (rotate-matrix
;;      (rotate-matrix (unit-matrix 3) deg-between-next axis-key)
;;      deg-between-axis (bef-axis axis-key))))

(defun sign
  (num)
  (/ num (abs num)))

;; calc matrix witch convert normal-vector to axis-vector
(defun matrix-from-normal
  (v &key (axis-key :z))
  (let* ( (axis-vector (axis-vector axis-key))
	  (vnext (axis-vector (next-axis axis-key)))
	  (bef-key (bef-axis axis-key))
	  (deg-between-next
	   (let* ( (key2d (append
			   (remove-if
			    #'(lambda (a) (equal a axis-key))
			     '(:x :y :z))
			   (list :zero)))
		   (v2d (reconf-vector v key2d)) )
;	     (describe v2d)
	     (if (zerop (norm v2d))
		 0
	       (* (sign (vget (v* vnext v2d) axis-key))
		  (acos (v. (normalize-vector v2d) vnext))))))
	  (deg-between-axis
	   (* (sign
	       (vget (rotate-vector (v* axis-vector v)
				    (- deg-between-next) axis-key)
		     bef-key))
	      (acos (v. axis-vector v)))) )
;    (describe deg-between-axis)
;    (describe deg-between-next)
    (rotate-matrix
     (rotate-matrix (unit-matrix 3) deg-between-next axis-key)
     deg-between-axis bef-key)))


(defun matrix2list
  (m)
  (let (buf)
    (dotimes (i (cdr (assoc 'dim0 (send m :slots))))
      (push (concatenate cons (matrix-row m i)) buf))
    (reverse buf)))
(defun list2matrix
  (l) (make-matrix (length l) (length (car l)) l))


(defun vx2matrix
  (v)
  (let ((a (aref v 0)) (b (aref v 1)) (c (aref v 2)))
    (make-matrix 3 3 (list (list 0 (* -1 c) b)
			   (list c 0 (* -1 a))
			   (list (* -1 b) a 0)))))

(defun vector-in-plane
  (vec plane-key)
  (let ((table '(:yz :zx :xy)))
    (labels ((itter (vec key)
		    (cond
		     ((null vec) nil)
		     ((equal key plane-key)
		      (cons 0 (itter (cdr vec)
				     (cadr (member key table)))))
		     (t (cons (car vec)
			      (itter (cdr vec)
				     (cadr (member key table))))))))
      (concatenate float-vector
		   (itter (concatenate cons vec) :yz)))))

(defun matrix-set
  (mom tar &optional (x 0) (y 0))
  (dotimes (dy (cdr (assoc 'dim0 (send tar :slots))))
    (dotimes (dx (cdr (assoc 'dim1 (send tar :slots))))
      (setf (aref mom (+ y dy) (+ x dx)) (aref tar dy dx))))
  mom)

(defun vector-set
  (mon tar &optional (row 0))
  (dotimes (drow (length tar))
    (setf (aref mon (+ row drow)) (aref tar drow)))
  mon)