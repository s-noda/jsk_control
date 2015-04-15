
(require "vector-util.l")

;; #f(x,y) data-list ---> (a,b) of y = ax + b
(defun min-power-2d
  (data-list)
  (let*
      ( (Exx (reduce
	      #'+
	      (mapcar
	       #'(lambda (v) (* (vget v :x) (vget v :x))) data-list)))
	(Ex (reduce
	     #'+
	     (mapcar
	      #'(lambda (v) (vget v :x)) data-list)))
	(Ey (reduce
	     #'+
	     (mapcar
	      #'(lambda (v) (vget v :y)) data-list)))
	(Exy (reduce
	      #'+
	      (mapcar #'(lambda (v) (* (vget v :x) (vget v :y))) data-list)))
	(n (length data-list))
	(mother (- (* n Exx) (* Ex Ex))) )
    (if (zerop mother)
	nil
      (list (/ (- (* n Exy) (* Ex Ey)) mother)
	    (/ (- (* Exx Ey) (* Exy Ex)) mother)))))


;; #f(x,y,z) data-list ---> (a,b,c,d,e,f) of ax+d = by+e = cz+f
(defun min-power-3d
  (data-list)
  (let ( (xyz (list :x :y :z))
	 (get2d
	  #'(lambda (key1 key2)
	      (mapcar
	       #'(lambda (v) (float-vector (vget v key1) (vget v key2)))
	       data-list))) )
    (reduce
     #'(lambda (result key)
	 (or result
	     (let* ( (but-key (remove-if #'(lambda (a) (equal a key)) xyz))
		     (but1 (funcall get2d (car but-key) key))
		     (but2 (funcall get2d (cadr but-key) key))
		     (min-power1 (min-power-2d but1))
		     (min-power2 (min-power-2d but2)) )
	       (cond
		((and (null min-power1) (null min-power2))  nil)
		(t
		 (let ( (ret
			 (list (cons key (list 1 0))
			       (cons (car but-key) min-power1)
			       (cons (cadr but-key) min-power2))) )
		   (list
		    (cdr (assoc :x ret))
		    (cdr (assoc :y ret))
		    (cdr (assoc :z ret)))))))))
     (cons nil xyz))))


(defun min-power-vector-3d
  (data-list)
  (concatenate float-vector
	       (mapcar #'(lambda (list)
			   (cond
			    ((null list) 0)
			    ((zerop (car list)) 1)
			    (t (/ 1.0 (car list)))))
		       (min-power-3d data-list))))


(defun min-power-plane-3d ;; return (a b c d), where ax+by+cz+d=0
  (data-list)
  (let* ((E1 0) (Ex 0) (Ey 0) (Ez 0) (Exx 0) (Eyy 0)
	 (Exy 0) (Eyz 0) (Ezx 0) ret)
    (mapcar
     #'(lambda (data)
	 (setq E1 (+ E1 1))
	 (setq Ex (+ Ex (aref data 0)))
	 (setq Ey (+ Ey (aref data 1)))
	 (setq Ez (+ Ez (aref data 2)))
	 (setq Exx (+ Exx (* (aref data 0) (aref data 0))))
	 (setq Eyy (+ Eyy (* (aref data 1) (aref data 1))))
	 (setq Exy (+ Exy (* (aref data 0) (aref data 1))))
	 (setq Eyz (+ Eyz (* (aref data 1) (aref data 2))))
	 (setq Ezx (+ Ezx (* (aref data 2) (aref data 0)))))
     data-list)
    (setq ret
	  (transform
	   (pseudo-inverse
	    (make-matrix
	     3 3
	     (list (list E1 Ex Ey) (list Ex Exx Exy) (list Ey Exy Eyy))))
	   (float-vector Ez Ezx Eyz)))
    (list (aref ret 1) (aref ret 2) 1 (aref ret 0))))

#|

;; test for reduce evaluate timing
(reduce
 #'(lambda (a b)
     (let ( (func #'(lambda (a) (progn (print a) t))) )
       (or (funcall func a) (funcall func b))))
 (list 1 2 3))