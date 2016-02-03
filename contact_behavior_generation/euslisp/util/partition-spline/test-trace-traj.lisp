#!/usr/bin/env roseus

(require "package://contact_behavior_generation/euslisp/util/partition-spline/partition-spline.lisp")

(defvar *ref-traj*
  (minjerk-interpole-partition-spline-vector
   :debug? nil :dimension 2 :x-max 800.0 :id-max 20
   :end-pos (float-vector 0 0)
   :mid-pos (list (float-vector -20 10) (float-vector 0 20)
		  (float-vector 20 10) (float-vector 0 0)
		  (float-vector -20 -10) (float-vector 0 -20)
		  (float-vector 20 -10))
   :mid-pos-x (list 100 200 300 400 500 600 700)))
(defvar *traj* (minjerk-interpole-partition-spline-vector :debug? nil :dimension 2 :x-max 1.0))

(defun trace-trajectory
  (&rest
   args
   &key
   (debug? t)
   ;;
   (tm 80.0)
   (max-vel 1.0)
   ;;
   (traj *traj*)
   (ref-traj *ref-traj*)
   ;;
   ;; misc
   (x-min (send traj :x-min))
   (x-max (send traj :x-max))
   (max-vel-for-traj (* max-vel tm))
   (dimension (send traj :dimension))
   (id-max (send traj :id-max))
   (dim-list (let ((id -1)) (mapcar #'(lambda (a) (incf id)) (make-list dimension))))
   (start-pos (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   (start-vel (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   ;; (start-acc (map float-vector #'(lambda (a) 0.0) (make-list dimension)))
   ;;
   (end-pos   (send ref-traj :calc tm))
   (end-vel   (send ref-traj :calc-delta tm :n 1))
   ;; (end-acc   (send ref-traj :calc-delta tm :n 2))
   ;;
   (pos-coeff-list
    (mapcar
     #'(lambda (pos tm)
	 (send traj :calc-pos-constraints-coeff-matrix-for-gain-vector
	       :dim-list dim-list :tm tm))
     (list start-pos end-pos)
     (list x-min  x-max)))
   (vel-coeff-list
    (mapcar
     #'(lambda (pos tm)
	 (send traj :calc-pos-constraints-coeff-matrix-for-gain-vector
	       :dim-list dim-list :tm tm :delta 1))
     (list start-vel end-vel)
     (list x-min x-max)))
   ;; (acc-coeff-list
   ;;  (mapcar
   ;;   #'(lambda (pos tm)
   ;; 	 (send traj :calc-pos-constraints-coeff-matrix-for-gain-vector
   ;; 	       :dim-list dim-list :tm tm :delta 2))
   ;;   (list start-acc end-acc)
   ;;   (list x-min x-max)))
   (pos-coeff-matrix
    (matrix-append (append pos-coeff-list vel-coeff-list)
		   '(1 0)))
   (pos-vector
    (apply #'concatenate
	   (cons float-vector
		 (flatten
		  (list start-pos end-pos
			start-vel end-vel
			;; start-acc end-acc
			)))))
   ;;
   (objective-matrix (unit-matrix (length (send traj :gain-vector))))
   ;;
   (vel-coeff-matrix
    (matrix-append
     (mapcar 'transpose
	     (send-all (send traj :partition-spline-list)
		       :calc-delta-matrix))
     '(1 1)))
   (vel-min-vector
    (fill (instantiate float-vector (send vel-coeff-matrix :get-val 'dim0))
	  (* -1 max-vel-for-traj)))
   (vel-max-vector
    (fill (instantiate float-vector (send vel-coeff-matrix :get-val 'dim0))
	  (* 1 max-vel-for-traj)))
   ;;
   (solve-qp-func 'solve-eiquadprog) ;;'solve-linear-equation)
   (gain-vector
    (let ((ret (instantiate float-vector (* id-max dimension))))
      (require "package://eus_qp/euslisp/eiquadprog.lisp")
      (or
       (funcall
	solve-qp-func
	:initial-state ret
	:eval-weight-matrix objective-matrix
	:equality-matrix pos-coeff-matrix
	:equality-vector pos-vector
	:inequality-matrix vel-coeff-matrix
	:inequality-min-vector vel-min-vector
	:inequality-max-vector vel-max-vector
	)
       ret)))
   (gain-vector-org (copy-seq (send traj :gain-vector)))
   (gain-matrix (send traj :convert-gain-vector-to-gain-matrix gain-vector))
   ;;
   (dif-eq (v- (transform pos-coeff-matrix gain-vector) pos-vector))
   (dif-ieq (transform vel-coeff-matrix gain-vector))
   (error-cnt 0)
   &allow-other-keys
   )
  (cond
   (debug?
    (warning-message 6 "[trace-trajectory] debug-mode: tm=~A~%" tm)
    (dotimes (i (length dif-eq))
      (cond
       ((> (aref dif-eq i) 1e-3)
	(warning-message 1 "dif-eq(~A): ~A > 1e-3~%" i (aref dif-eq i))
	(incf error-cnt))))
    (dotimes (i (length dif-ieq))
      (cond
       ((not (and (> (- (aref dif-ieq i) (aref vel-min-vector i)) -1e-3)
		  (< (- (aref dif-ieq i) (aref vel-max-vector i)) 1e-3)))
	(warning-message 1 "dif-ieq(~A): ~A !E [~A ~A]~%"
			 i (aref dif-ieq i)
			 (aref vel-min-vector i) (aref vel-max-vector i))
	(incf error-cnt))))
    (if (plusp error-cnt) (warning-message 1 "~A invalid~%" tm)
      (warning-message 6 "~A valid~%" tm))
    ))
  (cond
   ((not (plusp error-cnt))
    (send traj :put :fastest-time tm)
    traj)
   (t
    ;; revert
    (send traj :convert-gain-vector-to-gain-matrix gain-vector-org)
    nil))
  )

(defun binary-search-fastest-trace-trajectroy
  (&rest
   args
   &key
   (tm 800.0)
   (dif (/ tm 2))
   (min-dif 5)
   (tm-org tm)
   traj traj-buf)
  (while t
    (setq traj (apply 'trace-trajectory :tm tm args))
    (push traj traj-buf)
    (if traj
	(setq tm (- tm dif))
      (setq tm (+ tm dif)))
    (setq dif (/ dif 2))
    (if (< dif min-dif) (return-from nil nil)))
  (find-if 'identity traj-buf)
  )

(defun draw-trajectory
  (&key
   (tm 0)
   (tm-max (send *traj* :get :fastest-time))
   (tm-cnt 100)
   (tm-step (/ tm-max tm-cnt))
   )
  (cond
   ((not (and (boundp '*viewer*) *viewer*))
    (setq *car* (make-cube 10 10 10))
    (setq *qad* (make-cube 10 10 10))
    (send *qad* :set-color (float-vector 1 1 0))
    (send *car* :set-color (float-vector 0 1 0))
    (send *car* :newcoords (make-coords :pos (float-vector 10 10 0)))
    (objects (list (make-cube 100 100 100)))
    (send *irtviewer* :objects (list *car* *qad*))
    (send *viewer* :draw-objects)))
  (dotimes (i (- tm-cnt 1))
    (send *qad* :newcoords
	  (make-coords
	   :pos
	   (concatenate
	    float-vector
	    (send *traj* :calc (/ i (* 1.0 tm-cnt)))
	    (float-vector 0))))
    (send *car* :newcoords
	  (make-coords
	   :pos
	   (concatenate
	    float-vector
	    (send *ref-traj* :calc (* i tm-step))
	    (float-vector 0))))
    (send *viewer* :draw-objects)
    (unix::usleep (* 30 1000))
    (x::window-main-one)))
