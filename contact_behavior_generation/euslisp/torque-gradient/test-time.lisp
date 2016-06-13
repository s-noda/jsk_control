#!/usr/bin/env roseus

(require "random-contact-pose.lisp")

(setq *root-dir* "log.test_time")

(defun calc-grad-time-trial-proc
  (&key
   (move-target-keys '(:rarm :larm :rleg :lleg))
   (move-target
    (mapcar '(lambda (k) (send *robot* k :end-coords))
	    move-target-keys))
   (link-list (mapcar
	       '(lambda (mt) (send *robot* :link-list (send mt :parent)))
	       move-target))
   (union-link-list (union (flatten link-list) nil))
   (wrench-list
    (mapcar '(lambda (k)
	       (concatenate float-vector
			    (random-vector 3) (random-vector 3)))
	    move-target))
   (6dof? nil)
   (debug? nil)
   (tau (map float-vector '(lambda (l) (random 1.0)) union-link-list))
   ;;
   Jtau Jf move
   t1 t2
   &allow-other-keys
   )
  (mapcar '(lambda (j)
	     (send j :joint-angle
		   (+ (* (random 1.0) (- (send j :max-angle)
					 (send j :min-angle)))
		      (send j :min-angle))))
	  (send-all union-link-list :joint))
  (send-all union-link-list :worldcoords)
  (send-all move-target :worldcoords)
  (setq
   t1
   (bench2
    (progn
      (setq move tau)
      (setq
       Jtau
       (calc-torque-gradient-from-wrench-list-and-gradient
	:all-link-list union-link-list
	:move-target move-target
	:wrench wrench-list
	:6dof? 6dof?
	:debug? debug?))
      (setq move (transform (transpose Jtau) move)))))
  (setq
   t2
   (bench2
    (progn
      (setq move tau)
      (setq
       Jtau
       (calc-torque-gradient-with-force-approximation
	:move-target move-target
	:all-link-list union-link-list
	:6dof? 6dof?
	:as-list t
	:inverse? nil
	:debug? debug?))
      (mapcar
       #'(lambda (m) (setq move (transform m move)))
       Jtau))))
  (list t1 t2))

(defun calc-grad-time-trial-proc-statistics
  (&rest
   args
   &key
   (loop-cnt 1000)
   ;;
   (move-target-keys '(:rarm :larm :rleg :lleg))
   (move-target
    (mapcar '(lambda (k) (send *robot* k :end-coords))
	    move-target-keys))
   (link-list (mapcar
	       '(lambda (mt) (send *robot* :link-list (send mt :parent)))
	       move-target))
   (union-link-list (union (flatten link-list) nil))
   ;;
   buf
   average
   variance
   &allow-other-keys)
  (apply 'calc-grad-time-trial-proc args)
  (dotimes (i loop-cnt)
    (push
     (apply 'calc-grad-time-trial-proc args)
     buf))
  (setq average
	(list (/ (apply '+ (mapcar 'car buf)) (* 1.0 loop-cnt))
	      (/ (apply '+ (mapcar 'cadr buf)) (* 1.0 loop-cnt))))
  (setq variance
	(list (sqrt
	       (/
		(apply '+ (mapcar #'(lambda (d) (expt (- d (car average)) 2))
				  (mapcar 'car buf)))
		(* 1.0 loop-cnt)))
	      (sqrt
	       (/
		(apply '+ (mapcar #'(lambda (d) (expt (- d (cadr average)) 2))
				  (mapcar 'cadr buf)))
		(* 1.0 loop-cnt)))))
  (list
   ;; (cons :raw buf)
   (cons :n (length union-link-list))
   (cons :k (length move-target))
   (cons :average average)
   (cons :variance variance)
   (cons :loop-cnt loop-cnt)))

(defun calc-kn-matrix
  (&optional
   (data *grad-time-trial-dat*)
   (nmax (apply 'max (mapcar '(lambda (d) (cdr (assoc :n d))) data)))
   (kmax (apply 'max (mapcar '(lambda (d) (cdr (assoc :k d))) data)))
   (dtau_mat (make-matrix (+ 1 kmax) (+ 1 nmax)))
   (df_mat (make-matrix (+ 1 kmax) (+ 1 nmax))))
  (fill (send dtau_mat :get-val 'entity) 0)
  (fill (send df_mat :get-val 'entity) 0)
  (dolist (d data)
    (setf (aref dtau_mat (cdr (assoc :k d)) (cdr (assoc :n d)))
	  (nth 0 (cdr (assoc :average d))))
    (setf (aref df_mat (cdr (assoc :k d)) (cdr (assoc :n d)))
	  (nth 1 (cdr (assoc :average d)))))
  (list (transpose dtau_mat) (transpose df_mat)))

(defun calc-grad-time-trial
  (&rest
   args
   &key
   (move-target-keys '(:rarm :larm :rleg :lleg))
   (move-target
    (mapcar '(lambda (k) (send *robot* k :end-coords))
	    move-target-keys))
   (link-list (mapcar
	       '(lambda (mt) (send *robot* :link-list (send mt :parent)))
	       move-target))
   ;; (union-link-list (union (flatten link-list) nil))
   (6dof? nil)
   (debug? nil)
   tmp buf
   ;;
   (min-link-len (apply 'max (mapcar 'length link-list)))
   &allow-other-keys
   )
  (dotimes (i (length move-target-keys))
    (warning-message 2 "k=~A~%" i)
    (dotimes (j min-link-len)
      (warning-message 2 " -- n/k=~A~%" j)
      (setq
       tmp
       (apply
	'calc-grad-time-trial-proc-statistics
	:move-target-keys (subseq move-target-keys 0 (+ 1 i))
	:move-target (subseq move-target 0 (+ 1 i))
	:link-list
	(mapcar
	 #'(lambda (ll) (if (> (+ 1 j) (length ll)) ll (subseq ll 0 (+ j 1))))
	 (subseq link-list 0 (+ i 1)))
	args))
      (push tmp buf)))
  (setq *grad-time-trial-dat* buf)
  ;; nil
  )

(defun dump-time-values
  (&key
   (data *grad-time-trial-dat*)
   (nlist (union (flatten (mapcar '(lambda (d) (cdr (assoc :n d))) data)) nil))
   (klist (union (flatten (mapcar '(lambda (d) (cdr (assoc :k d))) data)) nil))
   (root-dir *root-dir*)
   buf
   )
  (unix::system (format nil "mkdir -p ~A" root-dir))
  (dolist (k klist)
    (setq buf (remove-if #'(lambda (d) (not (eq k (cdr (assoc :k d))))) data))
    (setq buf (sort buf #'(lambda (a b) (< (cdr (assoc :n a))
					   (Cdr (assoc :n b))))))
    (let* ((ptau (open (format nil "~A/TorqueGradient(k=~A)" root-dir k)
		       :direction :output))
	   (pf (open (format nil "~A/PseudoGradient(k=~A)" root-dir k)
		     :direction :output)))
      (dolist (d buf)
	(format ptau "~A ~A~%" (cdr (assoc :n d)) (cadr (assoc :average d)))
	(format pf "~A ~A~%" (cdr (assoc :n d)) (caddr (assoc :average d)))
	)
      (close ptau) (close pf))))

(defmethod FaceSet
    (:update nil nil))

(calc-grad-time-trial)
(dump-time-values)
(let* ((av (flatten (mapcar '(lambda (d) (cdr (assoc :average d)))
			    *grad-time-trial-dat*))))
  (format t "time in [~A ~A]~%" (apply 'min av) (apply 'max av)))

(unix::system
 (format nil (format nil "cd ~A; ./graph.sh;" *root-dir*)))
