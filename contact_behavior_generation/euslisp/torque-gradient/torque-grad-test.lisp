#!/usr/bin/env roseus

(require "human-ball-test.lisp")
(require "rotational-6dof.lisp")

;; (test-sphere-human-ball :key-list '(:rleg :lleg :rarm :larm) :stop 50 :debug-view :no-message :null-max 0.3 :gain1 0.01 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send *robot* :newcoords (make-coords :pos (float-vector 0 0 -650))))))

;; (test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm :larm) :stop 50 :debug-view nil :gain1 0.530954 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send (car (send *robot* :links)) :newcoords (make-coords :pos (float-vector 0 0 -650))))))

(cond
 ((substringp "true" (unix::getenv "TORQUE_GRAD_TEST"))
  (setq
   *test-human-ball-rsd*
   (test-sphere-human-ball-loop :loop-max 50 :key-list '(:rleg :lleg :rarm :larm) :rotation-axis '(t t t t) :stop 55 :null-max 0.5 :debug-view nil :gain1 0.05 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t :thre '(10 10 10 10) :rthre '(0.1 0.1 0.1 0.1)) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send *robot* :newcoords (make-coords :pos (float-vector 0 0 -650)))))))
  (dump-loadable-structure "log.test-human-ball.rsd" *test-human-ball-rsd*))
 (t
  (load "log.test-human-ball.rsd")
  ))

(defun parse-from-key
  (&optional (key :f))
  (mapcar
   #'(lambda (dl) (send-all dl :buf key))
   (cdr *test-human-ball-rsd*)))

(defun average-from-key
  (&optional
   (key :f)
   (dll (parse-from-key key))
   (len (length dll))
   (ret (instantiate float-vector (length (car dll)))))
  (dolist (dl dll)
    (dotimes (i (length ret))
      (if (numberp (nth i dl))
	  (setf (aref ret i) (+ (aref ret i) (nth i dl))))))
  (dotimes (i (length ret))
    (setf (aref ret i) (/ (aref ret i) len)))
  ret)

(defun midian-from-key
  (&optional
   (key :f)
   (dll (parse-from-key key))
   (len (length dll))
   (ret (instantiate float-vector (length (car dll))))
   ;;
   sep)
  (dotimes (i (length ret))
    (let* ((sp (mapcar #'(lambda (dl) (nth i dl)) dll)))
      (setq sp (sort sp '<))
      (push sp sep)))
  (setq sep (reverse sep))
  (dotimes (i (length ret))
    (setf (aref ret i) (nth (/ (length (nth i sep)) 2) (nth i sep))))
  ret)

(defun variance-from-key
  (&optional
   (key :f)
   (av (average-from-key key))
   (dll (parse-from-key key))
   (len (length dll))
   (ret (instantiate float-vector (length (car dll)))))
  (dolist (dl dll)
    (dotimes (i (length ret))
      (if (numberp (nth i dl))
	  (setf (aref ret i)
		(+ (aref ret i)
		   (expt (- (nth i dl) (aref av i)) 2))))))
  (dotimes (i (length ret))
    (setf (aref ret i) (sqrt (/ (aref ret i) len))))
  ret)

(defun show-human-ball-pose
  (rsd)
  (cond
   ((or (find *robot* (send *irtviewer* :objects))
	(not (find *human-ball* (send *irtviewer* :objects))))
    (objects (flatten (list *robot-hand* *human-ball*)))
    (send *irtviewer* :change-background (float-vector 1 1 1))))
  (send rsd :draw :friction-cone? nil :torque-draw? nil)
  (send *viewer* :draw-objects))

(defun format-list
  (p dl)
  (let* ((cnt 0))
    (map cons
	 #'(lambda (d)
	     (if (> cnt 0)
		 (format p " "))
	     (format p "~A" d)
	     (incf cnt))
	 dl)
    (format p "~%")
    ))

(defun dump-t1-t2-f1-f2
  (&key (path "t1_t2_f1_f2.log"))
  (let* ((p (open path :direction :output))
	 (tau (parse-from-key :f))
	 (tm (parse-from-key :time))
	 (t1-t2-f1-f2
	  (mapcar '(lambda (tml fl)
		     (list (nth 1 tml) (nth 2 tml)
			   (/ (nth 1 fl) (nth 0 fl))
			   (/ (nth 2 fl) (nth 0 fl))
			   (/ (nth 1 fl) (nth 2 fl))
			   ))
		  tm tau))
	 (av (average-from-key :dummy t1-t2-f1-f2))
	 (md (midian-from-key :dummy t1-t2-f1-f2))
	 (vr (variance-from-key :dummy av t1-t2-f1-f2))
	 )
    (format p ":raw~%")
    (dolist (d t1-t2-f1-f2)
      (format-list p d))
    (format p ":average~%")
    (format-list p av)
    (format p ":midian~%")
    (format-list p md)
    (format p ":variance~%")
    (format-list p vr)
    ;;
    (format p ":skip-cnt~%")
    (let* (sep
	   (ret (instantiate float-vector 2)))
      (dotimes (i (length ret))
	(push (mapcar #'(lambda (dl) (nth (+ i 2) dl)) t1-t2-f1-f2) sep)
	(setf (aref ret i) (count-if '(lambda (d) (> d 0.99)) (car sep)))
	;; (print (car sep))
	)
      (setq sep (reverse sep))
      (format p "0 0 ~A ~A 0~%"
	      (/ (aref ret 0) (length (nth 0 sep)))
	      (/ (aref ret 1) (length (nth 1 sep)))))
    ;;
    (close p)
    ))
