#!/usr/bin/env roseus

(require "human-ball-test.lisp")
(require "rotational-6dof.lisp")

;; (test-sphere-human-ball :key-list '(:rleg :lleg :rarm :larm) :stop 50 :debug-view :no-message :null-max 0.3 :gain1 0.01 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send *robot* :newcoords (make-coords :pos (float-vector 0 0 -650))))))

;; (test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm :larm) :stop 50 :debug-view nil :gain1 0.530954 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send (car (send *robot* :links)) :newcoords (make-coords :pos (float-vector 0 0 -650))))))

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
  (rsd &optional (str nil))
  (cond
   ((or (find *robot* (send *irtviewer* :objects))
	(not (find *human-ball* (send *irtviewer* :objects))))
    (objects (flatten (list *robot-hand* *human-ball*)))
    (send *irtviewer* :change-background (float-vector 1 1 1))))
  (send rsd :draw :friction-cone? nil :torque-draw? nil)
  (send *viewer* :draw-objects :flush nil)
  (let* ((y (- (send *viewer* :viewsurface :height) 10)))
    (mapcar
     #'(lambda (str)
	 (cond
	  ((stringp str)
	   (send *viewer* :viewsurface :color (float-vector 0 0 0))
	   (send *viewer* :viewsurface :string 10 y str)
	   (setq y (- y (text-height str)))
	   )))
     (reverse (flatten (list str)))))
  (send *viewer* :viewsurface :flush)
  )

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

(defun gen-t1-t2-f1-f2
  (&optional
   (tau (parse-from-key :f))
   (tm (parse-from-key :time))
   (t1-t2-f1-f2
    (mapcar '(lambda (tml fl)
	       (list (nth 1 tml) (nth 2 tml)
		     (/ (nth 1 fl) (nth 0 fl))
		     (/ (nth 2 fl) (nth 0 fl))
		     (/ (nth 1 fl) (nth 2 fl))
		     ))
	    tm tau)))
  t1-t2-f1-f2)

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

(defun gen-hist-data
  (&key
   (t1-t2-f1-f2 (gen-t1-t2-f1-f2))
   (f1-f2 (mapcar '(lambda (dl) (subseq dl 2 4)) t1-t2-f1-f2))
   (range '(0.5 1.0))
   (step 0.1)
   (x (let* (buf (x (car range)))
	(while (< x (cadr range))
	  (push x buf) (setq x (+ x step)))
	(if (not (eps= (car buf) (cadr range)))
	    (push (cadr range) buf))
	(coerce (reverse buf) float-vector)))
   (y
    (mapcar
     #'(lambda (d) (scale 0 x))
     (car f1-f2)))
   ;;
   val
   )
  (dolist (dl f1-f2)
    (dotimes (i (length dl))
      (setq val (max 0 (floor (/ (- (nth i dl) (car range)) step))))
      (if (< val (length (nth i y)))
	  (setf (aref (nth i y) val) (+ (aref (nth i y) val) 1)))))
  (list (cons :x x) (cons :y y)))

(defun dump-hist-plot-data
  (&key
   (hist-data (gen-hist-data))
   (path "plot_dat.py")
   )
  (labels
      ((dump-array
	(p v name)
	(format p "~A=np.array([" name)
	(dotimes (i (length v))
	  (format p "~A~A"
		  (if (> i 0) "," "")
		  (aref v i)))
	(format p "])~%")))
    (let* ((p (open path :direction :output)))
      (format p "import numpy as np~%")
      (dump-array p (cdr (assoc :x hist-data)) "X")
      (dump-array p (nth 0 (cdr (assoc :y hist-data))) "Y1")
      (dump-array p (nth 1 (cdr (assoc :y hist-data))) "Y2")
      (close p)))
  (unix::system "./hist.py")
  hist-data)

(defun sort-rsd
  (&optional
   (func '(lambda (f1 f2) (> (- (nth 2 f1) (nth 3 f1))
			     (- (nth 2 f2) (nth 3 f2))
			     )))
   (t1-t2-f1-f2 (gen-t1-t2-f1-f2))
   (t1-t2-f1-f2-cons (mapcar 'cons t1-t2-f1-f2 (Cdr *test-human-ball-rsd*)))
   )
  ;; (mapcar
  ;;'cdr
  (sort
   t1-t2-f1-f2-cons
   #'(lambda (a b) (funcall func (car a) (car b)))))

(defun text-height
  (str &optional (font x:font-courb24))
  (+ (aref (x::textdots str font) 0)
     (aref (x::textdots str font) 1)))

(in-package "GL")
(defmethod glviewsurface
  (:string
   (x y str &optional (fid x:font-courb24))
   (send self :makecurrent)
   (glMatrixMode GL_PROJECTION)
   (glPushMatrix)
   (send self :2d-mode)
   (unless (eq (get self :glxusexfont) fid)
     (setf (get self :glxusexfont) fid)
     (glxUseXfont fid 32 96 (+ 1000 32)))
   (glRasterPos2i (round x) (- (send self :height) (round y)))
   (glListBase 1000)
   (glCallLists (length str) GL_UNSIGNED_BYTE str)
   (send self :3d-mode)
   (glMatrixMode GL_PROJECTION)
   (glPopMatrix)
   (glMatrixMode GL_MODELVIEW)
   ))
(in-package "USER")

(defun show-sorted-rsd
  (&key
   (func '(lambda (f1 f2) (> (- (nth 2 f1) (nth 3 f1))
			     (- (nth 2 f2) (nth 3 f2))
			     )))
   (prefix "rsd")
   )
  (let* ((rsdl (sort-rsd func)))
    (show-human-ball-pose (nth 0 (cdar rsdl)) (format nil "Initial Posture"))
    (send *viewer* :viewsurface :write-to-image-file (format nil "~A0.jpg" prefix))
    (read-line)
    (show-human-ball-pose (nth 1 (cdar rsdl))
			  (list (format nil "Torque Gradient")
				(format nil "  (Objective:~A)"
					(nth 2 (caar rsdl)))))
    (send *viewer* :viewsurface :write-to-image-file (format nil "~A1.jpg" prefix))
    (read-line)
    (show-human-ball-pose (nth 2 (cdar rsdl))
			  (list (format nil "Pseudo Gradient")
				(format nil "  (Objective:~A)"
					(nth 3 (caar rsdl)))))
    (send *viewer* :viewsurface :write-to-image-file (format nil "~A2.jpg" prefix))
    ))

(send *robot* :legs :knee-p :min-angle 5)
(cond
 ((substringp "true" (unix::getenv "TORQUE_GRAD_TEST"))
  (setq
   *test-human-ball-rsd*
   (test-sphere-human-ball-loop :loop-max 100 :key-list '(:rleg :lleg :rarm :larm) :rotation-axis '(t t t t) :stop 55 :null-max 0.3 :debug-view nil :gain1 0.01 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t :thre (make-list 4 :initial-element 30) :rthre (make-list 4 :initial-element (deg2rad 10))) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send *robot* :newcoords (make-coords :pos (float-vector 0 0 -850)))))))
  (send-all (flatten (cdr *test-human-ball-rsd*)) :clear)
  (dump-loadable-structure (format nil "log.test-human-ball.rsd.~A" "tttt.3010.100.850")
                           *test-human-ball-rsd*))
 (t
  ;; (load "log.test-human-ball.rsd.tttt.3012.100")
  ))

#|

(show-sorted-rsd :prefix "pseudo_lt_torque")
(show-sorted-rsd
 :func '(lambda (f1 f2) (> (- (nth 3 f1) (nth 2 f1))
			   (- (nth 3 f2) (nth 2 f2))))
 :prefix "pseudo_gt_torque")
