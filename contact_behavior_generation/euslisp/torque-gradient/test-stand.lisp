#!/usr/bin/env roseus

(require "random-contact-pose.lisp")

(defvar *root-dir* "log.test_stand")
(unix::system (format nil "mkdir -p ~A" *root-dir*))

(defun dump-plot-data
  (tag d)
  (let* ((p (open (format nil "~A/~A" *root-dir* tag) :direction :output)))
    (dotimes (i (length d))
      (format p "~A ~A~%" i (nth i d)))
    (close p)))

(send *irtviewer* :change-background (float-vector 1 1 1))

;; non divergented parameters for standing posture optimization
(reset-pose)
(send *robot* :newcoords (make-coords))
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file
      (format nil "~A/horizontal_init.jpg" *root-dir*))
;; (send *robot* :newcoords (make-coords :rpy (list 0 (deg2rad 30) 0)))
(setq a (test-torque-ik :stop 100 :init nil :gain 0.05 :null-max 0.5 :torque-gradient-root-link-virtual-joint-weight (fill (instantiate float-vector 6) -0.002) :contact-wrench-optimize? t :gtol 1e-10))
(print (norm (apply 'concatenate (cons float-vector (append (send a :f/fmax) (send a :t/tmax))))))
(setq ap (reverse (send-all *rsd-queue* :buf :f)))
(dump-plot-data "horizontail_dtau" ap)
;; (test-torque-ik :init nil :gain nil :null-max 1.0)
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file
      (format nil "~A/horizontal_dtau.jpg" *root-dir*))

(reset-pose)
(send *robot* :newcoords (make-coords))
(setq b (test-torque-ik :stop 100 :init nil :gain 750.0 :null-max 0.5 :mode :force-approximation :torque-gradient-root-link-virtual-joint-weight (fill (instantiate float-vector 6) -1.0) :contact-wrench-optimize? t :gtol 1e-10))
(setq bp (reverse (send-all *rsd-queue* :buf :f)))
(dump-plot-data "horizontail_df" bp)
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file
      (format nil "~A/horizontal_df.jpg" *root-dir*))

;; rotate test, solvability
(reset-pose)
(send *robot* :newcoords (make-coords :rpy (list 0 (deg2rad 15) 0)))
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file "slope15_init.jpg")
(setq c (test-torque-ik :stop 100 :init nil :gain nil :null-max 0.5 :torque-gradient-root-link-virtual-joint-weight (fill (instantiate float-vector 6) -0.002) :force-gradient-root-link-virtual-joint-weight (fill (instantiate float-vector 6) -1.0) :contact-wrench-optimize? t :gtol 1e-10))
(print (norm (apply 'concatenate (cons float-vector (append (send c :f/fmax) (send c :t/tmax))))))
(setq cp (reverse (send-all *rsd-queue* :buf :f)))
(dump-plot-data "slop15_dtau" cp)
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file
      (format nil "~A/slope15_dtau.jpg" *root-dir*))
;; (test-torque-ik :init nil :gain nil :null-max 1.0)

(reset-pose)
(send *robot* :newcoords (make-coords :rpy (list 0 (deg2rad 15) 0)))
(setq d (test-torque-ik :stop 100 :init nil :gain 750.0 :null-max 0.5 :mode :force-approximation :torque-gradient-root-link-virtual-joint-weight (fill (instantiate float-vector 6) -1.0) :contact-wrench-optimize? t :gtol 1e-10))
(print (norm (apply 'concatenate (cons float-vector (append (send d :f/fmax) (send d :t/tmax))))))
(setq dp (reverse (send-all *rsd-queue* :buf :f)))
(dump-plot-data "slop15_df" dp)
(send *viewer* :draw-objects)
(send *viewer* :viewsurface :write-to-image-file
      (format nil "~A/slope15_df.jpg" *root-dir*))

