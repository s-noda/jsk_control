#!/usr/bin/env roseus


(defvar *robot-name* (or (unix:getenv "ROBOT") "HRP2JSKNT"))
(require (format nil "package://hrpsys_ros_bridge_tutorials/euslisp/~A-interface.l"
                 (string-downcase *robot-name*)))
(defvar *robot* (make-robot-model-from-name *robot-name*))
;; (defvar *ri*
;;   (eval (list 'instance
;;               (read-from-string (format nil "~A-interface" *robot-name*))
;;               :init)))

(objects (list *robot*))

(defun ankle-grgr
  (&key
   (down-step 5) ;;deg
   (rotate-max 36)
   (rotate-cnt 0)
   ;; (rotate-step) (round (/ rotate-step 360.0)))
   (leg :lleg)
   (ankle-p (send *robot* leg :ankle-p :joint-angle))
   (ankle-r (send *robot* leg :ankle-r :joint-angle))
   ;; (ret nil)
   (force-buf (float-vector 0 0 0))
   (max-buf)
   )
  (cond
   ((and (boundp '*ri*) *ri*)
    (dotimes (i 3)
      (send *robot* :angle-vector (send *ri* :state :potentio-vector)))))
  (dotimes (i (+ rotate-max 1))
    (setq rotate-cnt (/ (* 1.0 i) rotate-max))
    (send *robot* leg :ankle-p :joint-angle
	  (+ ankle-p (* down-step (sin (* 2 pi rotate-cnt)))))
    (send *robot* leg :ankle-r :joint-angle
	  (+ ankle-r (* down-step (cos (* 2 pi rotate-cnt)))))
    (send *viewer* :draw-objects)
    (cond
     ((and (boundp '*ri*) *ri*)
      (send *ri* :angle-vector (send *robot* :angle-vector) 500)
      (unix:usleep (round 0.5 0.5 1000 1000))
      (dotimes (i 3) (setq force-buf (send *ri* :state :force-vector leg)))))
    (if (or (not max-buf)
	    (< (car max-buf)
	       (norm force-buf)))
	(setq max-buf
	      (list (norm force-buf)
		    (+ ankle-p (* down-step (sin (* 2 pi rotate-cnt))))
		    (+ ankle-r (* down-step (cos (* 2 pi rotate-cnt)))))))
    )
  (send *robot* leg :ankle-p :joint-angle (cadr max-buf))
  (send *robot* leg :ankle-r :joint-angle (caddr max-buf))
  (send *viewer* :draw-objects)
  ;; ret
  max-buf
  )
