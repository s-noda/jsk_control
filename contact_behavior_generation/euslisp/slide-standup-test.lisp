#!/usr/bin/env roseus

(defvar *robot-type* :staro)
(require "motion-sequencer.lisp")
(require "dynamic-connector.lisp")

(require "motion-planners/motion-planner.lisp") (defun demo-motion-sequence (&rest args) (let* ((ret (apply 'demo-motion-sequence2 (append args (list :error-thre 0.7))))) (cons (car ret) (reverse (cdr ret)))))

(defun test-proc
  nil
  (let* ((rsd (demo-motion-sequence-with-timer
	       :motion-planner-names *motion-planner-names*
	       :ik-debug-view *ik-debug-view*
	       :now-rsd
	       ;;(instance robot-state-data2 :init
	       ;; :contact-states *facefall-contact-state*)
	       (optimize-brli :contact-states *facefall-contact-state*)
	       :remove-limb :rarm
	       ;; :log-stream t
	       ;;:loop-max 8
	       :loop-max
	       '(lambda (cnt &rest args)
		  (let* ((ret (simple-fullbody :target (list (list (cons :target :rarm)) (list (cons :target :larm)) (list (cons :target :rleg)) (list (cons :target :lleg))) :debug-view nil :stop 30 :centroid-thre 100 :warnp nil)))
		    (if ret (send *viewer* :draw-objects)) ret))
	       ;; :cs-filter-func '(lambda (&rest args) (car args))
	       :rms-loop-max 10
	       :tmax-hand-rate 1.0
	       :tmax-leg-rate 1.0
	       :log-file (format nil "~A/simple-floor" *log-root*)
	       :ref-order '(:rarm :larm :rleg :lleg :rarm :larm :rleg :lleg)
	       ;; :error-thre 0.7
	       )))
    (if (or (atom rsd) (not (find :loop-exeeded rsd))) nil rsd)
    ))

;; start test

(defvar *log-root* (format nil "log.slide_standup_test/~A" (log-surfix)))
(defvar *ik-debug-view* nil) ;;:no-message)
(unix:system (format nil "mkdir -p ~A" *log-root*))

(demo-climb-setup :simple-floor)

(send *best-facefall* :draw :friction-cone? nil)
(gen-init-rsd)
(setq *motion-planner-names*
      (list static-walk-motion-planner slide-motion-planner))
(setq *rsd1* (test-proc))

(send *best-facefall* :draw :friction-cone? nil)
(gen-init-rsd)
(setq *motion-planner-names*
      (list static-walk-motion-planner))
(setq *rsd2* (test-proc))

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

(let* ((func
	;;'(lambda (vl) (apply '+ (mapcar 'norm2 vl))))
        '(lambda (vl) (apply 'max (map cons 'abs (apply 'concatenate (flatten (list cons vl)))))))
       (cost2 (mapcar func
		      (mapcar
		       '(lambda (rsd)
			  (append (send rsd :t/tmax)
				  ;; (send rsd :f/fmax)
				  ))
		       (cdr *rsd2*))))
       (cost1 (mapcar func
		      (mapcar
		       '(lambda (rsd)
			  (append (send rsd :t/tmax)
				  ;; (send rsd :f/fmax)
				  ))
		       (cdr *rsd1*))))
       (max2 (apply 'max (butlast cost2 3)))
       (max1 (apply 'max (butlast cost1 3)))
       (pos2 (position max2 (reverse cost2)))
       (pos1 (position max1 (reverse cost1)))
       (mid2 (nth (/ (length cost2) 2) (sort cost2 '<)))
       (mid1 (nth (/ (length cost1) 2) (sort cost1 '<)))
       )
  ;; (send (nth pos1 (reverse *rsd1*)) :draw :rest (list *climb-obj*))
  (send (nth pos2 (reverse *rsd2*)) :draw :rest (list *climb-obj*) :torque-draw? nil)
  (send *viewer* :viewsurface :color (float-vector 0 0 0))
  (send *viewer* :viewsurface :string
	10 (- (- (send *viewer* :viewsurface :height) 10) (text-height "a"))
	(format nil "Detach Left Arm"))
  (send *viewer* :viewsurface :string
	10 (- (send *viewer* :viewsurface :height) 10)
	(format nil "maxTorqueRate=~A" (subseq (format nil "~A" (apply 'max (apply 'concatenate (flatten (cons cons (send (nth pos2 (reverse *rsd2*)) :t/tmax)))))) 0 5)))
  (send *viewer* :viewsurface :flush)
  (send *viewer* :viewsurface :write-to-image-file
	"worst_posture_without_swithing.jpg")
  ;;
  (send (nth pos1 (reverse *rsd1*)) :draw :rest (list *climb-obj*) :torque-draw? nil)
  (send *viewer* :viewsurface :color (float-vector 0 0 0))
  (send *viewer* :viewsurface :string
	10 (- (- (send *viewer* :viewsurface :height) 10) (text-height "a"))
	(format nil "Slide Left Leg"))
  (send *viewer* :viewsurface :string
	10 (- (send *viewer* :viewsurface :height) 10)
	(format nil "maxTorqueRate=~A" (subseq (format nil "~A" (apply 'max (apply 'concatenate (flatten (cons cons (send (nth pos1 (reverse *rsd1*)) :t/tmax)))))) 0 5)))
  (send *viewer* :viewsurface :flush)
  (send *viewer* :viewsurface :write-to-image-file
	"worst_posture_with_swithing.jpg")
  ;;
  (warning-message 1 "walk vs slide~%")
  (format t "max: ~A at ~A vs ~A at ~A~%" max2 pos2 max1 pos1)
  (format t "len: ~A  vs ~A~%" (length cost2) (length cost1))
  (format t "mid: ~A  vs ~A~%" mid2 mid1)
  (format t "ave: ~A  vs ~A~%"
	  (/ (apply '+ cost2) (length cost2))
	  (/ (apply '+ cost1) (length cost1)))
  )

#|

(send-all (send *climb-obj* :bodies) :set-color (float-vector 0.9 0.9 0.9))
(send *irtviewer* :objects (list *robot* *climb-obj*))
(send *best-facefall* :draw :friction-cone? nil)
(send *viewer* :draw-objects)
(send-all (send-all *contact-states* :target-coords) :draw-on
	  :flush nil :color (float-vector 0 1 0) :size 100 :width 5)
(send-all (append (list (send *robot* :get :larm-simple-floor-end-coords)
			(send *robot* :get :rarm-simple-floor-end-coords))
		  (send *robot* :legs :end-coords))
	  :draw-on :flush nil :color (float-vector 1 0 0) :size 300 :width 10)
(send *viewer* :viewsurface :flush)
(send *viewer* :viewsurface :write-to-image-file "contact-candidates.png")
