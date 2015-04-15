
;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsknt)
(require "../robot-state-data2.l")
(require "../optimize-brli.l")
(require "../util/spline.l")
(require "package://eus_nlopt/euslisp/nlopt-object.l")
(require "dynamic-torque-util.l")
(require "dynamic-trajectory.l")

(defclass dynamic-optimize
  :super nlopt-object
  :slots (start-av
	  end-av
	  start-base-coords
	  end-base-coords
	  start-base-vel
	  start-base-acc
	  end-base-vel
	  end-base-acc
	  fix-target
	  fix-coords
	  move-target
	  move-coords-list
	  contact-states
	  tfm-vector-list
	  f0-list-list
	  m0-list-list
	  split-cnt
	  freq
	  debug?
	  ;;
	  tfm-vector
	  normal-tfm-vector
	  trajectory
	  err
	  ))

(defmethod dynamic-optimize
  (:init
   (&rest
    args
    &key
    (start-av (progn
		(send *robot* :reset-pose)
		(send *robot* :fix-leg-to-coords (make-coords))
		(send *pickview* :look-all)
		(copy-object (send *robot* :angle-vector))))
    (start-base-coords
     (copy-seq
      (concatenate
       float-vector
       (send (send (car (send *robot* :links)) :worldcoords) :worldpos)
       (matrix-log (send (send (car (send *robot* :links)) :worldcoords) :worldrot)))))
    (end-av
     (progn
       (send *robot* :fullbody-inverse-kinematics
	     (list
	      (make-coords
	       :pos
	       (float-vector
		500 -50
		(+ 200 (aref (send *robot* :rleg :end-coords :worldpos) 2))))
	      (copy-object (send *robot* :rleg :end-coords :worldcoords))
	      (copy-object (send *robot* :lleg :end-coords :worldcoords)))
	     :rotation-axis (list nil t t)
	     :translation-axis (list t t t)
	     :move-target
	     (mapcar
	      #'(lambda (k) (send *robot* k :end-coords))
	      '(:rarm :rleg :lleg))
	     :min #F(-1000 -1000 -1000 -200 -200 -200)
	     :max #F(1000 1000 1000 200 200 200)
	     :root-link-virtual-joint-weight
	     #F(0.01 0.01 1 1 1 1)
	     :target-centroid-pos
	     (scale 0.5 (v+ (send *robot* :rleg :end-coords :worldpos)
			    (send *robot* :lleg :end-coords :worldpos)))
	     :revert-if-fail nil
	     :collision-avoidance-link-pair nil
	     :stop 100
	     :link-list
	     (mapcar
	      #'(lambda (k)
		  (send *robot* :link-list
			(send (send *robot* k :end-coords) :parent)))
	      '(:rarm :rleg :lleg))
	     :debug-view :no-message)
       (send *robot* :head :look-at (send *robot* :rarm :end-coords :worldpos))
       (copy-object (send *robot* :angle-vector))))
    (end-base-coords
     (copy-seq
      (concatenate
       float-vector
       (send (send (car (send *robot* :links)) :worldcoords) :worldpos)
       (matrix-log (send (send (car (send *robot* :links)) :worldcoords) :worldrot)))))
    (start-base-vel #F(0 0 0 0 0 0))
    (start-base-acc #F(0 0 0 0 0 0))
    (end-base-vel #F(0 0 0 0 0 0))
    (end-base-acc #F(0 0 0 0 0 0))
    (fix-target
     (list (send *robot* :rleg :end-coords)
	   (send *robot* :lleg :end-coords)))
    (fix-coords
     (mapcar #'(lambda (cs) (copy-object (send cs :worldcoords))) fix-target))
    (contact-states
     (remove-if
      #'(lambda (rsd) (not (find (send rsd :name) '(:rleg :lleg))))
      (send *sample-rsd* :contact-states)))
    (start-contact-force
     (mapcar #'(lambda (c) #F(0 0 300)) contact-states))
    (start-contact-moment
     (mapcar #'(lambda (c) #F(0 0 0)) contact-states))
    (end-contact-force
     (mapcar #'(lambda (c) #F(0 0 300)) contact-states))
    (end-contact-moment
     (mapcar #'(lambda (c) #F(0 0 0)) contact-states))
    (split-cnt 10)
    (f0-list-list
     (let (ret rate)
       (dotimes (i (+ split-cnt 1))
	 (setq rate (/ (* i 1.0) split-cnt))
	 (push
	  (mapcar #'(lambda (st en)
		      (v+ (scale rate st)
			  (scale (- 1 rate) en)))
		  start-contact-force end-contact-force)
	  ret))
       ret))
    (m0-list-list
     (let (ret rate)
       (dotimes (i (+ split-cnt 1))
	 (setq rate (/ (* i 1.0) split-cnt))
	 (push
	  (mapcar #'(lambda (st en)
		      (v+ (scale rate st)
			  (scale (- 1 rate) en)))
		  start-contact-moment end-contact-moment)
	  ret))
       ret))
    (move-target nil)
    (move-coords-list (make-list (+ split-cnt 1)))
    (freq 5)
    (max-eval -1)
    (max-time -1)
    (debug? t)
    ;;
    (initial-state (scale 0.5 (v+ start-base-coords end-base-coords)))
    (state-min-vector (v+ (vmin start-base-coords end-base-coords) #F(-300 -100 -0 0 0 0)))
    (state-max-vector (v+ (vmax start-base-coords end-base-coords) #F(300 100 0 0 0 0)))
    (state-dimension (length start-base-coords))
    (equality-dimension 0)
    (inequality-dimension
     (*
      (- split-cnt 1)
      (+ (length (cdr (send *robot* :links)))
	 (* (length contact-states) 6))))
    (ftol 1e-2)
    (xtol 1e-2)
    (eqthre 1e-2)
    &allow-other-keys
    )
   (send-super*
    :init
    :initial-state initial-state
    :state-min-vector state-min-vector
    :state-max-vector state-max-vector
    :state-dimension state-dimension
    :equality-dimension equality-dimension
    :inequality-dimension inequality-dimension
    ;;(* 6 (- split-cnt 1) (length force-contact))
    :ftol ftol
    :xtol xtol
    :eqthre eqthre
    :max-eval max-eval
    :max-time max-time
    args
    )
   (mapcar
    #'(lambda (k v) (send self k v))
    (list :start-av :end-av :start-base-coords :end-base-coords
	  :fix-target :fix-coords :move-target :move-coords-list
	  :start-base-vel :start-base-acc :end-base-vel :end-base-acc
	  :contact-states :f0-list-list :m0-list-list
	  :split-cnt :freq :debug?)
    (list start-av end-av start-base-coords end-base-coords
	  fix-target fix-coords move-target move-coords-list
	  start-base-vel start-base-acc end-base-vel end-base-acc
	  contact-states f0-list-list m0-list-list
	  split-cnt freq debug?))
   self
   )
  (:draw-animation
   (av-list coords-list zmp-list)
   (mapcar
    #'(lambda (av c zmp)
	(send *robot* :angle-vector (copy-object av))
	(base-coords-fix (copy-object c))
	(send *viewer* :draw-objects :flush nil)
	(send (or zmp
		  (concatenate
		   float-vector
		   (subseq (send *robot* :centroid) 0 2)
		   (list (aref (send *robot* :rleg :end-coords :worldpos) 2))))
	      :draw-on :flush nil :color #F(1 0 0))
	(send *viewer* :viewsurface :flush)
	(unix:usleep (round (* 1000 1000
			       (/ 1.0 (length av-list))))))
    av-list coords-list zmp-list))
  (:evaluation-function
   (v1 v2 &optional (draw? t))
   (setq trajectory
	 (gen-trajectory
	  :fix-target fix-target
	  :fix-coords fix-coords
	  :move-target move-target
	  :move-coords-list move-coords-list
	  :start-av start-av
	  :end-av end-av
	  :start-base-pos start-base-coords
	  :end-base-pos end-base-coords
	  :start-base-vel start-base-vel
	  :start-base-acc start-base-acc
	  :end-base-vel end-base-vel
	  :end-base-acc end-base-acc
	  :split-cnt split-cnt
	  :through-pos-list (list v1)
	  :draw? nil))
   (setq tfm-vector
	 (calc-trajectory-torque
	  :trajectory trajectory
	  :contact-states contact-states
	  :f0-list-list f0-list-list
	  :m0-list-list m0-list-list
	  :freq freq))
   ;;(if debug? (and (print 'tfm-vector) (pprint tfm-vector)))
   (if draw?
       (send self :draw-animation
	     (cdr (assoc :position trajectory))
	     (cdr (assoc :coords trajectory))
	     (append
	      (list nil)
	      (mapcar
	       #'(lambda (fm)
		   (calc-zmp-from-fm-list
		    fm
		    (send-all
		     (send-all contact-states :contact-coords)
		     :worldpos)))
	       (mapcar #'cdr tfm-vector))
	      (list nil))))
   (setq
    normal-tfm-vector
    (mapcar
     #'(lambda (tfm)
	 (cons
	  (map float-vector
	       #'/
	       (car tfm)
	       (send-all (send-all (cdr (send *robot* :links)) :joint) :max-joint-torque))
	  (mapcar
	   #'(lambda (cs fm)
	       (send cs :force-error-world fm))
	   contact-states (cdr tfm))))
     tfm-vector))
   (setq err
	 (*
	  1e+3
	  (apply
	   #'+
	   (flatten
	    (mapcar
	     #'(lambda (err)
		 (mapcar #'norm2 (butlast err)))
	     (cdr (assoc :error trajectory)))))))
   ;; (if debug? (and (print 'normalized-tfm-vector) (pprint tfm-vector)))
   (setf (aref v2 0)
	 (+
	  err
	  (/
	   (apply #'+
		  (mapcar
		   #'(lambda (v) (expt v 4))
		   (apply #'concatenate
			  (cons cons (flatten normal-tfm-vector)))))
	   split-cnt))
	 )
   (if draw? (format t "~A (err=~A) with ~A~%" (aref v2 0) err v1))
   0)
  (:evaluation-function-gradient
   (v1 v2)
   (send self :simple-jacobian :evaluation-function v1 v2 nil)
   0)
  (:equality-function (v1 v2) 0)
  (:equality-function-gradient (v1 v2) 0)
  (:inequality-function
   (v1 v2 &optional (draw? t) (index -1))
   (send self :evaluation-function v1 #F(0) nil)
   ;; (print fm-error)
   (map cons
	#'(lambda (v val)
	    (setf (aref v2 (incf index)) (- (* val val) 1.0)))
	v2
	(apply #'concatenate (cons cons (flatten tfm-vector))))
   ;;(cons cons
   ;;(mapcar #'(lambda (v) (subseq v 0 6)) (flatten (mapcar #'cdr tfm-vector))))))
   (if (and debug? draw?) (print v2))
   0)
  (:inequality-function-gradient
   (v1 v2)
   (send self :simple-jacobian :inequality-function v1 v2 nil)
   0)
  (:optimize
   (&rest args &key ret &allow-other-keys)
   (format (if (boundp '*log-stream*) *log-stream* t)
	   "[dynamic-optimize] initial-value ~A~%"
	   (or (cadr (member :initial-state args)) initial-state))
   (setq ret (send-super* :optimize args))
   ;;(send ret :buf :trajectory trajectory)
   (format (if (boundp '*log-stream*) *log-stream* t)
	   "[dynamic-optimize] optimized-value ~A~%" ret)
   ;;ret
   trajectory
   )
  (:get-trajectory
   (&rest args &key (optimize? nil) &allow-other-keys)
   (cond
    (optimize? (send* self :optimize args))
    (t (send self :evaluation-function
	     (or (cdr (assoc :initial-state args)) initial-state)
	     #F(0))
       trajectory)))
  )

(defun demo-dynamic-optimize
  nil
  (cond
   ((not
     (and (boundp '*viewer*) *viewer*))
    (pickview :no-menu t)
    (objects (list *robot*))))
  (send (instance dynamic-optimize :init :debug? nil)
	:optimize))

;; (demo-dynamic-optimize)
#|

(send *robot* :fullbody-inverse-kinematics
      (list
       (make-coords
	:pos
	(float-vector
	 300 -50
	 (+ 200 (aref (send *robot* :rleg :end-coords :worldpos) 2))))
       (copy-object (send *robot* :rleg :end-coords :worldcoords))
       (copy-object (send *robot* :lleg :end-coords :worldcoords)))
      :rotation-axis (list nil t t)
      :translation-axis (list t t t)
      :move-target
      (mapcar
       #'(lambda (k) (send *robot* k :end-coords))
       '(:rarm :rleg :lleg))
      :min #F(-1000 -1000 -1000 -200 -200 -200)
      :max #F(1000 1000 1000 200 200 200)
      :root-link-virtual-joint-weight
      #F(0.01 0.01 0.01 1 1 1)
      :target-centroid-pos
      (scale 0.5 (v+ (send *robot* :rleg :end-coords :worldpos)
		     (send *robot* :lleg :end-coords :worldpos)))
      :revert-if-fail nil
      :collision-avoidance-link-pair nil
      :stop 100
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list
		 (send (send *robot* k :end-coords) :parent)))
       '(:rarm :rleg :lleg))
      :debug-view :no-message)
(send *robot* :head :look-at (send *robot* :rarm :end-coords :worldpos))

(load "dynamic-motion-filter.l")
(send *robot* :reset-pose)
(send *robot* :newcoords (make-coords))
(demo-motion-filter
 :target-q
 #f(20.1763 17.1958 -80.2251 80.4068 -30.7286 -4.60553 0.0 -19.4429 -16.0008 -75.4336 68.328 -23.2319 3.2191 0.0 -42.9771 42.0 43.0 43.0 -43.8823 23.0146 59.6359 0.713711 34.7483 14.8923 -7.19139 0.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 0.0)
 :target-b
 (concatenate float-vector
	      (scale 1e-3 #f(-34.7965 24.5765 -40.7364))
	      (reverse '(0.333 0.427 0.384))))


(send *robot*
      :angle-vector
      (float-vector 20.1763 17.1958 -80.2251 80.4068 -30.7286 -4.60553 0.0 -19.4429 -16.0008 -75.4336 68.328 -23.2319 3.2191 0.0 -42.9771 42.0 43.0 43.0 -43.8823 23.0146 59.6359 0.713711 34.7483 14.8923 -7.19139 0.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 0.0))
(send *robot*
      :newcoords
      (make-coords
       :pos (scale 1 #f(-34.7965 24.5765 -40.7364))
       :rpy '(0.333 0.427 0.384)))

(motion-filter-play :filtered-motion a)

(setq
 a
 (instance
  dynamic-optimize
  :init
  :debug? nil
  :start-av
  (progn
    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))
    (send *robot* :reset-pose))
  :start-base-coords
  (copy-seq
   (concatenate
    float-vector
    (send (send (car (send *robot* :links)) :worldcoords) :worldpos)
    (matrix-log (send (send (car (send *robot* :links)) :worldcoords) :worldrot))))
  :end-av
  (progn
    (send *robot* :angle-vector
	  (copy-seq
	   #f(4.3623 1.90788 -118.526 68.8727 -0.384729 -2.96897 4.37937 1.92252 -118.459 69.3164 -0.896309 -2.99165 44.0 0.0 44.0 -9.99994 -34.2603 -35.1868 4.89789 0.994618 46.9578 5.79069 -44.8672 15.0 -86.8598 34.1718 17.9421 0.805567 20.8225 33.4117 -3.79171 -15.0)))
	  ;;#f(-2.312211e-05 -0.000194 -43.5218 84.5925 -41.0707 0.000235 2.312211e-05 0.000194 -43.5218 84.5925 -41.0707 -0.000235 44.0 0.0 0.0 0.0 -94.946 -51.7082 9.76871 0.787682 43.4207 71.0413 -8.7582 15.0 -86.8563 34.1764 17.9421 0.805568 20.8225 33.3932 -3.78892 -15.0)))
    (send *robot* :fix-leg-to-coords (make-coords))
    (copy-seq #f(4.3623 1.90788 -118.526 68.8727 -0.384729 -2.96897 4.37937 1.92252 -118.459 69.3164 -0.896309 -2.99165 44.0 0.0 44.0 -9.99994 -34.2603 -35.1868 4.89789 0.994618 46.9578 5.79069 -44.8672 15.0 -86.8598 34.1718 17.9421 0.805567 20.8225 33.4117 -3.79171 -15.0)))
  ;;(copy-seq #f(-2.312211e-05 -0.000194 -43.5218 84.5925 -41.0707 0.000235 2.312211e-05 0.000194 -43.5218 84.5925 -41.0707 -0.000235 44.0 0.0 0.0 0.0 -94.946 -51.7082 9.76871 0.787682 43.4207 71.0413 -8.7582 15.0 -86.8563 34.1764 17.9421 0.805568 20.8225 33.3932 -3.78892 -15.0)))
  :end-base-coords
  (copy-seq
   (concatenate
    float-vector
    (send (send (car (send *robot* :links)) :worldcoords) :worldpos)
    (matrix-log (send (send (car (send *robot* :links)) :worldcoords) :worldrot))))))
(send a :optimize)

(send a :evaluation-function #f(-34.8241 -10.9087 610.775 0.0 0.0 0.0) #F(0))
