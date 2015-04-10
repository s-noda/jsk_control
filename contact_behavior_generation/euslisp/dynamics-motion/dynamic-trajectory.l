
;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsk)
(require "my-util.l")
(require "../robot-state-data2.l")
(require "../optimize-brli.l")
(require "../util/spline.l")
(require "dynamic-torque-util.l")

(defun gen-vel-list
  (pos freq)
  (mapcar #'(lambda (v1 v2) (scale freq (v- v1 v2)))
	  (cdr pos) pos))

(defun ik-wrapper
  (&rest args)
  (apply #'simple-fullbody args))

(defun base-coords-fix
  (coords
   &key
   (cnt 3)
   (robot *robot*)
   (base (car (send robot :links)))
   (base-coords (copy-object (send base :worldcoords)))
   (debug? nil))
  (send *robot* :transform
	(send base-coords :transformation coords)
	:local)
  (send-all (send *robot* :links) :worldcoords)
  (cond
   ((> (norm (send coords :difference-position (send base :worldcoords))) 1)
    (if debug?
	(format t "[base-coords-fix] transformatino failure!! ~A/3~%" cnt))
    (cond
     ((zerop cnt) (format t "[base-coords-fix] abort!!~%"))
     (t (base-coords-fix coords :cnt (- cnt 1) :robot robot :base base))))))

;; (defun base-coords-fix
;;   (coords
;;    &key
;;    (robot *robot*)
;;    (base (car (send robot :links))))
;;   (let* ((robot-coords (send robot :worldcoords))
;; 	 (base-coords (send base :worldcoords))
;; 	 (diffpos (v- (send coords :worldpos)
;; 		      (send base-coords :worldpos)))
;; 	 (diffrot
;; 	  (m* (send coords :worldrot)
;; 	      (transpose (send base-coords :worldrot)))))
;;     (send robot :newcoords
;; 	  (make-coords
;; 	   :pos
;; 	   (v+ diffpos (send robot-coords :worldpos))
;; 	   :rot
;; 	   (m* diffrot (send robot-coords :worldrot))))
;;     (send robot :worldcoords)))

(defun xyzrpy
  (coords)
  (concatenate
   float-vector
   (send coords :worldpos)
   (matrix-log (send coords :worldrot))))

(defclass trajectory-elem
  :super propertied-object
  :slots (position
	  velocity
	  acceleration
	  coords
	  dx
	  ddx
	  w
	  dw
	  ))
(defmethod trajectory-elem
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((cadr args)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  (:init
   (&rest
    args
    &key
    (static? t)
    &allow-other-keys
    )
   (mapcar
    #'(lambda (val key)
	(send self key val))
    (cdr args) args)
   (cond
    (static?
     (send self :velocity (or velocity (scale 0 position)))
     (send self :acceleration (or acceleration (scale 0 position)))
     (send self :dx (or dx (float-vector 0 0 0)))
     (send self :ddx (or ddx (float-vector 0 0 0)))
     (send self :w (or w (float-vector 0 0 0)))
     (send self :dw (or dw (float-vector 0 0 0)))
     ))
   )
  )

(defun gen-dynamic-trajectory
  (&key
   freq position-list coords-list
   (ret
    (mapcar
     #'(lambda (pp p np pc c nc)
	 (let* ((pdp (scale freq (v- p pp)))
		(ndp (scale freq (v- np p)))
		(pdc (scale freq (v- (send c :worldpos)
				     (send pc :worldpos))))
		(ndc (scale freq (v- (send nc :worldpos)
				     (send c :worldpos))))
		(pw (scale freq
			   (transform
			    (send pc :worldrot)
			    (send pc :difference-rotation c))))
		(nw (scale freq
			   (transform
			    (send c :worldrot)
			    (send c :difference-rotation nc)))))
	   (instance trajectory-elem
		     :init
		     :position p
		     :velocity (scale 0.5 (v+ pdp ndp))
		     :acceleration (scale freq (v- ndp pdp))
		     :coords c
		     :dx (scale 0.5 (v+ pdc ndc))
		     :ddx (scale freq (v- ndc pdc))
		     :w (scale 0.5 (v+ pw nw))
		     :dw (scale freq (v- nw pw)))))
     (cddr position-list) (cdr position-list) position-list
     (cddr coords-list) (cdr coords-list) coords-list))
   (fix-length nil)
   &allow-other-keys)
  (cond
   (fix-length
    (append
     (list
      (instance trajectory-elem :init
		:position (nth 0 position-list)
		:velocity (scale 0 (nth 0 position-list))
		:acceleration (scale 0 (nth 0 position-list))
		:coords (nth 0 coords-list)
		:dx (float-vector 0 0 0)
		:ddx (float-vector 0 0 0)
		:w (float-vector 0 0 0)
		:dw (float-vector 0 0 0)))
     ret
     (list
      (instance trajectory-elem :init
		:position (nth (- (length position-list) 1) position-list)
		:velocity (scale 0 (nth 0 position-list))
		:acceleration (scale 0 (nth 0 position-list))
		:coords (nth (- (length coords-list) 1) coords-list)
		:dx (float-vector 0 0 0)
		:ddx (float-vector 0 0 0)
		:w (float-vector 0 0 0)
		:dw (float-vector 0 0 0)))))
   (t ret)))

(defun gen-trajectory
  (&key
   (fix-target
    (mapcar
     #'(lambda (k) (send *robot* k :end-coords))
     '(:rleg :lleg)))
   (fix-coords
    (progn
      (cond
       ((and (boundp '*fix-leg-coords*) *fix-leg-coords*)
	*fix-leg-coords*
	;(send *viewer* :draw-objects)
	)
       (t
	(mapcar
	 #'(lambda (cs) (copy-object (send cs :worldcoords)))
	 fix-target)))))
   (move-target nil)
   (move-coords-list nil)
   (base-cascoords
    (make-cascoords
     :init :link-list
     :parent (car (send *robot* :links))
     :coords (copy-object (send (car (send *robot* :links))
				:worldcoords))))
   (start-av (if (and (boundp '*start*) *start*)
		 (send *robot* :angle-vector *start*)
	       (copy-object (send *robot* :angle-vector))))
   (start-base-pos
    (progn
      (cond
       ((and (boundp '*fix-leg-coords*) *fix-leg-coords*)
	(send *robot* :fix-leg-to-coords
	      (car *fix-leg-coords*) :rleg)
	;(send *viewer* :draw-objects)
	))
      (xyzrpy (send (car (send *robot* :links)) :worldcoords))))
   (end-av (if (and (boundp '*end*) *end*)
	       (send *robot* :angle-vector *end*)
	     (copy-object (send *robot* :angle-vector))))
   (end-base-pos
    (progn
      (cond
       ((and (boundp '*fix-leg-coords*) *fix-leg-coords*)
	(send *robot* :fix-leg-to-coords
	      (car *fix-leg-coords*) :rleg)
	;(send *viewer* :draw-objects)
	))
      (xyzrpy (send (car (send *robot* :links)) :worldcoords))))
   (time-scale 1.0) ; for animation
   (split-cnt 10)
   (start-base-vel #F(0 0 0 0 0 0))
   (start-base-acc #F(0 0 0 0 0 0))
   (end-base-vel #F(0 0 0 0 0 0))
   (end-base-acc #F(0 0 0 0 0 0))
   (through-pos-list
    (list (scale 0.5 (v+ start-base-pos end-base-pos))))
   (through-vel-list
    (mapcar #'(lambda (hoge) nil) through-pos-list))
   (through-acc-list
    (mapcar #'(lambda (hoge) nil) through-pos-list))
   (av-split
    (let ((buf nil) tmp)
      (dotimes (i (+ split-cnt 1))
	(setq tmp (/ (* 1.0 (- split-cnt i)) split-cnt))
	(push (v+ (scale tmp end-av) (scale (- 1.0 tmp) start-av)) buf))
      buf))
   (time-list
    (let ((buf nil) (len (+ 1 (length through-pos-list))))
      (dotimes (i (+ 1 len))
	(push (* time-scale (/ (* 1.0 (- len i)) len)) buf))
      buf))
   (spline
    (solve-spline
     :time-list time-list
     :d (length start-base-pos)
     :p (mapcar
	 #'(lambda (v)
	     (cond
	      ((listp v) v)
	      ((vectorp v) (list (cons :vector v)))))
	 (append (list start-base-pos)
		 through-pos-list
		 (list end-base-pos)))
     :dp (mapcar
	  #'(lambda (v)
	      (cond
	       ((listp v) v)
	       ((vectorp v) (list (cons :vector v)))))
	  (append (list start-base-vel)
		  through-vel-list
		  (list end-base-vel)))
     :ddp (mapcar
	   #'(lambda (v)
	       (cond
		((listp v) v)
		((vectorp v) (list (cons :vector v)))))
	   (append (list start-base-acc)
		   through-acc-list
		   (list end-base-acc)))))
   (time-buf 0)
   (callback #'(lambda (av) nil))
   (draw? nil)
   ;;
   error-list
   av-list
   coords-list
   base-coords-tmp
   ik-target
   &allow-other-keys
   )
  (mapcar
   #'(lambda (av &rest target-coords)
       ;;(print target-coords)
       (send *robot* :angle-vector av)
       (setq base-coords-tmp
	     (calc-spline spline time-buf (length start-base-pos)))
       (setq base-coords-tmp
	     (make-coords :pos (subseq base-coords-tmp 0 3)
			  :rpy
			  (reverse
			   (coerce (subseq base-coords-tmp 3 6)
				   cons))))
       (base-coords-fix base-coords-tmp)
       (setq ik-target
	     (append
	      (mapcar
	       #'(lambda (cs fc)
		   (list (cons :target cs)
			 (cons :coords fc)))
	       fix-target fix-coords)
	      (mapcar
	       #'(lambda (mc tc)
		   (list (cons :target mc)
			 (cons :coords tc)))
	       move-target (flatten target-coords))
	      (list
	       (list (cons :target base-cascoords)
		     (cons :coords (copy-object base-coords-tmp))))))
       (ik-wrapper
	:target ik-target
	:balance-leg nil
	:target-centroid-pos nil
	:stop 30
	:revert-if-fail nil
	:collision-avoidance-link-pair nil
	:max #F(500 500 1000 300 300 300)
	:min #F(-500 -500 -1000 -300 -300 -300))
       (cond
	(draw?
	 (send *viewer* :draw-objects :flush nil)
	 (send (concatenate
		float-vector
		(subseq (send *robot* :centroid) 0 2)
		(float-vector
		 (aref (send *robot* :rleg :end-coords :worldpos) 2)))
	       :draw-on :flush nil :color #F(1 0 0))
	 (send *viewer* :viewsurface :flush)
	 (unix:usleep (round (* time-scale (/ 1.0 split-cnt) 1e+6)))))
       (funcall callback (send *robot* :angle-vector))
       (push (copy-object (send *robot* :angle-vector)) av-list)
       (push (copy-object base-coords-tmp) coords-list)
       (push
	(mapcar #'(lambda (tar)
		    (concatenate
		     float-vector
		     (scale 1e-3 (send (cdr (assoc :target tar))
				       :difference-position
				       (cdr (assoc :coords tar))))
		     (send (cdr (assoc :target tar))
			   :difference-rotation
			   (cdr (assoc :coords tar)))))
		ik-target)
	error-list)
       (setq time-buf
	     (+ time-buf (* time-scale (/ 1.0 split-cnt)))))
   av-split
   (if move-target
       (apply #'mapcar (cons #'list move-coords-list))
     (mapcar #'(lambda (hoge) nil) av-split)))
  (send (send base-cascoords :parent) :dissoc base-cascoords)
  (list (cons :error (reverse error-list))
	(cons :position (reverse av-list))
	(cons :coords (reverse coords-list))))

(defun demo-trajectory
  (&rest args)
  (let ((buf (apply #'gen-trajectory args)))
    (gen-dynamic-trajectory
     :freq 10.0
     :position-list (cdr (assoc :position buf))
     :coords-list (cdr (assoc :coords buf)))))

(defun calc-trajectory-torque
  (&rest
   args
   &key
   (freq 10.0)
   (split-cnt 10)
   (contact-states
    (remove-if
     #'(lambda (rsd) (not (find (send rsd :name) '(:rleg :lleg))))
     (send *sample-rsd* :contact-states)))
   (trajectory (apply #'gen-trajectory args))
   (dynamic-trajectory
    (gen-dynamic-trajectory
     :freq freq
     :position-list (cdr (assoc :position trajectory))
     :coords-list (cdr (assoc :coords trajectory))))
   (f0-list-list
    (make-list (length dynamic-trajectory)
	       :initial-element
	       (make-list (length contact-states)
			  :initial-element #F(0 0 0))))
   (m0-list-list
    (make-list (length dynamic-trajectory)
	       :initial-element
	       (make-list (length contact-states)
			  :initial-element #F(0 0 0))))
   (draw? nil)
   &allow-other-keys)
  ;;(setq fuga dynamic-trajectory)
  (mapcar
   #'(lambda (traj f0-list m0-list)
       (send *robot* :angle-vector
	     (copy-object (send traj :position)))
       (base-coords-fix (copy-object (send traj :coords)))
;;       (send *viewer* :draw-objects)
;;       (send (send traj :coords) :draw-on :flush t :color #F(1 0 0))
;;       (send (send (car (send *robot* :links)) :worldcoords) :draw-on :flush t :color #F(0 1 0))
;;       (read-line)
       (calc-torque-check
	:contact-states
	(mapcar
	 #'(lambda (cs f0 m0)
	     (list
	      (cons :f0 f0)
	      (cons :m0 m0)
	      (cons :link (send (send cs :contact-coords) :parent))
	      (cons :worldcoords
		    (copy-object
		     (send (send cs :contact-coords) :worldcoords)))))
	 contact-states f0-list m0-list)
	:root-angular-velocity (send traj :w)
	:root-spacial-velocity (scale 1e-3 (send traj :dx))
	:root-angular-acceleration (send traj :dw)
	:root-spacial-acceleration (scale 1e-3 (send traj :ddx))
	:dq (map float-vector #'deg2rad (send traj :velocity))
	:ddq (map float-vector #'deg2rad (send traj :acceleration))
	))
   dynamic-trajectory f0-list-list m0-list-list))



#|

