;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "../my-util.l")
(require "../contact-state.l")
(require "param-ladder.l")

(defvar *robot*) ;; (hrp2jsknts-simple-detail))

(defclass kirin-ladder
  :super param-ladder
  :slots ())

(defmethod kirin-ladder
  (:init
   (&rest
    args
    &key
    ((:name name) :kirin-ladder)
    buf
    &allow-other-keys)
   (send-super :init
	       :name name
	       :step-vector
	       (list (float-vector (/ 200 (tan (deg2rad 70))) 0 200)
		     (float-vector (/ 200 (tan (deg2rad 70))) 0 200)
		     (float-vector (/ 200 (tan (deg2rad 70))) 0 200)
		     (float-vector (/ 210 (tan (deg2rad 70))) 0 210)
		     (float-vector (/ 270 (tan (deg2rad 70))) 0 270)
		     (float-vector (/ 270 (tan (deg2rad 70))) 0 270))
	       :step-color #F(0.3 0.3 0.3)
	       :step-length '(420 390 360 330 290 250)
	       :step-bottom-length 450
	       :step-radius
	       (list
		(setq buf (list #F(40 5 0) #F(40 -5 0) #F(-40 -5 0) #F(-40 5 0)))
		(copy-object buf)
		(copy-object buf)
		(list #F(230 5 0) #F(230 -5 0) #F(-30 -5 0) #F(-30 5 0))
		(setq buf (list #F(1 1 0) #F(1 -1 0) #F(-1 -1 0) #F(-1 1 0)))
		(copy-object buf))
	       :handrail-height -30
	       :handrail-radius 15
	       :footrail-radius 15
	       :mirror 'butlast
	       )
   (send-all (append (flatten handrail-body)
		     (flatten mirror-bodies))
	     :put :no-grasp t)
   self
   )
  (:gen-contact-states
   nil
   (let (leg-can hand-can buf)
     (setq leg-can
	   (mapcar
	    #'(lambda (sl bd off)
		(mapcar
		 #'(lambda (k)
		     (setq buf
			   (transform (send *robot* k :end-coords :worldrot) #F(0 0 1)))
		     (instance
		      simple-contact-state :init :name k
		      :contact-coords
		      (cascoords-collection
		       :limb-key k
		       :coords (make-coords
				:pos
				(v+ (v- #F(80 0 0) off)
				    (send *robot* k :end-coords :worldpos))
				:rot
				(copy-object (send *robot* k :end-coords :worldrot))))
		      :contact-n #F(0 0 1) :force0 #F(0 0 0 0 0 0)
		      :ux 0.3 :uy 0.3 :uz 0.3 :lx 0.05 :ly 0.1
		      :target-coords
		      (make-coords
		       :pos (v+ (v+ off (send bd :worldpos))
				(float-vector
				 0
				 (* sl (if (eq k :rleg) -0.7 -0.3))
				 0))
		       :rot
		       (matrix-exponent
			(normalize-vector (v* #F(0 0 1) buf))
			(acos (v. buf #F(0 0 1)))))))
		 '(:rleg :lleg)))
	    (subseq (send self :step-length) 0 4)
	    (subseq (send self :step-body) 0 4)
	    (list #F(0 0 0) #F(0 0 0) #F(0 0 0) #F(50 0 0))
	    ))
     (setq hand-can
	   (mapcar
	    #'(lambda (rdlist bdlist)
		(mapcar
		 #'(lambda (rd bd k n offset)
		     (setq buf
			   (list
			    (transform (send *robot* k :end-coords :worldrot) n)
			    (float-vector (* -1 (cos (deg2rad 20)))
					  0
					  (sin (deg2rad 20)))))
		     (instance
		      simple-contact-state :init :name k
		      :contact-plane-obj k ;; tmp
		      :contact-coords (send *robot* k :end-coords)
		      :contact-n n
		      :force0 #F(50 50 -10 5 5 5)
		      :ux 0.7 :uy 0.7 :uz 0.1 :lx 0.04 :ly 0.1
		      :target-coords
		      (make-coords
		       :pos (v+ (v+ offset (send bd :worldpos))
				(scale 290 (normalize-vector rd)))
		       :rot
		       (matrix-exponent
			(normalize-vector (apply #'v* (reverse buf)))
			(acos (apply #'v. buf))))
		      ))
		 rdlist bdlist '(:rarm :larm)
		 (list (normalize-vector #F(-0.25 -1 -0.3))
		       (normalize-vector #F(-0.25 +1 -0.3)))
		 (list #F(-15 -50 0) #F(-15 50 0))
		 ))
	    (subseq handrail-direction 4)
	    (subseq handrail-body 4)))
     (mapcar
      #'(lambda (k)
	  (setq buf (remove-if #'(lambda (cs) (not (eq k (send cs :name))))
			       (flatten (append leg-can hand-can))))
	  (send-all buf :sequence-select buf))
      '(:rarm :larm :rleg :lleg))
     (append leg-can hand-can)))
  ;;
  (:gen-collision-check-list
   nil
   (list
    (list (cons :name :kirin-ladder)
	  (cons :n (scale -1 (send self :vertical-vector)))
	  (cons :a0 (send self :worldpos))
	  (cons :check-func
		'(lambda (env ccl)
		   (let* ((x (funcall (cdr (assoc :dist ccl)))))
		     (if (> (aref x 2) 1350) 1e+6)))))))
  )


#|
(load "param-ladder.l")
(objects (list (setq *kirin-ladder* (instance kirin-ladder :init))))

