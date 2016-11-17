;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "../robot-param.lisp")
(require "../my-util.lisp")
(require "../contact-state.lisp")
(require "param-ladder.lisp")

;(defvar *robot* (hrp2jsknts-simple-detail))

(defclass big-ladder
  :super param-ladder
  :slots ())

(defmethod big-ladder
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
	       (append
		(mapcar
		 #'(lambda (hoge)
		     (float-vector (/ 300 (tan (deg2rad 70))) 0 300))
		 (make-list 9))
		(list (float-vector (/ 1000 (tan (deg2rad 70))) 0 1000)))
	       :step-color #F(0.3 0.9 0.3)
	       :step-length (make-list 10 :initial-element 900)
	       :step-bottom-length 900
	       :step-radius
	       (append
		(mapcar
		 #'(lambda (hoge)
		     (list (float-vector 90 5 0) (float-vector 90 -5 0)
			   (float-vector -10 -5 0) (float-vector -10 5 0)))
		 (make-list 8))
		(list
		 (list (float-vector 800 5 0) (float-vector 800 -5 0)
		       (float-vector -10 -5 0) (float-vector -10 5 0)))
		(list
		 (list (float-vector 1 1 0) (float-vector 1 -1 0)
		       (float-vector -1 -1 0) (float-vector -1 1 0)))
		)
	       :handrail-height (* 700 (sin (deg2rad 70)))
	       :handrail-radius 15
	       :footrail-radius
	       (append (make-list 9 :initial-element 15) (list 1))
	       :mirror nil
	       )
   (gl::transparent self 0.5)
   self
   )
  (:gen-contact-states
   nil
   (let (leg-can hand-can buf)
     (setq leg-can
	   (mapcar
	    #'(lambda (sl bd)
		(mapcar
		 #'(lambda (k)
		     (instance
		      simple-contact-state :init :name k
		      :contact-coords
		      (cascoords-collection
		       :limb-key k
		       :coords (make-coords
				:pos
				(v+ #F(30 0 0)
				    (send *robot* k :end-coords :worldpos))
				:rot
				(copy-object (send *robot* k :end-coords :worldrot))))
		      :contact-n #F(0 0 1) :force0 #F(0 0 0 0 0 0)
		      :ux 0.5 :uy 0.5 :uz 0.5 :lx 0.1 :ly 0.1
		      :target-coords
		      (make-coords
		       :pos (v+ (map float-vector
				     #'*
				     #F(1 0 1)
				     (send bd :worldpos))
				(float-vector
				 0
				 (aref (send *robot* k :end-coords :worldpos) 1)
				 0))
		       :rot
		       (copy-object
			(send *robot* k :end-coords :worldrot)))))
		 '(:rleg :lleg)))
	    (subseq (send self :step-length) 0 2)
	    (subseq (send self :step-body) 0 2)
	    ))
     (setq hand-can
	   (mapcar
	    #'(lambda (rdlist bdlist)
		(mapcar
		 #'(lambda (rd bd k n)
		     (setq buf
			   (list
			    (transform (send *robot* k :end-coords :worldrot) n)
			    (normalize-vector
			     (scale
			      1
			      (float-vector
			       (*
				(cos (deg2rad 100))
				(* 1 (cos (deg2rad 30))))
			       (*
				(case k
				      (:rarm -1)
				      (:larm 1))
				(sin (deg2rad 100)))
			       (*
				(cos (deg2rad 100))
				(* -1 (sin (deg2rad 30)))))))))
		     (instance
		      simple-contact-state :init :name k
		      :contact-coords (send *robot* k :end-coords)
		      :contact-n n :force0 #F(80 80 80 20 20 20)
		      :ux 0.5 :uy 0.5 :uz 0.5 :lx 0.1 :ly 0.1
		      :target-coords
		      (make-coords
		       :pos (v+ (send bd :worldpos)
				(scale 100 (normalize-vector rd)))
		       :rot
		       (matrix-exponent
			(normalize-vector (apply #'v* (reverse buf)))
			(acos (apply #'v. buf))))
		      ))
		 (subseq rdlist 0 2)
		 (subseq bdlist 0 2)
		 '(:rarm :larm)
		 (list #F(0 -1 0) #F(0 1 0))
		 ))
	    (subseq handrail-direction 4 6)
	    (subseq handrail-body 4 6)))
     (mapcar
      #'(lambda (k)
	  (setq buf (remove-if #'(lambda (cs) (not (eq k (send cs :name))))
			       (flatten (append leg-can hand-can))))
	  (send-all buf :sequence-select buf))
      '(:rarm :larm :rleg :lleg))
     (append leg-can hand-can)))
  )

#|

(require "package://hrpsys_gazebo_atlas/models/drc_ladder.lisp")
(setq *ladder* (drc-ladder))
(send *ladder* :rotate (deg2rad -90) :z)
(send *ladder* :translate #F(1180 0 0) :world)

(setq *big-ladder* (instance big-ladder :init))
(if (not (and (boundp '*viewer*) *viewer*))
    (pickview :no-menu t))
(objects (list *robot* *ladder* *big-ladder*))

(setq *big-ladder* (instance big-ladder :init))
(if (not (and (boundp '*viewer*) *viewer*))
    (pickview :no-menu t))
(objects (list *robot* *big-ladder*))

(defvar *robot-type* :atlas)
(require "motion-sequencer.lisp")
(demo-climb-setup :big-ladder)
(demo-motion-sequence)
