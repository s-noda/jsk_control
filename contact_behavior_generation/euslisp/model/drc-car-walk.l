(defvar *robot-type* :staro)
(require "motion-sequencer.l")
(require "motion-planners/motion-planner.l")
(require "model/drc-car.l")
(send-all (send *robot* :arms :end-coords) :translate (float-vector -80 0 -95) :local)
(setq *climb-obj*
      (instance drc-car :init :with-car? t :trans 0.3))
(if (not (and (boundp '*viewer*) *viewer*)) (pickview :no-menu t))
(send *viewer* :viewsurface :bg-color #F(1 1 1))
(objects (list *robot* *climb-obj*))
(send *viewer* :draw-objects)
(staro-car-pose-ik)

(progn
  (setq *env-collision-check-list*
	(send *climb-obj* :gen-collision-check-list))
  (setq *contact-states* (send *climb-obj* :gen-contact-states))
  (send-all *contact-states* :set-val 'target-direction
	    '(lambda (tc ntc self &rest args)
	       (aref (v- (send tc :worldpos)
			 (send ntc :worldpos)) 1)))
  ;; (send-all *contact-states* :set-val 'gain '(1.5 1 10))
  (mapcar
   '(lambda (cs)
      (cond
       ((find (send cs :name) '(:rleg :lleg))
	(send cs :set-val 'gain '(1.5 1 10)))
       (t
	(send cs :set-val 'gain '(0.5 1 10)))))
   *contact-states*)
  nil)

(defun move-torque-ik
  (move
   &key
   (collision-avoidance-link-pair
    (apply
     'append
     (mapcar
      '(lambda (l)
	 (mapcar
	  #'(lambda (l2) (list l l2))
	  (send *robot* :torso :links)))
      (append (send *robot* :rarm :links)
	      (send *robot* :larm :links))))))
  (dotimes (i 10)
    (let* ((move-target (send-all cs :contact-coords))
	   (target-coords (send-all cs :target-coords))
	   (torso-end-coords (send *robot* :torso :end-coords))
	   (torso-target-coords (send torso-end-coords :copy-worldcoords)))
      (send torso-target-coords :translate move :world)
      (send *robot* :fullbody-inverse-kinematics
	    (cons torso-target-coords target-coords)
	    :move-target (cons torso-end-coords move-target)
	    :link-list
	    (mapcar '(lambda (cs) (send *robot* :link-list (send cs :parent)))
		    (cons torso-end-coords move-target))
	    :collision-avoidance-link-pair collision-avoidance-link-pair
	    :avoid-collision-distance 40
	    :avoid-collision-null-gain 0.8
	    :avoid-collision-joint-gain 0.8
	    :target-centroid-pos nil
	    :debug-view :no-message
	    :stop 10 :revert-if-fail nil
	    )
      )))

(send *robot* :torso :waist-p :min-angle 0)
(staro-car-pose-ik)
;; (send *robot* :translate (float-vector 0 100 0) :world)
(mapcar
 #'(lambda (k move)
     (let* (;;(k :rleg)
	    (cs
	     (car
	      (sort
	       (remove-if
		#'(lambda (cs)
		    (not (eq (send cs :name) k)))
		*contact-states*)
	       #'(lambda (cs1 cs2)
		   (< (+
		       (norm (send (send cs1 :target-coords)
				   :difference-position
				   (send *robot* k :end-coords)))
		       (norm (send (send cs1 :target-coords)
				   :difference-rotation
				   (send *robot* k :end-coords))))
		      (+
		       (norm (send (send cs2 :target-coords)
				   :difference-position
				   (send *robot* k :end-coords)))
		       (norm (send (send cs2 :target-coords)
				   :difference-rotation
				   (send *robot* k :end-coords))))))
	       ))))
       (send *robot* :inverse-kinematics
	     (send (send cs :target-coords)
		   :translate move :world)
	     :move-target (send cs :contact-coords)
	     :link-list
	     (send *robot* :link-list
		   (send (send cs :contact-coords) :parent))
	     :rotation-axis :z
	     :debug-view :no-message)
       ))
 '(:rleg :lleg :rarm)
 (list (float-vector -50 -50 0)
       (float-vector -80 50 0)
       (float-vector -50 -150 0)))

(send *robot* :inverse-kinematics
      (make-coords
       :pos
       (copy-seq (send (send *climb-obj* :get-val 'cup-pane) :worldpos))
       :rpy (list (deg2rad 135) 0 0))
      :move-target (send *robot* :larm :end-coords)
      :link-list (send *robot* :larm :links)
      :debug-view :no-message)

(setq cs
      (let* ((k '(:rarm :larm :rleg :lleg)))
	(now-contact-state
	 :limb-keys k
	 :contact-coords
	 (mapcar
	  '(lambda (k) (send *robot* k :end-coords))
	  k)
	 :contact-n (mapcar '(lambda (k) (float-vector 0 0 1)) k)
	 :force0 (mapcar '(lambda (k) (float-vector 0 0 0 0 0 0)) k)
	 )))
(move-torque-ik (float-vector 0 0 0))

(setq ret nil)
(push (pose-generate-with-contact-state cs) ret)

(move-torque-ik (float-vector 0 0 10))
(push (pose-generate-with-contact-state cs) ret)

(move-torque-ik (float-vector 0 10 0))
(pose-generate-with-contact-state cs)
(move-torque-ik (float-vector 0 10 0))
(push (pose-generate-with-contact-state cs) ret)

(move-torque-ik (float-vector 0 0 -10))
(push (pose-generate-with-contact-state cs) ret)

(push
 (pose-generate-with-contact-state
  (remove-if '(lambda (cs) (eq (send cs :name) :lleg)) cs))
 ret)

(send *robot* :lleg :move-end-pos (float-vector 0 300 0) :world)
(push
 (pose-generate-with-contact-state
  (remove-if '(lambda (cs) (eq (send cs :name) :lleg)) cs))
 ret)

(rsd-serialize :rsd-list ret :file "car-travis-slide.rsd")
(setq ret (flatten (rsd-deserialize :file "car-travis-slide.rsd")))
(rsd-play :rsd-list ret :graph nil)


(require "package://jsk_hrpsys_ros_bridge/euslisp/staro-interface.l")
(staro-init)
