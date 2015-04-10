;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "my-util.l")
(require "../contact-state.l")

(defvar *robot*) ;; (hrp2jsknts-simple-detail))

(defclass drc-ladder-with-cs-candidates
  :super drc-ladder
  :slots (ladder-pane right-pane left-pane))
(defmethod drc-ladder-with-cs-candidates
  (:init
   (&rest
    args
    &key
    (slope 60)
    (step-cnt 9)
    (step-height 267)
    (step-bottom-length 800)
    &allow-other-keys)
   ;; set param
   (send-super* :init args)
   (let* ((c (make-cube (/ (* step-cnt step-height) (sin (deg2rad slope)))
			step-bottom-length 1)))
     (send c :set-color (float-vector 1 0 0))
     (gl::transparent c 0.3)
     (send c :rotate (deg2rad (* -1 slope)) :y)
     (send c :translate
	   (float-vector (+ 100 (/ (* 0.5 step-cnt step-height) (tan (deg2rad slope))))
			 0
			 (/ (* 0.5 step-cnt step-height) (sin (deg2rad slope))))
	   :world)
     (setq ladder-pane c)
     )
   (let* ((c (make-cube (/ (* step-cnt step-height) (sin (deg2rad slope)))
			step-bottom-length 1)))
     (send c :set-color (float-vector 0 1 0))
     (gl::transparent c 0.3)
     (send c :rotate (deg2rad -90) :x)
     (send c :rotate (deg2rad (* -1 slope)) :z)
     (send c :translate
	   (float-vector (/ (* 0.5 step-cnt step-height) (tan (deg2rad slope)))
			 (* -0.5 (+ 100 step-bottom-length))
			 (/ (* 0.5 step-cnt step-height) (sin (deg2rad slope))))
	   :world)
     (setq right-pane c)
     )
   (let* ((c (make-cube (/ (* step-cnt step-height) (sin (deg2rad slope)))
			step-bottom-length 1)))
     (send c :set-color (float-vector 0 0 1))
     (gl::transparent c 0.3)
     (send c :rotate (deg2rad +90) :x)
     (send c :rotate (deg2rad (* +1 slope)) :z)
     (send c :translate
	   (float-vector (/ (* 0.5 step-cnt step-height) (tan (deg2rad slope)))
			 (* +0.5 (+ 100 step-bottom-length))
			 (/ (* 0.5 step-cnt step-height) (sin (deg2rad slope))))
	   :world)
     (setq left-pane c)
     )
   (send self :assoc ladder-pane)
   (send self :assoc right-pane)
   (send self :assoc left-pane)
   self
   )
  (:calc-dist-from-pane
   (pos pane
        &key
        (min-ccl -10)
        (vv (transform (send pane :worldrot)
                       (float-vector 0 0 1)))
        (a (send pane :worldpos))
        (top-face
         (find-if '(lambda (id) (find :top (send id :id)))
                  (send pane :faces))))
   (if (eq :inside (send top-face :insidep pos))
       (- (* 1 (v. vv (v- pos a))) min-ccl)
     1e+6))
  (:gen-contact-states
   nil
   ;; (let (leg-can hand-can buf hand-offset)
   ;;   (setq leg-can
   ;; 	   (mapcar
   ;; 	    #'(lambda (sl bd)
   ;; 		(mapcar
   ;; 		 #'(lambda (k)
   ;; 		     (setq buf
   ;; 			   (transform (send *robot* k :end-coords :worldrot) #F(0 0 1)))
   ;; 		     (instance
   ;; 		      simple-contact-state :init :name k
   ;; 		      :contact-coords
   ;; 		      (cascoords-collection
   ;; 		       :limb-key k
   ;; 		       :coords (make-coords
   ;; 				:pos
   ;; 				(v+ #F(20 0 0)
   ;; 				    (send *robot* k :end-coords :worldpos))
   ;; 				:rot
   ;; 				(copy-object (send *robot* k :end-coords :worldrot))))
   ;; 		      :contact-n #F(0 0 1) :force0 #F(0 0 0 0 0 0)
   ;; 		      :ux 0.3 :uy 0.3 :uz 0.1 :lx 0.05 :ly 0.1
   ;; 		      :target-coords
   ;; 		      (make-coords
   ;; 		       :pos (v+ (send bd :worldpos)
   ;; 				(float-vector
   ;; 				 0
   ;; 				 (* sl (if (eq k :rleg) -0.7 -0.3))
   ;; 				 0))
   ;; 		       :rot
   ;; 		       (m*
   ;; 			(matrix-exponent
   ;; 			 (float-vector 0 0
   ;; 				       (if (eq k :rleg) (deg2rad -0) (deg2rad 0))))
   ;; 			(matrix-exponent
   ;; 			 (normalize-vector (v* #F(0 0 1) buf))
   ;; 			 (acos (v. buf #F(0 0 1))))))))
   ;; 		 '(:rleg :lleg)))
   ;; 	    (subseq (send self :step-length) 0)
   ;; 	    (subseq (send self :step-body) 0)
   ;; 	    ))
   ;;   (setq hand-can
   ;; 	   (mapcar
   ;; 	    #'(lambda (rdlist bdlist)
   ;; 		(mapcar
   ;; 		 #'(lambda (rd bd k n offset)
   ;; 		     (setq buf
   ;; 			   (list
   ;; 			    (transform (send *robot* k :end-coords :worldrot) n)
   ;; 			    (transform
   ;; 			     (matrix-exponent (float-vector 0 0 (deg2rad (if (eq k :rarm) 60 -60))))
   ;; 			     (transform (send (send bd :worldcoords) :worldrot) #F(-1 0 0)))
   ;; 			    ;;(float-vector (* -1 (cos (deg2rad 20)))
   ;; 			    ;;0
   ;; 			    ;;(sin (deg2rad 20)))
   ;; 			    ))
   ;; 		     (instance
   ;; 		      simple-contact-state :init :name k
   ;; 		      :contact-coords (send *robot* k :end-coords)
   ;; 		      :contact-n n
   ;; 		      :force0 #F(50 50 -10 10 10 10)
   ;; 		      :ux 0.3 :uy 0.3 :uz 0.1 :lx 0.1 :ly 0.1
   ;; 		      ;; :rotation-axis :z
   ;; 		      :target-coords
   ;; 		      (make-coords
   ;; 		       :pos (v+ (v+ offset (send bd :worldpos))
   ;; 				(scale (if (null hand-offset)
   ;; 					   0
   ;; 					 (setq hand-offset -100))
   ;; 				       (normalize-vector rd)))
   ;; 		       :rot
   ;; 		       (matrix-exponent
   ;; 			(normalize-vector (apply #'v* (reverse buf)))
   ;; 			(acos (apply #'v. buf))))
   ;; 		      ))
   ;; 		 rdlist bdlist '(:rarm :larm)
   ;; 		 (list (normalize-vector #F(0 -1 0))
   ;; 		       (normalize-vector #F(0 +1 0)))
   ;; 		 (list #F(-0 -0 60) #F(-0 0 60))
   ;; 		 ))
   ;; 	    (subseq handrail-direction 1 10)
   ;; 	    (subseq handrail-body 1 10)))
   ;;   (mapcar
   ;;    #'(lambda (k)
   ;; 	  (setq buf (remove-if #'(lambda (cs) (not (eq k (send cs :name))))
   ;; 			       (flatten (append leg-can hand-can))))
   ;; 	  (send-all buf :sequence-select buf))
   ;;    '(:rarm :larm :rleg :lleg))
   ;;   (append leg-can hand-can))
   (flatten
    (list
     (let* ((place-contact-name (list :rarm :larm :rleg :lleg))
	    (buf
	     (gen-primitive-contact-states
	      self
	      :bodies (flatten step-body)
	      :grasp-contact-name nil
	      :place-contact-name place-contact-name
	      :place-n-vector (list (float-vector 0 0 1)
				    (float-vector 0 0 1)
				    (float-vector 0 0 1)
				    (float-vector 0 0 1))
	      :face-step 100
	      :max-depth 3
	      )))
       buf)
     (let* ((grasp-contact-name (list :rarm :larm))
	    (buf
	     (gen-primitive-contact-states
	      self
	      :bodies (flatten handrail-body)
	      :grasp-contact-name grasp-contact-name
	      :place-contact-name nil
	      :max-depth 3
	      )))
       buf)
     )
    ))
  (:gen-collision-check-list
   nil
   (mapcar
    #'(lambda (sym)
	(list (cons :name (read-from-string (format nil ":~A" sym)))
	      (cons :n
		    (scale -1
			   (transform
			    (send (send self :get-val sym) :worldrot)
			    (float-vector 0 0 1))))
	      (cons :a0 (send (send self :get-val sym) :worldpos))
	      (cons :check-func
		    (list 'lambda '(env ccl)
			  (list
			   'let
			   '((x (funcall (cdr (assoc :dist ccl)))))
			   (list 'send self :calc-dist-from-pane 'x
				 (send self :get-val sym)))))))
    '(ladder-pane left-pane right-pane)))
  )

;; (setq *robot-collision-check-list*
;;   (list
;;    (list
;;     (cons :name :head)
;;     (cons :move-target :head)
;;     (cons
;;      :dist
;;      '(lambda nil (send *robot* :head :end-coords :worldpos)))
;;     (cons :min 30))
;;    (list
;;     (cons :name :right-shoulder)
;;     (cons :move-target (send *robot* :rarm :shoulder-r))
;;     (cons
;;      :dist
;;      '(lambda nil
;; 	(send
;; 	 (send
;; 	  (send *robot* :rarm :shoulder-r) :child-link)
;; 	 :worldpos)))
;;     (cons :min 100))
;;    (list
;;     (cons :name :left-shoulder)
;;     (cons :move-target (send *robot* :larm :shoulder-r))
;;     (cons
;;      :dist
;;      '(lambda nil
;; 	(send
;; 	 (send
;; 	  (send *robot* :larm :shoulder-r) :child-link)
;; 	 :worldpos)))
;;     (cons :min 100))
;;    (list
;;     (cons :name :right-elbow)
;;     (cons :move-target (send *robot* :rarm :elbow-p))
;;     (cons
;;      :dist
;;      '(lambda nil
;; 	(send
;; 	 (send
;; 	  (send *robot* :rarm :elbow-p) :child-link)
;; 	 :worldpos)))
;;     (cons :min 60))
;;    (list
;;     (cons :name :left-elbow)
;;     (cons :move-target (send *robot* :larm :elbow-p))
;;     (cons
;;      :dist
;;      '(lambda nil
;; 	(send
;; 	 (send
;; 	  (send *robot* :larm :elbow-p) :child-link)
;; 	 :worldpos)))
;;     (cons :min 60))
;;    (list
;;     (cons :name :right-knee)
;;     (cons :move-target (send *robot* :rleg :knee-p))
;;     (cons
;;      :dist
;;      '(lambda nil
;; 	(send
;; 	 (send
;; 	  (send *robot* :rleg :knee-p) :child-link)
;; 	 :worldpos)))
;;     (cons :min 150))
;;    (list
;;     (cons :name :left-knee)
;;     (cons :move-target (send *robot* :lleg :knee-p))
;;     (cons
;;      :dist
;;      '(lambda nil
;; 	(send
;; 	 (send
;; 	  (send *robot* :lleg :knee-p) :child-link)
;; 	 :worldpos)))
;;     (cons :min 150))
;;    ))


#|

(setq *robot-type* :hrp2jsknts-collada)
(require "package://motion-sequencer.l")
(require "package://motion-planners/motion-planner.l")
(require "package://model/drc-ladder.l")
(pickview :no-menu t)
(objects (list (setq *climb-obj* (instance drc-ladder-with-cs-candidates :init))
	       (send *climb-obj* :get-val 'ladder-pane)
	       (send *climb-obj* :get-val 'right-pane)
	       (send *climb-obj* :get-val 'left-pane)
	       *robot*
	       ))
(progn (setq *contact-states* (flatten (send *climb-obj* :gen-contact-states))) nil)

(setq *env-collision-check-list*
      (if (find-method *climb-obj* :gen-collision-check-list)
	  (send *climb-obj* :gen-collision-check-list)
	(list
	 (list (cons :name (send *climb-obj* :name))
	       (cons :n (scale -1 (send *climb-obj* :vertical-vector)))
	       (cons :a0 (send *climb-obj* :worldpos)))
	 )))

(setq *contact-states*
      (remove-if
       #'(lambda (cs) (find (send cs :name) (list :larm)))
       *contact-states*))

;; (setq *kca-ik-param* (list :additional-weight-list nil))
(send-all *contact-states* :set-val 'gain '(1.3 1.0 10.0))

(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords :pos (float-vector -100 0 0)))
(send *robot* :larm :shoulder-p :joint-angle -180)

;; ;; for side walk
;; (send *robot* :arms :shoulder-p :joint-angle -180)
;; (send *robot* :rotate (deg2rad 90) :z)
;; (setq *dcsf-use-heuristic* nil)
;; (setq *contact-states*
;;       (remove-if
;;        #'(lambda (cs) (find (send cs :name) (list :rarm :larm)))
;;        *contact-states*))
;; (mapcar
;;  #'(lambda (cs)
;;      (cond
;;       ((find (send cs :name) '(:rleg :lleg))
;;        (send (send cs :target-coords) :rotate (deg2rad 75) :z))))
;;  *contact-states*)

;; ;; for back walk
;; (send *robot* :reset-pose)
;; (send *robot* :fix-leg-to-coords (make-coords :pos (float-vector -100 0 0)))
;; (send *robot* :larm :shoulder-p :joint-angle -180)
;; (send *robot* :arms :shoulder-p :joint-angle -180)
;; (send *robot* :rotate (deg2rad 180) :z)
;; (send *robot* :translate (float-vector 100 0 0) :world)
;; (setq *dcsf-use-heuristic* nil)
;; (setq *contact-states*
;;       (remove-if
;;        #'(lambda (cs) (find (send cs :name) (list :rarm :larm)))
;;        *contact-states*))
;; (mapcar
;;  #'(lambda (cs)
;;      (cond
;;       ((find (send cs :name) '(:rleg :lleg))
;;        (send (send cs :target-coords) :rotate (deg2rad 180) :z))))
;;  *contact-states*)

(setq *rsd*
      (demo-motion-sequence2
       :now-rsd
       (optimize-brli
	:robot
	*robot*
	:contact-states
	(now-contact-state
	 :limb-keys '(:lleg :rleg)
	 ))
       :loop-max 15
       :ik-debug-view :no-message
       :rms-loop-max 10
       :cs-filter-func
       #'(lambda (cs &rest args)
	   (setq cs (apply 'sequencial-select-filter (cons cs args)))
	   (setq cs
		 (remove-if
		  #'(lambda (cs)
		      (cond
		       ((eq (send cs :name) :rleg)
			(>
			 (norm
			  (send (send *robot* :lleg :end-coords) :difference-position
				(send cs :target-coords)))
			 550))
		       ((eq (send cs :name) :lleg)
			(>
			 (norm
			  (send (send *robot* :rleg :end-coords) :difference-position
				(send cs :target-coords)))
			 550))
		       (t nil)))
		  cs))
	   cs)
       ))


#|

(setq *robot-type* :hrp2jsknts-collada)
(require "package://robot-param.l")
(pickview :no-menu t)
(objects (list *robot* (setq *climb-obj* (instance drc-ladder-with-cs-candidates :init))))
(setq *contact-states* (flatten (send *climb-obj* :gen-contact-states)))

