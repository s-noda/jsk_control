
(require "apollo-org.l")

(defclass apollo-with-contact
  :super cascaded-link
  :slots (root-obj
	  name
	  apollo
	  luna-plane
	  ladder-pole-list
	  red-pole
	  blue-pole
	  green-pole
	  yellow-pole
	  foot-plane
	  ))
(defmethod apollo-with-contact
  (:init
   (&rest
    args
    &key
    ((:name nm) :apollo)
    &allow-other-keys)
   (send-super* :init :name nm args)
   (setq name nm)
   ;; gen-obj
   ; (setq root-obj (apollo))
   (setq root-obj (make-cube 1 1 1))
   (send root-obj :set-color #F(0 1 0))
   ;; (send root-obj :set-color #F(0 1 0))
   ;;
   (setq apollo (apollo))
   (send apollo :newcoords
	 (make-coords
	  :pos (float-vector 3350 0 -100)
	  :rpy '(-1.57 0 1.57)))
   ;;
   ;; (setq luna-plane (make-cube 10000 10000 10))
   ;; (send luna-plane :set-color (float-vector 1.0 0.9 0.9))
   ;;
   (setq ladder-pole-list (send self :gen-poles))
   ;;
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies
		   (flatten (list root-obj luna-plane ladder-pole-list))
		   :name name))
   ;;
   (mapcar
    #'(lambda (bd) (send root-obj :assoc bd))
    (flatten (list apollo luna-plane ladder-pole-list)))
   (send self :assoc root-obj)
   (setq bodies (list root-obj apollo))
   (setq links (list root-obj apollo))
   (setq joint-list nil)
   (send self :init-ending)
   ;;
   self
   )
  (:gen-poles
   (&key buf)
   (let* ((w 20) (h 300)
	  (rad (deg2rad 65)) (step 164)
	  a al)
     (dotimes (i 9)
       (print i)
       (setq a (make-cylinder w h))
       (send a :set-color (float-vector 1 0 0))
       (gl::transparent a 0.4)
       (send a :rotate (deg2rad 90) :x)
       (send a :translate
	     (v+ (float-vector 0 (/ h 2.0) 0)
		 (v+
		  (float-vector 605 50 564)
		  (scale i (float-vector (* step (cos rad)) 0 (* step (sin rad)))))
		 )
	     :world)
       (push a al)
       )
     (push (setq red-pole al) buf)
     ;; (send *irtviewer* :objects
     ;; (append (setq *foot-pole* al) (list *robot* *apollo*)))
     ;; (send *viewer* :draw-objects)
     )
   ;;
   (let* ((w 20) (h 340)
	  (rad (deg2rad 65)) (step 164)
	  a al)
     (dotimes (i 2)
       (print i)
       (setq a (make-cylinder w (* 4 h)))
       (send a :set-color (float-vector 0 1 0))
       (gl::transparent a 0.4)
       (send a :rotate (deg2rad 25) :y)
       (send a :translate
	     (v+
	      (float-vector 605 215 564)
	      (scale i (float-vector 0 (* -1 h) 0)))
	     :world)
       (push a al)
       )
     (push (setq green-pole al) buf)
     )
   ;;
   (let* ((w 20) (h 300)
	  (d 495)
	  a al)
     (dotimes (i 2)
       (print i)
       (setq a (make-cylinder w h))
       (send a :set-color (float-vector 0 0 1))
       (gl::transparent a 0.4)
       (send a :rotate (deg2rad 15) :y)
       (send a :translate
	     (v+
	      (float-vector 1300 -270 2150)
	      (scale i (float-vector 0 (* 1 d) 0)))
	     :world)
       (push a al)
       )
     (push (setq blue-pole al) buf)
     ;; (send *irtviewer* :objects
     ;; (append (setq *hand-pole* al) *foot-pole* (list *robot* *apollo*)))
     ;; (send *viewer* :draw-objects)
     )
   ;;
   (let* ((w 25) (h 300)
	  (d 495)
	  a al)
     (dotimes (i 2)
       (print i)
       (setq a (make-cylinder w h))
       (send a :set-color (float-vector 1 1 0))
       (gl::transparent a 0.4)
       (send a :rotate (deg2rad 78) :y)
       (send a :translate
	     (v+
	      (float-vector 1380 -270 2470)
	      (scale i (float-vector 0 (* 1 d) 0)))
	     :world)
       (push a al)
       )
     (push (setq yellow-pole al) buf)
     )
   ;;
   (let* ((w 400) (h 495)
	  a al)
     (dotimes (i 1)
       (print i)
       (setq a (make-cube w h 10))
       (send a :set-color (float-vector 1 0 1))
       (gl::transparent a 0.4)
       (send a :rotate (deg2rad -25) :y)
       (send a :translate
	     (float-vector 1480 -10 2245)
	     :world)
       (push a al)
       )
     (push (setq foot-plane al) buf)
     )
   (flatten buf))
  (:gen-collision-check-list
   nil
   (list
    (list (cons :name :apollo)
	  (cons :n (scale -1 (send self :vertical-vector)))
	  (cons :a0 (send (car red-pole) :worldpos)))))
  (:vertical-vector
   nil
   (float-vector (* -1 (sin (deg2rad 65)))
		 0 (cos (deg2rad 65))))
  ;; (:gen-contact-states nil nil)
  )




#|

eus apollo.l
;; (require "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
(require "../motion-sequencer.l")
(defvar *robot* (instance hrp2jsknts-robot :init))
(send *robot* :reset-manip-pose)
(send *robot* :fix-leg-to-coords (make-coords) :both)
(pickview :no-menu t)
(objects (list *robot* (setq *climb-obj* (instance apollo-with-contact :init))))

(setf (aref *g-vec* 2) 1600)

;; (send-all *contact-states* :set-val 'contact-plane-obj :ladder)

(progn
  (setq *contact-states*
	(flatten
	 (list
	  (gen-primitive-contact-states
	   *climb-obj*
	   :bodies (send *climb-obj* :get-val 'red-pole)
	   :grasp-contact-name;; '(:rarm :larm :rleg :lleg)
	   '(:rleg :lleg)
	   :place-contact-name nil
	   :grasp-contact-parameters
	   (mapcar
	    #'(lambda (k)
		(list :force0
		      (if (find k '(:rarm :larm))
			  (float-vector 50 50 -10 5 5 5)
			(float-vector 0 0 0 0 0 0))
		      :ux 0.7 :uy 0.7 :uz 0.1 :lx 0.04 :ly 0.1))
	    '(:rleg :lleg))
	   :grasp-z-vector
	   (mapcar #'(lambda (a)
		       (cond
			((eq a :rarm) (float-vector 0 0 -1))
			((eq a :larm) (float-vector 0 0 1))
			((eq a :rleg) (float-vector 0 -1 0))
			((eq a :lleg) (float-vector 0 -1 0))))
		   '(:rleg :lleg))
	   :draw? nil
	   )
	  (let* ((cs
		  (gen-primitive-contact-states
		   *climb-obj*
		   :bodies (append (send *climb-obj* :get-val 'green-pole)
				   )
		   :grasp-contact-name '(:rarm :larm)
		   :place-contact-name nil
		   :draw? nil
		   )))
	    (send-all (flatten cs) :set-val 'contact-plane-obj :green-ladder)
	    cs)
	  (gen-primitive-contact-states
	   *climb-obj*
	   :bodies (append (send *climb-obj* :get-val 'blue-pole)
			   (send *climb-obj* :get-val 'yellow-pole)
			   )
	   :grasp-contact-name '(:rarm :larm)
	   :place-contact-name nil
	   :draw? nil
	   )
	  (gen-primitive-contact-states
	   *climb-obj*
	   :bodies (send *climb-obj* :get-val 'foot-plane)
	   :place-contact-name '(:rleg :lleg)
	   :grasp-contact-name nil
	   :max-depth 1
	   ;; :face-step 10
	   :draw? nil
	   )
	  )
	 ))
  (send-all *contact-states* :set-val 'gain '(1.1 1 10 10))
  (send-all *contact-states* :set-val 'target-direction
	    '(lambda (tc ntc self &rest args) (* -1 (aref (v- (send tc :worldpos) (send ntc :worldpos)) 2))))
  nil)

(setq *env-collision-check-list*
      (send *climb-obj* :gen-collision-check-list))

(setq *dcsf-max-dist-scale* 1.4)
(setq *dcsf-min-dist-scale* 0.05)
(setq *dcsf-use-heuristic* nil)

(send *robot* :reset-manip-pose)
(send *robot* :legs :move-end-pos (float-vector 0 0 200))
(send *robot* :legs :crotch-p :joint-angle -15 :relative t)
(send *robot* :fix-leg-to-coords
      (make-coords
       :pos
       (v+
	(scale 2 (float-vector -90 0 -42))
	(send (car (send *climb-obj* :get-val 'foot-plane)) :worldpos)))
      :both)
(send *robot* :legs :ankle-p :joint-angle -25 :relative t)
(send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
(send *viewer* :draw-objects)

(send
 *pickview* :viewer :viewing :look
 (float-vector -500.0 5500.0 5075.0)
 (send *robot* :rleg :end-coords :worldpos))
(send *viewer* :draw-objects)

(progn (setq a (contact-states-filter-check :accept-draw? '(lambda (cs) (eq (send cs :name) :rleg)))) nil)


(require "../motion-planners/motion-planner.l")
;; (send *robot* :reset-pose)
;; (send *robot* :fix-leg-to-coords
;;       (make-coords :pos (float-vector 1010 40 1610))
;;       :both)
;; (send *robot* :rleg :move-end-pos (float-vector 0 25 0) :world)
;; (send *robot* :lleg :move-end-pos (float-vector 0 -25 0) :world)
;; (send *viewer* :draw-objects)
(let* ((rsd1 (optimize-brli
	      :contact-states
	      (now-contact-state :limb-keys '(:rleg :lleg))))
       (rsd2 (optimize-brli
	      :contact-states
	      (now-contact-state :limb-keys '(:rleg :lleg)))))
  (send rsd1 :buf :remove-limb :rleg)
  (send rsd2 :buf :remove-limb :lleg)
  (bench
   (setq
    ret
    (demo-motion-sequence2-with-timer
     :now-rsd rsd1
     :ret (list rsd1 rsd2)
     :rms-loop-max 10 :ik-debug-view nil
     :loop-max '(lambda (&rest args) (< (aref (send *robot* :centroid) 2) 1500)) ;;2390))
     :tag "apollo-climb-test"
     ;; '(lambda (&rest args) (probe-file "/tmp/hoge"))
     ;; :all-limbs '(:lleg :rarm :larm :rleg)
     ;; :remove-limb :lleg
     ))))


#|

(send *robot* :reset-manip-pose)
(send *robot* :fix-leg-to-coords (make-coords) :both)
(demo-motion-sequence
 :now-rsd
 (optimize-brli
  :contact-states
  (now-contact-state :limb-keys '(:rleg :lleg)))
 :rms-loop-max 10 :ik-debug-view nil
 :all-limbs '(:lleg :rarm :larm :rleg)
 :remove-limb :lleg)

(let* ((w 20) (h 340)
       (rad (deg2rad 65)) (step 164)
       a al)
  (dotimes (i 2)
    (print i)
    (setq a (make-cylinder w (* 4 h)))
    (send a :set-color (float-vector 0 1 0))
    (gl::transparent a 0.4)
    (send a :rotate (deg2rad 25) :y)
    (send a :translate
	  (v+
	   (float-vector 605 215 564)
	   (scale i (float-vector 0 (* -1 h) 0)))
	  :world)
    (push a al)
    )
  (send *pickview* :objects
	(append (setq *foot-pole* al) (list *robot* *apollo*)))
  (send *viewer* :draw-objects)
  )

(setq *contact-states*
      (flatten
       (gen-primitive-contact-states
	*climb-obj*
	:grasp-contact-name '(:rarm :larm :rleg :lleg)
	:grasp-contact-parameters
	(mapcar
	 #'(lambda (k)
	     (list :force0
		   (if (find k '(:rarm :larm))
		       (float-vector 50 50 -10 5 5 5)
		       (float-vector 0 0 0 0 0 0))
		   :ux 0.7 :uy 0.7 :uz 0.1 :lx 0.04 :ly 0.1))
	 '(:rarm :larm :rleg :lleg))
	;; :grasp-n-vector
	;; (mapcar #'(lambda (n)
	;; 	    (cond
	;; 	     ((eq n :rarm) (float-vector 0 +1 0))
	;; 	     ((eq n :larm) (float-vector 0 -1 0))
	;; 	     (t (float-vector 0 0 1))))
	;; 	'(:rarm :larm :rleg :lleg))
	:grasp-z-vector
	(mapcar #'(lambda (a)
		    (cond
		     ((eq a :rarm) (float-vector 0 0 -1))
		     ((eq a :larm) (float-vector 0 0 1))
		     ((eq a :rleg) (float-vector 0 -1 0))
		     ((eq a :lleg) (float-vector 0 -1 0))))
		'(:rarm :larm :rleg :lleg))
	:draw? nil
	)))

(defun default-contact-state-filter
  (cs &optional (v #F(0 0 1)) buf)
  (or
   (> (norm (v- (send (send cs :target-coords) :worldpos)
                (send (if (find-method *robot* (send cs :name))
                          (send *robot* (send cs :name) :root-link)
                        (car (send *robot* :links)))
                      :worldpos)))
      (cond
       ((and (find-method *robot* (send cs :name))
             (send *robot* (send cs :name) :links))
        (setq
         buf
         (apply
          #'+
          (mapcar
           #'(lambda (l1 l2) (norm (v- (send l1 :worldpos) (send l2 :worldpos))))
           (cdr (send *robot* (send cs :name) :links))
           (send *robot* (send cs :name) :links))))
        (* 1.1 buf))
       (t 500)))
   (< (apply #'min
             (mapcar
              #'(lambda (j)
                  (norm (v- (send j :worldpos)
                            (send (send cs :target-coords) :worldpos))))
              (let ((csl (send (send cs :contact-coords) :parent)))
                (remove-if #'(lambda (l)
                               (or (eq l csl)
                                   (find l (send csl :child-links))
                                   (find csl (send l :child-links))))
                           (send *robot* :links)))))
      100)))
