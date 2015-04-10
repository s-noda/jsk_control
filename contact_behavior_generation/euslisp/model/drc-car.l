(require "package://gazebo_drive_simulator/euslisp/polaris-xp900-with-handle.l")

(defvar *goal-contact-state*)

(defclass drc-car
  :super cascaded-link
  :slots (root-obj
	  name
	  drc-car
	  seat-pane
	  seat-back-pane
	  seat-front-pane
	  pedal-pane
	  pedal-front-pane
	  pedal-upper-pane
	  side-pane
	  top-pane
	  front-pane
	  stand-pane
	  handle-pane
	  cup-pane
	  slip-pane
	  ;;
	  front-left-pole
	  back-left-pole
	  top-left-pole
	  ))
(defmethod drc-car
  (:init
   (&rest
    args
    &key
    ((:name nm) :drc-car)
    (depth 30)
    (trans 0.3)
    (with-car? t)
    &allow-other-keys)
   (send-super* :init :name nm args)
   (setq name nm)
   ;; gen-obj
   (setq root-obj (make-cube 10 10 10))
   (send root-obj :set-color #F(0 1 0))
   ;;
   (setq drc-car
	 (instance polaris-xp900-with-handle :init))
   ;; (gl::transparent drc-car 0.5)
   ;;
   (let* ((p (make-cube 450 1400 depth)))
     (send p :set-color (float-vector 1 0 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 50 0 880) :world)
     (setq seat-pane p))
   (let* ((p (make-cube 450 700 depth)))
     (send p :set-color (float-vector 0 1 0))
     (if trans (gl::transparent p trans))
     (send p :rotate (deg2rad -80) :x)
     (send p :translate (float-vector 50 800 500) :world)
     (setq slip-pane p))
   (let* ((p (make-cube 550 1400 depth)))
     (send p :rotate (deg2rad 90) :y)
     (send p :set-color (float-vector 1 1 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 280 0 600) :world)
     (setq seat-front-pane p))
   (let* ((p (make-cube 500 1400 depth)))
     (send p :rotate (deg2rad 80) :y)
     (send p :set-color (float-vector 1 1 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector -250 0 1150) :world)
     (setq seat-back-pane p))
   ;;
   (let* ((p (make-cube 1050 1400 depth)))
     (send p :rotate (deg2rad -90) :y)
     (send p :set-color (float-vector 1 1 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 600 0 850) :world)
     (setq pedal-front-pane p))
   (let* ((p (make-cube 300 1500 depth)))
     (send p :set-color (float-vector 1 0 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 450 0 350) :world)
     (setq pedal-pane p))
   (let* ((p (make-cube 600 400 depth)))
     (send p :set-color (float-vector 0 0 1))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 300 900 250) :world)
     (setq side-pane p))
   (let* ((p (make-cube 260 400 depth)))
     (send p :set-color (float-vector 0 0 1))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 430 400 450) :world)
     (setq pedal-upper-pane p))
   (let* ((p (make-cube 700 1500 depth)))
     (send p :rotate (deg2rad 180) :y)
     (send p :set-color (float-vector 1 0 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector -50 0 1880) :world)
     (setq top-pane p))
   (let* ((p (make-cube 800 1500 depth)))
     (send p :set-color (float-vector 1 1 0))
     (send p :rotate (deg2rad -120) :y)
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 500 0 1550) :world)
     (setq front-pane p))
   (let* ((p (make-cube 2000 1000 depth)))
     (send p :set-color (float-vector 0 0 1))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 0 1000 0) :world)
     (setq stand-pane p))
   ;;
   (let* ((p (make-cube 380 380 10)))
     (send p :set-color (float-vector 0 1 0))
     (send p :rotate (deg2rad -30) :y)
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 430 480 1200) :world)
     (setq handle-pane p)
     ;; (objects (list *robot* *climb-obj* handle-pane))
     )
   (let* ((p (make-cube 150 150 depth)))
     (send p :rotate (deg2rad 45) :z)
     (send p :set-color (float-vector 1 0 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 580 700 980) :world)
     (setq cup-pane p))
   ;;
   (let* ((c (make-cylinder 40 800)))
     (send c :set-color (float-vector 0 1 0))
     (send c :rotate (deg2rad -30) :y)
     (send c :translate (float-vector 720 760 1200) :world)
     (setq front-left-pole c))
   (let* ((c (make-cylinder 40 700)))
     (send c :set-color (float-vector 0 1 0))
     (send c :translate (float-vector -420 760 1200) :world)
     (setq back-left-pole c))
   (let* ((c (make-cylinder 40 750)))
     (send c :set-color (float-vector 0 1 0))
     (send c :rotate (deg2rad 90) :y)
     (send c :translate (float-vector -400 760 1920) :world)
     (setq top-left-pole c))
   ;;
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies
		   (flatten (list seat-pane slip-pane seat-front-pane seat-back-pane pedal-front-pane pedal-pane pedal-upper-pane side-pane top-pane front-pane stand-pane handle-pane cup-pane front-left-pole back-left-pole top-left-pole))
		   :name name))
   ;;
   (mapcar
    #'(lambda (bd) (send root-obj :assoc bd))
    (flatten (list drc-car seat-pane slip-pane seat-front-pane seat-back-pane pedal-front-pane pedal-pane pedal-upper-pane side-pane top-pane front-pane stand-pane handle-pane cup-pane front-left-pole back-left-pole top-left-pole)))
   (send self :assoc root-obj)
   (setq bodies
	 (flatten (list root-obj (if with-car? drc-car))))
   ;; (flatten (list root-obj drc-car seat-pane seat-front-pane seat-back-pane pedal-front-pane pedal-pane)))
   (setq links
	 (flatten (list root-obj (if with-car? drc-car))))
   ;; (flatten (list root-obj drc-car seat-pane seat-front-pane seat-back-pane pedal-front-pane pedal-pane)))
   (setq joint-list nil)
   (send self :init-ending)
   ;;
   self
   )
  (:vertical-vector nil (float-vector 0 0 1))
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
    '(seat-pane seat-back-pane seat-front-pane pedal-pane
		pedal-front-pane front-pane stand-pane
		pedal-upper-pane
		;; top-pane
		)))
  (:gen-contact-states
   (&rest args)
   (flatten
    (mapcar
     #'(lambda (body name)
	 (let* ((place-contact-name (list :rarm :larm :rleg :lleg))
		(buf
		 (gen-primitive-contact-states
		  self
		  :bodies (list body)
		  ;; :contact-plane-obj name
		  :grasp-contact-name nil
		  :place-contact-name (cons :hip place-contact-name)
		  :place-cascoords
		  (cons
		   (send *robot* :get :hip-end-coords)
		   (mapcar '(lambda (k) (send *robot* k :end-coords))
			   place-contact-name))
		  :place-n-vector (list (float-vector 0 0 1)
					(float-vector 0 0 1)
					(float-vector 0 0 1)
					(float-vector 0 0 1)
					(float-vector 0 0 1))
		  :face-step 100
		  :max-depth 3
		  )))
	   (send-all (flatten buf) :set-val 'contact-plane-obj name)
	   buf))
     (list seat-pane pedal-pane stand-pane pedal-upper-pane side-pane)
     (list :seat-pane :pedal-pane :stand-pane :pedal-upper-pane :side-pane)
     )))
  )

(defun staro-car-pose
  nil
  (gen-staro-hip-end-coords)
  (send *robot* :angle-vector (float-vector 10.0 150.0 90.0 90.0 -110.0 0.0 -40.0 45.0 -24.3437 -32.5517 -25.2969 14.4575 -105.139 4.86937 -27.4592 -45.4563 0.0 0.0 -80.0 45.0 -26.0 0.0 0.0 0.0 -80.0 45.0 -26.0 0.0 0.0 0.0 0.0 0.0))
  (mapcar
   '(lambda (j av) (send j :joint-angle av))
   (send *robot* :rarm :joint-list)
   '(10.0 150.0 90.0 90.0 -110.0 0.0 -40.0 45.0))
  (mapcar
   '(lambda (j av) (send j :joint-angle av))
   (send *robot* :larm :joint-list)
   '(-10.0 -30.0 90.0 -90.0 110.0 0.0 40.0 45.0))
  (let* ((c (make-coords :pos (float-vector 100 150 850))))
    (dotimes (i 3)
      (send (car (send *robot* :links))
	    :transform
	    (send
	     (send (send *robot* :get :rleg-crotch-end-coords)
		   :copy-worldcoords)
	     :transformation
	     c)
	    :local)))
  (send *viewer* :draw-objects))

(defun staro-car-pose-ik
  nil
  (gen-staro-hip-end-coords)
  (dotimes (i 3)
    (send *robot* :angle-vector (float-vector 10.0 150.0 90.0 90.0 -110.0 0.0 -40.0 45.0 -24.3437 -32.5517 -25.2969 14.4575 -105.139 4.86937 -27.4592 -45.4563 0.0 0.0 -80.0 45.0 -26.0 0.0 0.0 0.0 -80.0 45.0 -26.0 0.0 0.0 0.0 0.0 0.0))
    (send *robot* :newcoords
	  (make-coords
	   :pos (float-vector -40 370.0 10)
	   :rpy (float-vector 0.0 0.0 0.0))))
  (send *viewer* :draw-objects))

(defun gen-staro-hip-end-coords
  nil
  (cond
   ((not (and (send *robot* :get :rleg-crotch-end-coords)
	      (send *robot* :get :lleg-crotch-end-coords)))
    (send *robot* :reset-pose)
    (send *robot* :legs :knee-p :joint-angle 87)
    (send *robot* :legs :crotch-p :joint-angle -67)
    (send *robot* :fix-leg-to-coords
	  (make-coords
	   :pos (float-vector 300 1000 0)
	   :rpy '(-1.57 0 0))
	  :both)
    (send *viewer* :draw-objects)
    (mapcar
     '(lambda (k kcrotch)
	(send *robot* :put
	      kcrotch
	      (make-cascoords
	       :name kcrotch
	       :parent (send *robot* k :crotch-p :child-link)
	       :coords
	       (let* ((org (send *robot* k :end-coords
				 :copy-worldcoords)))
		 (send org :translate (float-vector -100 0 450))
		 org))))
     '(:rleg :lleg)
     '(:rleg-crotch-end-coords :lleg-crotch-end-coords)))))

(setq *robot-collision-check-list*
  (list
   (list
    (cons :name :head)
    (cons :move-target :head)
    (cons
     :dist
     '(lambda nil (send *robot* :head :end-coords :worldpos)))
    (cons :min 30))
   (list
    (cons :name :right-shoulder)
    (cons :move-target (send *robot* :rarm :shoulder-r))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :rarm :shoulder-r) :child-link)
	 :worldpos)))
    (cons :min 100))
   (list
    (cons :name :left-shoulder)
    (cons :move-target (send *robot* :larm :shoulder-r))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :larm :shoulder-r) :child-link)
	 :worldpos)))
    (cons :min 100))
   (list
    (cons :name :right-elbow)
    (cons :move-target (send *robot* :rarm :elbow-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :rarm :elbow-p) :child-link)
	 :worldpos)))
    (cons :min 60))
   (list
    (cons :name :left-elbow)
    (cons :move-target (send *robot* :larm :elbow-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :larm :elbow-p) :child-link)
	 :worldpos)))
    (cons :min 60))
   (list
    (cons :name :right-knee)
    (cons :move-target (send *robot* :rleg :knee-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :rleg :knee-p) :child-link)
	 :worldpos)))
    (cons :min 30))
   (list
    (cons :name :left-knee)
    (cons :move-target (send *robot* :lleg :knee-p))
    (cons
     :dist
     '(lambda nil
	(send
	 (send
	  (send *robot* :lleg :knee-p) :child-link)
	 :worldpos)))
    (cons :min 30))
   ))

#|

(setq *rsd*
      (demo-motion-sequence2
       :now-rsd
       (optimize-brli
	:robot
	*robot*
	:contact-states
	(now-contact-state
	 :limb-keys '(:rleg :lleg)
	 :contact-coords
	 (list (send *robot* :get :rleg-crotch-end-coords)
	       (send *robot* :get :lleg-crotch-end-coords))
	 ))
       :loop-max
       '(lambda (cnt &rest args)
	  (and
	   (< (aref (send *robot* :rleg :end-coords :worldpos) 2) 100)
	   (< (aref (send *robot* :lleg :end-coords :worldpos) 2) 100)))
       ;; :ik-debug-view :no-message
       :rms-loop-max 10
       ;; :new-rsd-check-func
       ;; '(lambda (rsd)
       ;; 	  (let* ((ret nil))
       ;; 	    (dolist (r rsd)
       ;; 	      (setq ret
       ;; 		    (or ret
       ;; 			(progn
       ;; 			  (send r :draw :pv nil :torque-draw? nil)
       ;; 			  (send *robot* :self-collision-check))))
       ;; 	      )
       ;; 	    (not ret)))
       :cs-filter-func '(lambda (cs &rest args) cs)))

;;

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
	(send cs :set-val 'gain '(0.5 1 10)))
       )
      (send (send cs :get-val 'target-coords) :translate (float-vector 0 0 -10) :world)
      )
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
      (append (cdr (send *robot* :rarm :links))
	      (cdr (send *robot* :larm :links)))))))
  (dotimes (i 10)
    (let* ((move-target (send-all cs :contact-coords))
	   (target-coords (send-all cs :target-coords))
	   (torso-end-coords (car (last move-target)))
	   (torso-target-coords (car (last target-coords))))
      ;;(torso-end-coords (send *robot* :torso :end-coords))
      ;;(torso-target-coords (send torso-end-coords :copy-worldcoords)))
      (send torso-target-coords :translate move :world)
      (send *robot* :fullbody-inverse-kinematics
	    target-coords
	    :move-target move-target
	    :link-list
	    (mapcar '(lambda (cs) (send *robot* :link-list (send cs :parent)))
		    move-target)
	    :collision-avoidance-link-pair collision-avoidance-link-pair
	    :avoid-collision-distance 40
	    :avoid-collision-null-gain 0.8
	    :avoid-collision-joint-gain 0.8
	    :target-centroid-pos nil
	    :debug-view :no-message
	    :stop 15 ;; :revert-if-fail nil
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

;; (send *robot* :inverse-kinematics
;;       (make-coords
;;        :pos
;;        (copy-seq (send (send *climb-obj* :get-val 'cup-pane) :worldpos))
;;        :rpy (list (deg2rad 135) 0 0))
;;       :move-target (send *robot* :larm :end-coords)
;;       :link-list (send *robot* :larm :links)
;;       :debug-view :no-message)

(setq cs
      (let* ((k '(:rarm :rleg :lleg :torso)))
	(now-contact-state
	 :limb-keys k
	 :contact-coords
	 (mapcar
	  '(lambda (k)
	     (cond
	      ;;((eq k :torso)
	      ;; (send *robot* :get :rleg-crotch-end-coords))
	      (t (send *robot* k :end-coords))))
	  k)
	 :contact-n (mapcar '(lambda (k) (float-vector 0 0 1)) k)
	 :force0 (mapcar '(lambda (k) (float-vector 0 0 0 0 0 0)) k)
	 )))

(setq ret nil)
(move-torque-ik (float-vector 0 0 0))
(push (pose-generate-with-contact-state cs) ret)
(move-torque-ik (float-vector 0 0 5))
(push (pose-generate-with-contact-state cs) ret)
(move-torque-ik (float-vector 0 20 0))
(push (pose-generate-with-contact-state cs) ret)

(rsd-serialize :rsd-list ret :file "car-travis-slide.rsd")
(setq ret (flatten (rsd-deserialize :file "car-travis-slide.rsd")))
(rsd-play :rsd-list ret :graph nil)

(require "package://jsk_hrpsys_ros_bridge/euslisp/staro-interface.l")
(staro-init)
