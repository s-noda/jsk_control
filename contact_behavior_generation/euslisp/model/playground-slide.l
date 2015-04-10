
(defclass playground-slide
  :super cascaded-link
  :slots (root-obj
	  name
	  seat-pane
	  seat-back-pane
	  seat-front-pane
	  pedal-pane
	  pedal-front-pane
	  side-pane
	  stand-pane
	  handle-pane
	  slip-pane
	  ;;
	  front-left-pole
	  back-left-pole
	  top-left-pole
	  ;;
	  spin-disk-above
	  spin-disk-below
	  disk-spin-joint
	  disk-rotate-joint
	  ))

(defmethod playground-slide
  (:init
   (&rest
    args
    &key
    ((:name nm) :playground-slide)
    (depth 5)
    (trans 0.3)
    (with-car? t)
    &allow-other-keys)
   (send-super* :init :name nm args)
   (setq name nm)
   ;; gen-obj
   (setq root-obj (make-cube 10 10 10))
   (send root-obj :set-color #F(0 1 0))
   ;;
   (let* ((p (make-cube 450 700 depth)))
     (send p :set-color (float-vector 1 0 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 50 0 770) :world)
     (setq seat-pane p))
   (let* ((p (make-cube 390 190 depth)))
     (send p :set-color (float-vector 1 0 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 50 600 300) :world)
     (setq side-pane p))
   (let* ((p (make-cube 450 500 depth)))
     (send p :set-color (float-vector 0 1 0))
     (if trans (gl::transparent p trans))
     (send p :rotate (deg2rad -80) :x)
     (send p :translate (float-vector 50 410 510) :world)
     (setq slip-pane p))
   (let* ((p (make-cube 400 700 depth)))
     (send p :rotate (deg2rad 90) :y)
     (send p :set-color (float-vector 1 1 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 280 0 550) :world)
     (setq seat-front-pane p))
   (let* ((p (make-cube 500 700 depth)))
     (send p :rotate (deg2rad 80) :y)
     (send p :set-color (float-vector 1 1 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector -250 0 1050) :world)
     (setq seat-back-pane p))
   ;;
   (let* ((p (make-cube 400 700 depth)))
     (send p :rotate (deg2rad -90) :y)
     (send p :set-color (float-vector 1 1 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 600 0 550) :world)
     (setq pedal-front-pane p))
   (let* ((p (make-cube 300 700 depth)))
     (send p :set-color (float-vector 1 0 0))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 450 0 340) :world)
     (setq pedal-pane p))
   ;; (let* ((p (make-cube 300 400 depth)))
   ;;   (send p :set-color (float-vector 0 0 1))
   ;;   (if trans (gl::transparent p trans))
   ;;   (send p :translate (float-vector 300 550 250) :world)
   ;;   ;; (setq side-pane p)
   ;;   )
   (let* ((p (make-cube 2000 1000 depth)))
     (send p :set-color (float-vector 0 0 1))
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 0 650 0) :world)
     (setq stand-pane p))
   ;;
   (let* ((p (make-cube 380 380 10)))
     (send p :set-color (float-vector 0 1 0))
     (send p :rotate (deg2rad -30) :y)
     (if trans (gl::transparent p trans))
     (send p :translate (float-vector 430 130 1200) :world)
     (setq handle-pane p)
     )
   ;;
   (let* ((c (make-cylinder 40 800)))
     (send c :set-color (float-vector 0 1 0))
     (send c :rotate (deg2rad -30) :y)
     (send c :translate (float-vector 720 410 1200) :world)
     (setq front-left-pole c))
   (let* ((c (make-cylinder 40 700)))
     (send c :set-color (float-vector 0 1 0))
     (send c :translate (float-vector -420 410 1200) :world)
     (setq back-left-pole c))
   (let* ((c (make-cylinder 40 750)))
     (send c :set-color (float-vector 0 1 0))
     (send c :rotate (deg2rad 90) :y)
     (send c :translate (float-vector -400 410 1810) :world)
     (setq top-left-pole c))
   ;;
   (let* ((disk (make-cylinder 200 10)))
     (send disk :set-color (float-vector 0 0 1))
     (if trans (gl::transparent disk trans))
     (send disk :translate (float-vector 50 300 770) :world)
     (setq spin-disk-below disk)
     )
   (let* ((disk (make-cylinder 200 10)))
     (send disk :set-color (float-vector 0 0 1))
     (if trans (gl::transparent disk trans))
     (send disk :translate (float-vector 50 300 (+ 10 770)) :world)
     (setq spin-disk-above disk)
     )
   ;;
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies
		   (flatten (list seat-pane slip-pane seat-front-pane seat-back-pane pedal-front-pane pedal-pane side-pane stand-pane handle-pane front-left-pole back-left-pole top-left-pole))
		   :name name))
   (if spin-disk-above
       (setq spin-disk-above
	     (instance bodyset-link :init
		       (make-cascoords
			:coords (send spin-disk-above :copy-worldcoords))
		       :bodies (list spin-disk-above)
		       :name :spin-disk-above)))
   (if spin-disk-below
       (setq spin-disk-below
	     (instance bodyset-link :init
		       (make-cascoords
			:coords
			(send (send spin-disk-below :copy-worldcoords)
			      :translate (float-vector 0 50 0) :world))
		       :bodies (list spin-disk-below)
		       :name :spin-disk-below)))
   ;;
   ;; def joint
   (if spin-disk-below
       (let* ((plink root-obj)
	      (clink spin-disk-below)
	      (joint (instance rotational-joint :init :min -180.0 :max 180.0
			       :name "disk-rotate-joint"
			       :axis :x
			       :child-link clink
			       :parent-link plink))
	      )
	 (send clink :add-joint joint)
	 (send clink :add-parent-link plink)
	 (send plink :assoc clink)
	 (send joint :joint-angle 0)
	 (setq disk-rotate-joint joint)
	 ))
   (if spin-disk-above
       (let* ((plink spin-disk-below)
	      (clink spin-disk-above)
	      (joint (instance rotational-joint :init :min -180.0 :max 180.0
			       :name "disk-spin-joint"
			       :axis :z
			       :child-link clink
			       :parent-link plink))
	      )
	 (send clink :add-joint joint)
	 (send clink :add-parent-link plink)
	 (send plink :assoc clink)
	 (send (send joint :get-val 'default-coords) :newcoords
	       (make-coords :pos (float-vector 0 -50 10)))
	 (send joint :joint-angle 0)
	 (setq disk-spin-joint joint)
	 ))
   ;;
   (mapcar
    #'(lambda (bd) (send root-obj :assoc bd))
    (flatten (list seat-pane slip-pane seat-front-pane seat-back-pane pedal-front-pane pedal-pane side-pane stand-pane handle-pane front-left-pole back-left-pole top-left-pole)))
   (send self :assoc root-obj)
   (setq bodies (flatten (list root-obj spin-disk-below spin-disk-above)))
   (setq links (flatten (list root-obj spin-disk-below spin-disk-above)))
   (setq joint-list (flatten (list disk-spin-joint disk-rotate-joint)))
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
    nil))
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

;; (objects (list (setq *climb-obj* (instance playground-slide :init))))

#|

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(setq *robot* (instance hrp2jsknts-robot :init))
(objects (list *robot* (setq *climb-obj* (instance playground-slide :init))))

(send *robot* :fix-leg-to-coords
      (send
       (send (send *climb-obj* :get-val 'spin-disk-above) :copy-worldcoords)
       :translate (float-vector 50 0 0) :world)
      :both)
(send (send *climb-obj* :get-val 'spin-disk-above)
      :assoc *robot*)
(send (car (send *climb-obj* :joint-list)) :joint-angle 30)
(send (cadr (send *climb-obj* :joint-list)) :joint-angle -30)
(send *viewer* :draw-objects)

