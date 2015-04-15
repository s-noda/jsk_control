
(defvar *goal-contact-state*)

(defclass four-leg-seat
  :super cascaded-link
  :slots (root-obj
	  name
	  seat-plane
	  depth width height
	  thick
	  four-legs
	  ))
(defmethod four-leg-seat
  (:init
   (&rest
    args
    &key
    ((:name nm) :four-leg-seat)
    ((:depth d) 2000)
    ((:width w) 750)
    ((:height h) (- 440 0)) ;; offset 40 mm
    ((:thick thk) 100)
    &allow-other-keys)
   (send-super* :init :name nm args)
   (setq depth d width w height h thick thk)
   (setq name nm)
   ;; gen-obj
   (setq root-obj (make-cube 1 1 1))
   (send root-obj :set-color #F(0 1 0))
   ;;
   (setq seat-plane (make-cube depth width thick))
   (send seat-plane :set-color (float-vector 0.6 0.3 0.3))
   (send seat-plane :translate
	 (float-vector 0 0 (- height (/ thick 2.0)))
	 :world)
   ;;
   (let* ((radius 30)
	  (offset 30)
	  (width (- (- width (* 2 radius)) offset))
	  (depth (- (- depth (* 2 radius)) offset)))
     (setq four-legs
	   (mapcar
	    #'(lambda (y x r)
		(let* ((obj (make-cylinder r (- height (/ thick 2.0)))))
		  (send obj :translate (float-vector x y 0) :world)
		  (send obj :set-color (float-vector 0.8 0.8 0.8))
		  obj))
	    (list (/ width 2.0) (/ width 2.0) (/ width -2.0) (/ width -2.0))
	    (list (/ depth 2.0) (/ depth -2.0) (/ depth -2.0) (/ depth 2.0))
	    (list radius radius radius radius))))
   ;;
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies
		   (flatten (list four-legs seat-plane))
		   :name name))
   ;;
   (mapcar
    #'(lambda (bd) (send root-obj :assoc bd))
    (flatten (list four-legs seat-plane)))
   (send self :assoc root-obj)
   (setq bodies (list root-obj))
   (setq links (list root-obj))
   (setq joint-list nil)
   (send self :init-ending)
   ;;
   self
   )
  (:vertical-vector nil (float-vector 0 0 1))
  (:calc-dist-from-seat-plane
   (pos &key
	(min-ccl -10)
	(vv (send self :vertical-vector))
	(a (send seat-plane :worldpos))
	(xv (transform (send seat-plane :worldrot) #F(0 1 0)))
	(yv (transform (send seat-plane :worldrot) #F(1 0 0)))
	(x (abs (v. xv (v- pos a))))
	(y (abs (v. yv (v- pos a))))
	(w (/ width 2.0)) (h (/ depth 2.0)))
   (if (and (< x w) (< y h))
       (- (* 1 (v. vv (v- pos a))) min-ccl)
     1e+6))
  (:gen-collision-check-list
   nil
   (list
    (list (cons :name name)
	  (cons :n (scale -1 (send self :vertical-vector)))
	  (cons :a0 (send seat-plane :worldpos))
	  (cons :check-func
		(list 'lambda '(env ccl)
		      (list 'let
			    (append
			     '((x (funcall (cdr (assoc :dist ccl)))))
			     (list (list 'slf self)))
			    '(send slf :calc-dist-from-seat-plane x)))))))
  (:gen-contact-states
   (&key
    (dx 100) (dy 100)
    (width (* 100 (round (/ (min (/ width 2.0) 1000) 100))))
    (depth (* 100 (round (/ (min (/ depth 2.0) 1000) 100))))
    (x (* -1 width)) (y (* -1 height))
    (gain '(1.0 1.0 1.0))
    (cascoords
     (list (list (cons :name :hip)
		 (cons :cascoords
		       (list
			(send *robot* :get :hip-end-coords)))
		 (cons :translation-axis '(t))
		 (cons :rotation-axis '(t)))))
    ret
    )
   (while (<= x width)
     (setq y (* -1 depth))
     (while (<= y depth)
       (mapcar
	#'(lambda (data)
	    (push
	     (mapcar
	      #'(lambda (cs ta ra)
		  (instance
		   simple-contact-state
		   :init
		   :name (cdr (assoc :name data))
		   :contact-plane-obj :four-leg-seat
		   :contact-coords cs
		   :contact-n #F(0 0 1)
		   :force0 #F(0 0 0 0 0 0)
		   :ux 0.5 :uy 0.5 :uz 0.5 :lx 0.1 :ly 0.1
		   :gain gain
		   :target-direction
		   '(lambda (tc ntc self &rest args)
		      (* -1 (norm (v- (send (send *goal-contact-state* :target-coords) :worldpos)
				      (send tc :worldpos)))))
		   ;; (let ((val (v. (v- (send tc :worldpos) (send ntc :worldpos))
		   ;; 		     (v- (send (send *goal-contact-state* :target-coords) :worldpos)
		   ;; 			 (send ntc :worldpos)))))
		   ;; 	(* (if (> val 0) 1 -1) (sqrt (abs val)))))
		   :translation-axis ta
		   :rotation-axis ra
		   :target-coords
		   (make-coords :pos (v+ (send self :worldpos) (float-vector y x height)))))
	      (cdr (assoc :cascoords data))
	      (cdr (assoc :translation-axis data))
	      (cdr (assoc :rotation-axis data)))
	     ret))
	cascoords)
       (setq y (+ y dy)))
     (setq x (+ x dx)))
   ret)
  )

(defun setup-hip-end-coords
  nil
  (let* ((plink (car (send *robot* :links)))
	 (hip (instance bodyset-link :init
			(make-cascoords
			 :coords
			 (make-coords
			  :pos (copy-seq (send plink :worldpos))
			  ))
			:name "hip_sit_link"
			:bodies (list (make-cube 10 10 10))))
	 (joint (instance rotational-joint :init :min 0.0 :max 0.0
			  :name "hip_fixed_joint"
			  :child-link hip
			  :parent-link plink))
	 (hip-end-coords
	  (make-cascoords :coords (send hip :copy-worldcoords)
			  :parent hip
			  :name :hip-end-coords))
	 )
    (send *robot* :put :hip-end-coords hip-end-coords)
    (send hip :add-joint joint)
    (send hip :add-parent-link plink)
    (send plink :assoc hip)
    (send joint :set-val 'default-coords
	  (make-coords :pos (float-vector -50 0 -40)))
    (send joint :joint-angle 0)
    ;; (send *robot* :set-val 'joint-list
    ;; (append (send *robot* :get-val 'joint-list) (list joint)))
    (send *robot* :set-val 'links
	  (append (send *robot* :get-val 'links) (list hip)))
    ))

(defun now-hip-contact-state
  (&key (contact-plane-obj nil))
  (instance
   simple-contact-state
   :init
   :name :hip
   :contact-plane-obj contact-plane-obj
   :contact-coords (send *robot* :get :hip-end-coords)
   :contact-n #F(0 0 1)
   :force0 #F(0 0 0 0 0 0)
   :ux 0.05 :uy 0.05 :uz 0.05 :lx 0.05 :ly 0.05
   :gain '(0.7 1.0 100)
   :target-direction
   '(lambda (tc ntc self &rest args)
      (* -1 (norm (v- (send (send *goal-contact-state* :target-coords) :worldpos)
		      (send tc :worldpos)))))
   ;; (let ((val (v. (v- (send tc :worldpos) (send ntc :worldpos))
   ;; 		     (v- (send (send *goal-contact-state* :target-coords) :worldpos)
   ;; 			 (send ntc :worldpos)))))
   ;; 	(* (if (> val 0) 1 -1) (sqrt (abs val)))))
   :target-coords (send (send *robot* :get :hip-end-coords) :copy-worldcoords)))

