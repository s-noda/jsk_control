;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "../my-util.l")
(require "../contact-state.l")

(defvar *robot*) ;; (hrp2jsknts-simple-detail))

(defclass random-rock-wall
  :super cascaded-link
  :slots (root-obj
	  wall-obj
	  rock-obj-list
	  ))

(defmethod random-rock-wall
  (:init
   (&rest
    args
    &key
    ((:name name) :random-rock-wall)
    ((:wall-width ww) 1500)
    ((:wall-height wh) 3000)
    ((:wall-obj wo)
     (let ((buf (make-cube 10 ww wh)))
       (send buf :set-color #F(0.5 0.1 0.1))
       buf))
    ((:rock-obj-cnt roc) 150)
    ((:rock-obj-list rol)
     (mapcar #'(lambda (hoge)
		 (let* ((size (* 100 (+ 1.0 (random 1.0))))
			(c (make-cube size size size)))
		   (send c :locate
			 (float-vector (+ 0 (random 0))
				       (* (/ ww 2.0) (- (random 2.0) 1.0))
				       (* (/ wh 2.0) (- (random 2.0) 1.0)))
			 :world)
		   (send c :set-color #F(0.5 0.2 0.2))
		   (send c :rotate (deg2rad (+ 80 (* 40 (random 1.0)))) :y)
		   (send c :rotate (deg2rad (* 30 (- (random 2.0) 1.0))) :x)))
	     (make-list roc)))
    &allow-other-keys)
   ;; set param
   (send-super* :init :name name args)
   (setq wall-obj wo)
   (setq rock-obj-list rol)
   (setq root-obj (make-cube 1 1 1))
   (send root-obj :set-color #F(0 0 0))
   (mapcar #'(lambda (obj) (send root-obj :assoc obj))
	   (cons wall-obj rock-obj-list))
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies (append (list root-obj wall-obj) rock-obj-list)
		   :name name))
   (send root-obj :centroid (float-vector 0 0 (/ wh -2.0)))
   (send self :assoc root-obj)
   (setq bodies (list root-obj))
   (setq links (list root-obj))
   (send self :init-ending)
   self
   )
  (:vertical-vector nil #F(-1 0 0))
  (:gen-contact-states
   nil
   (let ((c) (n) (ic) (gn))
     (mapcar
      #'(lambda (ro)
	  (setq c (send (car (nthcdr 5 (send ro :faces))) :center-coordinates))
	  (setq n (transform (send c :worldrot) #F(0 0 1)))
	  (mapcar
	   #'(lambda (k dir gain)
	       (setq ic (copy-object c))
	       (case k
		     (:rarm
		      (send ic
			    :rotate (deg2rad -90) :x))
		     (:larm
		      (send ic :rotate (deg2rad 90) :x)))
	       (setq gn
		     (transform
		      (send *robot* k :end-coords :worldrot)
		      (if (find k '(:rarm :larm)) (scale -1 dir) dir)))
	       (instance
		simple-contact-state
		:init
		:name k
		:contact-coords; (cdr (assoc k *sample-contact-coords*))
		(if (find k '(:rarm :larm)) (send *robot* k :end-coords)
		  (cascoords-collection
		   :limb-key k
		   :coords (make-coords
			    :pos
			    (v+ #F(100 0 0)
				(send *robot* k :end-coords :worldpos))
			    :rot
			    (copy-object (send *robot* k :end-coords :worldrot)))))
		:contact-n dir
		:force0 #F(0 0 0 0 0 0)
		:ux 1.0 :uy 1.0 :uz 1.0 :lx 0.01 :ly 0.01
		:gain gain
		:target-coords
		(make-coords :pos (v- (send ic :worldpos)
				      (transform (send ic :worldrot) #F(50 0 0)))
			     :rot (send ic :worldrot))
		))
	   '(:rarm :larm :rleg :lleg)
	   (list #F(0 -1 0) #F(0 1 0) #F(0 0 1) #F(0 0 1))
	   (list '(1 1.1 30) '(1 1.1 30)
		 '(1 1.5 10) '(1 1.5 10))))
      rock-obj-list
      )))
  )
