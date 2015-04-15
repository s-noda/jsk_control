;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "../my-util.l")
(require "../contact-state.l")

(defvar *robot*) ;; (hrp2jsknts-simple-detail))

(defclass hasegawa-ladder
  :super cascaded-link
  :slots (root-obj
	  step-body
	  footrail-body
	  handrail-body
	  base-body
	  ;;
	  step-count
	  step-vector
	  step-radius
	  step-length
	  step-bottom-length
	  step-uxy
	  step-uz
	  step-color
	  ;;
	  footrail-radius
	  footrail-color
	  footrail-direction
	  ;;
	  handrail-step-vector
	  handrail-radius
	  handrail-color
	  handrail-direction
	  handrail-uxy
	  handrail-uz
	  handrail-height
	  ;;
	  vertical-vector
	  horizon-vector
	  ;;
	  mirror-bodies
	  ))
(defmethod hasegawa-ladder
  (:init
   (&rest
    args
    &key
    ((:name name) "hasegawa-ladder")
    ((:step-vector sv)
     (list (float-vector (/ 0 (tan (deg2rad 75))) 0 0)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   ;;(float-vector (/ 900 (tan (deg2rad 75))) 0 900)
	   ))
    ((:step-radius sr)
     (let (buf)
       (list
	(setq buf (list (float-vector -5 5 0) (float-vector -5 -5 0) (float-vector 80 -5 0) (float-vector 80 5 0)))
	(copy-object buf)
	(copy-object buf)
	(copy-object buf)
	(copy-object buf)
	(copy-object buf)
	(copy-object buf)
	(list (float-vector 1997 5 0) (float-vector 1997 -5 0) (float-vector -5 -5 0) (float-vector -5 5 0))
	;;(setq buf (list (float-vector 1 1 0) (float-vector 1 -1 0) (float-vector -1 -1 0) (float-vector -1 1 0)))
	)))
    ((:step-uxy suxy) 0.3)
    ((:step-uz suz) 0.3)
    ((:step-length sl) 800)
    ((:step-bottom-length sbl) 800)
    ((:step-color sco) (float-vector 0.717647 0.717647 0.72549))
    ((:footrail-radius fr) (list (float-vector -5 5 0) (float-vector -5 -5 0) (float-vector 80 -5 0) (float-vector 80 5 0)))
    ((:footrail-color fco) sco)
    ((:handrail-step-vector hsv)
     (list (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ (- 305 (* 220 (sin (deg2rad 15)))) (tan (deg2rad 75)))
			 0 (- 305 (* 220 (sin (deg2rad 15)))))
	   ;;
	   (float-vector (* -220 (cos (deg2rad 15))) 0 (* 220 (sin (deg2rad 15))))
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (* 220 (cos (deg2rad 15))) 0 (* -220 (sin (deg2rad 15))))
	   ;;
	   (float-vector (* -220 (cos (deg2rad 15))) 0 (* 220 (sin (deg2rad 15))))
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector (/ 305 (tan (deg2rad 75))) 0 305)
	   (float-vector 160 0 0)
	   ;;
	   (float-vector 0 0 (* -2 305))
	   ;;
	   (float-vector 0 0 900)
	   (float-vector 600 0 0)
	   (float-vector 0 0 -900)
	   ;;
	   (float-vector 0 0 900)
	   (float-vector 600 0 0)
	   (float-vector 0 0 -900)
	   ;;
	   (float-vector 0 0 900)
	   (float-vector 600 0 0)
	   (float-vector 0 0 -900)
	   ;;
	   (float-vector 0 0 (* -7 305))
	   (float-vector 0 0 (+ (* 7 305) 900))
	   ))
    ((:handrail-radius hr)
     (list nil nil nil
	   15 15 15 15
	   nil
	   15 15 15 15 15
	   nil
	   15 15 nil
	   15 15 nil
	   15 15 nil
	   nil 15
	   ))
    ((:handrail-uxy huxy) 0.3)
    ((:handrail-uz huz) 0.3)
    ((:handrail-height hh) 0)
    ((:handrail-color hco) sco)
    ((:step-count sc) (or (if (listp sv) (length sv)) 10))
    ((:base-width bw) 1500)
    ((:base-height bh) 258)
    ((:base-depth bd) 2773)
    ((:base-body bb) (make-cube bd bw bh))
    (mirror nil)
    buf
    &allow-other-keys)
   ;; set param
   (send-super* :init :name name args)
   (setq step-count sc)
   (setq step-vector (send self :filter-param :length sc :candidate sv :class vector))
   (setq step-radius (send self :filter-param :length sc :candidate sr :class
			   #'(lambda (val) (or (numberp val)
					       (listp val)))))
   (setq step-uxy (send self :filter-param :length sc :candidate suxy :class 'numberp))
   (setq step-uz (send self :filter-param :length sc :candidate suz :class 'numberp))
   (setq step-length (send self :filter-param :length sc :candidate sl :class 'numberp))
   (setq step-bottom-length sbl)
   (setq step-color (send self :filter-param :length sc :candidate sco :class vector))
   (setq footrail-radius (send self :filter-param :length sc :candidate fr :class
			       #'(lambda (val) (or (numberp val)
						   (listp val)))))
   (setq footrail-color (send self :filter-param :length sc :candidate fco :class vector))
   (setq handrail-step-vector
	 (send self :filter-param
	       :length (if (listp hsv) (length hsv) sc)
	       :candidate hsv
	       :class vector))
   (setq handrail-radius (send self :filter-param :length (length handrail-step-vector)
			       :candidate hr :class
			       #'(lambda (val) (or (numberp val) (listp val)))))
   (setq handrail-color (send self :filter-param :length (length handrail-step-vector)
			      :candidate hco :class vector))
   (setq handrail-uxy (send self :filter-param :length (length handrail-step-vector)
			    :candidate huxy :class 'numberp))
   (setq handrail-uz (send self :filter-param :length (length handrail-step-vector)
			   :candidate huz :class 'numberp))
   (setq handrail-height hh)
   ;; gen obj
   (setq root-obj (make-cube 1 1 1))
   (send root-obj :set-color sco)
   ;; (setf (get root-obj :face-color) sco)
   ;; mirror
   (cond
    (mirror
     (setq buf step-vector)
     (setq step-vector
	   (mapcar #'(lambda (v) (map float-vector #'* '(-1 1 1) v))
		   (if (functionp mirror) (funcall mirror step-vector)
		     step-vector)))
     ;;     (send self :gen-step)
     (send self :gen-footrail)
     (setq mirror-bodies (list step-body handrail-body footrail-body))
     (setq mirror (v. (float-vector 1 0 0) (reduce #'v+ step-vector)))
     (send-all (flatten mirror-bodies) :translate
	       (float-vector (* -2 mirror) 0 0) :world)
     (setq step-vector buf)))
   (setq base-body bb)
   (send base-body :translate
	 (float-vector (/ bd 2.0) 0 (/ bh -2.0))
	 :world)
   (send base-body :set-color sco)
   (send root-obj :assoc base-body)
   (send self :gen-step)
   (send self :gen-footrail)
   (send self :gen-handrail)
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies
		   (cons root-obj
			 (flatten (append step-body handrail-body
					  footrail-body (list base-body)
					  mirror-bodies)))
		   :name name))
   ;;
   (send root-obj :centroid
	 (let ((buf (flatten (send-all (send root-obj :faces) :vertices))))
	   (map float-vector
		#'*
		#f(1 1 0)
		(scale (/ 1.0 (length buf)) (reduce #'v+ buf)))))
   ;;
   (send self :assoc root-obj)
   (setq bodies (list root-obj))
   (setq links (list root-obj))
   (setq joint-list nil)
   (send self :init-ending)
   ;; (send self :translate (float-vector 0 0 bh) :world)
   (send-all
    (cdr (send root-obj :bodies))
    :translate (float-vector 0 0 bh) :world)
   self
   )
  (:filter-param
   (&key length candidate class)
   (let ((ret
	  (if (and (listp candidate) (eq length (length candidate)))
	      candidate
	    (mapcar #'(lambda (cnt) (copy-object candidate))
		    (make-list length)))))
     (cond
      ((or
	(not (eq length (length ret)))
	(not
	 (reduce
	  #'(lambda (a b) (and a b))
	  (mapcar
	   #'(lambda (el)
	       (if (functionp class)
		   (funcall class el)
		 (subclassp (class el) class)))
	   ret))))
       (throw :filter-param candidate))
      (t ret))))
  (:gen-step
   nil
   (setq
    step-body
    (let ((buf (float-vector 0 0 0)) (index -1))
      (mapcar
       #'(lambda (sv sr sl sc)
	   (let ((bod
		  (if (numberp sr)
		      (make-cylinder sr sl)
		    (make-prism sr (float-vector 0 0 sl)))))
	     (setq buf (v+ buf sv))
	     (send bod :rotate (deg2rad 90) :x)
	     (send bod :translate (float-vector 0 (/ sl 2.0) 0) :world)
	     (send bod :translate buf :world)
	     (send bod :name
		   (read-from-string (format nil ":step-link~A" (incf index))))
	     (send root-obj :assoc bod)
	     (send bod :set-color sc)
					;(setf (get bod :face-color) sc)
	     bod))
       step-vector
       step-radius
       step-length
       step-color))))
  (:gen-footrail
   nil
   (setq footrail-direction nil)
   (let ((now-r (float-vector 0 (/ step-bottom-length -2.0) 0))
	 (now-l (float-vector 0 (/ step-bottom-length +2.0) 0))
	 (ue-v (float-vector 0 0 0))
	 (ho-v (float-vector 0 0 0))
	 (buf (float-vector 0 0 0)) (index -1))
     (setq
      footrail-body
      (mapcar
       #'(lambda (sv sl fc fr)
	   (let* ((next-r (v+ (v+ buf sv) (float-vector 0 (/ sl -2.0) 0)))
		  (next-l (v+ (v+ buf sv) (float-vector 0 (/ sl 2.0) 0)))
		  (bod-r (if (listp fr)
			     (make-prism fr (norm (v- next-r now-r)))
			   (make-cylinder fr (norm (v- next-r now-r)))))
		  (bod-l (copy-object bod-r))
		  (dir-r (normalize-vector (v- next-r now-r)))
		  (dir-l (normalize-vector (v- next-l now-l))))
	     (setq footrail-direction
		   (append footrail-direction (list (list dir-r dir-l))))
	     (send bod-r :newcoords
		   (make-coords
		    :pos (copy-seq now-r)
		    :rot (matrix-exponent
			  (normalize-vector (v* (float-vector 0 0 1) dir-r))
			  (acos (v. (float-vector 0 0 1) dir-r)))))
	     (send bod-l :newcoords
		   (make-coords
		    :pos (copy-seq now-l)
		    :rot (matrix-exponent
			  (normalize-vector (v* (float-vector 0 0 1) dir-l))
			  (acos (v. (float-vector 0 0 1) dir-l)))))
	     (send bod-r :name
		   (read-from-string (format nil ":rfoot-link~A" (incf index))))
	     (send bod-l :name
		   (read-from-string (format nil ":lfoot-link~A" index)))
	     (send bod-r :set-color fc)
					;(setf (get bod-r :face-color) fc)
	     (send bod-l :set-color fc)
					;(setf (get bod-l :face-color) fc)
	     (send root-obj :assoc bod-r)
	     (send root-obj :assoc bod-l)
	     (setq ho-v (v+ ho-v sv))
	     (setq ue-v
		   (v+ ue-v
		       (normalize-vector
			(v* (v- next-r buf)
			    (v- next-l buf)))))
	     (setq buf (v+ buf sv))
	     (setq now-r next-r)
	     (setq now-l next-l)
	     (list bod-r bod-l)))
       step-vector
       (append step-length step-length step-length)
       footrail-color
       footrail-radius))
     (setq vertical-vector (scale (/ 1.0 step-count) ue-v))
     (setq horizon-vector (normalize-vector ho-v))
     footrail-body)
   )
  (:gen-handrail
   nil
   (setq handrail-direction nil)
   (let ((now-r (float-vector 0 (/ step-bottom-length -2.0) 0))
	 (now-l (float-vector 0 (/ step-bottom-length +2.0) 0))
	 (buf (float-vector 0 0 0)) (index -1)
	 next-r next-l)
     (setq
      handrail-body
      (apply
       #'append
       (mapcar
	#'(lambda (sv sl hc fr)
	    (setq next-r (v+ (v+ buf sv) (float-vector 0 (/ sl -2.0) 0)))
	    (setq next-l (v+ (v+ buf sv) (float-vector 0 (/ sl 2.0) 0)))
	    (if (null fr)
		(progn (setq now-r next-r)
		       (setq now-l next-l)
		       (setq buf (v+ buf sv))
		       nil)
	      (let* ((bod-r (if (listp fr)
				(make-prism fr (norm (v- next-r now-r)))
			      (make-cylinder fr (norm (v- next-r now-r)))))
		     (bod-l (copy-object bod-r))
		     (dir-r (normalize-vector (v- next-r now-r)))
		     (dir-l (normalize-vector (v- next-l now-l))))
		(setq handrail-direction
		      (append handrail-direction (list (list dir-r dir-l))))
		(send bod-r :newcoords
		      (make-coords
		       :pos (copy-seq now-r)
		       :rot (matrix-exponent
			     (normalize-vector (v* (float-vector 0 0 1) dir-r))
			     (acos (v. (float-vector 0 0 1) dir-r)))))
		(send bod-l :newcoords
		      (make-coords
		       :pos (copy-seq now-l)
		       :rot (matrix-exponent
			     (normalize-vector (v* (float-vector 0 0 1) dir-l))
			     (acos (v. (float-vector 0 0 1) dir-l)))))
		(send bod-r :name
		      (read-from-string (format nil ":rhand-link~A" (incf index))))
		(send bod-l :name
		      (read-from-string (format nil ":lhand-link~A" index)))
		(send bod-r :set-color hc) ;(setf (get bod-r :face-color) hc)
		(send bod-l :set-color hc)	;(setf (get bod-l :face-color) hc)
		(send root-obj :assoc bod-r)
		(send root-obj :assoc bod-l)
		(setq buf (v+ buf sv))
		(setq now-r next-r)
		(setq now-l next-l)
		(list (list bod-r bod-l)))))
	handrail-step-vector
	(apply #'append (make-list (length handrail-step-vector)
				   :initial-element step-length))
	handrail-color
	handrail-radius)))
     ;; (setq ue-v (scale handrail-height (float-vector -1 0 0)))
     (mapcar
      #'(lambda (b)
	  (send b :translate
		(scale handrail-height (float-vector -1 0 0))
		:world))
      (flatten handrail-body))
     handrail-body
     ))
  (:step-body nil step-body)
  (:step-vector nil step-vector)
  (:step-length nil step-length)
  (:handrail-body nil handrail-body)
  (:footrail-body nil footrail-body)
  (:vertical-vector nil vertical-vector)
  (:horizon-vector nil horizon-vector)
  ;;
  (:gen-contact-states
   nil
   (let (leg-can hand-can buf hand-offset)
     (setq leg-can
	   (mapcar
	    #'(lambda (sl bd)
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
				(v+ #F(70 0 0)
				    (send *robot* k :end-coords :worldpos))
				:rot
				(copy-object (send *robot* k :end-coords :worldrot))))
		      :contact-n #F(0 0 1) :force0 #F(0 0 0 0 0 0)
		      :ux 0.3 :uy 0.3 :uz 0.1 :lx 0.05 :ly 0.1
		      :target-coords
		      (make-coords
		       :pos (v+ (send bd :worldpos)
				(float-vector
				 0
				 (* sl (if (eq k :rleg) -0.7 -0.3))
				 0))
		       :rot
		       (m*
			(matrix-exponent
			 (float-vector 0 0
				       (if (eq k :rleg) (deg2rad -15) (deg2rad 15))))
			(matrix-exponent
			 (normalize-vector (v* #F(0 0 1) buf))
			 (acos (v. buf #F(0 0 1))))))))
		 '(:rleg :lleg)))
	    (subseq (send self :step-length) 0)
	    (subseq (send self :step-body) 0)
	    ))
     (setq hand-can
	   (mapcar
	    #'(lambda (rdlist bdlist)
		(mapcar
		 #'(lambda (rd bd k n offset)
		     (setq buf
			   (list
			    (transform (send *robot* k :end-coords :worldrot) n)
			    (transform
			     (matrix-exponent (float-vector 0 0 (deg2rad (if (eq k :rarm) 60 -60))))
			     (transform (send (send bd :worldcoords) :worldrot) #F(-1 0 0)))
			    ;;(float-vector (* -1 (cos (deg2rad 20)))
			    ;;0
			    ;;(sin (deg2rad 20)))
			    ))
		     (instance
		      simple-contact-state :init :name k
		      :contact-coords (send *robot* k :end-coords)
		      :contact-n n
		      :force0 #F(50 50 -10 10 10 10)
		      :ux 0.3 :uy 0.3 :uz 0.1 :lx 0.1 :ly 0.1
		      ;; :rotation-axis :z
		      :target-coords
		      (make-coords
		       :pos (v+ (v+ offset (send bd :worldpos))
				(scale (if (null hand-offset)
					   0
					 (setq hand-offset -100))
				       (normalize-vector rd)))
		       :rot
		       (matrix-exponent
			(normalize-vector (apply #'v* (reverse buf)))
			(acos (apply #'v. buf))))
		      ))
		 rdlist bdlist '(:rarm :larm)
		 (list (normalize-vector #F(0 -1 0))
		       (normalize-vector #F(0 +1 0)))
		 (list #F(-0 -0 60) #F(-0 0 60))
		 ))
	    (subseq handrail-direction 1 10)
	    (subseq handrail-body 1 10)))
     (mapcar
      #'(lambda (k)
	  (setq buf (remove-if #'(lambda (cs) (not (eq k (send cs :name))))
			       (flatten (append leg-can hand-can))))
	  (send-all buf :sequence-select buf))
      '(:rarm :larm :rleg :lleg))
     (append leg-can hand-can)))
  )
