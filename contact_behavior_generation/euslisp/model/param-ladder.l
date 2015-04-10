;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "../my-util.l")
(require "../contact-state.l")

(defvar *robot* (hrp2jsknts-simple-detail))
;; (defvar *sample-contact-coords*
;;   (append
;;    (mapcar
;;     #'(lambda (k)
;; 	(let ((buf (send *robot* k :end-coords)))
;; 	  (cons k (make-cascoords
;; 		   :init :link-list
;; 		   :name (read-from-string
;; 			  (format nil "~A-sample-coords" k))
;; 		   :pos (v+ #F(80 0 0) (send buf :worldpos))
;; 		   :rot (copy-object (send buf :worldrot))
;; 		   :parent (send buf :parent)))))
;;     '(:rleg :lleg))
;;    (mapcar #'(lambda (k) (cons k (send *robot* k :end-coords)))
;; 	   '(:rarm :larm))))

;(if (not (boundp '*pickview*)) (pickview :no-menu t))
(defun obj (l) (objects l) (send *pickview* :draw-objects))

(defun filter-param
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

(defclass param-ladder
  :super cascaded-link
  :slots (root-obj
	  step-body
	  footrail-body
	  handrail-body
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

(defmethod param-ladder
  (:init
   (&rest
    args
    &key
    ((:name name) :param-ladder)
    ((:step-vector sv) #F(0 0 200))
    ((:step-radius sr) 20)
    ((:step-uxy suxy) 0.3)
    ((:step-uz suz) 0.3)
    ((:step-length sl) 1000)
    ((:step-bottom-length sbl) (if (listp sl) (find-if #'numberp sl) sl))
    ((:step-color sco) #F(0.8 1 1))
    ((:footrail-radius fr) 30)
    ((:footrail-color fco) sco)
    ((:handrail-radius hr) 30)
    ((:handrail-uxy huxy) 0.3)
    ((:handrail-uz huz) 0.3)
    ((:handrail-height hh) 50)
    ((:handrail-color hco) sco)
    ((:step-count sc) (or (and (listp sv) (length sv))
			  10))
    (mirror nil)
    buf
    &allow-other-keys)
   ;; set param
   (send-super* :init :name name args)
   (setq step-count sc)
   (setq step-vector (filter-param :length sc :candidate sv :class vector))
   (setq step-radius (filter-param :length sc :candidate sr :class
				   #'(lambda (val) (or (numberp val)
						       (listp val)))))
   (setq step-uxy (filter-param :length sc :candidate suxy :class 'numberp))
   (setq step-uz (filter-param :length sc :candidate suz :class 'numberp))
   (setq step-length (filter-param :length sc :candidate sl :class 'numberp))
   (setq step-bottom-length sbl)
   (setq step-color (filter-param :length sc :candidate sco :class vector))
   (setq footrail-radius (filter-param :length sc :candidate fr :class
				       #'(lambda (val) (or (numberp val)
							   (listp val)))))
   (setq footrail-color (filter-param :length sc :candidate fco :class vector))
   (setq handrail-radius (filter-param :length sc :candidate hr :class
				       #'(lambda (val) (or (numberp val)
							   (listp val)))))
   (setq handrail-color (filter-param :length sc :candidate hco :class vector))
   (setq handrail-uxy (filter-param :length sc :candidate huxy :class 'numberp))
   (setq handrail-uz (filter-param :length sc :candidate huz :class 'numberp))
   (setq handrail-height hh)
   ;; gen obj
   (setq root-obj (make-cube 1 1 1))
   (send root-obj :set-color sco)
   ;(setf (get root-obj :face-color) sco)
   ;; mirror
   (cond
    (mirror
     (setq buf step-vector)
     (setq step-vector
	   (mapcar #'(lambda (v) (map float-vector #'* '(-1 1 1) v))
		   (if (functionp mirror) (funcall mirror step-vector)
		     step-vector)))
;     (send self :gen-step)
     (send self :gen-footrail)
     (setq mirror-bodies (list step-body handrail-body footrail-body))
     (setq mirror (v. #F(1 0 0) (reduce #'v+ step-vector)))
     (send-all (flatten mirror-bodies) :translate
	       (float-vector (* -2 mirror) 0 0) :world)
     (setq step-vector buf)))
   (send self :gen-step)
   (send self :gen-footrail)
   (send self :gen-handrail)
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies
		   (cons root-obj
			 (flatten (append step-body handrail-body footrail-body
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
   self
   )
  (:gen-step
   nil
   (setq
    step-body
    (let ((buf #F(0 0 0)) (index -1))
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
   (setq
    footrail-body
    (let ((now-r (float-vector 0 (/ step-bottom-length -2.0) 0))
	  (now-l (float-vector 0 (/ step-bottom-length +2.0) 0))
	  (buf #F(0 0 0)) (index -1))
      (mapcar
       #'(lambda (sv sl fc fr)
	   (let* ((next-r (v+ (v+ buf sv) (float-vector 0 (/ sl -2.0) 0)))
		  (next-l (v+ (v+ buf sv) (float-vector 0 (/ sl 2.0) 0)))
		  (bod-r
		   (if (listp fr)
		       (make-prism (copy-object fr) (norm (v- next-r now-r)))
		     (make-cylinder (copy-object fr) (norm (v- next-r now-r)))))
		  (bod-l ;;(copy-object bod-r))
		   (if (listp fr)
		       (make-prism (copy-object fr) (norm (v- next-r now-r)))
		     (make-cylinder (copy-object fr) (norm (v- next-r now-r)))))
		  (dir-r (normalize-vector (v- next-r now-r)))
		  (dir-l (normalize-vector (v- next-l now-l))))
	     (setq footrail-direction
		   (append footrail-direction (list (list dir-r dir-l))))
	     (send bod-r :newcoords
		   (make-coords
		    :pos (copy-seq now-r)
		    :rot (matrix-exponent
			  (normalize-vector (v* #F(0 0 1) dir-r))
			  (acos (v. #F(0 0 1) dir-r)))))
	     (send bod-l :newcoords
		   (make-coords
		    :pos (copy-seq now-l)
		    :rot (matrix-exponent
			  (normalize-vector (v* #F(0 0 1) dir-l))
			  (acos (v. #F(0 0 1) dir-l)))))
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
	     (setq buf (v+ buf sv))
	     (setq now-r next-r)
	     (setq now-l next-l)
	     (list bod-r bod-l)))
       step-vector
       step-length
       footrail-color
       footrail-radius))))
  (:gen-handrail
   nil
   (setq handrail-direction nil)
   (let ((now-r (float-vector 0 (/ step-bottom-length -2.0) 0))
	 (now-l (float-vector 0 (/ step-bottom-length +2.0) 0))
	 (ue-v #F(0 0 0))
	 (ho-v #F(0 0 0))
	 (buf #F(0 0 0)) (index -1))
     (setq
      handrail-body
      (mapcar
       #'(lambda (sv sl hc fr)
	   (let* ((next-r (v+ (v+ buf sv) (float-vector 0 (/ sl -2.0) 0)))
		  (next-l (v+ (v+ buf sv) (float-vector 0 (/ sl 2.0) 0)))
		  (bod-r
		   (if (listp fr)
		       (make-prism (copy-object fr) (norm (v- next-r now-r)))
		     (make-cylinder (copy-object fr) (norm (v- next-r now-r)))))
		  (bod-l ;;(copy-object bod-r))
		   (if (listp fr)
		       (make-prism (copy-object fr) (norm (v- next-r now-r)))
		     (make-cylinder (copy-object fr) (norm (v- next-r now-r)))))
		  (dir-r (normalize-vector (v- next-r now-r)))
		  (dir-l (normalize-vector (v- next-l now-l))))
	     (setq handrail-direction
		   (append handrail-direction (list (list dir-r dir-l))))
	     (send bod-r :newcoords
		   (make-coords
		    :pos (copy-seq now-r)
		    :rot (matrix-exponent
			  (normalize-vector (v* #F(0 0 1) dir-r))
			  (acos (v. #F(0 0 1) dir-r)))))
	     (send bod-l :newcoords
		   (make-coords
		    :pos (copy-seq now-l)
		    :rot (matrix-exponent
			  (normalize-vector (v* #F(0 0 1) dir-l))
			  (acos (v. #F(0 0 1) dir-l)))))
	     (send bod-r :name
		   (read-from-string (format nil ":rhand-link~A" (incf index))))
	     (send bod-l :name
		   (read-from-string (format nil ":lhand-link~A" index)))
	     (send bod-r :set-color hc)
	     ;(setf (get bod-r :face-color) hc)
	     (send bod-l :set-color hc)
	     ;(setf (get bod-l :face-color) hc)
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
       step-length
       handrail-color
       handrail-radius))
     (setq vertical-vector (scale (/ 1.0 step-count) ue-v))
     (setq horizon-vector (normalize-vector ho-v))
     (setq ue-v (scale handrail-height vertical-vector))
     (mapcar
      #'(lambda (b) (send b :translate ue-v :world))
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
  (:gen-contact-states
   nil
   (mapcar
    #'(lambda (sv sl bd)
	(mapcar
	 #'(lambda (k dir z gain)
	     (instance
	      simple-contact-state
	      :init
	      :name k
	      :contact-coords; (cdr (assoc k *sample-contact-coords*))
	      (if (find k '(:rarm :larm))
                  (send *robot* k :end-coords)
                (send *robot* :get (read-from-string (format nil "~A-end-coords2" k))))
					;:rotation-axis :y
	      :contact-n dir
	      :force0
	      (if (find k '(:rarm :larm))
		  #F(80 80 80 30 30 30)
		  #F(0 0 0 0 0 0))
	      :ux 0.3 :uy 0.3 :uz 0.1 :lx 0.05 :ly 0.05
	      :gain gain
	      :target-coords
              (let* ((zz
                      (case k
                        (:rarm (send bd :rotate-vector (float-vector 0 0 -1)))
                        (:larm (send bd :rotate-vector (float-vector 0 0 1)))
                        (t (float-vector 0 0 1)))))
                (make-coords
                 :pos (v+ (send bd :worldpos)
                          (float-vector
                           (if (find k '(:rarm :larm)) 0 0)
                           (* sl
			      (case k
				    (:rarm -0.75)
				    (:rleg -0.7)
				    (:larm -0.25)
				    (:lleg -0.3)))
                           0))
                 :rot
                 (if (memq k '(:rarm :larm))
                     (transpose
                      (make-matrix
                       3 3
                       (list z (v* zz z) zz)))
                   (transpose
                    (make-matrix
                     3 3
                     (list (v* (float-vector 0 1 0) z) (float-vector 0 1 0) z)))
                   )
                ))))
	 '(:rarm :rarm :rarm
	   :larm :larm :larm
	   :rleg :rleg :rleg
	   :lleg :lleg :lleg
	   )
	 (list #F(0 -1 0) #F(0 -1 0) #F(0 -1 0)
	       #F(0 1 0) #F(0 1 0) #F(0 1 0)
	       #F(0 0 1) #F(0 0 1) #F(0 0 1)
	       #F(0 0 1) #F(0 0 1) #F(0 0 1)
	       )
	 (list (rotate-vector #f(1 0 0) (deg2rad -45) :y)
               (rotate-vector #f(1 0 0) (deg2rad 0) :y)
               (rotate-vector #f(1 0 0) (deg2rad 45) :y)
	       (rotate-vector #f(1 0 0) (deg2rad -45) :y)
               (rotate-vector #f(1 0 0) (deg2rad 0) :y)
               (rotate-vector #f(1 0 0) (deg2rad 45) :y)
               (rotate-vector #f(0 0 1) (deg2rad 15) :y)
               (rotate-vector #f(0 0 1) (deg2rad 0) :y)
               (rotate-vector #f(0 0 1) (deg2rad -15) :y)
               (rotate-vector #f(0 0 1) (deg2rad 15) :y)
               (rotate-vector #f(0 0 1) (deg2rad 0) :y)
               (rotate-vector #f(0 0 1) (deg2rad -15) :y)
               )
	 (list '(1 1.1 1) '(1 1.1 1) '(1 1.1 1)
	       '(1 1.1 1) '(1 1.1 1) '(1 1.1 1)
	       '(1 1.4 1) '(1 1.4 1) '(1 1.4 1)
	       '(1 1.4 1) '(1 1.4 1) '(1 1.4 1)
	       )))
    (send self :step-vector)
    (send self :step-length)
    (send self :step-body)
    ))
;  (:centroid
;   nil
  )

#|

roseus motion-sequencer.l 
(load "/home/noda/ros/fuerte/jsk-ros-pkg/euslisp/jskeus/irteus/irtcollada.l")

(demo-climb-setup :kirin-ladder)
(collada::eus2collada *climb-obj* "/tmp")
(unix:system
 (format nil
	 "rosrun hrpsys_gazebo_tutorials eus2urdf_for_gazebo_pyscript.py ~a ~a/~a.dae"
	 "kirin-ladder" "/tmp" "kirin-ladder"))


(collada::eusmodel-description *climb-obj*)
(mapcar #'(lambda (a) (get a :face-color)) (send (car (send *climb-obj* :links)) :bodies))
(let* ((collada::ac (collada::link . acentroid))
       (collada::it (send collada::link :inertia-tensor))
       (collada::rt-cds (let ((collada::lnk collada::link))
			  (while (send collada::lnk :parent)
			    (setq collada::lnk (send collada::lnk :parent)))
			  (send (car (send collada::lnk :links)) :worldcoords)))
       (collada::mframe (send collada::link :copy-worldcoords))
       collada::rot collada::inertia))

