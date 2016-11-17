;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "euslib/demo/iori/hrp4r/hrp4r_tests/hrp4r-drive-car/drive-simulator-environment.lisp")
(require "../my-util.lisp")
(require "../contact-state.lisp")

(defclass garakuta-car
  :super cascaded-link
  :slots (root-obj
	  name
	  base-width
	  base-height
	  base-depth
	  front-width
	  front-height
	  front-depth
	  pole-radius
	  pole-height
	  ;;
	  base-cube
	  front-cube
	  front-pole
	  back-pole
	  ;;
	  corner-coords
	  back-coords
	  top-coords
	  ;;
	  detailed-model
	  ))
(defmethod garakuta-car
  (:init
   (&rest
    args
    &key
    ((:name name) :garakuta-car)
    ((:base-width bw) 1600)
    ((:base-height bh) 300)
    ((:base-depth bd) 800)
    ((:front-width fw) 700)
    ((:front-height fh) 1000);;1900)
    ((:front-depth fd) 800)
    ((:pole-radius pr) 50)
    ((:pole-height ph) 1900)
    &allow-other-keys)
   ;; set param
   (send-super* :init :name name args)
   (setq base-width bw
	 base-height bh
	 base-depth bd
	 front-width fw
	 front-height fh
	 front-depth fd
	 pole-radius pr
	 pole-height ph)
   (setq detailed-model (instance drive-simulator :init))
   ;; gen obj
   (setq root-obj (make-cube 1 1 1))
   (send root-obj :set-color #F(0 1 0))
   ;;
   (setq base-cube (make-cube base-width base-depth base-height))
   (send base-cube :translate
	 (float-vector 0 0 (/ base-height 2.0))
	 :world)
   (setq front-cube (make-cube front-width front-depth front-height))
   (send front-cube :translate
	 (float-vector (/ base-width 2.0)
		       0
		       (/ front-height 2.0))
	 :world)
   (setq front-pole (make-cylinder pole-radius pole-height))
   (send front-pole :translate
	 (float-vector (/ base-width 2.0) (/ base-depth 2.0) 0)
	 :world)
   (setq back-pole (make-cylinder pole-radius pole-height))
   (send back-pole :translate
	 (float-vector (/ base-width -2.0) (/ base-depth 2.0) 0)
	 :world)
   (mapcar
    #'(lambda (bd)
	(send bd :set-color #F(0 1 0))
	(gl::transparent bd 0.7))
    (list base-cube front-cube front-pole back-pole))
   (gl::transparent detailed-model 0.2)
   ;;
   (setq corner-coords
   	 (make-cascoords
   	  :coords
   	  (make-coords
   	   :pos
   	   (float-vector (/ (- base-width front-width) 2.0)
   			 (/ base-depth 2.0)
   			 base-height))))
   (setq top-coords
   	 (make-cascoords
   	  :coords
   	  (make-coords
   	   :pos
   	   (float-vector (/ (- base-width front-width) 2.0)
   			 (/ base-depth 2.0)
   			 front-height))))
   (setq back-coords
   	 (make-cascoords
   	  :coords
   	  (make-coords
   	   :pos
   	   (float-vector (/ base-width -2.0)
   			 (/ base-depth 2.0)
   			 base-height))))
   ;;
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies
		   (list root-obj base-cube front-cube front-pole back-pole)
		   :name name))
   ;;
   (send root-obj :centroid
	 (let ((buf (flatten (send-all (send root-obj :faces) :vertices))))
	   (map float-vector
		#'*
		#f(1 1 0)
		(scale (/ 1.0 (length buf)) (reduce #'v+ buf)))))
   ;;
   (mapcar
    #'(lambda (bd)
	(send root-obj :assoc bd))
    (list base-cube front-cube front-pole back-pole detailed-model
	  corner-coords top-coords back-coords
	  ))
   (send self :assoc root-obj)
   (setq bodies (list root-obj))
   (setq links (list root-obj))
   (setq joint-list nil)
   (send self :init-ending)
   self
   )
  (:dr
   nil
   (send *pickview* :objects (list self detailed-model))
   (send *viewer* :draw-objects :flush nil)
   (send-all (list corner-coords top-coords back-coords)
	     :draw-on :flush nil :color #f(1 0 0) :width 100)
   (send *viewer* :viewsurface :flush)
   )
  (:inner-vector
   nil (send (send corner-coords :copy-worldcoords) :worldpos))
  (:vertical-vector
   nil (normalize-vector
	(v* (v- (send back-coords :worldpos) (send corner-coords :worldpos))
	    (v- (send top-coords :worldpos) (send corner-coords :worldpos)))))
  (:horizon-vector
   nil (normalize-vector
	(v- (send back-coords :worldpos) (send corner-coords :worldpos))))
  (:gen-collision-check-list
   nil
   (list
    (list (cons :name name)
	  (cons :n (scale -1 (send self :vertical-vector)))
	  (cons :a0 (send self :inner-vector))
	  (cons :check-func
		(list 'lambda '(env ccl)
		      (list 'let
			    (list (list 'h (v- (send back-coords :worldpos)
					       (send corner-coords :worldpos)))
				  (list 'v (v- (send top-coords :worldpos)
					       (send corner-coords :worldpos)))
				  (list 'n (send self :vertical-vector))
				  (list 'a (send corner-coords :worldpos))
				  (list 'x '(funcall (cdr (assoc :dist ccl))))
				  )
			    '(setq x (v- x a))
			    '(setq x (v+ a (v- x (scale (v. x n) n))))
			    '(if (or
				  (> (aref (v- (v- x a) v) 2) 0)
				  (and (> (v. n (v* h x)) 0)
				       (> (v. n (v* x v)) 0)))
				 1e+6)))))))
  (:gen-contact-states
   nil
   (let* ((av (copy-object (send *robot* :angle-vector)))
	  (bc (send (car (send *robot* :links)) :copy-worldcoords))
	  ret)
     (send *robot* :reset-manip-pose)
     (send *robot* :fix-leg-to-coords
	   (let ((c (send *robot* :rleg :end-coords :copy-worldcoords)))
	     (make-coords :pos (v+ #F(-300 0 300)
				   (map float-vector #'* #F(1 1 0)
					(send c :worldpos)))
			  :rot (copy-object (send c :worldrot)))) :rleg)
     (send *robot* :inverse-kinematics
	   (make-coords :pos (v+ (send front-pole :worldpos)
				 #F(0 0 1550)))
	   :move-target (send *robot* :rarm :end-coords)
	   :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
	   :rotation-axis :z
	   ;; :translation-axis :z
	   :debug-view :no-message
	   )
     (send *robot* :inverse-kinematics
	   (make-coords :pos (v+ (send back-pole :worldpos)
				 #F(0 0 1550)))
	   :move-target (send *robot* :larm :end-coords)
	   :link-list (cddr (send *robot* :link-list (send *robot* :larm :end-coords :parent)))
	   :rotation-axis :z
	   ;; :translation-axis :z
	   :debug-view :no-message
	   )
     ;; (send *robot* :inverse-kinematics
     ;;       (let* ((c
     ;;               (make-coords :pos (v+ (send front-pole :worldpos)
     ;;                                     #F(-150 0 1850)))))
     ;;         (send c :rotate (deg2rad 90) :z)
     ;;         ;;(send c :rotate (deg2rad -90) :x)
     ;;         (send c :rotate (deg2rad 90) :x)
     ;;         (send c :rotate (deg2rad -160) :z)
     ;;         c)
     ;;       :move-target (send *robot* :rarm :end-coords)
     ;;       :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
     ;;       ;; :rotation-axis :z
     ;;       ;; :translation-axis :z
     ;;       :debug-view :no-message
     ;;       )
     ;; (read-line)
     (send-all (send *robot* :links) :worldcoords)
     (setq
      ret
      (mapcar
       #'(lambda (k dir tc gain)
	   (instance
	    simple-contact-state
	    :init
	    :name k
	    :contact-coords (send *robot* k :end-coords)
	    :contact-n dir
	    :force0
	    (if (find k '(:rarm :larm))
		#F(100 100 100 50 50 50)
		#F(0 0 0 0 0 0))
	    :ux 0.3 :uy 0.3 :uz 0.1 :lx 0.08 :ly 0.08
	    :gain gain
	    :target-coords tc))
       '(:rarm :larm :rleg :lleg)
       (list #F(-1 0 0) #F(-1 0 0) #F(0 0 1) #F(0 0 1))
       (mapcar #'(lambda (k c)
		   (send (send *robot* k :end-coords :copy-worldcoords) :translate (send c :worldpos) :local)
                   )
	       '(:rarm :larm :rleg :lleg)
               (list (make-coords :pos #F(-50 0 0)) (make-coords :pos #F(-50 0 0)) (make-coords) (make-coords)))
       (list '(1 1 1) '(1 1 1) '(1 1 1) '(1 1 1))
       ))
     (send (car ret) :rotation-axis :z)
     (send *robot* :angle-vector av)
     (send (car (send *robot* :links)) :move-to bc)
     ;;(send-all (send *robot* :links) :worldcoords)
     (send *robot* :fix-leg-to-coords (make-coords))
     (list ret)))
  )

#|

(defun ud nil (setq a (instance garakuta-car :init)) (send a :rotate (deg2rad 0) :z) (send a :dr)) (defun dc (c) (send-all (if (listp c) c (list c)) :draw-on :flush t :color #F(1 0 0) :size 1000)) (defun it nil (progn (defvar *car* (instance drive-simulator :init)) (pickview :no-menu t) (objects (list *car*))))

(it)
(setq *robot-type* :staro)
(load "../robot-param.lisp")

(ud)
(objects *staro*)
(send a :rotate (deg2rad -90) :z)
(send a :translate #F(-600 0 0) :world)
(send *viewer* :draw-objects)


(send *robot* :reset-manip-pose)
(send *robot* :fix-leg-to-coords
      (let ((c (send *robot* :rleg :end-coords :copy-worldcoords)))
	(make-coords :pos (v+ #F(0 0 300)
			      (map float-vector #'* #F(1 1 0)
				   (send c :worldpos)))
		     :rot (copy-object (send c :worldrot)))) :rleg)
(send *robot* :inverse-kinematics
      (make-coords :pos (v+ (send (send a :get-val 'front-pole) :worldpos) #F(0 0 1000)))
      :move-target (send *robot* :rarm :end-coords)
      :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
      :rotation-axis :z
      :translation-axis :z
      :debug-view :no-message
      )

(mapcar
 #'(lambda (k dir tc gain)
     (instance
      simple-contact-state
      :init
      :name k
      :contact-coords (send *robot* k :end-coords)
      :contact-n dir
      :force0
      (if (find k '(:rarm :larm))
	  #F(200 200 200 30 30 30)
	  #F(0 0 0 0 0 0))
      :ux 0.3 :uy 0.3 :uz 0.1 :lx 0.1 :ly 0.1
      :gain gain
      :target-coords tc))
 '(:rarm :rleg :lleg)
 (list #F(-1 0 0) #F(0 0 1) #F(0 0 1))
 (mapcar #'(lambda (k)
	     (send *robot* k :end-coords :copy-worldcoords))
	 '(:rarm :rleg :lleg))
 (list '(1 1 1) '(1 1 1) '(1 1 1))
 )


rosd climb/work && roseus "(progn (load \"motion-sequencer.l\") (demo-climb-setup :garakuta-car) (setq *ret* (demo-motion-sequence :all-limbs '(:rarm :rleg :lleg) :now-rsd (instance robot-state-data :init :contact-states (now-contact-state :limb-keys '(:rarm :larm :rleg :lleg))) :tmax-leg-rate 0.7)))"
