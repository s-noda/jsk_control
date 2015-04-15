;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "../my-util.l")
(require "../contact-state.l")

(defvar *robot*) ;; (hrp2jsk-simple-detail))

(defun link2cascoords
  (name link)
  (send *robot* name :end-coords))
;;  (make-cascoords :init :link-list
;;		  :name name
;;		  :coords (copy-object (send link :worldcoords))
;;		  :parent link))

(defclass simple-floor
  :super cascaded-link
  :slots (root-obj
	  floor-obj
	  width height z
	  ))

(defmethod simple-floor
  (:init
   (&rest
    args
    &key
    ((:name name) :simple-floor)
    ((:width ww) 5000)
    ((:height wh) 5000)
    ((:z wz) 0)
    &allow-other-keys)
   ;; set param
   (send-super* :init :name name args)
   (setq root-obj (make-cube 1 1 1))
   (setq floor-obj (make-cube ww wh 5))
   (setq width ww)
   (setq height wh)
   (setq z wz)
   (send floor-obj :translate (float-vector 0 0 z) :world)
   (send floor-obj :set-color #F(0.7 0.7 0.9))
   (send root-obj :set-color #F(0 0 0))
   (send root-obj :assoc floor-obj)
   (setq root-obj
	 (instance bodyset-link :init (make-cascoords)
		   :bodies (list root-obj floor-obj)
		   :name name))
   (send self :assoc root-obj)
   (setq bodies (list root-obj))
   (setq links (list root-obj))
   (send self :init-ending)
   self
   )
  (:vertical-vector nil #F(0 0 1))
  (:gen-contact-states
   (&key
    (dx 300) (dy 300)
    (width (min width 800))
    (height (min height 800))
    (x (* -1 width)) (y (* -1 height))
    (gain '(0.7 1.0 100))
    (cascoords
     (list (list (cons :name :rarm)
		 (cons :cascoords
		       (list
			(send *robot* :rarm :end-coords)
			;; (link2cascoords :rarm (send *robot* :rarm :elbow-p :child-link))
			))
		 (cons :translation-axis '(t t))
		 (cons :rotation-axis '(:z :z)))
	   (list (cons :name :larm)
		 (cons :cascoords
		       (list
			(send *robot* :larm :end-coords)
			;; (link2cascoords :larm (send *robot* :larm :elbow-p :child-link))
			))
		 (cons :translation-axis '(t t))
		 (cons :rotation-axis '(:z :z)))
	   (list (cons :name :rleg)
		 (cons :cascoords
		       (list
			(send *robot* :rleg :end-coords)
			;; (link2cascoords :rleg (send *robot* :rleg :knee-p :child-link))
			))
		 (cons :translation-axis '(t t))
		 (cons :rotation-axis '(:z :z)))
	   (list (cons :name :lleg)
		 (cons :cascoords
		       (list
			(send *robot* :lleg :end-coords)
			;; (link2cascoords :lleg (send *robot* :lleg :knee-p :child-link))
			))
		 (cons :translation-axis '(t t))
		 (cons :rotation-axis '(:z :z)))))
    ret
    )
   (while (<= x width)
     (setq y (* -1 height))
     (while (<= y height)
       (mapcar
	#'(lambda (data)
	    (push
	     (mapcar
	      #'(lambda (cs ta ra)
		  (instance
		   simple-contact-state
		   :init
		   :name (cdr (assoc :name data))
		   :contact-plane-obj :floor ;; floor-obj
		   :contact-coords cs
		   :contact-n #F(0 0 1)
		   :force0 #F(0 0 0 0 0 0)
		   :ux 0.5 :uy 0.5 :uz 0.5 :lx 0.05 :ly 0.05
		   :gain gain
		   :target-direction
		   '(lambda (tc ntc self &rest args)
		      (let ((val (v. (v- (send tc :worldpos) (send ntc :worldpos))
				     (v- (send *robot* :centroid) (send ntc :worldpos)))))
			(* (if (> val 0) 1 -1) (sqrt (abs val)))))
		   :translation-axis ta
		   :rotation-axis ra
		   :target-coords
		   (make-coords :pos (float-vector x y (+ 10 z)))))
	      (cdr (assoc :cascoords data))
	      (cdr (assoc :translation-axis data))
	      (cdr (assoc :rotation-axis data)))
	     ret))
	cascoords)
       (setq y (+ y dy)))
     (setq x (+ x dx)))
   ret)
  )

(defun gen-init-rsd
  (&key ret (contact-plane-obj :floor))
  (send *best-facefall* :draw :friction-cone? nil)
  (mapcar
   #'(lambda (cs)
       (send cs :set-val 'name
             (find-if
              #'(lambda (name)
                  (substringp
                   (format nil "~A" name)
                   (format nil "~A" (send (send cs :contact-coords) :name))))
              '(:rarm :larm :rleg :lleg))))
   (send *best-facefall* :contact-states))
  (mapcar
   #'(lambda (cs)
       (send cs :set-val 'rotation-axis t)
       (setf (aref (send (send cs :target-coords) :worldpos) 2)
             (if (find (send cs :name) '(:rarm :larm)) 10 10))
       (send cs :set-val 'contact-plane-obj contact-plane-obj))
   (append *contact-states* (send *best-facefall* :contact-states)))
  (simple-fullbody
   :target
   (list (list (cons :target :rarm))
         (list (cons :target :larm))
         (list (cons :target :rleg))
         (list (cons :target :lleg))
         (list (cons :target :torso)
               (cons :coords
                     (make-coords :pos (float-vector 0 0 500)))))
   :rotation-axis (list t t t t nil)
   :translation-axis (list t t t t (float-vector 0 0 1))
   :debug-view :no-message
   :balance-leg nil
   :target-centroid-pos nil
   :revert-if-fail nil)
  (simple-fullbody
   :target
   (mapcar
    #'(lambda (cs)
        (list (cons :target (send cs :contact-coords))
              (cons :coords (send cs :target-coords))))
    (send *best-facefall* :contact-states))
   :rotation-axis (list t t t t)
   :translation-axis (list t t t t)
   :debug-view :no-message
   :balance-leg nil
   :target-centroid-pos nil)
  (setq
   ret
   (optimize-brli :contact-states
                  (send *best-facefall* :contact-states)))
  (send ret :buf :time 5.0)
  (send *best-facefall* :buf :time 0.0)
  (connect-rsd
   :rsd-list (list ret *best-facefall*)
   :collision-avoid-trajectory-limb nil
   :contact-wrench-optimize-limb
   '(:rarm :larm :rleg :lleg)
   :linear-interpole-limb '(:revert-if-fail nil))
  (mapcar
   #'(lambda (cs)
       (send cs :set-val 'rotation-axis t)
       (setf (aref (send (send cs :target-coords) :worldpos) 2)
             (if (find (send cs :name) '(:rarm :larm)) 10 10))
       (send cs :set-val 'contact-plane-obj contact-plane-obj))
   (append *contact-states* (send *best-facefall* :contact-states)))
  ret
  )
