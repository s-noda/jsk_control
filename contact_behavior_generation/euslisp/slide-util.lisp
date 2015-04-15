;;  #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *log-stream* t)

;; @abstract
;;; (defun slide-condition-check (&rest args) nil)
(defun contact-plane-condition-check
  (cs1 cs2 &key (maxu 0.9))
  (and (send cs1 :contact-plane-obj)
       (send cs2 :contact-plane-obj)
       (eq (send cs1 :contact-plane-obj)
	   (send cs2 :contact-plane-obj))
       t
       ))

(defun move-dir-from-contact-states
  (from to)
  ;; (concatenate
  ;;  float-vector
  (normalize-vector
   (transform
    (send (send from :target-coords) :worldrot)
    (send (send from :target-coords)
	  :difference-position
	  (send to :target-coords))))
  ;; (normalize-vector
  ;;  (transform
  ;;   (send (send from :target-coords) :worldrot)
  ;;   (scale 1e-3
  ;; 	    (send (send from :target-coords)
  ;; 		  :difference-rotation
  ;; 		  (send to :target-coords))))))
  )

(defun calc-friction-coeff-from-contact-state
  (&key
   from to
   (move-dir (move-dir-from-contact-states from to))
   (local-move-dir)
   (local-move-dir-r)
   (slide-contact-state (list from))
   (u (mapcar
       #'(lambda (move-dir scs)
	   (abs
	    (v. (map float-vector
		     #'abs
		     (setq
		      local-move-dir
		      (transform
		       (transpose (send scs :z-rot))
		       (subseq move-dir 0 3))))
		(map float-vector
		     #'*
		     #F(1 1 0)
		     (subseq (transform (send scs :slip-matrix)
					#F(0 0 1 0 0 0)) 0 3)))))
       (if (listp move-dir) move-dir (list move-dir))
       slide-contact-state))
   (ur (if (> (length (car (flatten (list move-dir)))) 3)
	   (mapcar
	    #'(lambda (move-dir scs)
		(abs
		 (v. (map float-vector
			  #'abs
			  (setq
			   local-move-dir-r
			   (transform
			    (transpose (send scs :z-rot))
			    (subseq move-dir 3 6))))
		     (subseq (transform (send scs :slip-matrix)
					#F(0 0 1 0 0 0)) 3 6))))
	    (if (listp move-dir) move-dir (list move-dir))
	    slide-contact-state))))
  (list u ur local-move-dir local-move-dir-r))

(defun slide-friction-condition-extra-args
  (&rest
   args
   &key
   (contact-states (now-contact-state))
   (slide-contact-state (list (car (last contact-states))))
   from to
   (move-dir (move-dir-from-contact-states from to))
   (uur (calc-friction-coeff-from-contact-state
	 :move-dir move-dir
	 :slide-contact-state slide-contact-state))
   (u (car uur))
   (ur (cadr uur))
   (local-move-dir (caddr uur))
   (local-move-dir-r (cadddr uur))
   (friction-rate 0.8)
   (move-dir2
    (mapcar
     #'(lambda (move-dir u cs)
	 (concatenate float-vector
		      (transform
		       (send cs :z-rot) ;; sT RT f -> (Rs)T f
		       (float-vector (aref local-move-dir 0)
				     (aref local-move-dir 1)
				     (* +1 u friction-rate)))
		      (float-vector 0 0 0)))
     (if (listp move-dir) move-dir (list move-dir))
     u slide-contact-state))
   (move-dir2r
    (if (> (length (car (flatten (list move-dir)))) 3)
	(mapcar
	 #'(lambda (move-dir u cs)
	     (concatenate float-vector
			  (transform
			   (send cs :z-rot)
			   (float-vector 0 0 (* +1 u friction-rate)))
			  (transform
			   (send cs :z-rot)
			   (float-vector 0 ;;(aref move-dir 3)
					 0 ;;(aref move-dir 4)
					 (aref local-move-dir-r 5)))))
	 (if (listp move-dir) move-dir (list move-dir))
	 ur slide-contact-state)))
   ;;
   &allow-other-keys
   )
  (format *log-stream*
	  "[slide-friction-condition-extra-args]~%   v=~A u=~A~%"
	  (list move-dir2 move-dir2r) (list u ur))
  (cond
   ((and (or (not (vectorp local-move-dir)) (< (norm local-move-dir) 1e-3))
	 (or (not (vectorp local-move-dir-r)) (< (norm local-move-dir-r) 1e-3)))
    (format *log-stream*
	    "  no move and no slide (move=~A~%"
	    local-move-dir local-move-dir-r))
   (t
    (list
     :local-move-dir
     (normalize-vector (float-vector (aref local-move-dir 0)
				     (aref local-move-dir 1) 0))
     :limb-fmax-scale;; 0.0
     (mapcar #'(lambda (cs) (if (find cs slide-contact-state) 0.0 1.0))
	     contact-states)
     :extra-equality-matrix
     (make-matrix (* (length slide-contact-state)
		     (if (> (length (car (flatten (list move-dir)))) 3) 2 1))
		  (* (length contact-states) 6)
		  (append
		   (let ((id -1))
		     (apply
		      #'append
		      (mapcar
		       #'(lambda (u move-dir2 cs)
			   ;; (incf id)
			   (setq id (position cs contact-states))
			   (if (>= id 0)
			       (list
				(concatenate cons (instantiate float-vector (* 3 2 id))
					     move-dir2
					     (instantiate float-vector
							  (- (* (length contact-states) 6)
							     (* 3 (+ (* 2 id) 2))))))))
		       u move-dir2 slide-contact-state)))
		   (if (> (length (car (flatten (list move-dir)))) 3)
		       (let ((id -1))
			 (apply
			  #'append
			  (mapcar
			   #'(lambda (u move-dir2 cs)
			       ;; (incf id)
			       (setq id (position cs contact-states))
			       (if (>= id 0)
				   (list
				    (concatenate cons (instantiate float-vector (* 3 2 id))
						 move-dir2
						 (instantiate float-vector
							      (- (* (length contact-states) 6)
								 (* 3 (+ (* 2 id) 2))))))))
			   ur move-dir2r slide-contact-state))))))
     :extra-equality-vector
     (instantiate float-vector (* (length slide-contact-state)
				  (if (> (length (car (flatten (list move-dir)))) 3) 2 1)))))))

(defun slide-friction-condition-check
  (&rest
   args
   )
  (apply
   #'optimize-brli
   (append
    args
    (list
     :tmax-leg-rate 1.0
     :tmax-hand-rate 1.0)
    (apply #'slide-friction-condition-extra-args args))))

(defun demo-staro-hand-sliding
  (&key
   (now-cs) (next-cs) (u 0.3) (ux u) (uy u) (uz u)
   ret graph)
  (cond
   ((not (eq *robot-type* :staro))
    (setq *robot-type* :staro)
    (load "robot-param.l")
    (demo-climb-setup :simple-floor)))
  ;;
  (setq *kca-cog-gain* 5)
  ;; initiallize
  (send *robot* :reset-pose)
  (send *robot* :legs :ankle-p :joint-angle -80)
  (send *robot* :fix-leg-to-coords (make-coords))
  (let* ((key '(:rarm :larm :rleg :lleg))
	 (mt (mapcar #'(lambda (k) (send *robot* k :end-coords)) key))
	 (tc (send-all mt :copy-worldcoords))
	 (ll (mapcar #'(lambda (mt) (send *robot* :link-list (send mt :parent)))
		     mt)))
    (mapcar
     #'(lambda (tc)
	 (send tc :newcoords
	       (make-coords
		:pos (map float-vector #'*
			  '(1 1 0)
			  (send tc :worldpos)))))
     tc)
    (send *robot* :fullbody-inverse-kinematics
	  tc
	  :move-target mt
	  :link-list ll
	  :rotation-axis '(:z :z :z :z)
	  ;; :rotation-axis '(:xy :xy :xy :xy)
	  :translation-axis '(:xy :xy :xy :xy)
	  :min #F(-1000 -1000 -1000 -500 -500 -500)
	  :max #F(+1000 +1000 +1000 +500 +500 +500)
	  :target-centroid-pos nil
	  ;; :revert-if-fail nil
	  :debug-view :no-message))
  ;; (dotimes (i 5)
  ;; (send *robot* :inverse-kinematics (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #F(100 0 0) :world) :move-target (send *robot* :rarm :end-coords) :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent) (send *robot* :rarm :root-link)))
  ;; (send *robot* :inverse-kinematics (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #F(100 0 0) :world) :move-target (send *robot* :larm :end-coords) :link-list (send *robot* :link-list (send *robot* :larm :end-coords :parent) (send *robot* :larm :root-link))))
  ;;
  (setq now-cs
	(now-contact-state
	 :limb-keys
	 '(:rarm :larm :rleg :lleg)
	 :ux (make-list 4 :initial-element ux)
	 :uy (make-list 4 :initial-element uy)
	 :uz (make-list 4 :initial-element uz)
	 :lx (make-list 4 :initial-element 0.05)
	 :ly (make-list 4 :initial-element 0.05)
	 :contact-n
	 (make-list 4 :initial-element #F(0 0 1))
	 :force0
	 (make-list 4 :initial-element #F(0 0 0 0 0 0))))
  ;;
  (send *robot* :inverse-kinematics (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #F(100 0 0) :world) :move-target (send *robot* :rarm :end-coords) :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent) (send *robot* :rarm :root-link)))
  ;;
  (setq next-cs
	(now-contact-state
	 :limb-keys
	 '(:rarm :larm :rleg :lleg)
	 :ux (make-list 4 :initial-element ux)
	 :uy (make-list 4 :initial-element uy)
	 :uz (make-list 4 :initial-element uz)
	 :lx (make-list 4 :initial-element 0.05)
	 :ly (make-list 4 :initial-element 0.05)
	 :contact-n
	 (make-list 4 :initial-element #F(0 0 1))
	 :force0
	 (make-list 4 :initial-element #F(0 0 0 0 0 0))))
  ;;
  (setq
   ret
   (pose-generate-with-contact-state-with-time
    now-cs
    :rest-contact-states nil
    :non-stop t
    :stop 100
    :ik-debug-view :no-message
    :centroid-thre-rate 0.99
    :tau-gain 1.0
    :tmax-leg-rate 0.15
    :tmax-hand-rate 0.15
    :best-rsd? nil
    :optimize-brli-args
    (slide-friction-condition-extra-args
     :contact-states now-cs
     :from (car now-cs) :to (car next-cs))))
  ;; graph
  (setq
   graph
   (cons
    (create-graph
     (send :all-limbs :pname)
     :size '(400 400)
     :name-list (list "MAX" "BRLV" "T/Tmax" "F/Fmax")
     :inc-data-list
     (append
      (list (make-list (length ret) :initial-element 1))
      (let (tmp)
	(setq
	 tmp
	 (mapcar
	  #'(lambda (rsd)
	      (setq buf
		    (list (or (apply #'concatenate float-vector (send rsd :t/tmax))
			      #F(0))
			  (or (apply #'concatenate float-vector (send rsd :f/fmax))
			      #F(0))))
	      (list
	       (/ (+ (norm2 (car buf)) (norm2 (cadr buf)))
		  (+ (length (car buf)) (length (cadr buf))))
	       (/ (norm2 (car buf)) (length (car buf)))
	       (/ (norm2 (cadr buf)) (length (cadr buf)))))
	  (reverse ret)))
	(list (mapcar 'cadr tmp)
	      (mapcar 'car tmp)
	      (mapcar 'caddr tmp))))
     :range (list #F(0 0) (float-vector (length ret) 2)))
    (mapcar #'(lambda (k)
		(create-graph
		 (send k :pname)
		 :size '(400 400)
		 :name-list (list "MAX" "BRLV" "T/Tmax" "F/Fmax")
		 :inc-data-list
		 (append
		  (list (make-list (length ret) :initial-element 1))
		  (let (tmp)
		    (setq
		     tmp
		     (mapcar
		      #'(lambda (rsd)
			  (setq buf
				(list (or (send rsd :t/tmax k) #F(0))
				      (or (send rsd :f/fmax k) #F(0))))
			  (list
			   (/ (+ (norm2 (car buf)) (norm2 (cadr buf)))
			      (+ (length (car buf)) (length (cadr buf))))
			   (/ (norm2 (car buf)) (length (car buf)))
			   (/ (norm2 (cadr buf)) (length (cadr buf)))))
		      (reverse ret)))
		    (list (mapcar 'cadr tmp)
			  (mapcar 'car tmp)
			  (mapcar 'caddr tmp))))
		 :range (list #F(0 0) (float-vector (length ret) 2))
		 ))
	    '(:rarm :larm :rleg :lleg))))
  (place-graph-in-order)
  (send-all graph :fit-draw)
  (send-all graph :simple-draw-with-line)
  ret)


#|

u = 1.0
send (car a) :contact-forces :rarm
#f(-32.4756 31.1823 156.096 0.020187 0.554742 1.30999)
send (car a) :contact-forces :larm
#f(-38.7328 -46.2522 223.946 0.126881 0.639045 -4.38726)

u = 0.01
send (car a) :contact-forces :rarm
send (car a) :contact-forces :larm
#f(-0.273543 0.038621 345.834 1.66003 15.9387 0.024487)
#f(-0.202917 -0.211669 249.049 -2.9271 12.4524 -0.132638)

(require "util/gp-util.l")
(mapcar
 #'(lambda (graph)
     (if (or (substringp "rarm" (send graph :name))
	     (substringp "larm" (send graph :name)))
	 (graph-panel2gp-graph graph :ylabel "NORM" :xlabel "LOOP COUNT" :save? t :ratio 0.7)))
 *graph-sample*)


roseus
(defvar *robot-type* :hrp2jsknts-collada)
(require "motion-sequencer.l")
(demo-climb-setup :kirin-ladder)
(setq
 a
 (demo-motion-sequence
  :loop-max 8
  :tmax-hand-rate 0.6
  :tmax-leg-rate 0.6
  ))

(require "dynamic-connector.l")
(connect-rsd :rsd-list a)

(rsd-serialize :rsd-list a :file "slide-kirin-climb.rsd")
