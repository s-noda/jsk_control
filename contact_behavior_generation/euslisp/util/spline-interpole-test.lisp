;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsknt-collada)
(require "../dynamics-motion/dynamic-trajectory.l")
(require "../util/partition-spline/partition-spline.l")

(defun minjerk-coords-interpole
  (&key
   (coords-list (list (make-coords)
		      ;;(make-coords :pos #F(1 0 0))
		      (make-coords :pos #F(0 1 0))))
   (time-list (list 1.0))
   (cnt 30)
   (step (/ (apply #'+ time-list) (* 1.0 (- cnt 1))))
   ;;
   (coords-list-check
    (cond
     ((< (length coords-list) 2) 'error)
     ((eq (length coords-list) 2)
      (setq coords-list (list (nth 0 coords-list)
			      (apply #'midcoords (cons 0.5 coords-list))
			      (nth 1 coords-list))))
     (t 'ok)))
   (6dof-list
    (mapcar #'(lambda (c pre_c)
		(concatenate float-vector
			     (send c :worldpos)
			     (v+ (matrix-log (send pre_c :worldrot))
				 (send pre_c :difference-rotation c))))
	    coords-list (cons (make-coords) (cdr coords-list))))
   (spline
    (pos-list-interpolate-spline-minjerk 6dof-list time-list step))
   ;;(list #F(0 0) #F(0 1) #F(1 0)) (list 10) 1.0)
   ret
   )
  (mapcar
   #'(lambda (v)
       (make-coords :pos (subseq v 0 3)
		    :rot (matrix-exponent (subseq v 3 6))))
   (cadr (member :data spline)))
  )

(defun minjerk-angle-vector-interpole
  (&rest
   args
   &key
   (robot *robot*)
   (start-av (scale 0 (send robot :angle-vector)))
   mid-av
   (end-av (send robot :angle-vector))
   (start-coords (make-coords))
   mid-coords
   (end-coords (copy-object (send robot :worldcoords)))
   (start-coords-vector
    (concatenate float-vector
		 (send start-coords :worldpos)
		 (matrix-log (send start-coords :worldrot))))
   (mid-coords-vector
    (mapcar
     #'(lambda (mid prev)
	 (concatenate float-vector
		      (send mid :worldpos)
		      (v+
		       (matrix-log (send prev :worldrot))
		       (send prev
			     :difference-rotation
			     mid))))
     mid-coords (cons start-coords mid-coords)))
   (end-coords-vector
    (concatenate float-vector
		 (send end-coords :worldpos)
		 (v+
		  (matrix-log (send (or (car (last mid-coords)) start-coords)
				    :worldrot))
		  (send (or (car (last mid-coords)) start-coords)
			:difference-rotation
			end-coords))))
   (start-vector (concatenate float-vector start-av start-coords-vector))
   (mid-vector (mapcar
		#'(lambda (av cv) (concatenate float-vector av cv))
		mid-av mid-coords-vector))
   (end-vector (concatenate float-vector end-av end-coords-vector))
   (time-list (list 0.1))
   (cnt 30)
   (step (/ (apply #'+ time-list) (* 1.0 (- cnt 1.0))))
   (vector-list (append (list start-vector) mid-vector (list end-vector)))
   ;;
   (vector-list-check
    (cond
     ((<= (length vector-list) 2) 'error)
     ;; ((eq (length vector-list) 2) ;; euslib virtual_interpole needs at least 3 points
     ;;  (setq vector-list (list (nth 0 vector-list)
     ;; 			      (scale 0.5 (apply #'v+ vector-list))
     ;; 			      (nth 1 vector-list))))
     (t 'ok)))
   (start-vel (scale 0 start-vector))
   (start-acc (scale 0 start-vector))
   (end-vel (scale 0 end-vector))
   (end-acc (scale 0 end-vector))
   (vel-vector-list
    (append
     (list start-vel)
     (make-list (- (length vector-list) 2)
		:initial-element (scale 0 start-vector))
     (list end-vel)))
   (acc-vector-list
    (append
     (list start-acc)
     (make-list (- (length vector-list) 2)
		:initial-element (scale 0 start-vector))
     (list end-acc)))
   ;;(make-list (length vector-list) :initial-element 0.0))
   ;; (hoge (progn
   ;; (print vector-list) (print time-list) (print step)))
   (spline
    (pos-list-interpolate-spline-minjerk
     vector-list time-list step
     ;; :interpolator-class linear-interpolator
     :vel-vector-list vel-vector-list
     :acc-vector-list acc-vector-list
     ))
   ;;(list #F(0 0) #F(0 1) #F(1 0)) (list 10) 1.0)
   ret
   &allow-other-keys
   )
  ;; (format t " ~A vs ~A~%" start-coords-vector end-coords-vector)
  spline
  )

(defun minjerk-angle-vector-submanifold
  (&rest
   args
   &key
   (robot *robot*)
   (start-av (scale 0 (send robot :angle-vector)))
   (end-av (copy-object (send robot :angle-vector)))
   (start-coords (copy-object (send robot :worldcoords)))
   (end-coords (copy-object (send robot :worldcoords)))
   (start-traj (instance trajectory-elem :init :position start-av :coords start-coords))
   (end-traj (instance trajectory-elem :init :position end-av :coords end-coords))
   ;;
   (time-step 0.1)
   (cnt 30)
   (min-time 0.5)
   ;;
   (check-func #'(lambda (traj-list) nil))
   (filter-func #'(lambda (traj-list id) (nth id traj-list)))
   ;; (first? t)
   (init
    (progn (setq start-traj (funcall filter-func (list start-traj) 0))
	   (setq end-traj (funcall filter-func (list end-traj) 0))))
   ;;
   (ret (apply #'minjerk-angle-vector-interpole args))
   (traj
    (progn
      (gen-dynamic-trajectory
       :freq (/ 1.0 time-step)
       :position-list
       (mapcar
	#'(lambda (v) (subseq v 0 (- (length v) 6)))
	(cadr (member :data ret)))
       :coords-list
       (mapcar
	#'(lambda (v)
	    (let* ((p (subseq v (- (length v) 6) (- (length v) 3)))
		   (r (subseq v (- (length v) 3) (- (length v) 0)))
		   (m (matrix-exponent r)))
	      (make-coords :pos p :rot m)))
	(cadr (member :data ret)))
       :fix-length t)))
   (depth 0)
   (mid-id (round (/ (length traj) 2.)))
   mid
   ;;
   (start-vel (instantiate float-vector (+ (length start-av) 6)))
   (end-vel (instantiate float-vector (+ (length start-av) 6)))
   (start-acc (instantiate float-vector (+ (length start-av) 6)))
   (end-acc (instantiate float-vector (+ (length start-av) 6)))
   &allow-other-keys
   )
  (format t "~A ~A~%" (make-list (+ 1 depth) :initial-element "-") cnt)
  (cond
   ((>= min-time (* cnt time-step)) traj)
   ((funcall check-func traj) traj)
   (t (setq mid (funcall filter-func traj mid-id))
      (append (butlast
	       (apply #'minjerk-angle-vector-submanifold
		      (append
		       (list :start-av (send start-traj :position)
			     :start-coords (send start-traj :coords)
			     :end-av (send mid :position)
			     :end-coords (send mid :coords)
			     :cnt (+ mid-id 0)
			     :depth (+ depth)
			     :init nil
			     ;; :start-vel start-vel
			     ;; :start-acc start-acc
			     ;; :end-vel (concatenate float-vector
			     ;; 			   (send mid :velocity)
			     ;; 			   (send mid :w))
			     ;; :end-acc (concatenate float-vector
			     ;; 			   (send mid :acceleration)
			     ;; 			   (send mid :dw))
			     )
		       args)))
	      (list mid)
	      (cdr
	       (apply #'minjerk-angle-vector-submanifold
		      (append
		       (list :start-av (send mid :position)
			     :start-coords (send mid :coords)
			     :end-av (send end-traj :position)
			     :end-coords (send end-traj :coords)
			     :cnt (+ (- (length traj) mid-id) 1)
			     :depth (+ depth 1)
			     :init nil
			     ;; :start-vel (concatenate float-vector
			     ;; 			     (send mid :velocity)
			     ;; 			     (send mid :w))
			     ;; :start-acc (concatenate float-vector
			     ;; 			     (send mid :acceleration)
			     ;; 			     (send mid :dw))
			     ;; :end-vel end-vel
			     ;; :end-acc end-acc
			     )
		       args)))))))

(defun animate-minjerk-angle-vector-submanifold-test
  (&rest args &key
	 (ik-params
	  (list :centroid-thre 50 :stop 30 :revert-if-fail nil))
	 (fix-limbs '(:rleg :lleg :rarm :larm))
	 (time-step 0.1)
	 (animate? t)
	 (debug-view :no-message)
	 (init
	  (progn
	    (mapcar
	     #'(lambda (j) (send j :joint-angle
				 (+ (send j :min-angle)
				    (* (random 1.0)
				       (- (send j :max-angle)
					  (send j :min-angle))))))
	     (append (send *robot* :rarm :joint-list)
		     (send *robot* :larm :joint-list)
		     (send *robot* :torso :joint-list)))
	    ;;(send *robot* :newcoords
	    ;; (make-coords :rpy (random-vector 3.14)))
	    ;;(send *robot* :reset-manip-pose)
	    (send *robot* :fix-leg-to-coords (make-coords))
	    (send *viewer* :draw-objects)))
	 ;;
	 (target-cascoords
	  (mapcar #'(lambda (k) (send *robot* k :end-coords)) fix-limbs))
	 ;; (target-coords (send-all target-cascoords :copy-worldcoords))
	 (target-coords
	  #'(lambda (&rest args)
	      (send-all target-cascoords :worldcoords)
	      (send-all target-cascoords :copy-worldcoords)))
	 (check-func
	  #'(lambda (traj-list)
	      (not
	       (flatten
		(mapcar
		 #'(lambda (traj)
		     (send *robot* :angle-vector
			   (copy-object (send traj :position)))
		     (dotimes (i 3)
		       (send *robot* :newcoords
			     (copy-object (send traj :coords))))
		     (send *viewer* :draw-objects)
		     ;;(format t " check constratins ")
		     ;;(print
		     (mapcar
		      #'(lambda (tc c)
			  (or (> (norm (send tc :difference-position c)) 5)
			      (> (norm (send tc :difference-rotation c)) (deg2rad 3))))
		      (if (functionp target-coords)
			  (funcall target-coords) target-coords)
		      target-cascoords))
		 ;; )
		 traj-list)))
	      ;;t
	      ))
	 (filter-func
	  #'(lambda (traj id)
	      (send *robot* :angle-vector
		    (copy-object (send (nth id traj) :position)))
	      (dotimes (i 3)
		(send *robot* :newcoords
		      (copy-object (send (nth id traj) :coords))))
	      ;; (send *robot* :fix-leg-to-coords
	      ;; 	    (car (if (functionp target-coords)
	      ;; 		     (funcall target-coords) target-coords))
	      ;; 	    (car fix-limbs))
	      (apply
	       #'send
	       (append
		(list *robot* :fullbody-inverse-kinematics
		      (if (functionp target-coords)
			  (funcall target-coords) target-coords))
		ik-params
		(list
		 :move-target target-cascoords
		 :link-list (mapcar
			     #'(lambda (k)
				 (send *robot* :link-list
				       (send k :parent)))
			     target-cascoords)
		 :debug-view debug-view
		 :min #F(-1000 -1000 -1000 -400 -400 -400)
		 :max #F(1000 1000 1000 400 400 400))))
	      (instance trajectory-elem :init
			:position (copy-object (send *robot* :angle-vector))
			:coords (copy-object (send *robot* :worldcoords)))))
	 (ret
	  (apply #'minjerk-angle-vector-submanifold
		 (append
		  (list :check-func check-func
			:filter-func filter-func)
		  args)
		 ))
	 &allow-other-keys
	 )
  ;;(send *pickview* :objects (list *robot*))
  (if animate?
      (mapcar
       #'(lambda (traj)
	   (send *robot* :angle-vector (copy-object (send traj :position)))
	   (dotimes (i 3) (send *robot* :newcoords (copy-object (send traj :coords))))
	   (send *viewer* :draw-objects)
	   (x::window-main-one)
	   (unix::usleep (round (* time-step 1000 1000)))
	   )
       ret))
  ret)

(defun minjerk-coords-interpole-angle-vector
  (&key
   (robot *robot*)
   (coords-list (list (make-coords)
		      (make-coords :pos #F(1 0 0))
		      (make-coords :pos #F(0 1 0))))
   (cnt 15)
   (interpolated-coords-list
    (minjerk-coords-interpole
     :coords-list coords-list :cnt cnt))
   (arm :rarm)
   (move-target (send robot arm :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (animate t)
   (real? nil)
   (interpole-time 3000)
   ret
   )
  (dolist (c interpolated-coords-list)
    (send robot :inverse-kinematics c
	  :move-target move-target
	  :link-list link-list
	  :dif-pos-ratio 0.1
	  :dif-rot-ratio 0.1
	  ;;:debug-view :no-message
	  :stop 7
	  :revert-if-fail nil)
    (cond
     ((and animate (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects)
      (unix:usleep (* 100 1000))))
    (push (copy-object (send robot :angle-vector)) ret))
  (cond
   ((and real? (or (and (functionp real?) (funcall real?))
		   (not (functionp real?))))
    (send *ri* :angle-vector-sequence
	  (reverse ret)
	  (make-list (length ret)
		     :initial-element
		     (/ (* 1.0 interpole-time) (length ret))))))
  (reverse ret))

(defun test-spline-interpole
  nil
  (let* ((end (progn
		(send *robot* :init-pose)
		(copy-object (send *robot* :larm :end-coords :copy-worldcoords))))
	 (start (progn
		  (send *robot* :reset-fight-pose)
		  (copy-object (send *robot* :larm :end-coords :copy-worldcoords)))))
    (minjerk-coords-interpole-angle-vector
     :robot *robot*
     :coords-list (list start end)
     :arm :larm)))

(defun demo-reach-ladder-spline-interpole
  (&key (remove-limb :rarm)
	(loop-max 1)
	(init1 (progn (init-pose) (send *robot* :reset-manip-pose)
		      (send *robot* :arms :shoulder-p :joint-angle 0)))
	(start-av (copy-object (send *robot* :angle-vector)))
	(start-coords (copy-object (send *robot* :worldcoords)))
	(fix-limbs '(:rleg :lleg :rarm :larm))
	(target-cascoords
	 (mapcar #'(lambda (k) (send *robot* k :end-coords))
		 fix-limbs))
	target-coords
	(cnt 30)
	(init2
	 (progn
	   (cond
	    ((not (and (boundp '*climb-obj*) *climb-obj*))
	     (setq *robot-type* :hrp2jsknt-collada)
	     ;; (require "spline-interpole-test.l")
	     (require "../../motion-sequencer.l")
	     (demo-climb-setup :kirin-ladder)
	     (return-from demo-reach-ladder-spline-interpole nil)
	     ))
	   (demo-motion-sequence :loop-max loop-max
				 :all-limbs fix-limbs
				 :remove-limb remove-limb)))
	(end-av (copy-object (send *robot* :angle-vector)))
	(end-coords (copy-object (send *robot* :worldcoords)))
	(debug-view :no-message)
	(ik-params nil)
	)
  (setq target-coords
	(mapcar #'(lambda (k) (send *robot* k :end-coords :copy-worldcoords)) fix-limbs))
  (let* ((ladder (instance bodyset-link :init (make-cascoords)
			   :bodies (list *climb-obj*)))
	 (joint1 (instance rotational-joint :init :min 0.0 :max 0.0
			   :child-link ladder
			   :parent-link (car (last (send *robot* :rarm :links)))))
	 (joint2 (instance rotational-joint :init :min 0.0 :max 0.0
			   :child-link ladder
			   :parent-link (car (last (send *robot* :larm :links)))))
	 (hands
	  (list (send (send *robot* :hand :rarm) :links)
		(send (send *robot* :hand :larm) :links)))
	 (hands2
	  (list
	   (cadr (reverse (send *robot* :rarm :links)))
	   (cadr (reverse (send *robot* :larm :links)))))
	 (hands-joint
	  (mapcar
	   #'(lambda (hh1 h2)
	       (mapcar
		#'(lambda (h1)
		    (send h1 :add-joint
			  (instance rotational-joint :init
				    :min 0.0 :max 0.0
				    :child-link h1
				    :parent-link h2)))
		hh1))
	   hands hands2))
	 )
    (send ladder :add-joint joint1)
    (send ladder :add-joint joint2)
    (animate-minjerk-angle-vector-submanifold-test
     :fix-limbs fix-limbs
     :end-av end-av
     :end-coords end-coords
     :start-av start-av
     :start-coords start-coords
     :target-cascoords target-cascoords
     :target-coords
     #'(lambda (&rest args)
	 (mapcar
	  #'(lambda (k c)
	      (if (eq k remove-limb)
		  (send *robot* k :end-coords :worldcoords)
		c))
	  fix-limbs target-coords))
     :init nil
     :ik-params
     (append
      ik-params
      (list :collision-avoidance-link-pair
	    (mapcar
	     #'(lambda (l) (list l ladder))
	     (cond
	      ((eq remove-limb :rarm)
	       (append (car hands)
		       (send *robot* :link-list
			     (send *robot* :rarm :end-coords :parent))))
	      ((eq remove-limb :larm)
	       (append (cadr hands)
		       (send *robot* :link-list
			     (send *robot* :larm :end-coords :parent))))))
	    :link-list
	    (mapcar
	     #'(lambda (k mt)
		 (cond
		  ((eq k :rarm)
		   (append
		    (car hands)
		    (send *robot* :link-list (send mt :parent))))
		  ((eq k :larm)
		   (append
		    (cadr hands)
		    (send *robot* :link-list (send mt :parent))))
		  (t (send *robot* :link-list (send mt :parent)))))
	     fix-limbs target-cascoords)
	    :avoid-collision-distance 100
	    :avoid-collision-null-gain 3.
	    :avoid-collision-joint-gain 3.
	    :thre '(5 5 5 5)
	    :rthre (list (deg2rad 3) (deg2rad 3)
			 (deg2rad 3) (deg2rad 3))
	    :revert-if-fail nil
	    :stop 30
	    ))
     :debug-view debug-view
     :check-func #'(lambda (&rest args) nil)
     :cnt cnt
     )
    )
  )

#|
(progn (if (not (and (boundp '*robot*) *robot*)) (setq *robot* (hrp2jsknt-simple-detail))) (cond ((not (and (boundp '*viewer*) *viewer*)) (pickview :no-menu t) (objects *robot*))) (send *robot* :reset-manip-pose) (reach-audience-table :real? nil))

(setq *robot* (hrp2jsknt-simple-detail))
(pickview :no-menu t)
(objects *robot*)

;;

roseus spline-interpole-test.l 
(demo-reach-ladder-spline-interpole)
;; (init-pose) (demo-motion-sequence :loop-max 2)

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
(hrp2jsknts-init)

(progn (init-pose) (send *robot* :reset-manip-pose) (send *robot* :arms :shoulder-p :joint-angle 0))

(setq a (demo-reach-ladder-spline-interpole :remove-limb :rarm :init (progn (init-pose) (send *robot* :reset-manip-pose) (send *robot* :arms :shoulder-p :joint-angle 0))))
(dump-loadable-structure "log/rarm-reach.log" a)
(setq a (demo-reach-ladder-spline-interpole :remove-limb :larm :init nil))
(dump-loadable-structure "log/larm-reach.log" a)

(open-hand-pose)

(load "log/rarm-reach.log")
(send *ri* :angle-vector-sequence (send-all a :position) (make-list (length a) :initial-element 1200))

(load "log/larm-reach.log")
(send *ri* :angle-vector-sequence (send-all a :position) (make-list (length a) :initial-element 1200))

(hook-hand-pose)
(grasp-hand-pose)

(send *ri* :start-impedance :arms :m-p 20 :d-p 10000 :k-p 0.1)


(require "package://hrpsys_gazebo_tutorials/euslisp/hand-command-publisher.l")
(setq *hrp2* *robot*)
(send *ri* :init-hand)

(defun rs (&optional (scale 1))
  (send *robot* :reset-pose)
  (send *robot* :rarm :shoulder-r :joint-angle -60)
  (send *robot* :larm :shoulder-r :joint-angle +60)
  (send *ri* :angle-vector
	(scale scale (send *robot* :angle-vector))))

(if (find-method *robot* :hand)
    (mapcar
     #'(lambda (k hl)
	 (mapcar
	  #'(lambda (h)
	      (cond
	       ((null (send h :parent-link)))
	       ((null (send (send h :parent-link) :parent-link))
		(send h :add-parent-link
		      (send *robot* k :end-coords :parent))
		(send (send *robot* k :end-coords :parent)
		      :add-child-links h))))
	  hl))
     '(:rarm :larm)
     (list (send *robot* :hand :rarm :links)
	   (send *robot* :hand :larm :links))))
