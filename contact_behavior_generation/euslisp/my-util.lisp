;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "util/vector-util.lisp")

(defvar *window-main-one-loop-running* nil)
(defvar *log-stream* t)

(sys:make-thread 5)
(defun window-main-one-loop
  nil
  (setq *window-main-one-loop-running* t)
  (sys::thread
   #'(lambda nil
       (print 'window-main-one-loop-start)
       (while *window-main-one-loop-running*
	 (unix:usleep 100000)
	 (x::window-main-one))
       (print 'window-main-one-loop-dead))))

(defun simple-fullbody
  (&rest
   args
   &key
   (robot *robot*)
   (center #f(0 0 0))
   (target-centroid-pos nil)
   (torso-null-space nil)
   (balance-leg :both)
   (target nil) ;; ( ((:target . :rleg) (:move . #f(0 0 0)) (:translation-axis . :x)) ...
   collision-avoidance-link-pair
   &allow-other-keys)
  (let* ((ll-buf)
	 (rest-arg (remove-args-values args
                                       (list :robot :center :balance-leg
                                             :target :target-centroid-pos
                                             :torso-null-space)))
	 (axis-move-key
	  (list :rotation-axis :translation-axis :thre))
	 (target-centroid-pos
	  (if (find :target-centroid-pos args)
	      target-centroid-pos
	    (v+ center
;		(send robot :calc-target-centroid-pos
;		      balance-leg '(:lleg :rleg))
		(scale
		 (/ 1.0 (case balance-leg
			      (:both 2)
			      (t 1)))
		 (reduce
		  #'v+
		  (append
		   (list #f(0 0 0) #f(0 0 0))
		   (mapcar
		    #'(lambda (k)
			(send robot k :end-coords :worldpos))
		    (case balance-leg
			  (:rleg '(:rleg))
			  (:lleg '(:lleg))
			  (:both '(:rleg :lleg)))))))
		)))
	 (axis-move-with-leg
	  (if balance-leg
	      (let ((lleg (if (find-if
			       #'(lambda (a)
				   (or (eq (cdr (assoc :target a)) :lleg)
				       (and
					(find-method a :name)
					(reg-match
					 "lleg"
					 (format nil "~A" (send a :name))))))
			       target)
			      nil (list (list (cons :target :lleg)))))
		    (rleg (if (find-if
			       #'(lambda (a)
				   (or (eq (cdr (assoc :target a)) :rleg)
				       (and
					(find-method a :name)
					(reg-match
					 "rleg"
					 (format nil "~A" (send a :name))))))
			       target)
			      nil
			    (list (list (cons :target :rleg))))))
		(case balance-leg
		  (:lleg (append lleg target))
		  (:rleg (append rleg target))
		  (:both (append lleg rleg target))))
	    target))
;	 move-target-buf
	 dissoc-buf
	 (fill-axis-move
	  (mapcar
	   #'(lambda (am)
	       (append
		(let* ((obj (cdr (assoc :target am)))
		       (move (cdr (assoc :move am)))
		       (rpy (cdr (assoc :rpy am)))
		       (worldcoords (cdr (assoc :coords am)))
		       (dissoc? nil)
		       link-list target-coords move-target)
		  (setq move-target
			(cond ((keywordp obj)
			       (send robot obj :end-coords))
			      ((eq 'cascaded-coords (send (class obj) :name))
			       obj)
			      ((or (eq 'robot-link (send (class obj) :name))
				   (eq 'bodyset-link (send (class obj) :name)))
			       (setq dissoc? t)
			       (make-cascoords :init :link-list
					       :coords (send obj :worldcoords)
					       :parent obj))
			      (t
			       (setq dissoc? t)
			       (make-cascoords :init :link-list
					       :pos
					       (copy-seq (send (send obj :child-link) :worldpos))
					       :rot
					       (copy-object
						(send (send obj :child-link) :worldrot))
					       :name
					       (if (find-method obj :name)
						   (send obj :name)
						 :unknown)
					       :parent (send obj :child-link)))
			      ))
		  (if dissoc?
		      (push move-target dissoc-buf))
		  (setq link-list
			(send robot :link-list (send move-target :parent)))
		  (setq target-coords
			(send (send move-target :worldcoords)
			      :copy-worldcoords))
		  (if worldcoords (setq target-coords worldcoords))
		  (if move (send target-coords :translate move :world))
		  (if rpy
		      (apply #'send (append (list target-coords :rpy) rpy)))
		  (list (cons :move-target move-target)
			(cons :target-coords target-coords)
			(cons :link-list link-list)))
		(mapcar
		 #'(lambda (key)
		     (let ((obj (assoc key am)))
		       (case key
			 (:thre (cons :thre (if obj (cdr obj) 5)))
			 (:rotation-axis (cons :rotation-axis (if obj (cdr obj) t)))
			 (:translation-axis (cons :translation-axis (if obj (cdr obj) t))))
		       ))
		 axis-move-key)
		)
	       )
	   axis-move-with-leg))
	 (get-target-link #'(lambda (key)
			      (mapcar #'(lambda (am)
					  (cdr (assoc key am))) fill-axis-move)))
	 (torso-null-space
	  (if torso-null-space
	      (let* ((dof
		      (send robot :calc-target-joint-dimension
			    (mapcar
			     #'(lambda (k)
				 (send robot :link-list
				       (send robot k :end-coords :parent)))
			     (mapcar #'(lambda (tar) (cdr (assoc :target tar)))
				     axis-move-with-leg))))
		    (tv (instantiate float-vector (+ dof 6))))
		(dotimes (i (length torso-null-space))
		  (setf (aref tv i) (aref torso-null-space i)))
		tv)))
	 ret
	 )
    (setq
     ret
     (apply #'send
	    (append
	     (list robot :fullbody-inverse-kinematics
		   (funcall get-target-link :target-coords)
		   :link-list (setq ll-buf (funcall get-target-link :link-list))
		   :move-target (funcall get-target-link :move-target)
		   :target-centroid-pos target-centroid-pos
		   :collision-avoidance-link-pair
		   (progn
		     (setq ll-buf (union (flatten ll-buf) nil))
		     (remove-if #'(lambda (ll)
				    (or (not (find (car ll) ll-buf))
					(not (find (cadr ll) ll-buf))))
				collision-avoidance-link-pair))
		   )
	     (if torso-null-space
		 (list :null-space #'(lambda nil torso-null-space))
	       nil)
	     (reduce
	      #'append
	      (mapcar #'(lambda (key)
			  (list key (funcall get-target-link key)))
		      axis-move-key))
	     rest-arg)
	    ))
    (mapcar #'(lambda (mt) (send (send mt :parent) :dissoc mt)) dissoc-buf)
    ret
    ))

(defun init-pose (&key (robot *robot*) (move 0) (rot-z 0) (debug-view nil))
  (send robot :reset-pose)
  ;; (send robot :newcoords (make-coords))
  (send robot :fix-leg-to-coords (make-coords) :both)
  (simple-fullbody
   :debug-view debug-view
   :centroid-thre 0.1
   :target-centroid-pos
   (scale 0.5 (apply #'v+ (send robot :legs :end-coords :worldpos)))
   :target
   (list
    (list (cons :target :rleg)
	  (cons :rpy (list rot-z 0 0))
	  (cons :move (float-vector 0 move 0)))
    (list (cons :target :lleg)
	  (cons :rpy (list (* -1 rot-z) 0 0))
	  (cons :move (float-vector 0 (* -1 move) 0)))))
  (send robot :fix-leg-to-coords (make-coords) :both)
  ;; (send robot :locate
  ;; (float-vector
  ;; 0 0  (* -1 (aref (send (send robot :lleg :end-coords) :worldpos) 2))))
  )

(defun force-sensor-cascoords
  (&key (robot *robot*))
  (if (and (boundp '*force-sensor-cascoords*)
	   *force-sensor-cascoords*)
      *force-sensor-cascoords*
    (setq *force-sensor-cascoords*
	  (mapcar #'(lambda (k)
		      (cons k (send (send robot k :end-coords :parent)
				    :assoc
				    (make-cascoords
				     :name
				     (read-from-string
				      (format nil "~A-force-sensor" k))
				     :coords
				     (send (send robot :force-sensor k)
					   :worldcoords)))))
		  '(:rarm :larm :rleg :lleg)))))

(defun ankle-end-coords
  (leg-key &key (robot *robot*))
  (if (and (boundp '*ankle-coords*)
	   *ankle-coords*)
      (cdr (assoc leg-key *ankle-coords*))
    (cdr
     (assoc leg-key
	    (setq *ankle-coords*
		  (let ((rleg (make-cascoords
			       :name :rleg-ankle-end-coords
			       :coords
			       (send robot :rleg :end-coords :copy-worldcoords)))
			(lleg (make-cascoords
			       :name :lleg-ankle-end-coords
			       :coords
			       (send robot :lleg :end-coords :copy-worldcoords))))
		    (send (send (send robot :rleg :end-coords :parent) :parent)
			  :assoc
			  rleg)
		    (send (send (send robot :lleg :end-coords :parent) :parent)
			  :assoc
			  lleg)
		    (list (cons :rleg rleg)
			  (cons :lleg lleg))))))))

(defvar *cascoords-collection*)
(defun cascoords-collection
  (&key
   (limb-key :rarm)
   (angle-vector (copy-seq (send *robot* :angle-vector)))
   (root-pos (copy-seq (send *robot* :worldpos)))
   (root-rot (copy-object (send *robot* :worldrot)))
   (pos #F(0 0 0))
   (rot (unit-matrix 3))
   (coords (make-coords :pos pos :rot rot))
   (parent-link-name (send (send *robot* limb-key :end-coords :parent) :name))
   (parent-link (find-if #'(lambda (a)
                             (substringp (format nil "~A" parent-link-name)
                                         (format nil "~A" (send a :name))))
			 (if (find-method *robot* limb-key)
			     (send *robot* limb-key :links)
			   (send *robot* :links)))))
  (send *robot* :angle-vector angle-vector)
  (send *robot* :newcoords (make-coords :pos root-pos :rot root-rot))
  (or
   (find-if #'(lambda (cd)
		(and (eq parent-link (send cd :parent))
		     (< (norm (send cd :difference-position coords)) 1)
		     (< (norm (send cd :difference-rotation coords)) (deg2rad 1))))
	    *cascoords-collection*)
   (car
    (push (make-cascoords :init :link-list
			  :name (read-from-string
				 (format nil "~A-cascoords-collection-~A"
					 limb-key (length *cascoords-collection*)))
			  :coords coords :parent parent-link)
	  *cascoords-collection*))))

(defun object-serializer
  (obj)
  (cond
   ((or (numberp obj) (arrayp obj) (symbolp obj)) obj)
   ((and (listp obj) ;; dot-pair
	 (cdr (last obj))
	 (atom (cdr (last obj))))
    (append (mapcar #'object-serializer obj)
	    (object-serializer (cdr (last obj)))))
   ((and (listp obj) (not (functionp obj)))
    (mapcar #'object-serializer obj))
   ((and (listp obj) (functionp obj))
    (list 'lambda nil
	  (list 'read-from-string
		(format nil "'~A"
			(cons 'lambda
			      (subseq obj (- (length obj) 2) (length obj)))))))
   ((eq coordinates (class obj))
    (list 'lambda nil
	  (list 'make-coords
		:name (send obj :name)
		:pos (send obj :worldpos)
		:rot (send obj :worldrot))))
   ((eq cascaded-coords (class obj))
    (list
     'lambda nil
     (list 'cascoords-collection
	   :limb-key
	   (find-if
	    #'(lambda (k)
                (or
                 (substringp (format nil "~A" k)
                             (format nil "~A" (send (send obj :parent) :name)))
                 (substringp (format nil "~A" k)
                             (format nil "~A" (send obj :name)))
                 ))
	    '(:rarm :larm :rleg :lleg))
	   :parent-link-name (send (send obj :parent) :name)
	   :angle-vector (copy-seq (send *robot* :angle-vector))
	   :root-pos (copy-seq (send *robot* :worldpos))
	   :root-rot (copy-object (send *robot* :worldrot))
	   :pos (copy-seq (send obj :worldpos))
	   :rot (copy-object (send obj :worldrot)))))
   (t
    (cons
     (send (class obj) :name)
     (mapcar
      #'(lambda (key-val)
;	  (print key-val)
	  (cons (read-from-string (format nil ":~A" (car key-val)))
		(object-serializer (cdr key-val))))
      (send obj :slots))))))

(defun object-deserializer
  (obj)
  (let (top)
    (cond
     ((and (functionp obj)
	   (not (and (listp obj) (eq (car obj) 'lambda-closure))))
      (funcall obj))
     ((atom obj) obj)
     ((and (symbolp (car obj))
	   (boundp (car obj))
	   (or (setq top (eval (car obj))) t)
	   (classp top))
      (eval
       (append
	(list 'instance top :init)
	(apply
	 #'append
	 (mapcar
	  #'(lambda (nm)
	      (list (car nm)
		    (object-deserializer (cdr nm))))
	  (cdr obj))))))
     (t (setq top (mapcar #'object-deserializer obj))
	(cond
	 ((and (cdr (last obj)) (atom (cdr (last obj))))
	  (cons 'cons (append top (list (object-deserializer (cdr (last obj)))))))
	 ((listp top) (cons 'list top))
	 (t top))))))

(defun update-pose-from-real
  (&key (robot *real-robot*)
	(dummy? t)
	(draw? t)
	(pos nil)
	(rot nil))
  (if (and (boundp '*ci*) (find-method *ci* :state))
      (update-pose
       :robot robot
       :pos
       (or pos (send (send *ci* :state :worldcoords) :worldpos))
       :rot
       (or rot
	   (send
	    (make-coords
	     :rpy
	     (reverse
	      (map float-vector
		   #'*
		   '(1 1 0)
		   (send *ci* :state :rpy-vector))))
	    :worldrot))
       :draw? draw?
       :angle-vector (send *ci* :state :potentio-vector))
    (if dummy?
	(update-pose :robot robot
		     :draw? draw?
		     :pos (or pos (send robot :worldpos))
		     :rot (or rot (send robot :worldrot)))
      nil)))

(defun update-pose
  (&key
   (robot *robot*)
   (draw? t)
   (pos (send robot :worldpos))
   (rot (send robot :worldrot))
   (angle-vector (send robot :angle-vector))
   (hand-angle-vector (send robot :arms :hand :angle-vector))
   )
  (send robot :angle-vector angle-vector)
  (send robot :newcoords (make-coords :pos pos :rot rot))
  (mapcar #'(lambda (k v) (send robot k :hand :angle-vector v))
	  '(:rarm :larm) hand-angle-vector)
  (if (and draw? (boundp '*irtviewer*))
      (send *irtviewer* :draw-objects))
  (list (cons :pos (send robot :worldpos))
	(cons :rot (send robot :worldrot))
	(cons :angle-vector (send robot :angle-vector))
	(cons :hand-angle-vector (send robot :arms :hand :angle-vector))))

(defun update-sensor-value-from-real
  (&key
   (robot *robot*)
   (torque-max 0.7)
   (dummy? t)
   )
  (if (and (boundp '*ci*) (find-method *ci* :state))
      (let ((target-keys  '(:rarm :larm :rleg :lleg)))
	(update-sensor-value
	 :robot robot
	 :torque-max torque-max
	 :target-keys target-keys
	 :target-coords
	 (mapcar #'(lambda (k)
		     (cdr (assoc k (force-sensor-cascoords :robot robot))))
		 target-keys)
;	 (mapcar #'(lambda (k) (send robot k :end-coords)) target-keys)
	 :force-list
	 (mapcar #'(lambda (k) (send *ci* :state :filtered-absolute-force-vector k))
		 target-keys)
	 :moment-list
	 (mapcar #'(lambda (k) (send *ci* :state :filtered-absolute-moment-vector k))
		 target-keys)
	 :zmp-vector
	 (map float-vector
	      #'(lambda (a b) (or a b))
	      (list nil nil
		    (aref
		     (send robot :calc-target-centroid-pos
			   :both '(:rleg :lleg))
		     2))
	      (send (send robot :worldcoords)
		    :transform-vector
		    (send
		     (send *ci* :state :worldcoords)
		     :inverse-transform-vector
		     (scale 1e+3 (send *ci* :state :zmp-vector)))))
	  :torque-vector (send *ci* :state :torque-vector)
	 )
	)
    (if dummy? (update-sensor-value :robot robot :torque-max torque-max) nil)))

(defun update-sensor-value
  (&key
   (robot *robot*)
   (force-arrow-scale '(2 2 1 1))
   (torque-max 0.7)
;   (leg-torque-max torque-max)
;   (hand-torque-max torque-max)
   (joint-list
    (if (find-method robot :actuators)
	(send-all (send-all (send robot :actuators) :parent) :joint)
      (send robot :joint-list)))
   (arm-coords
    (mapcar #'(lambda (k) (send robot k :end-coords)) '(:rarm :larm)))
;    (mapcar
;     #'(lambda (k) (cdr (assoc k (force-sensor-cascoords :robot robot))))
;     '(:rarm :larm)))
   (toe-vector)
   (leg-key '(:rleg :lleg))
   (leg-coords
    (mapcar
     #'(lambda (k)
	 (if (vectorp toe-vector)
	     (make-cascoords
	      :pos (v+ (send robot k :end-coords :worldpos)
		       toe-vector)
	      :rot (copy-object
		    (send robot k :end-coords :worldrot))
	      :name (read-from-string
		     (format nil "~A-toe" k))
	      :parent
	      (if (< (aref toe-vector 0) 40)
		  (progn
		    (send robot k :toe-p :joint-torque 0)
		    (send (send robot k :end-coords :parent) :parent))
		(send robot k :end-coords :parent)))
	   (cdr (assoc k (force-sensor-cascoords :robot robot)))))
     leg-key))
   (arm-force (list #f(0 0 0) #f(0 0 0)))
   (arm-moment (list #f(0 0 0) #F(0 0 0)))
   (z-height
    (/ (aref
	(reduce #'v+ (cons #f(0 0 0) (send-all leg-coords :worldpos)))
	2)
       (length leg-coords)))
   (zmp-vector
    (calc-zmp :z-height z-height
	      :hand-force arm-force
	      :hand-moment arm-moment))
   (acc-vector #f(0 0 0))
   (limb-force-moment
    (send robot
	  :calc-ext-force-moment-for-torque-vector-from-ext-forces
	  arm-force arm-moment arm-coords
	  :foot-target-coords leg-coords
	  :acc-vector acc-vector))
   (target-coords (cadr (member :target-coords limb-force-moment)))
   (target-keys
    (mapcar #'(lambda (c)
		(or
		 (find-if #'(lambda (l)
			      (reg-match (format nil "~A" l)
					 (format nil "~A" (send c :name))))
			  '(:rarm :larm :rleg :lleg))
		 (send c :name)))
	    target-coords))
   (rest-keys (remove-if #'(lambda (k) (find k target-keys)) '(:rarm :larm :rleg :lleg)))
   (force-list (cadr (member :force-list limb-force-moment)))
   (moment-list (cadr (member :moment-list limb-force-moment)))
   (torque-vector
    (send robot :torque-vector
	  :target-coords (append target-coords
				 (mapcar #'(lambda (k) (send robot k :end-coords)) rest-keys))
	  :force-list (append force-list
			      (make-list (length rest-keys) :initial-element #F(0 0 0)))
	  :moment-list (append moment-list
			       (make-list (length rest-keys) :initial-element #F(0 0 0)))))
   (draw? t)
   (flush? t)
   torque-vector-rate
   overthre? )
  (setf (aref zmp-vector 2) z-height)
  (when draw?
					;-- draw torque
    (if (boundp '*viewer*)
	(send robot :draw-torque *viewer* :color #f(1 0 0)
	      :torque-vector torque-vector))
					;-- draw zmp
    (send zmp-vector :draw-on
;	  :flush t
	  :color #f(0 1 0) :width 10)
					;-- draw force&moment
    (if (boundp '*irtviewer*)
	(progn
	  (format *log-stream* "[force & moment log]~%")
	  (mapcar
	   #'(lambda (c f m scale)
	       (format *log-stream* "   >> in ~A, f=~A | m=~A~%" (send c :name) f m)
	       (send *irtviewer* :viewer :viewsurface :color #f(0 1 0))
	       (send *irtviewer* :viewer :draw-arrow
		     (send c :worldpos)
		     (v+ (send c :worldpos) (scale scale m)))
	       (send *irtviewer* :viewer :viewsurface :color #f(0 0 1))
	       (send *irtviewer* :viewer :draw-arrow
		     (send c :worldpos)
		     (v+ (send c :worldpos) (scale scale f))))
	   target-coords
	   force-list
	   moment-list
	   force-arrow-scale)
	  (if flush?
	      (send *irtviewer* :viewer :viewsurface :flush))
	  )))
  ;-- torque thre check
  (setq torque-vector-rate
	(map cons
	     #'(lambda (j)
		 (let ((ret (/ (* 1.0 (abs (send j :joint-torque)))
			       (send j :max-joint-torque))))
		   (if (> ret torque-max)
		       (progn
			 (setq overthre? t)
			 (format *log-stream* "[torque overthre] ~A/~A=~A(>~A) in ~A !!!!!~%"
				 (send j :joint-torque)
				 (send j :max-joint-torque)
				 ret
				 torque-max
				 (send j :name))))
		   ret))
	     joint-list))
  (if (vectorp toe-vector)
      (mapcar #'(lambda (c)
		  (send (send c :parent) :dissoc c))
	      leg-coords))
  ;-- ret
  (list (cons :target-pos (send-all target-coords :worldpos))
	(cons :target-rot (send-all target-coords :worldrot))
	(cons :target-keys target-keys)
	(cons :force-list force-list)
	(cons :moment-list moment-list)
	(cons :torque-vector torque-vector)
	(cons :torque-vector-rate torque-vector-rate)
	(cons :overthre? overthre?)
	(cons :zmp-vector zmp-vector)))

(defun calc-zmp
  (&key
   (robot *robot*)
   (hand-keys '(:rarm :larm))
   (hand-pos
    (mapcar
     #'(lambda (h) (send (send robot :force-sensor h) :worldpos))
     hand-keys))
   (centroid (send robot :centroid))
   (innertia (scale (* -1e-6 (send robot :weight)) *g-vec*))
   (hand-force (list #F(0 0 0) #F(0 0 0)))
   (hand-moment (list #f(0 0 0) #F(0 0 0)))
   (force-list (cons innertia hand-force))
   (moment-list (cons #f(0 0 0) hand-moment))
   (pos-list (cons centroid hand-pos))
   (leg-key :both)
   (z-height (send robot :calc-target-centroid-pos leg-key '(:rleg :lleg))))
  (labels ((cross-matrix
	    (v)
	    (make-matrix
	     3 3
	     (list (list 0 (* -1 (aref v 2)) (aref v 1))
		   (list (aref v 2) 0 (* -1 (aref v 0)))
		   (list (* -1 (aref v 1)) (aref v 0) 0))))
	   (diag
	    (v)
	    (let ((ret (make-matrix (length v) (length v))))
	      (dotimes (i (length v))
		(setf (aref ret i i) (aref v i)))
	      ret)))
    (let* ((moment-from-zero
	    (reduce #'v+
		    (append
		     (list #f(0 0 0) #f(0 0 0))
		     (map cons #'(lambda (x f) (v* x f))
			  pos-list
			  force-list))))
	   (fx (cross-matrix (reduce #'v+ (append
					   (list #f(0 0 0) #f(0 0 0))
					   force-list))))
	   (z-height (if (vectorp z-height)
			 z-height (float-vector 0 0 z-height))))
;      (format t "~A/~A/~A/~%"  moment-from-zero fx z-height)
      (v+ z-height
	  (transform
	   (diag #F(1 1 0))
	   (transform
	    (pseudo-inverse (m* (m* (diag #f(1 1 0)) fx) (diag #F(1 1 0))))
	    (scale -1
		   (v+
		    (transform fx z-height)
		    moment-from-zero))))))))

(defun dynamic-calc-torque
  (&key
   (robot *robot*)
   av-list
   (dt 0.1)
   (arm-key '(:rarm :larm))
   (leg-key '(:rleg :lleg))
   (arm-force (list #F(0 0 0) #F(0 0 0)))
   (arm-moment (list #F(0 0 0) #F(0 0 0)))
   (arm-force-coords
    (mapcar #'(lambda (k) (cdr (assoc k (force-sensor-cascoords))))
	    arm-key))
   (toe-vector nil) ;#F(0 0 0))
   (leg-force-coords
    (mapcar
     #'(lambda (k)
	 (if toe-vector
	     (make-cascoords
	      :pos (v+ toe-vector
		       (send robot k :end-coords :worldpos))
	      :rot (copy-object (send robot k :end-coords :worldrot))
	      :name k
	      :parent
	      (if (< (aref toe-vector 0) 40)
		  (send (send robot k :end-coords :parent) :parent)
		(send robot k :end-coords :parent)))
	   (cdr (assoc k (force-sensor-cascoords)))))
     leg-key))
   (fix-leg-key :lleg)
   (max-torque-rate 0.7)
   (average-time 3)
   (draw? t)
   (fix-leg-coords (send robot fix-leg-key :end-coords :copy-worldcoords)))
					;(make-coords)))
  (let* ((rest-key
	  (remove-if
	   #'(lambda (k) (find k (append arm-key leg-key)))
	   '(:rarm :larm :rleg :lleg)))
	 (c)
	 dc ddc
	 (rpy)
	 w dw
	 (I)
	 (av-rad-list (mapcar #'(lambda (av) (scale (/ pi 180.0) av)) av-list))
	 (jav (mapcar #'(lambda (a b) (scale (/ 1.0 dt) (v- a b)))
		      (cdr av-rad-list) av-rad-list))
	 (jaa (mapcar #'(lambda (a b) (scale (/ 1.0 dt) (v- a b)))
		      (cdr jav) jav))
	 fm
	 tv
	 tv-rate
	 error?
	 (index -1)
	 (average-torque)
	 )
    (mapcar
     #'(lambda (av)
	 (send robot :angle-vector av)
	 (send robot :fix-leg-to-coords fix-leg-coords fix-leg-key)
	 (if draw? (send *irtviewer* :draw-objects))
	 (push (scale 1e-3 (copy-seq (send robot :centroid))) c)
	 (push (coerce (car (rpy-angle (send robot :worldrot))) float-vector) rpy)
	 (push (easy-calc-inertia-matrix) I)
	 )
     av-list)
    ;; centroid
    (setq c (reverse c))
    (setq dc (mapcar #'(lambda (a b) (scale (/ 1.0 dt) (v- a b))) (cdr c) c))
    (setq ddc (mapcar #'(lambda (a b) (scale (/ 1.0 dt) (v- a b))) (cdr dc) dc))
    ;; rpy w
    (setq I (reverse I))
    (setq rpy (reverse rpy))
    (setq w
	  (mapcar
	   #'(lambda (next prev)
	       (map float-vector
		    #'(lambda (val)
			(/
			 (car
			  (sort
			   (list val (- val (* 2 pi)) (+ val (* 2 pi)))
			   #'(lambda (a b) (< (abs a) (abs b)))))
			 dt))
		    (v- next prev)))
	   (cdr rpy) rpy))
    (setq dw (mapcar #'(lambda (a b) (scale (/ 1.0 dt) (v- a b))) (cdr w) w))
    ;;
    (mapcar
     #'(lambda (ddc dw c I jaa jav av)
	 (send robot :angle-vector av)
	 (send robot :fix-leg-to-coords fix-leg-coords fix-leg-key)
	 (if draw? (send *irtviewer* :draw-objects))
	 (setq
	  fm
	  (easy-calc-dynamic-force
	   :ddc ddc
	   :dw dw
	   :c c
	   :I I
	   :force-list arm-force
	   :moment-list arm-moment
	   :fix-force-coords arm-force-coords
	   :float-force-coords leg-force-coords))
	 (push
	  (send robot
		:torque-vector
		:force-list
		(append
		 (cdr (assoc :force-list fm))
		 (mapcar #'(lambda (rk) #F(0 0 0)) rest-key))
		:moment-list
		(append
		 (cdr (assoc :moment-list fm))
		 (mapcar #'(lambda (rk) #F(0 0 0)) rest-key))
		:target-coords
		(append
		 (cdr (assoc :target-coords fm))
		 (mapcar #'(lambda (rk) (send robot rk :end-coords))
			 rest-key))
		:jvv jav
		:jav jaa)
	  tv))
     ddc dw c I jaa jav av-list)
    (if toe-vector
	(mapcar #'(lambda (c) (send (send c :parent) :dissoc c)) leg-force-coords))
    (setq tv (reverse tv))
    (setq tv-rate
	  (mapcar
	   #'(lambda (tv)
	       (map float-vector
		    #'/
		    tv
		    (send-all
		     (if (find-method robot :actuators)
			 (send-all
			  (send-all
			   (send robot :actuators)
			   :parent)
			  :joint)
		       (send robot :joint-list))
		     :max-joint-torque)))
	   tv))
;; 3 sec average
    (setq
     average-torque
     (mapcar
      #'(lambda (hoge)
	  (map cons
	       #'(lambda (tor name)
		   (if (> tor max-torque-rate)
		       (format t "[torque emergency] at ~A, rate = ~A~%"
			       name tor))
		   tor)
	       (scale (/ 1.0 (/ average-time dt))
		      (reduce #'v+
			      (subseq tv-rate
				      (incf index)
				      (round (+ index (/ average-time dt))))))
	       (send-all (send-all
			  (if (find-method robot :actuators)
			      (send-all
			       (send robot :actuators)
			       :parent)
			    :joint)
			  (send robot :joint-list))
			 :name)))
      (make-list (- (length tv-rate) (round (/ average-time dt))))))
    (list
     (cons :dt dt)
     (cons :torque-vectors tv)
     (cons :torque-vectors-rate tv-rate)
     (cons :average-torque-vectors average-torque)
     (cons :ddc ddc)
     (cons :dw dw)
     (cons :jvv jav)
     (cons :jav jaa))
    ))

(defun easy-calc-dynamic-force
  (&key
   (robot *robot*)
   (I (easy-calc-inertia-matrix))
   (c (scale 1e-3 (send robot :centroid)))
   (m (* 1e-3 (send robot :weight)))
   (dw #F(0 0 0))
   (ddc #F(0 0 0))
   (force-list
    (list #F(0 0 0) #F(0 0 0)))
   (moment-list
    (list #F(0 0 0) #F(0 0 0)))
   (fix-force-coords
    (mapcar
     #'(lambda (k) (cdr (assoc k (force-sensor-cascoords))))
     '(:rarm :larm)))
   (float-force-coords
    (mapcar
     #'(lambda (k) (cdr (assoc k (force-sensor-cascoords))))
     '(:rleg :lleg))))
;  (setq c #F(0 0 0))
;  (setq ddc #F(0 0 0))
;  (setq dw #F(0 0 0))
  (let* ((fix-G
	  (send robot :calc-grasp-matrix
		(mapcar #'(lambda (cd) (v- (send cd :worldpos) (scale 1e+3 c)))
			fix-force-coords)))
	 (float-G
	  (send robot :calc-grasp-matrix
		(mapcar #'(lambda (cd) (v- (send cd :worldpos) (scale 1e+3 c)))
			float-force-coords)))
	 (centroid-f
	  (concatenate float-vector
		       (scale m (v- ddc #F(0 0 -9.8)))
		       (scale 1 (transform I dw))
;		       (v* c (scale m (v- ddc #F(0 0 -9.8))))
		       ))
	 (fix-force
	  (apply
	   #'concatenate
	   (cons float-vector
		 (flatten
		  (mapcar #'(lambda (f m) (list f m))
			  force-list
			  moment-list)))))
	 (float-force
	  (transform
	   (pseudo-inverse float-G)
	   (v- centroid-f (transform fix-G fix-force)))))
    (list (cons :force-list
		(append force-list
			(let ((index -6))
			  (mapcar
			   #'(lambda (hoge)
			       (setq index (+ index 6))
			       (subseq float-force index (+ index 3)))
			   float-force-coords))))
	  (cons :moment-list
		(append moment-list
			(let ((index -3))
			  (mapcar
			   #'(lambda (hoge)
			       (setq index (+ index 6))
			       (subseq float-force index (+ index 3)))
			   float-force-coords))))
	  (cons :target-coords
		(append fix-force-coords float-force-coords)))))

(defun easy-calc-inertia-matrix
  (&key (robot *robot*))
  (let ((c (send robot :centroid))
	(link-list
	 (flatten
	  (append
	   (mapcar
	    #'(lambda (k)
		(send robot :link-list (send robot k :end-coords :parent)))
	    '(:head :rleg :lleg))
	   (mapcar
	    #'(lambda (k)
		(send robot k :link-list (send robot k :end-coords :parent)))
	    '(:rarm :larm)))))
	buf)
    (labels
	((cross-matrix
	  (v)
	  (let ((x (aref v 0))
		(y (aref v 1))
		(z (aref v 2)))
	    (make-matrix
	     3 3
	     (list
	      (list 0 (* -1 z) y)
	      (list z 0 (* -1 x))
	      (list (* -1 y) x 0))))))
      (reduce
       #'m+
       (mapcar
	#'(lambda (l)
	    (setq buf (cross-matrix
		       (scale
			(* 1e-6 (send l :weight))
			(v- (send l :worldpos) c))))
	    (m* buf buf))
	link-list)))))

(defun easy-calc-existable-ext-force ;; given force dir ,only torque limit
  (&key
   (robot *robot*)
   (force-dir #f(1 0 0 0 0 0))
   (force-zero #F(0 0 0 0 0 0))
   (limb-key :rarm)
   (move-target (send robot limb-key :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (joint-list (send-all link-list :joint))
   (torque-rate 0.5)
   (torque-max
    (map float-vector
	 #'(lambda (a) (* torque-rate a))
	 (send-all joint-list :max-joint-torque)))
   )
  (labels
      ((filter (list ok-func)
	       (cond
		((null list) nil)
		((funcall ok-func (car list))
		 (cons (car list)
		       (filter (cdr list) ok-func)))
		(t (filter (cdr list) ok-func)))))
    (let* ((J (send robot limb-key
		    :calc-jacobian-from-link-list
		    link-list
		    :move-target move-target
		    :transform-coords (make-coords)
		    :translation-axis '(t)
		    :rotation-axis '(t)))
	   (Jt (transpose J))
	   (dyn-1
	    (progn
	      (send robot :torque-vector
		    :target-coords (list move-target)
		    :force-list (list #f(0 0 0))
		    :moment-list (list #f(0 0 0)))
	      (coerce (send-all joint-list :joint-torque) float-vector)))
	   (Jta (transform Jt force-dir))
	   (Jtb (transform Jt force-zero))
	   (large (map cons
		       #'(lambda
			   (small large j)
			   (cond
			    ((minusp j) (/ small j))
			    ((plusp j) (/ large j))
			    (t *inf*)))
		       (v- (v- dyn-1 Jtb) torque-max)
		       (v+ (v- dyn-1 Jtb) torque-max)
		       Jta))
	   (small (map cons
		       #'(lambda
			   (small large j)
			   (cond
			    ((minusp j) (/ large j))
			    ((plusp j) (/ small j))
			    (t (* -1 *inf*))))
		       (v- (v- dyn-1 Jtb) torque-max)
		       (v+ (v- dyn-1 Jtb) torque-max)
		       Jta))
	   (max (apply #'min large))
	   (min (apply #'max small)))
      (list
       (cons :max (v+ (scale max force-dir) force-zero))
       (cons :min (v+ (scale min force-dir) force-zero))))))

(defun random-existable-forces
  (&key
   (robot *robot*)
   (cnt 100)
   (limb :rarm))
  (let ((fave) (mave))
    (dotimes (i cnt)
      (let* ((force-dir (concatenate float-vector (random-vector) #f(0 0 0)))
	     (moment-dir (concatenate float-vector #f(0 0 0) (random-vector)))
	     (moment
	      (easy-calc-existable-ext-force
	       :force-dir moment-dir
	       :move-target
	       (make-cascoords
		:pos (send robot limb :end-coords :worldpos)
		:rot (send robot limb :end-coords :worldrot)
		:parent (send robot limb :end-coords :parent)
		:limb-key limb)))
	     (force
	      (easy-calc-existable-ext-force
	       :force-dir force-dir
	       :move-target
	       (make-cascoords
		:pos (send robot limb :end-coords :worldpos)
		:rot (send robot limb :end-coords :worldrot)
		:parent (send robot limb :end-coords :parent)
		:limb-key limb))))
	(mapcar
	 #'(lambda (k)
	     (push (subseq (cdr (assoc k force)) 0 3) fave)
	     (push (subseq (cdr (assoc k moment)) 3 6) mave)
#|
	     (send robot  :torque-vector
		   :target-coords (list (send robot limb :end-coords))
		   :force-list (list (car fave))
		   :moment-list (list #f(0 0 0)))
	     (format t " [~A]~%  force = ~A~%  torque = ~A/~A~%"
		     k (cdr (assoc k force))
		     (mapcar #'/
			     (send robot limb :joint-list :joint-torque)
			     (send robot limb :joint-list :max-joint-torque))
		     (apply #'max
			    (mapcar #'(lambda (a b) (abs (/ a b)))
				    (send robot limb :joint-list :joint-torque)
				    (send robot limb :joint-list :max-joint-torque))))
|#
	     )
	 '(:min :max))
	(send *irtviewer* :viewer :viewsurface :color #f(1 0 0))
	(send *irtviewer* :viewer :draw-arrow
	      (send robot limb :end-coords :worldpos)
	      (v+ (send robot limb :end-coords :worldpos)
		  (subseq (cdr (assoc :max force)) 0 3)))
	(send *irtviewer* :viewer :viewsurface :color #f(0 0 1))
	(send *irtviewer* :viewer :draw-arrow
	      (send robot limb :end-coords :worldpos)
	      (v+ (send robot limb :end-coords :worldpos)
		  (subseq (cdr (assoc :min force)) 0 3)))
	(send *irtviewer* :viewer :viewsurface :color #f(0 1 0))
	(send *irtviewer* :viewer :draw-arrow
	      (send robot limb :end-coords :worldpos)
	      (v+ (send robot limb :end-coords :worldpos)
		  (scale 10 (subseq (cdr (assoc :max moment)) 3 6))))
	(send *irtviewer* :viewer :draw-arrow
	      (send robot limb :end-coords :worldpos)
	      (v+ (send robot limb :end-coords :worldpos)
		  (scale 10 (subseq (cdr (assoc :min moment)) 3 6))))
	(send *irtviewer* :viewer :viewsurface :flush)
	))
    (list
     (cons :force fave)
     (cons :moment mave)
     )
    ))

(defun existable-ext-moment ;; eq zmp
  (&key
   (robot *robot*)
   (rate 0.7))
  (labels
      ((get-minmax
	(target
	 origin
	 &optional
	 (max (copy-seq (car target)))
	 (min (copy-seq (car target)))
	 )
	(if (null target)
	    (list (cons :max (v+ origin max))
		  (cons :min (v+ origin min)))
	  (get-minmax
	   (cdr target)
	   origin
	   (map float-vector
		#'(lambda (val max ori)
		    (max (- val ori) max))
		(car target) max origin)
	   (map float-vector
		#'(lambda (val min ori)
		    (min (- val ori) min))
		(car target) min origin)))))
    (let* ((convex
	    (get-minmax
	     (flatten
	      (append
	       (send robot :lleg :foot :bottom :vertices)
	       (send robot :rleg :foot :bottom :vertices)))
	     (scale
	      0.5
	      (v+
	       (send (send robot :force-sensor :lleg)
		     :worldpos)
	       (send (send robot :force-sensor :rleg)
		     :worldpos)
	       ))))
	   (moment
	      (mapcar
	       #'(lambda (x)
		   (v* (cdr x)
		       (scale
			(* 1e-9 (send robot :weight) rate)
			*g-vec*)))
	       convex)))
      (mapcar
       #'(lambda (v)
	   (send (cdr v) :draw-on
		 :color #F(0 1 0) :width 10))
       convex)
      (if flush?
	  (send *irtviewer* :viewer :viewsurface :flush))
      (get-minmax moment #F(0 0 0)))))

(defun concatenate-matrix-diagonal
  (&rest args)
  (matrix-append args '(1 1)))

(defmethod coordinates
  (:difference-position
   (coords &key (translation-axis t))
   (let ((dif-pos
	  (send self :inverse-transform-vector (send coords :worldpos))))
     (case
      translation-axis
      ((:x :xx) (setf (elt dif-pos 0) 0))
      ((:y :yy) (setf (elt dif-pos 1) 0))
      ((:z :zz) (setf (elt dif-pos 2) 0))
      ((:xy :yx) (setf (elt dif-pos 0) 0) (setf (elt dif-pos 1) 0))
      ((:yz :zy) (setf (elt dif-pos 1) 0) (setf (elt dif-pos 2) 0))
      ((:zx :xz) (setf (elt dif-pos 2) 0) (setf (elt dif-pos 0) 0)))
     (if (vectorp translation-axis)
	 (let ((ra
		(transform
		 (inverse-matrix (send self :worldrot))
		 translation-axis)))
	   (scale (v. dif-pos ra) ra))
       dif-pos)))
  (:difference-rotation
   (coords &key (rotation-axis t))
   (labels
    ((need-mirror-for-nearest-axis
      (coords0 coords1 axis)
      (let* ((a0 (send coords0 :axis axis))
             (a1 (send coords1 :axis axis))
             (a1m (v- a1))
             (dr1 (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))
             (dr1m (scale (acos (v. a0 a1m)) (normalize-vector (v* a0 a1m)))))
        (< (norm dr1) (norm dr1m)))))
    (let (dif-rotmatrix dif-rot a0 a1)
      (case
       rotation-axis
       ((:x :y :z)
        (setq a0 (send self :axis rotation-axis)
              a1 (send coords :axis rotation-axis))
        (setq dif-rot 
              (transform (transpose (send self :worldrot))
                         (scale (acos (v. a0 a1)) (normalize-vector (v* a0 a1))))))
       ((:xx :yy :zz)
        (let ((axis (case rotation-axis (:xx :x) (:yy :y) (:zz :z))) a0 a2)
          (setq a0 (send self :axis axis))
          (setq a2 (send coords :axis axis))
          (unless (need-mirror-for-nearest-axis self coords axis) (setq a2 (v- a2)))
          (setq dif-rot (transform (transpose (send self :worldrot))
                                   (scale (acos (v. a0 a2)) (normalize-vector (v* a0 a2)))))))
       ((:xm :ym :zm)
        (let ((rot (send coords :worldrot)))
          (unless (need-mirror-for-nearest-axis self coords (case rotation-axis (:xm :y) (:ym :z) (:zm :x)))
            (setq rot (rotate-matrix rot pi (case rotation-axis (:xm :x) (:ym :y) (:zm :z)))))
          (setq dif-rotmatrix (m* (transpose (send self :worldrot)) rot))
          (setq dif-rot (user::matrix-log dif-rotmatrix))
          ))
       (nil
        (setq dif-rot (float-vector 0 0 0)))
       (t
        (setq dif-rotmatrix (m* (transpose (send self :worldrot)) (send coords :worldrot)))
        (setq dif-rot (user::matrix-log dif-rotmatrix))
        ))
      dif-rot)))
  )

(defmethod face
  (:center-coordinates
   ()
   (let* ((z normal)
          (x (if (< (abs (- 1 (abs (v. z #f(1 0 0))))) 0.0001)
                 #f(0 1 0)
                 #f(1 0 0)))
          (y (normalize-vector (v* z x)))
          )
     (setq x (v* y z))
     (instance coordinates :init
               :rot (transpose (matrix x y z))
               :pos (cadr (send self :centroid))))))

(defmethod joint
  (:worldcoords (&rest args) (send (send self :child-link) :copy-worldcoords))
  (:worldpos (&rest args) (send (send self :worldcoords) :worldpos))
  (:worldrot (&rest args) (send (send self :worldcoords) :worldrot)))

(defmethod glviewsurface
  (:bg-color (color)
	     (send *irtviewer* :change-background color)))

;; ;;(defmethod hrp2
;; (defmethod cascaded-link ;; tempolarily
;;   (:calc-ext-force-moment-for-torque-vector-from-ext-forces
;;    (ext-forces
;;     ext-moments
;;     arm-target-coords
;;     &key
;;     (centroid (scale 1e-3 (send self :centroid)))
;;     (foot-target-coords (send self :legs :end-coords))
;;     (acc-vector #f(0 0 0)))
;;    (let* ((weight-force (scale (* 1e-3 (send self :weight))
;; 			       (v- #f(0 0 9.8) acc-vector)))
;;           (retfm
;;            (transform
;;             (pseudo-inverse
;; 	     (send self :calc-grasp-matrix (send-all
;; 					    foot-target-coords :worldpos)))
;;             (v-
;; 	     (concatenate
;; 	      float-vector
;; 	      weight-force
;; 	      (v* centroid;(send self :centroid))
;; 		  weight-force))
;;              (transform (send self :calc-grasp-matrix
;; 			      (send-all
;; 			       arm-target-coords :worldpos))
;; 			(or
;; 			 (apply #'concatenate float-vector
;; 				(flatten
;; 				 (mapcar
;; 				  #'(lambda (f m) (list f m))
;; 				  ext-forces
;; 				  ext-moments))
;; 				)
;; 			 (instantiate float-vector
;; 				      (* 6 (length arm-target-coords)))
;; 			 )
;; 			))))
;;           (retf) (retm))
;;      (dotimes (i (length foot-target-coords))
;;        (push (subseq retfm (* i 6) (+ (* i 6) 3)) retf))
;;      (dotimes (i (length foot-target-coords))
;;        (push (subseq retfm (+ (* i 6) 3) (+ (* i 6) 6)) retm))
;;      (list :target-coords (append arm-target-coords foot-target-coords)
;;            :force-list (append ext-forces (reverse retf))
;;            :moment-list (append ext-moments (reverse retm)))
;;      )
;;    )
;;   )


;; (defmethod rotational-joint
;;   (:joint-angle
;;    (&optional v &key relative &allow-other-keys)
;;    "(self class &optional v &key relative &allow-other-keys) ;; v and return value are joint-angle scalar ;; scalar is rotational value[deg]"
;;    (let ()
;;      (when v
;;        (if relative (setq v (+ v joint-angle)))
;;        (cond ((> v max-angle)
;;               ;(unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate max-angle(~A)~%" self v max-angle))
;;               (setq v max-angle)))
;;        (cond ((< v min-angle)
;;               ;(unless relative (warning-message 3 ";; ~A :joint-angle(~A) violate min-angle(~A)~%" self v min-angle))
;;               (setq v min-angle)))
;;        (setq joint-angle v)
;;        (send child-link :replace-coords default-coords)
;;        (send child-link :rotate (deg2rad joint-angle) axis))
;;      joint-angle)))

;; (defun bar-coords
;;   (body &key (ignore nil))
;;   (let ( (ret (make-coords)) )
;;     (send ret :replace-coords
;; 	  (matrix-from-normal
;; 	   (normalize-vector
;; 	    (let ( (tan (min-power-vector-3d (send body :vertices))) )
;; 	      (if ignore (vset tan ignore 0) tan))))
;; 	  (send body :centroid))
;;     ret))

(defun pm
  (m)
  (format t "#2f~%")
  (dotimes (i (length (matrix-column m 0)))
    (format t "|")
    (dotimes (j (length (matrix-row m 0)))
      (format t "~A~0,2f " (if (minusp (aref m i j)) "-" "+") (abs (aref m i j))))
    (format t "|~%")))

(defun remove-args-values
  (args keys)
  (labels ((itter
	    (rest)
	    (cond
	     ((null rest) nil)
	     ((not (keywordp (car rest)))
	      (warning-message 1 "[remove-args-values] invalid non-keyword arg detected~%") nil)
	     (t (append
		 (if (find (car rest) keys)
		     nil
		   (list (car rest) (cadr rest)))
		 (itter (cddr rest)))))))
	  (itter args)))

