(defvar *robot-type* :hrp2jsknt-collada)

(require "ik-with-torque.lisp")
(require (if (ros::rospack-find "eus_gui") "package://eus_gui/graph-sample.lisp" "../util/graph-sample.lisp"))
(require "rotational-6dof.lisp")

(defvar *robot-hand* (send *robot* :links))

(defun my-collision-check-list
  nil
  (let* ((ll (send *robot* :links) l1)
	 ret)
    (while ll
      (setq l1 (car ll))
      (dolist (l2 (cdr ll))
	(if (or (find l1 (send *robot* :link-list l2 (send *robot* :link "BODY")))
		(find l2 (send *robot* :link-list l1 (send *robot* :link "BODY")))
		(find l1 (send l2 :child-links))
		(find l2 (send l1 :child-links))
		)
	    nil
	  (push (list l1 l2) ret)))
      (setq ll (cdr ll)))
    ret))

(defun my-collision-check
  nil
  (mapcar
   #'(lambda (ll)
       (if (plusp (apply 'pqp-collision-check ll))
	   ll nil))
   (my-collision-check-list)))

(defun random-contact-pose
  (&key
   (key-list '(:rleg :lleg :rarm :larm))
   &allow-other-keys
   )
  ;;
  (mapcar
   '(lambda (j) (send j :joint-angle
		      (+ (* (random 1.0)
			    (- (send j :max-angle) (send j :min-angle)))
			 (send j :min-angle))))
   (send *robot* :joint-list))
  (send *robot* :newcoords
	(make-coords :rpy (scale (deg2rad 30) (random-vector))))
  (send-all (send *robot* :links) :worldcoords)
  (cond
   ((not (and (boundp '*viewer*) *viewer*))
    (objects (flatten (list *robot-hand*)))
    (send *irtviewer* :change-background (float-vector 1 1 1))))
  (send *viewer* :draw-objects)
  (if (not
       ;;(send *robot* :self-collision-check))
       (flatten (my-collision-check)))
      (copy-seq (send *robot* :angle-vector)))
  )

(defun random-torso-move
  (&key
   (key-list '(:rleg :lleg :rarm :larm))
   (thre (make-list (length key-list) :initial-element 5))
   (rotation-axis
    (subseq
     ;; (list :z :z :x :x)
     (list t t t t)
     0 (length key-list)))
   (torque-ik-args
    (list :key-list key-list
	  :wrench-key-list key-list
	  :init nil
	  :null-max 0.3
	  :gtol 0.001
	  :stop 3
	  :linear-brli nil
	  :debug-view nil
	  :contact-wrench-optimize? t
	  :rotation-axis rotation-axis))
   rest-torque-ik-args
   (move-max-max 1000)
   (move-step 100)
   (move-max move-max-max) ;;(random move-max-max))
   (move-dir (random-vector))
   (target-coords
    (mapcar
     '(lambda (k) (send (send *robot* k :end-coords) :copy-worldcoords))
     key-list))
   )
  (while
      (send *robot* :fullbody-inverse-kinematics
	    (cons
	     (send (send (send *robot* :torso :end-coords) :copy-worldcoords)
		   :translate (scale move-step move-dir))
	     target-coords)
	    :move-target
	    (mapcar
	     '(lambda (k) (send *robot* k :end-coords))
	     (cons :torso key-list))
	    :link-list
	    (mapcar
	     '(lambda (k)
		(send *robot* :link-list (send (send *robot* k :end-coords) :parent)))
	     (cons :torso key-list))
	    :rotational-axis (cons nil rotation-axis)
	    :stop 30 :warnp nil
	    :thre (cons (/ move-step 2) thre)
	    :target-centroid-pos nil)
    (if (flatten (my-collision-check)) (return-from nil nil))
    (setq move-max (- move-max move-step))
    (if (minusp move-max) (return-from nil nil))
    (print 'nop)
    )
  (send *viewer* :draw-objects)
  (progn
    (apply 'simple-calc-torque-gradient
	   (append rest-torque-ik-args torque-ik-args))
    (or (send *now-rsd* :full-constrainted)
	(send *now-rsd* :buf :contact-wrench-optimize-skip)))
  )

(defun gen-solvable-random-contact-pose
  (&rest
   args
   &key
   (stop 10)
   (solvable? :both)
   (ret nil) buf flag
   &allow-other-keys)
  (apply 'random-contact-pose args)
  (dotimes (i stop)
    (setq buf (random-torso-move))
    (cond
     ((eq solvable? :both)
      (push *now-rsd* ret)
      (if buf (setq solvable? nil) (setq solvable? t))
      )
     ((eq solvable? buf)
      (push *now-rsd* ret)
      (return-from nil (setq flag t)))
     (t 'nop))
    )
  (if flag ret)
  )

(defun statistics-random-contact-pose
  (&rest
   args
   &key
   (loop-max 100)
   (key-list '(:rarm :larm :rleg :lleg))
   inital-av
   (ok-cnt 0) (ik-ng-cnt 0) (qp-ng-cnt 0)
   torque-ik-args rest-torque-ik-args
   &allow-other-keys)
  (setq torque-ik-args
	(list :key-list key-list
	      :wrench-key-list key-list
	      :init nil
	      :null-max 0.3
	      :gtol 0.001
	      :stop 100
	      :linear-brli nil
	      :debug-view nil
	      ;; :rotation-axis (list :z :z :x :x)
	      :contact-wrench-optimize? t
	      :robot *robot*
	      ))
  (dotimes (i loop-max)
    (cond
     ((null
       (setq inital-av
	     (print (apply #'random-contact-pose :key-list key-list
			   args))))
      (incf ik-ng-cnt))
     ((progn
	(apply 'simple-calc-torque-gradient
	       (append rest-torque-ik-args torque-ik-args))
	(not
	 (or (send *now-rsd* :full-constrainted)
	     (send *now-rsd* :buf :contact-wrench-optimize-skip))))
      (incf qp-ng-cnt))
     (t (incf ok-cnt))))
  (format t "~A + ~A + ~A vs ~A~%" ok-cnt ik-ng-cnt qp-ng-cnt loop-max))

(defun now-contact-state
  (&key (limb-keys '(:rarm :larm :rleg :lleg))
	(contact-coords
	 (mapcar
	  (function
	   (lambda (k)
	     (if
		 (find k '(:rarm :larm))
		 (send *robot* k :end-coords)
	       (let
		   ((c (send *robot* k :end-coords)))
		 (cascoords-collection
		  :limb-key
		  k
		  :coords
		  (send (send c :worldcoords) :copy-worldcoords)
		  :parent-link
		  (if (send (send c :parent) :parent-link)
		      (send (send c :parent) :parent-link)
		    (car (flatten (send (send c :parent) :child-links))))
		  )))))
	  limb-keys))
	(contact-n
	 (mapcar
	  (function
	   (lambda (k)
	     (case
		 k
	       (:rarm #f(-1.0 0.0 0.0))
	       (:larm #f(-1.0 0.0 0.0))
	       (t #f(0.0 0.0 1.0)))))
	  limb-keys))
	(ux (make-list (length limb-keys) :initial-element 0.4))
	(uy (make-list (length limb-keys) :initial-element 0.4))
	(uz (make-list (length limb-keys) :initial-element 0.4))
	(lx (make-list (length limb-keys) :initial-element 0.06))
	(ly (make-list (length limb-keys) :initial-element 0.06))
	(gain
	 (mapcar
	  (function
	   (lambda (k)
	     (if (find k '(:rarm :larm)) '(1 1.4 10) '(1 10 10))))
	  limb-keys))
	(force0
	 (mapcar
	  (function
	   (lambda (k)
	     (scale 0 #F(1 1 1 1 1 1))
	     ;;(if (find k '(:rarm :larm))
	     ;;(scale 0 #f(80.0 80.0 80.0 20.0 20.0 20.0))
	     ;;#f(0.0 0.0 0.0 0.0 0.0 0.0))
	     ))
	  limb-keys))
	(target-coords
	 (send-all contact-coords :copy-worldcoords)))
  (mapcar
   (function
    (lambda (nm cc cn ux uy uz lx ly f0 tc)
      (instance
       simple-contact-state
               :init
               :name
               nm
               :contact-coords
               cc
               :contact-n
               cn
               :ux
               ux
               :uy
               uy
               :uz
               uz
               :lx
               lx
               :ly
               ly
               :force0
               f0
               :target-coords
               tc)))
      limb-keys
      contact-coords
      contact-n
      ux
      uy
      uz
      lx
      ly
      force0
      target-coords))

(defun gen-database-random-contact-pose
  nil
  (setq both nil
	true nil
	false nil)
  (let* ((len 100) buf)
    (while (< (length both) len)
      (warning-message 2 "[both] ~A/~A~%" (length both) len)
      (setq buf (gen-solvable-random-contact-pose :solvable? :both))
      (if buf (push buf both)))
    (while (< (length true) len)
      (warning-message 2 "[true] ~A/~A~%" (length both) len)
      (setq buf (gen-solvable-random-contact-pose :solvable? t))
      (if buf (push buf true)))
    (while (< (length false) len)
      (warning-message 2 "[false] ~A/~A~%" (length both) len)
      (setq buf (gen-solvable-random-contact-pose :solvable? nil))
      (if buf (push buf false)))
    (list both true false))
  (send-all (flatten (list both true false)) :clear)
  (dump-loadable-structure "random_contact_pose.both.rsd" both)
  (dump-loadable-structure "random_contact_pose.true.rsd" true)
  (dump-loadable-structure "random_contact_pose.false.rsd" false)
  )

(defun test-random-contact-pose
  (&rest
   args
   &key
   (key-list '(:rleg :lleg :rarm :larm))
   (rotation-axis
    (subseq
     ;; (list :z :z :x :x)
     (list t t t t)
     0 (length key-list)))
   (debug-view :no-message)
   (callback nil)
   (gain1) (gain2)
   (rest-torque-ik-args)
   (stop 100) (null-max 0.3)
   torque-ik-args
   ;;
   (test-mode :optimality) ;; :solvability
   (dataset (progn (if (not (and (boundp 'both) both))
		       (load "random_contact_pose.both.rsd"))
		   both))
   (data-id (random (length dataset)))
   (data-buf (nth (min (length dataset) data-id) dataset))
   data
   &allow-other-keys
   )
  (let* (init-rsd torque brlv)
    (setq *ik-convergence-user-check* 0.0)
    (setq torque-ik-args
	  (list :key-list key-list
		:wrench-key-list key-list
		:init nil
		:null-max null-max
		:gtol 0.001
		:stop stop
		:linear-brli nil
		:debug-view debug-view
		;;:root-link-virtual-joint-weight
		;;(float-vector 0.1 0.1 0.1 0.01 0.01 0.01)
		:rotation-axis rotation-axis))
    ;;
    (cond
     ((eq test-mode :optimality)
      (setq data (find-if '(lambda (rsd)
			     (or (send rsd :full-constrainted)
				 (send rsd :buf :contact-wrench-optimize-skip)))
			  data-buf)))
     ((eq test-mode :solvability)
      (setq data (find-if '(lambda (rsd)
			     (not
			      (or (send rsd :full-constrainted)
				  (send rsd :buf :contact-wrench-optimize-skip))))
			  data-buf))))
    (if (not data) (throw :no-valid-pose-exception nil))
    ;; (send data :draw :friction-cone? nil)
    (setq *now-rsd* data)
    ;;
    (setq init-rsd *now-rsd*)
    (setq torque-ik-args
	  (append torque-ik-args
		  (list :now-rsd init-rsd
			:best-rsd init-rsd)))
    (send init-rsd :draw :rest nil :friction-cone? nil)
    (send *viewer* :draw-objects)
    (setq *rsd-queue* nil)
    (setq
     torque
     (apply
      'test-torque-ik
      (append
       rest-torque-ik-args
       torque-ik-args
       (list :mode :normal :gain gain1))))
    (if (and (eq test-mode :optimality) (not torque))
	(return-from test-random-contact-pose nil))
    (send *viewer* :draw-objects)
    (if torque (send torque :buf :gain *simple-calc-torque-gradient-gain*))
    ;; (setq init (car (last *rsd-queue*)))
    (send init-rsd :draw :rest nil :friction-cone? nil)
    (send *viewer* :draw-objects)
    (setq *rsd-queue* nil)
    (setq
     brlv
     (apply
      'test-torque-ik
      (append
       rest-torque-ik-args
       torque-ik-args
       (list
	:mode :force-approximation
	:root-link-virtual-joint-weight
	(float-vector 0.1 0.1 0.1 0.01 0.01 0.01)
	:additional-weight-list
	(mapcar
	 #'(lambda (l) (list l 0.1))
	 (remove-if #'(lambda (j) (null (send j :joint)))
		    (send *robot* :torso :links)))
	:gain gain2
	))))
    (if (and (eq test-mode :optimality) (not brlv))
	(return-from test-random-contact-pose nil))
    (send *viewer* :draw-objects)
    (if brlv (send brlv :buf :gain *simple-calc-torque-gradient-gain*))
    (format t
	    (concatenate
	     string
	     " [test-random-contact-pose] time ~A/~A = ~A~%"
	     "                            eval ~A/~A = ~A~%")
	    (if brlv (send brlv :buf :time))
	    (if torque (send torque :buf :time))
	    (if (and brlv torque)
		(/ (send brlv :buf :time)
		   (send torque :buf :time)))
	    ;; (/ (norm (send brlv :torque-vector))
	    ;; 	(norm (send init :torque-vector)))
	    ;; (/ (norm (send torque :torque-vector))
	    ;; 	(norm (send init :torque-vector)))
	    ;; (/ (norm (send brlv :torque-vector))
	    ;; 	(norm (send torque :torque-vector)))
	    (if brlv
		(/ (norm (send brlv :buf :tau))
		   (norm (send init-rsd :buf :tau))))
	    (if torque
		(/ (norm (send torque :buf :tau))
		   (norm (send init-rsd :buf :tau))))
	    (if (and brlv torque)
		(/ (norm (send brlv :buf :tau))
		   (norm (send torque :buf :tau))))
	    )
    (if (functionp callback) (funcall callback torque brlv))
    (send-all (flatten (list init-rsd torque brlv)) :clear)
    (list init-rsd torque brlv)))

(defun loop-test-random-contact-pose
  (&rest
   args
   &key
   (key-list '(:rleg :lleg :rarm :larm))
   (loop-max 100)
   (debug-view :no-message)
   (fail-cnt 0)
   ret buf
   (surfix "")
   (graph (create-graph
	   (format nil "~A~A" surfix key-list)
	   :name-list (list "Torque gradient"
			    "Torque-Force gradient")
	   :range (list #F(0 0) #F(10. 1.5))
	   ))
   (data1 (car (send graph :data)))
   (data2 (cadr (send graph :data)))
   cnt
   init torq brlv
   (gain1) ;; 0.030954)
   (gain2) ;; 1.814612e+05)
   &allow-other-keys
   )
  (setq cnt loop-max)
  (do-until-key
   (if (< (decf cnt) 0) (return-from nil nil))
   (sys::gc)
   (setq
    buf
    (apply
     'test-random-contact-pose
     :key-list key-list :debug-view debug-view :gain1 gain1 :gain2 gain2
     :data-id cnt
     args))
   (cond
    ((or (not buf)
	 ;;(<
	 ;;(length (flatten (send-all (cdr buf) :full-constrainted)))
	 ;;2)
	 )
     (incf fail-cnt))
    ((or (null (nth 1 buf))
	 (null (nth 2 buf)))
     (push buf ret))
    (t (push buf ret)
       (setq init (nth 0 buf))
       (setq torq (nth 1 buf))
       (setq brlv (nth 2 buf))
       (if (not (numberp gain1)) (setq gain1 (send torq :buf :gain)))
       (if (not (numberp gain2)) (setq gain2 (send brlv :buf :gain)))
       (cond
	(graph
	 (mapcar
	  #'(lambda (buf)
	      (send buf :buf :t/tmax
		    (map float-vector
			 #'(lambda (d) (min d 1.0))
			 (send buf :buf :tau))))
	  buf)
	 (send data1
	       :add
	       (float-vector
		(send torq :buf :time)
		(/ (norm (send torq :buf :t/tmax))
		   (norm (send init :buf :t/tmax)))))
	 (send data2
	       :add
	       (float-vector
		(send brlv :buf :time)
		(/ (norm (send brlv :buf :t/tmax))
		   (norm (send init :buf :t/tmax)))))
	 (send graph :fit-draw :line-graph? nil)
	 (send graph :color #xFFFFFF)
	 (send graph :draw-line
	       (send graph :screen-pos #F(0 1))
	       (send graph :screen-pos #F(100 1)))
	 (send graph :color #xFF0000)
	 (send graph :plot-screen
	       (scale (/ 1.0 (length (send data1 :data)))
		      (reduce #'v+ (append (send data1 :data)
					   (list #F(0 0) #F(0 0)))))
	       10)
	 (send graph :color #x00FF00)
	 (send graph :plot-screen
	       (scale (/ 1.0 (length (send data2 :data)))
		      (reduce #'v+ (append (send data2 :data)
					   (list #F(0 0) #F(0 0)))))
	       10)
	 (send graph :repaint)
	 ))
       )))
  (cons fail-cnt ret))

