(defvar *robot-type* :jaxon)

(require "gradient-util.lisp")
(require "../optimize-brli.lisp")

(defvar *robot* (instance jaxon-robot :init))
(defvar *ankle-coords*
  (mapcar
   #'(lambda (k)
       (make-cascoords
	:name (read-from-string
	       (format nil "~A-ankle-end-coords" k))
	:parent (send (send *robot* k :end-coords :parent) :parent)
	:coords (send *robot* k :end-coords :copy-worldcoords)))
   '(:rleg :lleg)))

(defvar *ik-convergence-user-check* 0.0)
(defmethod cascaded-link
  (:ik-convergence-check
   (success dif-pos dif-rot
    rotation-axis translation-axis thre rthre
    centroid-thre target-centroid-pos
    centroid-offset-func cog-translation-axis
    &key (update-mass-properties t) (success-buf t))
   (dotimes (i (length dif-pos))
     (setq success-buf
	   (and success-buf
		(if (elt translation-axis i)
		    (< (norm (elt dif-pos i)) (elt thre i)) t)
		(if (elt rotation-axis i)
		    (< (norm (elt dif-rot i)) (elt rthre i)) t))))
   (if target-centroid-pos
       (setq success-buf
	     (and
	      success-buf
	      (send self :cog-convergence-check
		    centroid-thre target-centroid-pos
		    :centroid-offset-func centroid-offset-func
		    :translation-axis cog-translation-axis
		    :update-mass-properties update-mass-properties))))
   (if success-buf
       (cond
	;;((null *best-rsd*) (setq *best-rsd* *now-rsd*))
	((or
	  (and *now-rsd* (not *best-rsd*))
	  (and *best-rsd* *now-rsd*
	       (not (and (or (send *best-rsd* :full-constrainted)
			     (send *best-rsd* :buf :contact-wrench-optimize-skip))
			 (numberp (send *best-rsd* :buf :f))))
	       (and (or (send *now-rsd* :full-constrainted)
			(send *now-rsd* :buf :contact-wrench-optimize-skip))
		    (numberp (send *now-rsd* :buf :f))))
	  (and *best-rsd* *now-rsd*
	       (or (send *now-rsd* :full-constrainted)
		   (send *now-rsd* :buf :contact-wrench-optimize-skip))
	       (numberp (send *best-rsd* :buf :f))
	       (numberp (send *now-rsd* :buf :f))
	       (> (send *best-rsd* :buf :f)
		  (send *now-rsd* :buf :f))))
	 (format t "[update-best-rsd] ~A~%" (send *now-rsd* :buf :f))
	 (setq *best-rsd* *now-rsd*))))
   (and (numberp *ik-convergence-user-check*)
	(< *ik-convergence-user-check* 0.1) success success-buf)))

(defvar *prev-x*)
(defvar *prev-f*)
(defun tol-check
  (x f
     &key
     (xtol 0.004)
     (ftol 0.004)
     (gtol 0.001)
     (buf 0))
  (cond
   ((and
     ;; *prev-x* (< (abs (/ (- x *prev-x*) x)) xtol)
     ;; *prev-f* (< (abs (/ (- f *prev-f*) f)) ftol)
     *prev-x* *prev-f*
     (< (setq buf (/ (abs (- f *prev-f*))
		     (max 1e-6 (abs *prev-x*))))
	gtol)
     )
    ;;(throw :ik-stop :xtol-exceeded)
    (format t " ~A(~A)~%" :tol-exceeded buf)
    (setq *ik-convergence-user-check*
	  (* *ik-convergence-user-check* 0.8))
    )
   (t
    (setq *ik-convergence-user-check*
	  (min 1.0 (* *ik-convergence-user-check* 1.2)))
    ))
  (setq *prev-x* x)
  (setq *prev-f* f)
  buf
  )

(setq *log-stream* nil)
(defvar *best-rsd*)
(defvar *now-rsd*)
(defvar *rsd-queue*)
(defun simple-calc-contact-wrench
  (&key
   (robot *robot*)
   ret buf
   (key-list '(:rleg :lleg))
   (wrench-key-list '(:rleg :lleg))
   (contact-states (now-contact-state :limb-keys wrench-key-list))
   (rest-contact-states
    (now-contact-state :limb-keys
		       (set-difference key-list wrench-key-list)))
   (debug? nil)
   (contact-wrench-optimize? nil)
   &allow-other-keys)
  (setq
   ret
   (apply
    #'optimize-brli
    (append
     (list
      :robot robot
      :linear-brli nil
      :eiquadprog-function 'solve-eiquadprog-raw ;;-with-error
      :contact-states contact-states
      :rest-contact-states rest-contact-states
      :debug? nil)
     (if contact-wrench-optimize? nil (list :ret-buf nil)))
    ))
  (setq *now-rsd* ret)
  (if contact-wrench-optimize?
      (if (not (send *now-rsd* :full-constrainted))
	  (progn (setq *ik-convergence-user-check* 0.0)
		 (print 'not-full-constrainted-rsd)))
    (send *now-rsd* :buf :contact-wrench-optimize-skip t))
  (setq *rsd-queue*
	(subseq (cons *now-rsd* *rsd-queue*) 0 100))
  (setq buf
	(mapcar
	 #'cons
	 (send ret :contact-keys)
	 (send ret :contact-forces)))
  (setq buf
	(mapcar
	 #'(lambda (k) (cdr (assoc k buf)))
	 key-list))
  (cond
   (debug?
    (format t "   evaluated contact wrench~%")
    (pprint buf)))
  buf
  ;;(list #F(0 0 0 0 0 0) #F(0 0 0 0 0 0) #F(0 0 300 0 0 0) #F(0 0 300 0 0 0))
  )

(defvar *simple-calc-torque-gradient-gain* 1.0)
(defun simple-calc-torque-gradient
  (&rest
   args
   &key
   (robot (copy-object *robot*))
   (key-list '(:rleg :lleg))
   (wrench-key-list key-list)
   (wrench-list 'simple-calc-contact-wrench)
   (move-target
    (mapcar #'(lambda (k)
		(cond
		 ((eq k :rleg) (car *ankle-coords*))
		 ((eq k :lleg) (cadr *ankle-coords*))
		 (t (send *robot* k :end-coords))))
	    key-list))
   (link-list
    (mapcar #'(lambda (mt) (send *robot* :link-list (send mt :parent)))
	    move-target))
   (root-link (car (send *robot* :links)))
   (ul (send *robot* :calc-union-link-list link-list))
   (6dof? nil)
   (torque-gradient-root-link-virtual-joint-weight
    #F(0.001 0.001 0.001 0.001 0.001 0.001))
   (gain nil) ;;1e-3)
   (null-max 0.3)
   (cnt -1)
   (stop 50)
   (debug? nil)
   (torque-normalize? t)
   target-centroid-pos
   (dt/dx)
   (mode :normal) ;;:force-approximation) ;;:normal)
   (xtol 0.001)
   (ftol 0.001)
   (gtol 0.001)
   &allow-other-keys)
  (if (> cnt (* 0.9 stop))
      (setq *ik-convergence-user-check* 0.0))
  ;;(throw :ik-stop :loop-exceeded))
  (let* ((tm (instance mtimer :init))
	 (copy-rsd
	  (progn
	    (send robot :angle-vector
		  (copy-seq (send *robot* :angle-vector)))
	    (send robot :newcoords
		  (copy-object (send *robot* :worldcoords)))
	    (send-all (send *robot* :links) :worldcoords)
	    (send-all (send robot :links) :worldcoords)))
	 (wrench-list
	  (if (functionp wrench-list)
	      (apply wrench-list args)
	    wrench-list))
	 (gtm (instance mtimer :init))
	 (tau
	  (calc-torque-from-wrench-list-and-gradient
	   :all-link-list (remove root-link ul)
	   :move-target move-target
	   :wrench wrench-list
	   :6dof? 6dof?
	   :debug? debug?))
	 Jtau move g)
    ;; (print tau)
    (if torque-normalize?
	(setq tau
	      (map float-vector #'/ tau
		   (send-all
		    (send-all (remove root-link ul) :joint)
		    :max-joint-torque))))
    (cond
     (*now-rsd*
      (send *now-rsd* :buf :tau tau)
      (send *now-rsd* :buf :f (norm tau))
      (send *now-rsd* :buf :lname
	    (send-all (remove root-link ul) :name))))
    (cond
     ((eq mode :normal)
      (setq
       Jtau
       (calc-torque-gradient-from-wrench-list-and-gradient
	:all-link-list (remove root-link ul)
	:move-target move-target
	:wrench wrench-list
	:6dof? 6dof?
	:debug? debug?))
      (setq move (transform (transpose Jtau) tau))
      )
     ((eq mode :force-approximation)
      (setq
       Jtau
       (calc-torque-gradient-with-force-approximation
	:move-target move-target
	:all-link-list (remove root-link ul)
	:6dof? 6dof?
	:as-list t
	:debug? debug?))
      ;;(setq move (transform (transpose Jtau) tau))
      (setq move tau)
      (mapcar
       #'(lambda (m) (setq move (transform (transpose m) move)))
       Jtau)
      )
     ((eq mode :old-force-approximation)
      (map cons
	   #'(lambda (l tau)
	       (if (find-method l :joint)
		   (send (send l :joint) :joint-torque tau)))
	   (remove root-link ul) tau)
      (setq
       move
       (brlv-to-cog
	;;:robot robot
	:all-link-list (remove root-link ul)
	:link-list link-list
	:move-target move-target
	;;:force-list wrench-list
	:torque-list
	(mapcar #'(lambda (ll)
		    (send-all (send-all ll :joint) :joint-torque))
		link-list)
	:6dof? 6dof?
	:target-centroid-pos target-centroid-pos
	:debug? debug?))
      (if target-centroid-pos (setq move (scale 0 move)))
      )
     )
    (setq move
	  (child-reverse-filter
	   move
	   (send-all (remove root-link ul) :joint) :6dof? 6dof?))
    (cond
     (6dof?
      (setq move
	    (concatenate
	     float-vector
	     (subseq move (- (length move) 6) (length move))
	     (subseq move 6)))
      (dotimes (i 6)
	(setf (aref move i)
	      (* (aref torque-gradient-root-link-virtual-joint-weight i)
		 (aref move i)))))
     (t
      (setq
       move
       (concatenate
	float-vector #F(0 0 0 0 0 0) move))
      ))
    (cond
     (debug?
      (format t "[/joint-angle]~%")
      (format t "  tau=~A~%" tau)
      (format t "  move=~A~%" (scale -1 (normalize-vector move)))
      (format t "  root-link ~A~%"
	      (scale -1 (normalize-vector (subseq move 0 6))))
      (mapcar
       #'(lambda (lm)
	   (format t "  ~A(~A=~A) ~A~%"
		   (send (car lm) :name)
		   (send (send (car lm) :joint) :name)
		   (send (send (car lm) :joint) :axis-vector :world)
		   (cdr lm)))
       (sort
	(map cons #'cons ul (scale -1 (normalize-vector (subseq move 6))))
	#'(lambda (l1 l2) (> (abs (cdr l1)) (abs (cdr l2))))))))
    (cond
     ((and (not (numberp gain)) (eq cnt 0))
      (setq gain
	    (setq *simple-calc-torque-gradient-gain*
		  (/ null-max (norm move))))
      (format t "[torque-gradient] auto gain tune ~A~%" gain)
      )
     ((not (numberp gain))
      (setq gain *simple-calc-torque-gradient-gain*)))
    (setq g (* -1 (min (* 2. null-max) (* gain (norm move)))))
    (cond
     ((eq cnt 0)
      (print 'initialize)
      (setq *prev-f* nil)
      (setq *prev-x* nil)
      (setq *ik-convergence-user-check* 1.0)
      (return-from simple-calc-torque-gradient nil)
      ))
    (setq move
	  (scale g (normalize-vector move)))
    (setq dt/dx
	  (tol-check (norm move) (norm tau) :gtol gtol))
    (if (numberp *ik-convergence-user-check*)
	(setq move (scale *ik-convergence-user-check* move)))
    (format t "[~A] tau=~A dx=~A dt/dx=~A time=~A/~A~%"
	    cnt
	    (norm tau)
	    (abs g)
	    dt/dx
	    (send gtm :stop) (send tm :stop))
    ;; throw exception
    move))

(defun test-torque-ik
  (&rest
   args
   &key
   (tm (instance mtimer :init))
   (init
    (progn
      (cond
       ((not (and (boundp '*viewer*) *viewer*))
	(objects (list *robot*))))
      (send *robot* :reset-pose)
      ;;(send *robot* :angle-vector
      ;;(scale 0 (send *robot* :angle-vector)))
      (send *robot* :newcoords (make-coords))
      (send-all (send *robot* :links) :worldcoords)
      (send *viewer* :draw-objects)))
   (stop 50)
   (filter #'identity)
   (key-list '(:rleg :lleg))
   (move-target
    (mapcar #'(lambda (k)
		(cond
		 ((eq k :rleg) (car *ankle-coords*))
		 ((eq k :lleg) (cadr *ankle-coords*))
		 (t (send *robot* k :end-coords))))
	    key-list))
   (link-list
    (mapcar #'(lambda (mt) (send *robot* :link-list (send mt :parent)))
	    move-target))
   (target-coords (send-all move-target :copy-worldcoords))
   (translation-axis (mapcar #'(lambda (k) t) key-list))
   (rotation-axis translation-axis)
   (thre (mapcar #'(lambda (k) 50) key-list))
   (rthre (mapcar #'(lambda (k) (deg2rad 12)) key-list))
   (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.1 0.1))
   (avoid-weight-gain 0.01)
   target-centroid-pos
   (cnt -1)
   (debug-view :no-message)
   ;;
   (best-rsd nil)
   (now-rsd nil)
   collision-avoidance-link-pair
   (min #F(-1000 -1000 -1000 -500 -500 -500))
   (max #F(1000 1000 1000 500 500 500))
   robot
   ret
   &allow-other-keys)
  (setq args
	(append
	 (list :robot (setq robot (copy-object *robot*))
	       :move-target move-target
	       :link-list link-list
	       )
	 args))
  (setq *best-rsd* best-rsd)
  (setq *now-rsd* now-rsd)
  ;;(print args)
  (setq
   ret
   (catch :ik-stop
     (send* *robot* :fullbody-inverse-kinematics
	    (funcall filter target-coords)
	    :move-target (funcall filter move-target)
	    :link-list (funcall filter link-list)
	    ;;:union-link-list (remove rl ul)
	    :collision-avoidance-link-pair
	    collision-avoidance-link-pair
	    :null-space
	    #'(lambda nil
		(x::window-main-one)
		(incf cnt)
		(apply #'simple-calc-torque-gradient
		       (append
			(list :cnt cnt
			      :target-centroid-pos target-centroid-pos)
			args)))
	    :target-centroid-pos (print target-centroid-pos)
	    :min min :max max
	    :avoid-weight-gain avoid-weight-gain
	    :translation-axis (funcall filter translation-axis)
	    :rotation-axis (funcall filter rotation-axis)
	    :debug-view debug-view;;:no-message
	    ;;:stop (* 10 stop)
	    :stop stop
	    :thre thre :rthre rthre
	    ;; :warnp nil
	    :root-link-virtual-joint-weight
	    root-link-virtual-joint-weight
	    args
	    )))
  (cond
   (*now-rsd*
    (send *now-rsd* :buf :time (send tm :stop))
    (send *best-rsd* :buf :time (send *now-rsd* :buf :time))
    (format t "    time: ~A~%" (send *now-rsd* :buf :time))
    (or *best-rsd* (if ret *now-rsd*))))
  )
