(defvar *robot*)

(require "symbol-function.lisp")
(require "../math/matrix-util.lisp")

(defun symbol-forward-kinematics
  (&key
   (robot *robot*)
   (parent nil)
   (link (car (send robot :links)))
   (id 0)
   (q (read-from-string (format nil "q~A" id)))
   (dq (read-from-string (format nil "dq~A" id)))
   (ddq (read-from-string (format nil "ddq~A" id)))
   (fe
    (instance symbol-matrix :init :data
	      (mapcar
	       #'(lambda (id2)
		   (read-from-string (format nil "f~A~A" id2 id)))
	       '(x y z))))
   (ne
    (instance symbol-matrix :init :data
	      (mapcar
	       #'(lambda (id2)
		   (read-from-string (format nil "n~A~A" id2 id)))
	       '(x y z))))
   (x (if (send link :joint)
	  (case (send (send link :joint) :get-val 'axis)
		(:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
		(:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
		(:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
		(t (send (send link :joint) :get-val 'axis)))))
   (first? (send robot :angle-vector
		 (scale 0 (send robot :angle-vector))))
   )
  ;;(print id)
  (labels
   ((set-joint-transformation
     (link
      &key
      (joint-coords
       (send
	(send (send link :parent-link) :copy-worldcoords)
	:transform
	(copy-object
	 (send (send link :joint) :get-val 'default-coords))))
      ;; (send (send link :copy-worldcoords)
      ;; 	     :transform
      ;; 	     (send
      ;; 	      (copy-object (send (send link :joint) :get-val 'default-coords))
      ;; 	      :transformation
      ;; 	      (make-coords))))
      (parent-joint-coords
       (send (send link :parent-link) :get :Jc))
      (parent2joint
       (send (copy-object parent-joint-coords)
	     :transformation
	     (copy-object joint-coords)))
      (joint2link
       (send (copy-object joint-coords)
	     :transformation
	     (send link :copy-worldcoords)))
      )
     ;;
     (send link :put :I
	   (instance symbol-matrix :init :data
		     (scale-matrix 1e-9 (send link :inertia-tensor))))
     (send link :put :c
	   (instance symbol-matrix :init :data
		     (scale 1e-3 (send link :get-val 'acentroid))))
     (send link :put :m (* 1e-3 (send link :weight)))
     (send link :put :mg
	   (instance symbol-matrix :init :data
		     (scale (* 1e-3 (send link :weight)) #F(0 0 -9.8))))
     ;;
     (send link :put :Jc joint-coords)
     (send link :put :Rd
	   (instance symbol-matrix :init :data
		     (send joint2link :rot)))
     (send link :put :Pd
	   (instance symbol-matrix :init :data
		     (scale
		      1e-3
		      (send joint2link :pos))))
     ;;
     (send link :put :Ra
	   (instance symbol-matrix
		     :init
		     :data
		     (send parent2joint :rot)))
     (send link :put :Pa
	   (instance symbol-matrix
		     :init
		     :data
		     (scale 1e-3 (send parent2joint :pos))))
     )
    (next-call
     (&optional buf)
     (if (send link :child-links)
	 (mapcar
	  #'(lambda (child)
	      (setq
	       buf
	       (symbol-forward-kinematics
		:robot robot
		:parent link
		:link child
		:id (+ id 1)
		:first? nil))
	      (setq id (apply #'max (flatten buf)))
	      buf)
	  (send link :child-links)) id))
    )
   (cond
    ((null parent)
     (send link :put :fe fe)
     (send link :put :ne ne)
     (set-joint-transformation
      link
      :joint-coords (send link :copy-worldcoords)
      :parent-joint-coords (send link :copy-worldcoords))
     (send link :put :Pl (instance symbol-matrix :init :data '(x y z)))
     (send link :put :Pj (instance symbol-matrix :init :data '(x y z)))
     ;;(send link :put :Pd (instance symbol-matrix :init :data '(0 0 0)))
     (send link :put :Rl (symbol-rpy-matrix 'r 'p 'w))
     (send link :put :Rj (symbol-rpy-matrix 'r 'p 'w))
     ;;(send link :put :Rd (instance symbol-matrix :init :data (unit-matrix 3)))
     ;;
     (send link :put :dPl (instance symbol-matrix :init :data '(dx dy dz)))
     (send link :put :ddPl (instance symbol-matrix :init :data '(ddx ddy ddz)))
     (send link :put :dRl (instance symbol-matrix :init :data '(dr dp dw)))
     (send link :put :ddRl (instance symbol-matrix :init :data '(ddr ddp ddw)))
     ;;
     (next-call))
    (t
     (send link :put :q q)
     (send link :put :dq dq)
     (send link :put :ddq ddq)
     (send link :put :fe fe)
     (send link :put :ne ne)
     (set-joint-transformation link)
     ;;
     (send link :put :Rj
	   (send (send parent :get :Rj) :*
		 (send
		  (send link :get :Ra) :*
		  (symbol-rodrigues x q))))
     (send link :put :Pj
	   (send (send parent :get :Pj)  :+
		 (send
		  (send parent :get :Rj) :*
		  (send link :get :Pa))))
     (send link :put :Rl
	   (send (send link :get :Rj) :* (send link :get :Rd)))
     (send link :put :Pl
	   (send
	    (send link :get :Pj) :+
	    (send (send link :get :Rj) :* (send link :get :Pd))))
     ;;
     (send link :put :x
	   (send (send link :get :Rj) :* x))
     ;;
     (send link :put :/dPl
	   (send (send link :get :Pl) :cross (send link :get :x)))
     (send link :put :/dRl (send link :get :x))
     (send link :put :dPl
	   (send (send parent :get :dPl) :+
		 (send (send link :get :/dPl) :scale dq)))
     (send link :put :dRl
	   (send (send parent :get :dRl) :+
		 (send (send link :get :/dRl) :scale dq)))
     ;;
     (send link :put :/ddPl
	   (send
	    (send (send parent :get :dRl) :cross
		  (send link :get :/dPl))
	    :+
	    (send (send parent :get :dPl) :cross
		  (send link :get :/dRl))))
     (send link :put :/ddRl
	   (send (send parent :get :dRl) :cross
		 (send link :get :/dRl)))
     (send link :put :ddPl
	   (send
	    (send parent :get :ddPl) :+
	    (send
	     (send (send link :get :/ddPl) :scale dq) :+
	     (send (send link :get :/dPl) :scale ddq))))
     (send link :put :ddRl
	   (send
	    (send parent :get :ddRl) :+
	    (send
	     (send (send link :get :/ddRl) :scale dq) :+
	     (send (send link :get :/dRl) :scale ddq))))
     (format t "[~A] ~A/~A~%" (send link :name) id (length (send robot :links)))
     (mapcar
      #'(lambda (k)
     	  (if (and (class (send link :get k))
     		   (subclassp (class (send link :get k)) symbol-matrix))
     	      (send link :put k (send (send link :get k) :simplify)))
     	  )
      (mapcar #'car (send link :plist)))
     (next-call)))))

(defun symbol-inverse-dynamics
  (&key
   (robot *robot*)
   (link (car (send robot :links)))
   (child-links (send link :child-links))
   )
  (let* ((m (send link :get :m))
	 (mg (send link :get :mg))
	 (R (send link :get :Rl))
	 (Rt (send R :transpose))
	 (c (send (send R :* (send link :get :c)) :+ (send link :get :Pl)))
	 (iner (send (send R :* (send link :get :I)) :* Rt))
	 (c-hat (send c :cross-matrix))
	 (I (send iner :+
		  (send (send c-hat :* (send c-hat :transpose)) :scale m)))
	 (dp (send link :get :dPl))
	 (ddp (send link :get :ddPl))
	 (dr (send link :get :dRl))
	 (ddr (send link :get :ddRl))
	 momentum angular-momentum
	 force moment
	 )
    ;;hoge
    (setq momentum
	  (send (send dp :+ (send dr :cross c)) :scale m))
    (setq angular-momentum
	  (send (send (send c :cross dp) :scale m) :+ (send I :* dr)))
    (setq force
	  (send
	   (send (send ddp :+ (send ddr :cross c)) :scale m)
	   :+ (send dr :cross momentum)))
    (setq moment
	  (send
	   (send
	    (send (send (send c :cross ddp) :scale m)
		  :+ (send I :* ddr))
	    :+
	    (send dp :cross momentum))
	   :+
	   (send dr :cross angular-momentum)))
    (setq force (send force :- (send mg :+ (send link :get :fe))))
    (setq moment (send moment :-
		       (send (send c :cross mg) :+ (send link :get :ne))))
    (setq force (send force :simplify))
    (setq moment (send moment :simplify))
    (dolist (child child-links)
      (symbol-inverse-dynamics :robot robot :link child)
      (setq force (send force :+ (send child :get :f)))
      (setq moment (send moment :+ (send child :get :n))))
    (setq force (send force :simplify))
    (setq moment (send moment :simplify))
    (send link :put :f force)
    (send link :put :n moment)
    (send link :put :mm momentum)
    (send link :put :am angular-momentum)
    (if (and (send link :joint) (send link :parent-link))
	(send link :put :t
	      (send
	       (send (send (send (send link :get :/dPl) :transpose) :* force)
		     :+
		     (send (send (send link :get :/dRl) :transpose) :*  moment))
	       :simplify)))
    (format t "[~A] ~A > 0~%" (send link :name)
	    (let ((id 0) (l link))
	      (while (setq l (send l :parent-link)) (incf id))
	      id))
    nil
    ))

(defun calc-link-coords
  (&key
   (link (send  *robot* :rarm :end-coords :parent))
   ;; for debug vv
   (debug? nil)
   (timer (if debug? (instance mtimer :init)))
   (timebuf nil)
   (answer (if debug? (send link :copy-worldcoords)))
   (symbols
    (append '(x y z r p w dx dy dz dr dp dw ddx ddy ddz ddr ddp ddw)
	    (send-all (cdr (send *robot* :links)) :get :q)
	    (send-all (cdr (send *robot* :links)) :get :dq)
	    (send-all (cdr (send *robot* :links)) :get :ddq)))
   (values
    (concatenate
     cons
     (scale 1e-3 (send (send (car (send *robot* :links)) :worldcoords) :worldpos))
     (reverse
      (car
       (rpy-angle
     	(send (send (car (send *robot* :links)) :worldcoords) :worldrot))))
     ;;(matrix-log (send (send (car (send *robot* :links)) :worldcoords) :worldrot))
     (list 0 0 0 0 0 0)
     (list 0 0 0 0 0 0)
     (mapcar
      #'deg2rad
      (send-all (send-all (cdr (send *robot* :links)) :joint) :joint-angle))
     (make-list (length (cdr (send *robot* :links))) :initial-element 0)
     (make-list (length (cdr (send *robot* :links))) :initial-element 0)))
   (pos
    (scale
     1e+3
     (coerce
      (flatten
       (send (send (send link :get :Pl) :eval symbols values) :data))
      float-vector)))
   (rot
    (make-matrix
     3 3
     (send (send (send link :get :Rl) :eval symbols values) :data)))
   (ret (make-coords :pos pos :rot rot))
   dif
   )
  (if (and debug? timer) (setq timer (send timer :stop)))
  (if (and timebuf (> (length timebuf) 0))
      (setf (aref timebuf 0) timer))
  (cond
   (answer
    (format t
	    (concatenate
	     string
	     "[calc-link-pos]~%"
	     "   name = ~A~%"
	     "   diff = ~A~%"
	     "   joint angle = ~A~%"
	     "   Pa = ~A~%"
	     "   Pd = ~A~%"
	     "   Ra = ~A~%"
	     "   Rd = ~A~%"
	     "   ok? = ( ~A < 1e-3 ) = ~A~%"
	     "   time = ~A sec~%"
	     )
	    (if (send link :joint) (send (send link :joint) :name)
	      (send link :name))
	    ;;answer (norm (v- answer pos))
	    (setq
	     dif
	     (concatenate
	      float-vector
	      (scale 1e-3 (send answer :difference-position ret))
	      (send answer :difference-rotation ret)))
	    (if (send link :joint) (send (send link :joint) :joint-angle))
	    (flatten (send (send link :get :Pa) :data))
	    (flatten (send (send link :get :Pd) :data))
	    (reverse (car (rpy-angle (make-matrix 3 3 (send (send link :get :Ra) :data)))))
	    (reverse (car (rpy-angle (make-matrix 3 3 (send (send link :get :Rd) :data)))))
	    (norm dif)
	    (< (norm dif) 1e-3)
	    timer
	    )))
  (if (and (vectorp debug?) (> (length debug?) 0))
      (setf (aref debug? 0) (norm dif)))
  ;;(print (send link :worldcoords))
  ret)

(defun calc-joint-torque
  (&key
   (link (send  *robot* :rarm :end-coords :parent))
   (root (car (send *robot* :links)))
   (link-list (cdr (send *robot* :links)))
   ;; for debug vv
   (debug? nil)
   (timer (if debug? (instance mtimer :init)))
   (timebuf nil)
   (answer (if debug? (send link :copy-worldcoords)))
   (symbols
    (append '(x y z r p w dx dy dz dr dp dw ddx ddy ddz ddr ddp ddw)
	    (send-all (cdr (send *robot* :links)) :get :q)
	    (send-all (cdr (send *robot* :links)) :get :dq)
	    (send-all (cdr (send *robot* :links)) :get :ddq)
	    (flatten (send-all (send-all (send *robot* :links) :get :fe) :data))
	    (flatten (send-all (send-all (send *robot* :links) :get :ne) :data))
	    ))
   (values
    (concatenate
     cons
     (scale 1e-3 (send (send (car (send *robot* :links)) :worldcoords) :worldpos))
     (reverse
      (car
       (rpy-angle
     	(send (send (car (send *robot* :links)) :worldcoords) :worldrot))))
     ;;(matrix-log (send (send (car (send *robot* :links)) :worldcoords) :worldrot))
     (list 0 0 0 0 0 0)
     (list 0 0 0 0 0 0)
     (mapcar
      #'deg2rad
      (send-all (send-all (cdr (send *robot* :links)) :joint) :joint-angle))
     (make-list (length (cdr (send *robot* :links))) :initial-element 0)
     (make-list (length (cdr (send *robot* :links))) :initial-element 0)
     (make-list (* 3 (length (send *robot* :links))) :initial-element 0)
     (make-list (* 3 (length (send *robot* :links))) :initial-element 0)
     ))
   (symbol-torque
    (if (not debug?)
	(flatten
	 (mapcar
	  #'(lambda (tau)
	      (print tau)
	      (send (send tau :eval symbols values) :data))
	  (send-all link-list :get :t)))))
   (torque
    (progn (send *robot* :calc-torque-from-vel-acc)
	   (send-all (send-all link-list :joint) :joint-torque)))
   dif
   )
  (if (and debug? timer) (setq timer (send timer :stop)))
  (if (and timebuf (> (length timebuf) 0))
      (setf (aref timebuf 0) timer))
  (if debug?
      (map cons
	   #'(lambda (l tau)
	       (format (if debug? t nil)
		       (concatenate string
				    "[~A]~%"
				    "torque:   ~A vs ~A~%"
				    "momentum: ~A vs ~A~%"
				    "angularm: ~A vs ~A~%"
				    "force:    ~A vs ~A~%"
				    "moment:   ~A vs ~A~%"
				    )
		       (send l :name)
		       (caar (send (send (send l :get :t) :eval symbols values) :data))
		       tau
		       (flatten (send (send (send l :get :mm) :eval symbols values) :data))
		       (send l :get-val 'momentum)
		       (flatten (send (send (send l :get :am) :eval symbols values) :data))
		       (send l :get-val 'angular-momentum)
		       (flatten (send (send (send l :get :f) :eval symbols values) :data))
		       (send l :get-val 'force)
		       (flatten (send (send (send l :get :n) :eval symbols values) :data))
		       (send l :get-val 'moment)
		       ))
	   link-list torque))
  (if (and (vectorp debug?) (> (length debug?) 0))
      (setf (aref debug? 0) (norm dif)))
  ;;(print (send link :worldcoords))
  torque)

;;
;; (send-all (send *robot* :joint-list) :set-val 'joint-min-max-target nil)
(defun test-calc-link-coords
  (&key
   (link (send *robot* :rarm :end-coords :parent))
   (newcoords
    (make-coords
     :pos (scale 1e+3 (random-vector 1.0))
     :rpy (scale 3.14 (random-vector 1.0))))
   (angle-vector
    (map float-vector
	 #'(lambda (a) (- (random 180.0) 90))
	 (send *robot* :angle-vector)))
   ret t1 t2 (dif #F(0)))
  (setq t1
	(bench2
	 (progn
	   (send *robot* :newcoords newcoords)
	   (send *robot* :angle-vector angle-vector)
	   ;;(send-all (send *robot* :links) :worldcoords)
	   (send link :worldcoords))))
  (setq t2 (bench2 (setq ret (calc-link-coords :link link :debug? dif))))
  (format t "   eus-time = ~A/~A=1/~A~%" t1 t2 (/ t2 t1))
  ret
  )

;; not tested yet
(defun calc-symbol-jacobian
  (&optional
   (limb :rarm)
   (link-list (cdr (send *robot* :links))))
  (let* ((l (send *robot* limb :end-coords :parent))
	 (pos
	  (send (send *robot* limb :end-coords :parent)
		:get :Pj))
	 (ret
	  (send
	   (send pos :gradient-vector
		 (send-all link-list :get :q))
	   :simplify)))
    (send l :put :jacobian l)
    ret))

(defun calc-jacobian-gradient
  (&key
   (move-target (send *robot* :rarm :end-coords))
   (link (send move-target :parent))
   (link-list (send *robot* :link-list link))
   (J (send *robot* :calc-jacobian-from-link-list
            link-list
            :move-target move-target
            :transform-coords (make-coords)
            :translation-axis '(t)
            :rotation-axis '(t)))
   (a (if (send link :joint)
	  (case (send (send link :joint) :get-val 'axis)
		(:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
		(:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
		(:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
		(t (send (send link :joint) :get-val 'axis)))))
   (R
    (send
     (send
      (send (send link :parent-link) :copy-worldcoords)
      :transform
      (copy-object
       (send (send link :joint) :get-val 'default-coords)))
     :worldrot))
   (x (scale 1e-3
	     (send (send move-target :copy-worldcoords) :worldpos)))
   (rpy (reverse (car (rpy-angle R))))
   (rpy-gradient-matrix
    (progn
      (defvar *rpy-matrix-gradient* (symbol-rpy-gradient 'r 'p 'y))
      (mapcar
       #'(lambda (rpy-grad)
	   (make-matrix
	    3 3
	    (send (send rpy-grad :eval '(r p y) rpy) :data)))
       *rpy-matrix-gradient*)))
   (rpy-gradient
    (reduce #'m+
	    (map cons #'scale-matrix a rpy-gradient-matrix)))
   )
  (m*
   (matrix-append
    (list
     (matrix-append
      (list (outer-product-matrix (transform R a))
	    (m* (outer-product-matrix (scale -1 x))
		rpy-gradient))
      '(0 1))
     (matrix-append
      (list (make-matrix 3 3) rpy-gradient)
      '(0 1)))
    '(1 0))
   J))



#|

(mapcar
 #'(lambda (l)
     (let* ((j
	     (send (send (send l :parent-link) :copy-worldcoords)
		   :transform
		   (send (send l :joint) :get-val 'default-coords))))
       (concatenate
	float-vector
	(v- (send j :worldpos) (send (send l :joint) :worldpos)))))
 (remove-if #'(lambda (a) (not (and (send a :joint) (send a :parent-link))))
	    (send *robot* :links)))


(mapcar
 #'(lambda (j)
     (let* ((paxis (case (send j :get-val 'axis)
			 (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
			 (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
			 (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
			 (t (send j :get-val 'axis))))
	    (ax (normalize-vector
		 (send (send (send (send j :parent-link) :copy-worldcoords) :transform
			     (send j :get-val 'default-coords))
		       :rotate-vector paxis)))
	    (ax2 (normalize-vector
		  (send (send (send (send j :child-link) :copy-worldcoords) :transform
			      (send (send j :get-val 'default-coords)
				    :transformation
				    (make-coords)))
			:rotate-vector paxis))))
       (print (matrix-log (send (send j :get-val 'default-coords) :worldrot)))
       (print (matrix-log (send (send j :child-link) :worldrot)))
       (print (matrix-log
	       (send (send (send j :child-link) :transformation
			   (send j :parent-link))
		     :worldrot)))
       (print "")
       ;;(format t "ax = ~A <vs> ax2 = ~A~%" ax ax2)
       (norm (v- ax ax2))))
 (send *robot* :joint-list))
