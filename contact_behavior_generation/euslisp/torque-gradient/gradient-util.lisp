
(require "../math/matrix-util.lisp")
(require "symbol-function.lisp")

(defun remove-args-values-loop
  (args key-list)
  ;;args)
  (while (flatten (mapcar
		   #'(lambda (k) (find k args))
		   key-list))
    (setq args (remove-args-values args key-list)))
  args)

(defun coordinates-matrix
  (c &key
     (scale 1)
     (gradient? nil))
  (let ((ret (make-matrix 4 4)))
    (dotimes (dx 3)
      (dotimes (dy 3)
	(setf (aref ret dx dy)
	      (aref (send c :worldrot) dx dy))))
    (dotimes (i 3)
      (setf (aref ret i 3) (* scale (aref (send c :worldpos) i))))
    (if (not gradient?) (setf (aref ret 3 3) 1))
    ret))

(defun matrix-coordinates
  (mat)
  (let ((rot (make-matrix 3 3))
	(pos #F(0 0 0)))
    (dotimes (dx 3)
      (dotimes (dy 3)
	(setf (aref rot dx dy) (aref mat dx dy))))
    (dotimes (i 3)
      (setf (aref pos i) (aref mat i 3)))
    (make-coords :rot rot :pos pos)))

(defun outer-product-vector
  (wx &key (tag nil))
  (cond
   ((> (norm (send (m+ wx (transpose wx)) :get-val 'entity)) 1e-3)
    ;;(format t "[outer-product-vector(~A)] invalid wx detected ~A~%" tag wx))
    (throw :invalid_outer_product_matrix wx))
   (t
    (float-vector
     (* -1 (aref wx 1 2)) (aref wx 0 2) (* -1 (aref wx 0 1))))))

(defun twist-gradient-from-transformation-gradient
  (dR R &key (tag nil))
  (outer-product-vector (m* dR (transpose R)) :tag tag))

(defun twist-gradientx2-from-transformation-gradientx2
  (ddR R
       &key
       (tag nil)
       (Rt (transpose R))
       (dw1dw2 (m* ddR Rt))
       (dw2dw1 (m* R (transpose ddR)))
       (ddw (m- dw1dw2 dw2dw1)))
  (outer-product-vector ddw :tag tag))

(defun 6dof-gradient-coords
  (coords delta-symbol-list
	  &key
	  (debug? t)
	  (meter? t)
	  (xyzabc-symbol '(x y z a b c))
	  rot pos)
  (format debug? "[6dof-gradient-coords] ~A E ~A~%"
	  delta-symbol-list
	  xyzabc-symbol)
  (cond
   ((null delta-symbol-list) coords)
   ((null
     (flatten
      (mapcar #'(lambda (s) (find s (subseq xyzabc-symbol 0 3)))
	      delta-symbol-list)))
    (cond
     ((eq (length (union nil delta-symbol-list)) 1)
      (format debug? "   eus-gradient~%")
      (labels ((exp-gradient-itterator
		(sl)
		(cond
		 ((null sl) (send (send coords :worldcoords) :worldrot))
		 (t
		  (m*
		   (make-matrix
		    3 3
		    (case (car sl)
			  ('a '((0 0 0) (0 0 -1) (0 1 0)))
			  ('c '((0 -1 0) (1 0 0) (0 0 0)))
			  ('b '((0 0 1) (0 0 0) (-1 0 0)))))
		   (exp-gradient-itterator (cdr sl)))))))
	      (setq rot (exp-gradient-itterator delta-symbol-list))
	      (make-coords :rot rot)))
     ;; stupid code
     (t ;;(not (plusp (norm2 (matrix-log (send coords :rot)))))
      (make-coords :rot (make-matrix 3 3)))
     (nil
      (setq rot
   	    (symbol-rodrigues-gradient
   	     delta-symbol-list
   	     :xyz-symbol (subseq xyzabc-symbol 3 6)))
      (setq rot
   	    (send rot
   		  :eval
   		  (subseq xyzabc-symbol 3 6)
   		  (coerce (matrix-log (send coords :worldrot)) cons)))
      (setq rot (make-matrix 3 3 (send rot :data)))
      (make-coords :rot rot))))
   ((eq 1 (length delta-symbol-list))
    (setq pos
	  (map float-vector
	       #'(lambda (s) (if (eq s (nth 0 delta-symbol-list)) 1 0))
	       (subseq xyzabc-symbol 0 3)))
    (make-coords :rot (make-matrix 3 3) :pos pos))
   (t (make-coords :rot (make-matrix 3 3)))))

(defmethod joint
  (:joint-worldcoords
   nil
   (if (and parent-link (find-method parent-link :copy-worldcoords))
       (send (send (send parent-link :worldcoords) :copy-worldcoords)
	     :transform default-coords)))
  (:parent-joint nil
		 (let (pj)
		   (if (and child-link
			    (find-method child-link :parent-link)
			    (setq pj (send child-link :parent-link))
			    (setq pj (send pj :joint)))
		       pj)))
  (:parent-joint-list
   nil
   (let ((pj (send self :parent-joint)))
     (if pj (cons self (send pj :parent-joint-list)) (list self))))
  (:joint-worldcoords-from-local-transformation
   (&optional (tc
	       (send self :parent-link2joint-transformation)))
   (let ((pj (send self :parent-joint)))
     (if pj
	 (send pj :joint-worldcoords-from-local-transformation
	       (send (send pj :parent-link2joint-transformation)
		     :transform
		     (send (send pj :joint2link-transformation)
			   :transform
			   tc
			   :local)
		     :local))
       (send
	(send (send parent-link :worldcoords) :copy-worldcoords)
	:transform tc :local))))
  (:joint-worldcoords-test
   nil
   (let ((w1 (send self :joint-worldcoords))
	 (w2 (send self :joint-worldcoords-from-local-transformation)))
     (concatenate
      float-vector
      (scale 1e-3 (send w1 :difference-position w2))
      (send w1 :difference-rotation w2))))
  (:parent-link2joint-transformation ;; constant value expected
   nil (send (send (send parent-link :worldcoords) :copy-worldcoords)
	     :transformation
	     (send self :joint-worldcoords)
	     :local))
  (:joint2link-transformation ;; rodrigues
   nil (send (send self :joint-worldcoords)
	     :transformation
	     (send (send child-link :worldcoords) :copy-worldcoords)
	     :local))
  (:local-axis-vector
   nil
   (or
    (send self :get :local-axis-vector)
    (send self :put :local-axis-vector
	  (let* ((paxis (case axis
			  (:x #f(1 0 0)) (:y #f(0 1 0)) (:z #f(0 0 1))
			  (:xx #f(1 0 0)) (:yy #f(0 1 0)) (:zz #f(0 0 1))
			  (:-x #f(-1 0 0)) (:-y #f(0 -1 0)) (:-z #f(0 0 -1))
			  (t (normalize-vector axis))))
		 (j0 (send self :joint-angle))
		 (jmin (progn
			 (send self :joint-angle (send self :min-angle))
			 (send (send self :child-link) :worldcoords)
			 (send (send (send self :child-link) :copy-worldcoords) :worldrot)))
		 (jmax (progn
			 (send self :joint-angle (send self :max-angle))
			 (send (send self :child-link) :worldcoords)
			 (send (send (send self :child-link) :copy-worldcoords) :worldrot)))
		 (axs (normalize-vector (matrix-log (m* (transpose jmin) jmax)))))
	    (send self :joint-angle j0)
	    (cond
	     ((> (norm2 (v- axs paxis)) 1e-3)
	      (send self :put :child-reverse t)
	      (format t "[local-axis-vector] reverse joint detected ~A~%"
		      (send self :name))))
	    axs))))
  (:axis-vector
   (&optional (wrt :local))
   (let ((paxis (send self :local-axis-vector)))
     (case wrt
       (:local paxis)
       (:world (transform (send (send self :joint-worldcoords) :worldrot)
			  paxis)))))
  )

(defmethod rotational-joint
  (:joint2link-gradient-matrix
   (&key
    (n 1) (meter? t))
   (coordinates-matrix
    (send self :joint2link-gradient-coords :n n)
    :gradient? t))
  (:joint2link-gradient-coords
   (&key
    (n 1) (meter? t))
   (let* ((ax (outer-product-matrix
	       (send self :axis-vector :local)))
	  (ax^n ax))
     (dotimes (i (- n 1)) (setq ax^n (m* ax ax^n)))
     (make-coords
      :rot
      (m* ax^n
	  (send (send self :joint2link-transformation) :worldrot)))))
  )

(defun fast-general-gradient-worldcoords ;; N order
  (&rest
   args
   &key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (joint-list (reverse (send-all link-list :joint))) ;; start with end-effector
   (target-joint-list nil)
   (root-link (car (send robot :links)))
   (meter? t)
   (debug? t)
   &allow-other-keys
   )
  ;;(setq hoge  args)
  (cond
   ((find-if #'(lambda (j)
		 (and (not (find j joint-list))
		      (not (find root-link (flatten (list j))))))
	     target-joint-list)
    (format debug? "[fggw] zero gradient detected~%")
    (make-coords :rot (make-matrix 3 3)))
   ((find-if #'(lambda (j)
		 (not (and (class j)
			   (subclassp (class j) rotational-joint))))
	     target-joint-list)
    (format debug? "[fggw] non rotational-joint detected~%")
    ;;(format t "   ~A vs " (send move-target :worldrot))
    (apply #'general-gradient-worldcoords args))
   ((null
     (setq target-joint-list
	   (sort target-joint-list
		 #'(lambda (j1 j2) (> (position j1 joint-list)
				      (position j2 joint-list))))))
    (format debug? "[fggw] joint /E joint-list~%")
    (apply #'general-gradient-worldcoords args))
   (t
    (let* ((ax-list (send-all target-joint-list :axis-vector :world))
	   (Rj (send (car (last target-joint-list)) :joint-worldcoords))
	   (R (send Rj :transformation
		    (send move-target :copy-worldcoords)
		    :local))
	   (ax^n
	    (if (eq (length ax-list) 1)
		(outer-product-matrix (car ax-list))
	      (reduce #'m* (mapcar #'outer-product-matrix ax-list))))
	   ;;Rb Rj R
	   )
      (setq R
	    (make-coords
	     :pos (transform (send Rj :worldrot) (send R :worldpos))
	     :rot (m* (send Rj :worldrot) (send R :worldrot))))
      ;; (setq Rj (send (car (last target-joint-list)) :joint-worldcoords))
      ;; (setq Rb (send move-target :copy-worldcoords))
      ;; (setq R
      ;; 	    (make-coords
      ;; 	     :pos (v- (send Rb :worldpos) (send Rj :worldpos))
      ;; 	     :rot (m* (transpose (send Rj :worldrot))
      ;; 		      (send Rb :worldrot))))
      (make-coords
       :pos (transform ax^n (scale (if meter? 1e-3 1) (send R :worldpos)))
       :rot (m* ax^n (send R :worldrot)))))))

(defun general-gradient-worldcoords
  (&key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (joint-list (reverse (send-all link-list :joint)))
   (target-joint-list nil)
   (root-link (car (send robot :links)))
   (meter? t)
   (debug? t)
   ;; buf
   (joint (car joint-list))
   (tc
    (coordinates-matrix
     (send (send
	    (send
	     (if (and joint (send joint :child-link))
		 (send joint :child-link) root-link)
	     :worldcoords)
	    :copy-worldcoords)
	   :transformation
	   (send (send move-target :worldcoords) :copy-worldcoords)
	   :local)
     :scale (if meter? 1e-3 1)))
   &allow-other-keys
   )
  (cond
   ((or (null joint)
	(eq joint (send root-link :joint)))
    (let ((ret
	   (matrix-coordinates
	    (m*
	     (if (assoc root-link target-joint-list)
		 (coordinates-matrix
		  (6dof-gradient-coords
		   (send (send root-link :worldcoords) :copy-worldcoords)
		   (flatten
		    (mapcar
		     #'cdr
		     (remove-if
		      #'(lambda (l) (not (find root-link (flatten (list l)))))
		      target-joint-list)))
		   :debug? debug?
		   :meter? meter?)
		  :gradient? t)
	       (coordinates-matrix
		(send (send root-link :worldcoords) :copy-worldcoords)
		:scale (if meter? 1e-3 1)))
	     tc))))
      ret))
   (t
    (general-gradient-worldcoords
     :robot robot
     :move-target move-target
     :link-list link-list
     :joint-list (cdr joint-list)
     :joint (cadr joint-list)
     :target-joint-list target-joint-list
     :root-link root-link
     :tc
     (m*
      (coordinates-matrix
       (send joint :parent-link2joint-transformation)
       :scale (if meter? 1e-3 1))
      (m*
       (cond
	((zerop (count joint (flatten target-joint-list)))
	 (coordinates-matrix
	  (send joint :joint2link-transformation)
	  :scale (if meter? 1e-3 1)))
	(t
	 (send joint :joint2link-gradient-matrix
	       :n (count joint target-joint-list)
	       :meter? meter?)))
       tc))
     :meter? meter?
     :debug? debug?))
   )
  )

;; gradient-vector :target-joint (list (car (send *robot* :links)) 'c)
(defun gradient-vector
  (&rest
   args
   &key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (joint-list (reverse (send-all link-list :joint)))
   (target-joint (nth (random (+ (length joint-list) 0)) joint-list))
   (debug? t)
   (meter? t)
   (ret
    (fast-general-gradient-worldcoords
     :robot robot
     :move-target move-target
     :link-list link-list
     :joint-list joint-list
     :target-joint-list (list target-joint)
     :meter? meter?
     :debug? debug?))
   (p (send ret :pos))
   ;; (hoge (print args))
   (r (twist-gradient-from-transformation-gradient
			(send ret :rot)
			(send move-target :worldrot)
			:tag
			(if (find-method target-joint :name)
			    (send target-joint :name))))
   &allow-other-keys)
  ;;(print args)
  (setq ret (concatenate float-vector p r))
  (cond
   ((and (listp target-joint)
	 (class (car target-joint))
	 (subclassp (class (car target-joint)) cascaded-coords))
    (format debug? "[gradient-vector] 6dof link detected~%")
    ret)
   (debug?
    (let ((ax  (if target-joint (send target-joint :axis-vector :world)))
	  (p-p (if target-joint
		   (scale
		    (if meter? 1e-3 1)
		    (v- (send move-target :worldpos)
			(send (send target-joint :joint-worldcoords) :worldpos)))))
	  dif ref)
      (setq ref (concatenate float-vector (v* ax p-p) ax))
      (if (not (find target-joint joint-list))
	  (setq ref #F(0 0 0 0 0 0)))
      (format t
	      "[gradient-vector] ~A~% ref=~A~% ans=~A~% dif=~A~%"
	      (send target-joint :name)
	      ref
	      ret
	      (norm2 (setq dif (v- ref ret))))
      (cond
       ((> (norm2 dif) 1e-6)
	(format t "[gradient-vector] too large error!!!~%"))
       (t ret))
      ))
   (t ret)))

(defun gradient-matrix
  (&rest
   args
   &key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (all-link-list (send robot :link-list (send move-target :parent)))
   ;;(joint-list (reverse (send-all link-list :joint)))
   (all-joint-list (send-all all-link-list :joint))
   (6dof? t)
   (meter? t)
   (debug? t)
   (vector-length 6)
   &allow-other-keys
   )
  (setq args (remove-args-values-loop args '(:link-list)))
  (transpose
   (make-matrix
    (+ (length all-joint-list) (if 6dof? 6 0))
    vector-length
    (append
     (mapcar
      #'(lambda (j)
	  (subseq
	   (apply #'gradient-vector (append (list :target-joint j) args))
	   0 vector-length)
	  )
      (append
       all-joint-list
       (if 6dof?
	   (mapcar #'(lambda (k)
		       (list (car (send robot :links)) k))
		   '(x y z a b c))))))))
  )

(defun test-gradient-matrix
  (&rest
   args
   &key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (all-link-list (send robot :link-list (send move-target :parent)))
   ;;(joint-list (reverse (send-all link-list :joint)))
   (all-joint-list (send-all all-link-list :joint))
   (meter? t)
   (debug? t)
   (vector-length 6)
   ;;
   (g1 (apply 'gradient-matrix (append (list :6dof? nil) args)))
   (g2 (send robot :calc-jacobian-from-link-list all-link-list
	     :move-target move-target
	     :transform-coords (make-coords)
	     :translation-axis '(t)
	     :rotation-axis '(t)))
   &allow-other-keys
   )
  (format t "----- mine~%")
  (format-array g1)
  (format t "----- eus impl~%")
  (format-array g2)
  (format t "----- diff mat~%")
  (format-array (m- g1 g2))
  (format t "----- diff norm~%")
  (print (norm (send (m- g1 g2) :get-val 'entity)))
  )

(defun gradientx2-vector
  (&key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (joint-list (reverse (send-all link-list :joint)))
   (target-joint1 (nth (random (- (length joint-list) 0)) joint-list)) ;; near to base-link
   (target-joint2 (nth (random (+ 1 (position target-joint1 joint-list)))
		       joint-list))
   (target-joint-list (list target-joint1 target-joint2))
   (debug? t)
   (meter? t)
   (grad^2
    (fast-general-gradient-worldcoords
     :robot robot
     :move-target move-target
     :link-list link-list
     :joint-list joint-list
     :target-joint-list (list target-joint1 target-joint2)
     :meter? meter?
     :debug? debug?))
   (p (send grad^2 :pos))
   (r (twist-gradientx2-from-transformation-gradientx2
       (send grad^2 :rot)
       (send move-target :worldrot)
       :tag (list
	     (if (find-method target-joint1 :name) (send target-joint1 :name))
	     (if (find-method target-joint2 :name) (send target-joint2 :name)))))
   buf
   &allow-other-keys)
  (setq ret (concatenate float-vector p r))
  (cond
   ((remove-if
     #'(lambda (j) (and j (class j) (subclassp (class j) rotational-joint)))
     (flatten target-joint-list))
    (format debug? "[gradient^2-vector] linear joint detected~%")
    ret)
   ((and target-joint-list debug?)
    (let ((ax (send-all target-joint-list :axis-vector :world))
	  (p-p (scale
		(if meter? 1e-3 1)
		(v- (send move-target :worldpos)
		    (send (send target-joint2 :joint-worldcoords) :worldpos))))
	  dif ref)
      (setq ref (concatenate float-vector
			     (reduce #'v* (reverse (append ax (list p-p))))
			     (apply #'v* ax)))
      (if (not (and (find target-joint1 joint-list)
		    (find target-joint2 joint-list)))
	  (setq ref #F(0 0 0 0 0 0)))
      (format t
	      "[gradient^2-vector] [EF]<-[~A]<-[~A]~% ref=~A~% ans=~A~% dif=~A~%"
	      (send target-joint2 :name)
	      (send target-joint1 :name)
	      ref
	      ret
	      (norm2 (setq dif (v- ref ret))))
      (cond
       ((> (norm2 dif) 1e-6)
	(format t "[gradient^2-vector] too large error!!!~%"))
       (t ret))
      ))
   (t ret)))

(defun gradientx2-matrix
  (&rest
   args
   &key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (all-link-list (send robot :link-list (send move-target :parent)))
   ;;(joint-list (reverse (send-all link-list :joint)))
   (all-joint-list (send-all all-link-list :joint))
   (target-joint (nth (random (length all-joint-list)) all-joint-list))
   (6dof? t)
   (meter? t)
   (debug? t)
   &allow-other-keys
   )
  (setq args (remove-args-values-loop args '(:link-list)))
  (transpose
   (make-matrix
    (+ (length all-joint-list) (if 6dof? 6 0))
    6
    (append
     (mapcar
      #'(lambda (j)
	  (let ((j1 target-joint) (j2 j) buf)
	    (cond
	     ((and
	       (class j1)
	       (subclassp (class j1) joint)
	       (find j2 (send j1 :parent-joint-list)))
	      (setq buf j1)
	      (setq j1 j2)
	      (setq j2 buf)))
	    (apply #'gradientx2-vector
		   (append (list :target-joint1 j1
				 :target-joint2 j2)
			   args))))
      (append
       all-joint-list
       (if 6dof?
	   (mapcar #'(lambda (k)
		       (list (car (send robot :links)) k))
		   '(x y z a b c))))))))
  )

(defun gravity-move-target
  (all-link-list)
  (mapcar #'(lambda (l)
	      (send l :worldcoords)
	      (send l :put :gravity-move-target
		    (or (send l :get :gravity-move-target)
			(make-cascoords
			 :name
			 (read-from-string
			  (format nil "~A-gravity-move-target" (send l :name)))
			 :pos (copy-seq (send l :centroid))
			 :parent l))))
	  all-link-list))

(defun inertia-move-target
  (link-list
   &key
   (robot *robot*)
   (weight (send robot :weight))
   buf)
  ;;(send-all (send robot :links) :worldcoords)
  (send robot :update-mass-properties)
  (mapcar
   #'(lambda (l)
       (setq buf (send l :get :inertia-move-target))
       (if buf (send (send buf :parent) :dissoc buf))
       (send l :put :weight-rate (/ (send l :get :m-til) weight))
       (send l :put :inertia-move-target
	     (make-cascoords
	      :name
	      (read-from-string
	       (format nil "~A-inertia-move-target" (send l :name)))
	      :pos (copy-seq (send l :get :c-til))
	      :parent l)))
   link-list))

;; gradient samples
(defun cog-gradient-matrix
  (&rest
   args
   &key
   (robot *robot*)
   (all-link-list (cdr (send robot :links)))
   (all-joint-list (send-all all-link-list :joint))
   ;;
   (weight-list
    (mapcar #'(lambda (l) (send l :weight))
	    all-link-list))
   (gravity-move-target (gravity-move-target all-link-list))
   (gravity-jacobian
    (mapcar
     #'(lambda (mt w)
	 (scale-matrix
	  w
	  (apply #'gradient-matrix
		 (append (list :move-target mt
			       :all-joint-list all-joint-list
			       :vector-length 3)
			 args))))
     gravity-move-target weight-list))
   ;;
   (6dof? t)
   (meter? t)
   (debug? nil)
   &allow-other-keys
   )
  (scale-matrix
   (/ 1.0 (send robot :weight));;(apply #'+ weight-list))
   (reduce #'m+ gravity-jacobian)))

(defun fast-cog-gradient-matrix
  (&rest
   args
   &key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (all-link-list (cdr (send robot :links)))
   (all-joint-list (send-all all-link-list :joint))
   ;;
   (6dof? t)
   (inertia-move-target
    (progn
      (inertia-move-target
       (cons root-link all-link-list)
       :robot robot)
      (append
       (send-all (send-all all-joint-list :child-link) :get :inertia-move-target)
       (if 6dof?
	   (make-list
	    6 :initial-element
	    (send root-link :get :inertia-move-target))))))
   (weight-rate-list
    (append
     (send-all (send-all all-joint-list :child-link) :get :weight-rate)
     (if 6dof? (list 1 1 1 1 1 1))))
   ;;
   (meter? t)
   (debug? t)
   (vector-length 3)
   &allow-other-keys
   )
  (setq args (remove-args-values-loop args '(:link-list)))
  (transpose
   (make-matrix
    (+ (length all-joint-list) (if 6dof? 6 0))
    vector-length
    (mapcar
     #'(lambda (j mt w)
	 (scale
	  w
	  (subseq
	   (apply #'gradient-vector
		  (append (list :target-joint j
				:move-target mt)
			  args))
	   0 vector-length))
	 )
     (append
      all-joint-list
      (if 6dof?
	  (mapcar #'(lambda (k)
		      (list (car (send robot :links)) k))
		  '(x y z a b c))))
     inertia-move-target
     weight-rate-list)))
  )

(defun fast-cog-gradientx2-matrix-column
  (&rest
   args
   &key
   (robot *robot*)
   (move-target (send robot :rleg :end-coords))
   (root-link (car (send robot :links)))
   (all-link-list (cdr (send robot :links)))
   ;;(joint-list (reverse (send-all link-list :joint)))
   (all-joint-list (send-all all-link-list :joint))
   (target-joint (nth (random (length all-joint-list)) all-joint-list))
   ;;
   (6dof? t)
   (inertia-move-target
    (progn
      (inertia-move-target (cons root-link all-link-list))
      (append
       (send-all (send-all all-joint-list :child-link) :get :inertia-move-target)
       (if 6dof?
	   (make-list
	    6 :initial-element
	    (send root-link :get :inertia-move-target))))))
   (weight-rate-list
    (append
     (send-all (send-all all-joint-list :child-link) :get :weight-rate)
     (if 6dof? (list 1 1 1 1 1 1))))
   ;;
   (meter? t)
   (debug? t)
   &allow-other-keys
   )
  (setq args (remove-args-values-loop args '(:link-list)))
  (transpose
   (make-matrix
    (+ (length all-joint-list) (if 6dof? 6 0))
    6
    (append
     (mapcar
      #'(lambda (j)
	  (let ((j1 target-joint) (j2 j) buf l)
	    (cond
	     ((and
	       (class j1)
	       (subclassp (class j1) joint)
	       (find j2 (send j1 :parent-joint-list)))
	      (setq buf j1)
	      (setq j1 j2)
	      (setq j2 buf)))
	    (setq l
		  (cond
		   ((find-method j2 :child-link)
		    (send j2 :child-link))
		   ((and (list j2) (find root-link j2))
		    root-link)))
	    (scale
	     1;;(send l :get :weight-rate)
	     (apply #'gradientx2-vector
		    (append (list :move-target
				  (send l :get :inertia-move-target)
				  :target-joint1 j1
				  :target-joint2 j2)
			    args)))))
      (append
       all-joint-list
       (if 6dof?
	   (mapcar #'(lambda (k)
		       (list root-link k))
		   '(x y z a b c))))))))
  )

(defun fast-cog-gradientx2-matrix
  (&rest
   args
   &key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (all-link-list (cdr (send robot :links)))
   (all-joint-list (send-all all-link-list :joint))
   ;;
   (6dof? t)
   (meter? t)
   (debug? t)
   ;;
   (gradientx2-matrix
    (mapcar #'(lambda (j) (apply #'fast-cog-gradientx2-matrix-column
				 (append
				  (list :target-joint j)
				  args)))
	    (append
	     all-joint-list
	     (if 6dof?
		 (mapcar #'(lambda (k)
			     (list (car (send robot :links)) k))
			 '(x y z a b c))))))
   (weight-list
    (concatenate
     cons
     (mapcar
      #'(lambda (j)
	  (* 1e-3 (send (send j :child-link) :get :m-til)))
      all-joint-list)
     (if 6dof? (scale (* 1e-3 (send robot :weight))
		      #F(1 1 1 1 1 1)))))
   (wrench-list
    (mapcar #'(lambda (m) (scale m #F(0 0 -9.8 0 0 0)))
	    weight-list))
   (torque-gradient
    (make-matrix
     (+ (if 6dof? 6 0) (length all-joint-list))
     (+ (if 6dof? 6 0) (length all-joint-list))
     (mapcar
      #'(lambda (m w) (transform (transpose m) w))
      gradientx2-matrix wrench-list)))
   &allow-other-keys
   )
  (if (and debug? (boundp 'pm))
      (mapcar #'pm hessian-matrix))
  (scale-matrix -1 torque-gradient))

(defun calc-torque-gradient-from-wrench-and-gradient
  (&rest
   args
   &key
   (robot *robot*)
   (wrench (instantiate float-vector 6))
   (move-target (send robot :rleg :end-coords))
   (all-link-list (send robot :link-list (send move-target :parent)))
   ;;(joint-list (reverse (send-all link-list :joint)))
   (all-joint-list (send-all all-link-list :joint))
   (6dof? t)
   (meter? t)
   (debug? nil)
   (hessian-matrix
    (mapcar #'(lambda (j) (apply #'gradientx2-matrix
				 (append
				  (list :target-joint j)
				  args)))
	    (append
	     all-joint-list
	     (if 6dof?
		 (mapcar #'(lambda (k)
			     (list (car (send robot :links)) k))
			 '(x y z a b c))))))
   (torque-gradient
    (make-matrix
     (+ (if 6dof? 6 0) (length all-joint-list))
     (+ (if 6dof? 6 0) (length all-joint-list))
     (mapcar
      #'(lambda (m) (transform (transpose m) wrench))
      hessian-matrix)))
   &allow-other-keys
   )
  (if (and debug? (boundp 'pm))
      (mapcar #'pm hessian-matrix))
  (scale-matrix -1 torque-gradient))

(defun calc-torque-gradient-from-wrench-list-and-gradient
  (&rest
   args
   &key
   (robot *robot*)
   (all-link-list (cdr (send robot :links)))
   (all-joint-list (send-all all-link-list :joint))
   ;;
   (wrench (list (instantiate float-vector 6)))
   (move-target (list (send robot :rleg :end-coords)))
   (cog-gradientx2
    (apply #'fast-cog-gradientx2-matrix args))
   ;;(gravity-vector #F(0 0 -9.8 0 0 0))
   ;; (gravity-wrench
   ;;  (mapcar #'(lambda (l)
   ;; 		(scale (* 1e-3 (send l :weight)) gravity-vector))
   ;; 	    all-link-list))
   ;; (gravity-move-target (gravity-move-target all-link-list))
   ;; (wrench-list (append wrench gravity-wrench))
   ;; (move-target-list (append move-target gravity-move-target))
   ;;
   (6dof? t)
   (meter? t)
   (debug? t)
   ;;
   ret
   )
  (setq
   ret
   (reduce
    #'m+
    (cons
     cog-gradientx2
     (mapcar
      #'(lambda (w mt)
	  (apply #'calc-torque-gradient-from-wrench-and-gradient
		 (append
		  (list :robot robot
			:wrench w
			:move-target mt
			;;:link-list link-list
			:all-joint-list all-joint-list
			:debug? debug?
			)
		  args)))
      wrench move-target))))
  ret)

(defun child-reverse-filter
  (target joint-list &key (6dof? t))
  (cond
   ((not (eq (length target) (+ (if 6dof? 6 0) (length joint-list))))
    (format t "[child-reverse] vector dimension missmatch ~A vs ~A~%"
	    target joint-list)))
  (send-all joint-list :local-axis-vector)
  (map float-vector
       #'*
       target
       (append
	(mapcar
	 #'(lambda (j) (if (send j :get :child-reverse) -1 1))
	 joint-list)
	(if 6dof? '(1 1 1 1 1 1)))))

(defun calc-torque-from-wrench-and-gradient
  (&rest
   args
   &key
   (robot *robot*)
   (wrench (instantiate float-vector 6))
   (move-target (send robot :rleg :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (joint-list (reverse (send-all link-list :joint)))
   (all-joint-list joint-list)
   (dof (length all-joint-list))
   (6dof? t)
   (meter? t)
   (debug? nil)
   (jacobian (apply #'gradient-matrix args))
   vbuf
   &allow-other-keys
   )
  (if (not (vectorp vbuf))
      (setq vbuf (instantiate float-vector (+ dof (if 6dof? 6 0)))))
  (if debug? (pm jacobian))
  ;; (scale -1 (transform (transpose jacobian) wrench vbuf) vbuf)
  (dotimes (i dof)
    (setf (aref vbuf i)
	  (* -1 (v. (matrix-column jacobian i) wrench))))
  ;; (if 6dof? (setq ret (concatenate float-vector ret (float-vector 0 0 0 0 0 0))))
  vbuf)

(defun calc-torque-from-wrench-list-and-gradient
  (&rest
   args
   &key
   (robot *robot*)
   (all-link-list (send robot :link-list
		    (send robot :rleg :end-coords :parent)))
   (all-joint-list (send-all all-link-list :joint))
   ;;
   (wrench (list (instantiate float-vector 6)))
   (move-target (list (send robot :rleg :end-coords)))
   ;;
   (gravity-wrench (scale (* 1e-3 (send robot :weight))
			  #F(0 0 -9.8)))
   (cog-jacobian
    (apply #'fast-cog-gradient-matrix
	   (append
	    (list :all-joint-list all-joint-list)
	    args)))
   (wrench-list wrench)
   (move-target-list move-target)
   ;; (gravity-vector #F(0 0 -9.8 0 0 0))
   ;; (gravity-wrench
   ;;  (mapcar #'(lambda (l)
   ;; 		(scale (* 1e-3 (send l :weight)) gravity-vector))
   ;; 	    all-link-list))
   ;; (gravity-move-target (gravity-move-target all-link-list))
   ;; (wrench-list (append wrench gravity-wrench))
   ;; (move-target-list (append move-target gravity-move-target))
   ;;
   (dof (length all-joint-list))
   (6dof? t)
   (meter? t)
   (debug? nil)
   vbuf vbufbuf
   &allow-other-keys
   )
  (if (not (vectorp vbuf))
      (setq vbuf (instantiate float-vector (+ dof (if 6dof? 6 0)))))
  (if (not (vectorp vbufbuf)) (setq vbufbuf (copy-seq vbuf)))
  ;; (scale -1 (transform (transpose cog-jacobian) gravity-wrench vbuf) vbuf)
  (dotimes (i dof)
    (setf (aref vbuf i)
	  (* -1 (v. (matrix-column cog-jacobian i) gravity-wrench))))
  (mapcar
   #'(lambda (w mt)
       (apply #'calc-torque-from-wrench-and-gradient
	      (append
	       (list :robot robot
		     :wrench w
		     :move-target mt
		     ;;:link-list link-list
		     :all-joint-list all-joint-list
		     :debug? debug?
		     :vbuf vbufbuf
		     )
	       args))
       (v+ vbuf vbufbuf vbuf)
       )
   wrench-list move-target-list)
  ;;
  (cond
   (debug?
    (let* ((tau (send robot :torque-vector
		      :target-coords move-target-list
		      :force-list (mapcar '(lambda (v) (subseq v 0 3)) wrench-list)
		      :moment-list (mapcar '(lambda (v) (subseq v 3 6)) wrench-list)))
	   (tau2 (coerce (send-all all-joint-list :joint-torque) float-vector)))
      (format t "tau-check: ~A vs ~A = ~A (~A)~%"
	      vbuf tau2 (v- vbuf tau2) (norm2 (v- vbuf tau2)))
      (if (> (norm2 (v- vbuf tau2)) 1e-1)
	  (format t " --- too large error~%"))
      )))
  vbuf
  )

;; (defun calc-torque-from-wrench-list-and-gradient
;;   (&rest
;;    args
;;    &key
;;    (robot *robot*)
;;    (all-link-list (send robot :link-list
;; 		    (send robot :rleg :end-coords :parent)))
;;    (all-joint-list (send-all all-link-list :joint))
;;    ;;
;;    (wrench (list (instantiate float-vector 6)))
;;    (move-target (list (send robot :rleg :end-coords)))
;;    (gravity-vector #F(0 0 -9.8 0 0 0))
;;    (gravity-wrench
;;     (mapcar #'(lambda (l)
;; 		(scale (* 1e-3 (send l :weight)) gravity-vector))
;; 	    all-link-list))
;;    (gravity-move-target (gravity-move-target all-link-list))
;;    (wrench-list (append wrench gravity-wrench))
;;    (move-target-list (append move-target gravity-move-target))
;;    ;;
;;    (6dof? t)
;;    (meter? t)
;;    (debug? nil)
;;    )
;;   (reduce #'v+
;; 	  (mapcar
;; 	   #'(lambda (w mt)
;; 	       (apply #'calc-torque-from-wrench-and-gradient
;; 		      (append
;; 		       (list :wrench w
;; 			     :move-target mt
;; 			     ;;:link-list link-list
;; 			     :all-joint-list all-joint-list
;; 			     :debug? debug?
;; 			     )
;; 		       args)))
;; 	   wrench-list move-target-list)))

(defun test-calc-torque-from-wrench-and-gradient
  (&key
   (robot *robot*)
   (move-target (list (send robot :rleg :end-coords)))
   (joint-list
    (send-all
     (flatten (mapcar
	       #'(lambda (l) (send robot :link-list (send l :parent)))
	       move-target))
     :joint))
   (force-list (mapcar #'(lambda (mt) (random-vector 600.0)) move-target))
   (moment-list (mapcar #'(lambda (mt) (random-vector 60.0)) move-target))
   (wrench-list
    (mapcar #'(lambda (f m) (concatenate float-vector f m))
	    force-list moment-list))
   (6dof? nil)
   ref ans
   )
  (random-robot-state)
  (format t
	  (concatenate
	   string
	   "[calc-torque-from-wrench-and-gradient-test]~%"
	   "  ref=~A~%"
	   "  ans=~A~%"
	   "  dif=~A~%"
	   "  dif=~A~%")
	  (progn
	    (send robot :torque-vector
		  :force-list force-list
		  :moment-list moment-list
		  :target-coords move-target)
	    (setq ref (send-all joint-list :joint-torque)))
	  (setq ans
		(child-reverse-filter
		 (calc-torque-from-wrench-list-and-gradient
		  :robot robot
		  :move-target move-target
		  :wrench wrench-list
		  :all-joint-list joint-list
		  :debug? t
		  :6dof? 6dof?
		  )
		 joint-list :6dof? 6dof?))
	  (coerce (map cons #'- ref ans) float-vector)
	  (norm2 (coerce (map cons #'- ref ans) float-vector))))

;; for test
(defun random-robot-state
  nil
  (mapcar
   #'(lambda (j)
       (send j :joint-angle
	     (+ (send j :min-angle)
		(random (* 1.0 (- (send j :max-angle) (send j :min-angle)))))))
   (send *robot* :joint-list))
  (send *robot* :newcoords
	(make-coords :pos (scale 300 (random-vector 1.0))
		     :rpy (random-vector 3.14)))
  (send-all (send *robot* :links) :worldcoords)
  t
  )

(defun joint-worldcoords-test
  nil
  (random-robot-state)
  (apply #'+
	 (mapcar
	  #'norm2
	  (send-all (send *robot* :joint-list) :joint-worldcoords-test)))
  )

(defun matrix-log-test
  ;; check matrix-log value vs twist-gradient-from-transformation-gradient
  (&key
   (rpy (random-vector 3.14))
   (drpy/dt (random-vector 3.14))
   (dt 1e-3)
   (drpy (scale dt drpy/dt))
   (Rrpy (send (make-coords :rpy (reverse rpy)) :worldrot))
   (Rrpy+drpy (send (make-coords :rpy (reverse (v+ drpy rpy))) :worldrot))
   (dRrpy (m- Rrpy+drpy Rrpy))
   (dRrpy/dt (scale-matrix (/ 1.0 dt) dRrpy))
   (w-from-twist-gradient
    (twist-gradient-from-transformation-gradient dRrpy/dt Rrpy))
   (w-from-matrix-log
    (scale (/ 1.0 dt)
	   (transform
	    Rrpy
	    (matrix-log (m* (transpose Rrpy) Rrpy+drpy)))))
   )
  (format t "[matrix-log-test]~% rpy=~A~% w1 =~A~% w2 =~A~%"
	  drpy/dt w-from-twist-gradient w-from-matrix-log)
  )

(defun calc-torque-force-matrix
  (&rest
   args
   &key
   (robot *robot*)
   (move-target (list (send robot :rleg :end-coords)))
   (link-list
    (mapcar #'(lambda (mt) (send robot :link-list (send mt :parent)))
	    move-target))
   (joint-list (mapcar #'(lambda (ll) (send-all ll :joint)) link-list))
   (torque-list nil)
   ;;(mapcar #'(lambda (jl) (send-all jl :joint-torque)) joint-list))
   (jacobian-list
    (mapcar #'(lambda (mt ll)
		(apply #'gradient-matrix
		       (append
			(list :move-target mt
			      :all-link-list ll
			      :6dof? nil)
			args)))
	    move-target link-list))
   (Jt-1
    (matrix-append
     (mapcar #'(lambda (j) (pseudo-inverse (transpose j))) jacobian-list)
     '(1 1)))
   &allow-other-keys)
  (if torque-list
      (setq torque-list
	    (apply #'concatenate (cons float-vector torque-list))))
  (if torque-list (transform Jt-1 torque-list) Jt-1))

(defun cog-gradient-matrix-from-contact-force
  (&key
   (robot *robot*)
   (mg (* 1e-3 (send robot :weight) 9.8))
   (g-1 (make-matrix
	 3 6
	 (list (list 0 0 0 0 (/ 1.0 mg) 0)
	       (list 0 0 0 (/ -1.0 mg) 0 0)
	       (list 0 0 0 0 0 0))))
   (move-target (list (send robot :rleg :end-coords)))
   (contact-force nil) ;;(list #F(0 0 0 0 0 0)))
   (center (scale (/ 1.0 (length move-target))
		  (funcall
		   (if (< (length move-target) 2)
		       #'car
		     #'(lambda (l) (reduce #'v+ l)))
		   (send-all move-target :worldpos))))
   (mat
    (m* g-1 (send robot :calc-grasp-matrix
		  (mapcar
		   #'(lambda (mt) (v- (send mt :worldpos) center))
		   move-target))))
   &allow-other-keys
   )
  (if contact-force
      (transform
       mat
       (apply #'concatenate (cons float-vector contact-force)))
    mat))

(defun brlv-to-cog
  (&rest
   args
   &key
   (robot *robot*)
   (debug? t)
   (move-target
    (mapcar
     #'(lambda (k) (send robot k :end-coords))
     '(:rarm :larm :rleg :lleg)))
   (torque-list nil)
   (force-list
    (make-list (length move-target)
	       :initial-element #F(0 0 0 0 0 0)))
   (link-list
    (mapcar #'(lambda (mt) (send robot :link-list (send mt :parent)))
	    move-target))
   (all-link-list (union (flatten link-list) nil))
   (all-joint-list (send-all all-link-list :joint))
   ;;(before (print 'before))
   (cog-jacobian
    (apply #'fast-cog-gradient-matrix
	   (append
	    (list :all-joint-list all-joint-list)
	    args)))
   ;;(after (print 'after))
   (cog-force-gradient
    (apply #'cog-gradient-matrix-from-contact-force
	   (append (list :move-target move-target) args)))
   (torque-force-matrix
    (apply #'calc-torque-force-matrix
	   (append
	    (list :move-target move-target
		  :link-list link-list
		  :torque-list torque-list)
	    args)))
   target-centroid-pos
   &allow-other-keys
   )
  ;; (if (vectorp torque-force-matrix)
  ;;     (setq torque-force-matrix
  ;; 	    (v- (scale 1e-2 torque-force-matrix)
  ;; 		(apply
  ;; 		 #'concatenate
  ;; 		 (cons float-vector force-list)))))
  ;; (print torque-force-matrix)
  ;;(setq cog-jacobian
  ;;(send robot :calc-cog-jacobian-from-link-list
  ;;:link-list all-link-list))
  (cond
   ((and debug?
	 (vectorp torque-force-matrix))
    (let* ((xyzabc '(:x :y :z :a :b :c))
	   (id 0)
	   (filter-v (scale 0 torque-force-matrix))
	   cog-move-buf
	   (cog-move #F(0 0 0))
	   )
      (format t "[brlv-cog-move]~%")
      (pm (scale-matrix 600 cog-force-gradient))
      (dotimes (id (length move-target))
	(dotimes (x 6)
	  (setq filter-v (scale 0 filter-v))
	  (setf (aref filter-v (+ x (* id 6))) -1)
	  (setq filter-v
		(map float-vector #'*
		     filter-v
		     torque-force-matrix))
	  (format t " ~A(~A)=~A ~A~%"
		  (nth x xyzabc)
		  id
		  (aref filter-v (+ x (* id 6)))
		  (setq cog-move-buf
			(transform cog-force-gradient filter-v)))
	  (setq cog-move (v+ cog-move cog-move-buf))
	  ))
      (format t "     cog-move=~A~%" cog-move)
      )))
  (if (and target-centroid-pos (vectorp torque-force-matrix))
      (v+
       (send robot :centroid)
       (transform cog-force-gradient torque-force-matrix)
       target-centroid-pos))
  (apply
   (if (vectorp torque-force-matrix) #'transform #'m*)
   (list (m* (transpose cog-jacobian) cog-force-gradient)
	 (scale -1 torque-force-matrix))))

(defun calc-torque-gradient-with-force-approximation
  (&rest
   args
   &key
   (robot *robot*)
   (debug? t)
   (move-target
    (mapcar
     #'(lambda (k) (send robot k :end-coords))
     '(:rarm :larm :rleg :lleg)))
   (mg (* -1e-3 (send robot :weight) 9.8))
   (mgx (matrix-append
	 (list (make-matrix 3 3)
	       (outer-product-matrix (float-vector 0 0 mg)))
	 '(1 0)))
   (center (scale (/ 1.0 (length move-target))
		  (funcall
		   (if (< (length move-target) 2)
		       #'car
		     #'(lambda (l) (reduce #'v+ l)))
		   (send-all move-target :worldpos))))
   (G (send robot :calc-grasp-matrix
	    (mapcar
	     #'(lambda (mt) (v- (send mt :worldpos) center))
	     move-target)))
   (link-list
    (mapcar #'(lambda (mt) (send robot :link-list (send mt :parent)))
	    move-target))
   (all-link-list (union (flatten link-list) nil))
   (all-joint-list (send-all all-link-list :joint))
   ;;(before (print 'before))
   (cog-jacobian
    (apply #'fast-cog-gradient-matrix
	   (append
	    (list :all-joint-list all-joint-list)
	    args)))
   (jacobian-list
    (mapcar #'(lambda (mt)
		(apply #'gradient-matrix
		       (append
			(list :move-target mt
			      :all-link-list all-link-list
			      )
			args)))
	    move-target))
   (Jf
    (matrix-append
     (mapcar #'(lambda (j) (transpose j)) jacobian-list)
     '(0 1)))
   (as-list nil)
   (inverse? t)
   (ret
    (if inverse?
	(list Jf
	      (pseudo-inverse-loop G)
	      ;;(transpose G)
	      mgx cog-jacobian)
      (list (pseudo-inverse-loop Jf)
	    G
	    (pseudo-inverse-loop mgx)
	    (transpose cog-jacobian))))
   &allow-other-keys
   )
  (if as-list ret (reduce #'m* ret))
  )

(defun time-trial-torque-vs-brlv-gradient
  (&key
   (move-target
    (mapcar
     #'(lambda (k) (send *robot* k :end-coords))
     '(:rarm :larm :rleg :lleg)))
   (loop-cnt 10)
   (average 0))
  (format t "[time-trial-torque-vs-brlv-gradient]~%")
  (dotimes (i loop-cnt)
    (let (tm1 tm2)
      (random-robot-state)
      (setq tm2 (bench2 (brlv-to-cog :debug? nil :move-target move-target)))
      (setq tm1 (bench2 (calc-torque-gradient-from-wrench-list-and-gradient
			 :move-target move-target :debug? nil)))
      (format t "  ~A/~A) ~A vs ~A: average=~A~%"
	      (+ 1 i) loop-cnt
	      tm1 tm2
	      (/ (setq average (+ average (/ tm1 tm2))) (+ i 1)))))
  (setq average (/ average loop-cnt)))

#|

(format t "[axis-test]~%")
(mapcar
 #'(lambda (j)
     (let* ((j0 (progn
		  (send j :joint-angle (send j :min-angle))
		  (send *robot* :angle-vector
			(copy-seq (send *robot* :angle-vector)))
		  (send-all (send *robot* :links) :worldcoords)
		  (send *viewer* :draw-objects)
		  ;;(read-line)
		  (send (send (send j :child-link) :copy-worldcoords) :worldrot)
		  ))
	    (j1 (progn
		  (send j :joint-angle (send j :max-angle))
		  (send *robot* :angle-vector
			(copy-seq (send *robot* :angle-vector)))
		  (send-all (send *robot* :links) :worldcoords)
		  (send *viewer* :draw-objects)
		  ;;(read-line)
		  (send (send (send j :child-link) :copy-worldcoords) :worldrot)))
	    (ans (send j :axis-vector :local))
	    (axs (normalize-vector (matrix-log (m* (transpose j0) j1)))))
       ;; (print (m* (transpose j0) j1))
       (format t " ~A:  ~A vs ~A (dif=~A)~%"
	       (send j :name) ans axs (norm2 (v- ans axs)))))
 (send *robot* :joint-list))

(let* ((robot *robot*)
       (all-link-list (send robot :link-list
			    (send robot :rleg :end-coords :parent)))
       (all-joint-list (send-all all-link-list :joint))
       (gravity-wrench (scale (* 1e-3 (send robot :weight))
			      #F(0 0 -9.8)))
       (cog-jacobian
	(apply #'fast-cog-gradient-matrix
	       (append
		(list :all-joint-list all-joint-list
		      :debug? nil)))))
  (scale -1 (transform (transpose cog-jacobian) gravity-wrench)))
