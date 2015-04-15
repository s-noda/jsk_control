;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

;;(require "my-util.l")

(defvar *log-stream* t)


(defun make-trapezoid
  (bottom-points
   top-points
   &rest args
   &key
   (primitive (list ':trapezoid bottom-points top-points))
   &allow-other-keys)
  (let ((bottom-verts (mapcar #'list bottom-points))
	(top-verts (mapcar #'list top-points))
	faces)
    (push (make-face-from-vertices (nreverse top-verts))
	  faces)
    (send (first faces) :id (list :top))
    (nconc faces
	   (geometry::make-side-faces
	    top-verts (reverse bottom-verts)))
    (push (make-face-from-vertices  bottom-verts) faces)
    (send (first faces) :id (list :bottom))
    (send-lexpr (instantiate *body-class*) :init
		:faces faces
		:approximated t
		:primitive primitive
		args)))


(defclass simple-contact-state
  :super object
  :slots (name
	  contact-coords
	  contact-n
	  contact-plane-obj
	  ;;
	  slip-matrix
	  force0
	  default-target-coords
	  target-coords
	  z-rot
	  ;;
	  eval-scale
	  gain
	  f/fmax
	  ;;
	  translation-axis
	  rotation-axis
	  ;;
	  friction-cone-object
	  force-vector-object
	  ;;
	  target-direction
	  sequence-select
	  fz-max
	  ;;
	  buf
	  ))

(defmethod simple-contact-state
  (:init
   (&key
    ((:name nm) :rarm)
    ((:contact-coords cc) (send *robot* nm :end-coords))
    ((:contact-n cn) #F(0 -1 0))
    ((:contact-plane-obj cpo) nil)
    (ux 0.3) (uy 0.3) (uz 0.1) (lx 0.05) (ly 0.05)
    ((:slip-matrix sm) (send self :gen-slip-matrix ux uy uz lx ly))
    ((:force0 f0) #F(80 80 80 20 20 20))
    ((:target-coords tc) (copy-object (send cc :worldcoords)))
    ((:default-target-coords ts) tc)
    ((:gain g) '(1 1 10))
    ((:translation-axis ta) t)
    ((:rotation-axis ra) t)
    ((:target-direction td)
     '(lambda (tc ntc self &rest args) (aref (v- (send tc :worldpos) (send ntc :worldpos)) 2)))
    ((:sequence-select ss) nil)
    ((:fz-max fm) (if (find nm '(:rarm :larm))
		      (float-vector 0 0 1e+5 0 0 0) #F(0 0 1e+5 0 0 0)))
    ((:eval-scale es) 1.0)
    &allow-other-keys)
   (setq name nm)
   ;; (send-super :name nm)
   (setq contact-coords cc)
   (setq contact-n cn)
   (setq contact-plane-obj cpo)
   (setq slip-matrix sm)
   (setq force0 f0)
   (setq default-target-coords ts)
   (setq target-coords tc)
   (setq eval-scale es)
   (setq gain g)
   (setq translation-axis ta)
   (setq rotation-axis ra)
   (setq target-direction td)
   (setq sequence-select ss)
   (setq fz-max fm)
   )
  (:fix-coords
   nil
   (setq target-coords (copy-object (send contact-coords :worldcoords))))
  (:copy
   (&rest args)
   (eval
    (append
     '(instance simple-contact-state :init)
     (apply
      #'append
      (mapcar
       #'(lambda (sym)
	   (let ((key (read-from-string (format nil ":~A" sym))))
	     (cond
	      ((find key args)
	       (list key (cadr (member key args))))
	      (t (list key sym)))))
       (mapcar #'car (send self :slots)))))))
  (:gen-slip-matrix
   (ux uy uz lx ly)
   (make-matrix 6 6
		(list (list 0 0 ux 0 0 0)
		      (list 0 0 uy 0 0 0)
		      ;(coerce fz-max cons)
		      (list 0 0 1e+5 0 0)
		      (list 0 0 ly 0 0 0)
		      (list 0 0 lx 0 0 0)
		      (list 0 0 uz 0 0 0))))
  (:z-rot
   (&optional z)
   (or
    (setq z-rot z)
    ;;(m*
    ;;(send contact-coords :worldrot)
    ;;(matrix-exponent
    ;;(normalize-vector (v* (float-vector 0 0 1) contact-n))
    ;;(acos (v. (float-vector 0 0 1) contact-n))))))
    (let ((n (transform (send contact-coords :worldrot)
     			contact-n)))
      ;;(transpose
      (matrix-exponent
       (normalize-vector (v* (float-vector 0 0 1) n))
       (acos (v. (float-vector 0 0 1) n))))))
  (:z-coords
   nil
   (make-coords :pos (send contact-coords :worldpos)
		:rot (transpose (send self :z-rot))))
;; 		(m*
;; 		      (send contact-coords :worldrot)
;; ;		      (send (make-coords) :worldrot)
;; 		      (matrix-exponent
;; 		       (normalize-vector (v* #F(0 0 1) contact-n))
;; 		       (acos (v. #F(0 0 1) contact-n))))))
  (:force-error
   (f)
   (if (< (norm f) 1e-3)
       (scale 0 f)
     (map float-vector
	  #'(lambda (f fmax) (/ f (if (zerop fmax) 1e-5 fmax)))
	  f (v+ force0
		;;(if (minusp (aref (v+ force0 f) 2))
		;;(progn
		;; (format *log-stream*
		;; " minus force detected in contact-state :force-error ~A~%"
		;; f)
		;; (scale 0 f))
		(transform slip-matrix f)))))
  (:force-error-world
   (fw)
   (let* (;; (n (transform (send contact-coords :worldrot) contact-n))
	  ;; (llt
	  ;; (matrix-exponent
	  ;; (normalize-vector (v* #F(0 0 1) n))
	  ;; (acos (v. #F(0 0 1) n))))
	  (llt (send self :z-rot))
	  (fl (concatenate float-vector
			   (transform (transpose llt) (subseq fw 0 3))
			   (transform (transpose llt) (subseq fw 3 6))))
	  (fe (send self :force-error fl))
	  (ret (concatenate float-vector
			    (transform llt (subseq fe 0 3))
			    (transform llt (subseq fe 3 6)))))
     (setq f/fmax fe)
     ret))
  (:force-check
   (f
    &optional (f-error (send self :force-error f)))
   (< (apply #'max (mapcar #'abs (coerce f-error cons))) 1.01))
  (:clear nil
	  (mapcar
	   #'(lambda (d)
	       (if (and (class (cdr d)) (subclassp (class (cdr d)) simple-contact-state))
		   (send self :buf (car d) nil)))
	   (send self :buf))
	  (setq contact-plane-obj
		(if (and contact-plane-obj (class contact-plane-obj)
			 (find-method contact-plane-obj :name))
		    (or (send contact-plane-obj :name) contact-plane-obj)
		  contact-plane-obj))
	  (setq sequence-select nil)
	  (setq friction-cone-object nil)
	  (setq force-vector-object nil))
  (:sequence-select (&rest val)
		    (if val (setq sequence-select (car val)) sequence-select))
  (:gen-friction-cone-object
   (&key (offset-pos #F(0 0 0)))
   (let* ((mat slip-matrix)
	  (coords (send contact-coords :worldcoords))
	  ;; (n (transform (send coords :worldrot) contact-n))
	  (f0 force0)
	  (len 300)
	  (bot (subseq f0 0 3))
	  (top (subseq (v+ f0 (transform mat (float-vector 0 0 len 0 0 0)))
		       0 3))
	  (bot-list
	   (reverse
	    (list
	     (float-vector (* 1 (aref bot 0)) (* 1 (aref bot 1)) 0)
	     (float-vector (* -1 (aref bot 0)) (* 1 (aref bot 1)) 0)
	     (float-vector (* -1 (aref bot 0)) (* -1 (aref bot 1)) 0)
	     (float-vector (* 1 (aref bot 0)) (* -1 (aref bot 1)) 0))))
	  (top-list
	   (reverse
	    (list
	     (float-vector (* 1 (aref top 0)) (* 1 (aref top 1)) len)
	     (float-vector (* -1 (aref top 0)) (* 1 (aref top 1)) len)
	     (float-vector (* -1 (aref top 0)) (* -1 (aref top 1)) len)
	     (float-vector (* 1 (aref top 0)) (* -1 (aref top 1)) len))))
	  (bod (make-trapezoid bot-list top-list))
	  )
     (send bod :newcoords
	   (make-coords :pos (v+ offset-pos  (send coords :worldpos))
			:rot (send self :z-rot)))
			;;(m*
			;;(send coords :worldrot)
			;;(matrix-exponent
			;;(normalize-vector (v* #F(0 0 1) contact-n))
			;;(acos (v. #F(0 0 1) contact-n))))))
     (send bod :set-color #F(1 0 0))
     (gl::transparent bod 0.5)
     (setq friction-cone-object bod))
   )
  (:gen-force-vector-object
   (force &key (offset-pos #F(0 0 0)) (s 1) (max 500))
   (let* ((scale (+ 1 (* s (norm force))))
	  (stick (make-cylinder 15 (min max scale)))
	  (hat (make-cone (float-vector 0 0 50) 40))
	  (nf (scale (/ 1.0 scale) force))
	  bod)
     (send stick :newcoords
	   (make-coords :pos
			(v+
			 offset-pos
			 (send contact-coords :worldpos))
			:rot
			(matrix-exponent
			 (normalize-vector (v* #F(0 0 1) nf))
			 (acos (v. #F(0 0 1) nf)))))
     (send hat :newcoords
	   (copy-object (send stick :worldcoords)))
     (send hat :translate
	   (float-vector 0 0 (min max scale))
	   :local)
     (setq bod (body+ hat stick))
     (send bod :set-color #F(0 0 1))
     (setq force-vector-object bod)))
  (:difference-position
   nil
   (send contact-coords :difference-position target-coords
	 :translation-axis translation-axis))
  (:difference-rotation
   nil
   (send contact-coords :difference-rotation target-coords
	 :rotation-axis translation-axis))
  (:buf (&rest args)
	(cond
	 ((null args) buf)
	 ((= 1 (length args)) (cdr (assoc (car args) buf)))
	 (t (setq buf
		  (cons (cons (car args) (cadr args))
			(remove-if #'(lambda (kv) (eq (car kv) (car args))) buf))))))
  (:nomethod
   (&rest args)
   (cond
    ((and (car args)
	  (find-method (car args) :pname)
	  (setq args
		(find-if #'(lambda (a)
			     (and
			      (car a)
			      (find-method (car a) :pname)
			      (stringp (send (car a) :pname))
			      (string= (send (car a) :pname)
				       (send (car args) :pname))))
			 (send self :slots))))
     (cdr args))
    (t nil)))
  (:best-n (brli) (transform (send target-coords :worldrot) #F(0 0 1)))
  (:best-x (x) (send target-coords :worldpos))
  (:eval
   (now-cs now-rsd
	      &key
	      (now-tc
	       (if (find-method now-cs :target-coords)
		   (send now-cs :target-coords)
		 (send self :contact-coords :copy-worldcoords)))
	      ;; (x (send
	      ;; 	  (if (find-method now-cs :target-coords)
	      ;; 	      (send now-cs :target-coords)
	      ;; 	    (send self :contact-coords))
	      ;; 	  :worldpos))
	      (brli (send now-rsd :gbrli))
	      (force-zero (send *robot* :centroid)) (gain gain))
   (cond
    ((and sequence-select
	  (listp sequence-select))
     (let (can)
       (cond
	((find now-cs sequence-select)
	 (setq can (or (cadr (member now-cs sequence-select))
		       now-cs)))
	(t
	 (setq can
	       (car (sort
		     sequence-select
		     #'(lambda (s1 s2)
			 (< (+ (norm2 (send now-tc :difference-position
					    (send s1 :target-coords)))
			       (norm2 (send now-tc :difference-rotation
					    (send s1 :target-coords))))
			    (+ (norm2 (send now-tc :difference-position
					    (send s2 :target-coords)))
			       (norm2 (send now-tc :difference-rotation
					    (send s2 :target-coords)))))))))))
       (if (eq can self) 1e+10 0)))
    (t
     (let* (;; (n (send self :best-n brli))
	    (now-n (transform (send now-tc :worldrot) contact-n))
	    ;; (y (send self :best-x x))
	    (tc target-coords)
	    (n (transform (send tc :worldrot) contact-n));;(transform (send self :z-rot) #F(0 0 1)))
	    (ret (apply
		  #'+
		  (mapcar #'*
			  (if (eq (length gain) 3) (append gain (last gain)) gain)
			  (list (funcall target-direction tc now-tc self)
				;; (* -1 (norm (v- (send now-tc :worldpos) y)))
				(* -1
				   (sqrt
				    (+ (norm2 (send now-tc :difference-position tc))
				       (norm2 (send now-tc :difference-rotation tc)))))
				(v. brli
				    (concatenate
				     float-vector
				     n
				     (v* (scale 1e-3
						(v- (send tc :worldpos) force-zero))
					 n)))
				(if now-cs
				    (* -1
				       (v. brli
					   (concatenate
					    float-vector
					    now-n
					    (v* (scale 1e-3
						       (v- (send now-tc :worldpos) force-zero))
						now-n)))) 1)
				)))))
       ;;     (setq target-coords
       ;;	   (make-coords :pos y
       ;;			:rot
       ;;			(matrix-exponent
       ;;			 (normalize-vector (v* n now-n))
       ;;			 (acos (v. n now-n)))))
       ; ;(format *log-stream* "eval-contact-state ~A with ~A = ~A~%" name target-coords ret)
       (* eval-scale (exp (* 1e-3 ret)))
       ))))
  )

(defclass rotatable-contact-state
  :super simple-contact-state
  :slots (rotate-mode))
(defmethod rotatable-contact-state
  (:init
   (&rest args &key ((:rotate-mode rm) :point) &allow-other-keys)
   (setq rotate-mode rm)
   (send-super* :init args))
  (:rotation-axis
   (axis &key
	 (move-target contact-coords)
	 (target-coords target-coords))
   (let* (a0 a1)
     (cond
      ((vectorp rotate-mode)
       (setq a0 (send move-target :axis rotation-mode)
	     a1 (send target-coords :axis rotation-mode))
       (transform (transpose (send move-target :worldrot))
		  (scale (acos (v. a0 a1))
			 (normalize-vector (v* a0 a1)))
		  axis)))
     (if (< (v. (transform (send move-target :worldrot)
			   contact-n)
		(transform (send target-coords :worldrot)
			   contact-n)) 1e-6)
	 axis
       (scale 0 axis axis)))
   axis
   )
  )

(defun sequence-select-filter
  (cs-list)
  (mapcar
   #'(lambda (k)
       (let ((buf (remove-if #'(lambda (cs) (not (eq k (send cs :name))))
			     cs-list)))
	 (send-all buf :sequence-select buf)))
   (union (send-all cs-list :name) nil)))

(defun now-contact-state
  (&key
   (limb-keys '(:rarm :larm :rleg :lleg))
   (contact-coords
    (mapcar #'(lambda (k)
		(if (find k '(:rarm :larm))
		    (send *robot* k :end-coords)
		  (let ((c (send *robot* k :end-coords)))
		    (cascoords-collection
		     :limb-key k
		     :coords (copy-object (send c :worldcoords))
		     :parent-link (send (send c :parent) :parent)))))
	    limb-keys))
;    (mapcar #'(lambda (k) (send *robot* k :end-coords)) limb-keys))
   (contact-n
    (mapcar
     #'(lambda (k)
	 (case k (:rarm #F(0 -1 0)) (:larm #F(0 1 0)) (t #F(0 0 1))))
     limb-keys))
   (ux (make-list (length limb-keys) :initial-element 0.3))
   (uy (make-list (length limb-keys) :initial-element 0.3))
   (uz (make-list (length limb-keys) :initial-element 0.1))
   (lx (make-list (length limb-keys) :initial-element 0.1))
   (ly (make-list (length limb-keys) :initial-element 0.1))
   (gain (mapcar #'(lambda (k)
		     (if (find k '(:rarm :larm))
			 '(1 1.4 10)
		       '(1 10 10)))
		 limb-keys))
   (force0 (mapcar #'(lambda (k)
		       (if (find k '(:rarm :larm))
			   #F(80 80 80 20 20 20)
			   #F(0 0 0 0 0 0)))
		   limb-keys))
   (target-coords (send-all contact-coords :copy-worldcoords)))
  (mapcar
   #'(lambda (nm cc cn ux uy uz lx ly f0 tc)
       (instance simple-contact-state
		 :init
		 :name nm :contact-coords cc :contact-n cn
		 :ux ux :uy uy :uz uz :lx lx :ly ly
		 :force0 f0 :target-coords tc))
   limb-keys contact-coords contact-n
   ux uy uz lx ly force0 target-coords))

(defvar *dcsf-max-dist-scale* 1.1)
(defvar *dcsf-min-dist-scale* -0.1)
(defvar *dcsf-use-heuristic* t)
(defun default-contact-state-filter
  (cs &optional (v #F(0 0 1)) buf)
  (or
   (let ((d (norm (v- (send (send cs :target-coords) :worldpos)
		      (send (if (find-method *robot* (send cs :name))
				(send *robot* (send cs :name) :root-link)
			      (car (send *robot* :links)))
			    :worldpos)))))
     (cond
      ((and (find-method *robot* (send cs :name))
	    (send *robot* (send cs :name) :links))
       (setq
	 buf
	 (apply
	  #'+
	  (mapcar
	   #'(lambda (l1 l2) (norm (v- (send l1 :worldpos) (send l2 :worldpos))))
	   (cdr (send *robot* (send cs :name) :links))
	   (send *robot* (send cs :name) :links))))
       ;; (format *log-stream* "[default-contact-state-filter] ~A length = ~A~%"
       ;;(send cs :name) buf)
       (or (> d (* *dcsf-max-dist-scale* buf))
	   (< d (* *dcsf-min-dist-scale* buf))))
      (t (or (> d (* *dcsf-max-dist-scale* 500))
	     (< d (* *dcsf-min-dist-scale* 500))))))
   (< (apply #'min ;; collision
	     (mapcar
	      #'(lambda (j)
		  (norm (v- (send j :worldpos)
			    (send (send cs :target-coords) :worldpos))))
	      (let ((csl (send (send cs :contact-coords) :parent)))
		(remove-if #'(lambda (l)
			       (or (eq l csl)
				   (find l (send csl :child-links))
				   (find csl (send l :child-links))))
			   (send *robot* :links)))))
      100)
   (and *dcsf-use-heuristic*
	(find (send cs :name) '(:rleg :lleg)) ;; heuristic
	(or
	 (> (norm (map float-vector #'* '(1 1 0)
		       (matrix-log (send (send cs :target-coords) :worldrot))))
	    (deg2rad 45))
	 (> (v. v (send (send cs :target-coords) :worldpos))
	    (v. v (send (send *robot* (send cs :name) :knee-p) :worldpos)))))
   ;; (if (find (send cs :name) '(:rarm :larm))
   ;; 500 800
   ;; )) ;; unreachable
   ;; (< (apply #'min
   ;; 	     (mapcar
   ;; 	      #'(lambda (j)
   ;; 		  (norm (v- (send j :worldpos)
   ;; 			    (send (send cs :target-coords) :worldpos))))
   ;; 	      (flatten
   ;; 	       (mapcar #'(lambda (k) (send *robot* k :joint-list))
   ;; 		       (remove (send cs :name) '(:rleg :lleg :rarm :larm))))))
   ;;    200)
   ))

(defun contact-states-filter-check
  (&key (contact-states *contact-states*)
	(contact-state-filter 'default-contact-state-filter)
	(accept-log) (reject-log)
	(accept-cs) (reject-cs)
	(accept-draw? nil)
	(func '(lambda (cs log)
		 (cond
		  ((assoc (send cs :name) log)
		   (setf (aref (cdr (assoc (send cs :name) log)) 0)
			 (+ (aref (cdr (assoc (send cs :name) log)) 0) 1)))
		  (t
		   (setq log (cons (cons (send cs :name) (float-vector 1))
				   log)))
		  )
		 log)))
  (if accept-draw? (send *viewer* :draw-objects))
  (mapcar
   #'(lambda (cs)
       (cond
	((funcall contact-state-filter cs)
	 (push cs reject-cs)
	 (setq reject-log (funcall func cs reject-log)))
	(t
	 (if (or (and (functionp accept-draw?)
		      (funcall accept-draw? cs))
		 (and (not (functionp accept-draw?))
		      accept-draw?))
	     (send (send cs :target-coords) :draw-on :flush t))
	 (push cs accept-cs)
	 (setq accept-log (funcall func cs accept-log)))))
   contact-states)
  (format t "[contact-state-candidates-check]~%")
  (format t " accept: ~A~%" accept-log)
  (format t " reject: ~A~%" reject-log)
  (list (list :accept accept-log accept-cs)
	(list :reject reject-log reject-cs))
  )

;; tmp overwrite
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
     (cond
      ((vectorp translation-axis)
       (let ((ra
	      (transform
	       (inverse-matrix (send self :worldrot))
	       translation-axis)))
	 (scale (v. dif-pos ra) ra)))
      ((and (class translation-axis)
	    (find-method translation-axis :translation-axis))
       (funcall translation-axis :translation-axis dif-pos))
      (t dif-pos))))
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
      (cond
       ((and (class rotation-axis)
	     (find-method rotation-axis :rotation-axis))
	(funcall rotation-axis :rotation-axis dif-rot))
       (t dif-rot))
      )))
  )

(defun face-circle-split
  (face
   &key
   (outer-vl (butlast (send face :vertices)))
   (vl outer-vl)
   (face-step 30)
   (area (send face :area))
   (min-face-area (* face-step face-step))
   (c (if (> (length vl) 1)
	  (scale (/ 1.0 (length vl)) (reduce #'v+ vl))))
   (depth 0)
   (max-depth 5)
   ;; ret
   )
  (cond
   ((or (> depth max-depth)
	(< (length vl) 2)
	(< area min-face-area)
	(flatten
	 (mapcar
	  #'(lambda (a b) ;; n = a-c+((c-a)(b-a)/(b-a)(b-a))(b-a)
	      (< (norm (v+ (v- a c)
			   (scale (/ (v. (v- c a) (v- b a))
				     (max (v. (v- b a) (v- b a))
					  1e-6))
				  (v- b a))))
		 face-step))
	  outer-vl (append (last outer-vl) outer-vl))))
    nil)
   (t
    (cons
     c
     (apply
      #'append
      (mapcar
       #'(lambda (a b)
	   (face-circle-split
	    face
	    :area
	    (* (norm (v- b a))
	       (norm
		(v+ (v- a c)
		    (scale (/ (v. (v- c a) (v- b a))
			      (max (v. (v- b a) (v- b a))
				   1e-6))
			   (v- b a)))))
	    :outer-vl outer-vl
	    :vl (list a b c)
	    :face-step face-step
	    :depth (+ depth 1)
	    :max-depth max-depth
	    ;; :ret ret)
	    ))
       vl (append (last vl) vl)))))))

(defun cylinder-split
  (cylinder
   &key
   (top-pos
    (send (send cylinder :find-named-face :top) :worldpos))
   (bot-pos
    (send (send cylinder :find-named-face :bottom) :worldpos))
   (c-pos (scale 0.5 (v+ top-pos bot-pos)))
   (radius
    (sqrt (/ (send cylinder :volume)
	     (* 3.14 (norm (v- top-pos bot-pos))))))
   (grasp-min-radius 10)
   (grasp-max-radius 40)
   (grasp-step 100)
   (depth 0)
   (max-depth 5)
   )
  ;; (format t "[cylider-split] radius: ~A < ~A < ~A~%"
  ;; grasp-min-radius radius grasp-max-radius)
  (cond
   ((or (> depth max-depth)
	(< (norm (v- top-pos bot-pos)) grasp-step)
	(> radius grasp-max-radius)
	(< radius grasp-min-radius))
    nil)
   (t
    (cons
     c-pos
     (append
      (cylinder-split cylinder
		      :top-pos top-pos
		      :bot-pos c-pos
		      :radius radius
		      :grasp-min-radius grasp-min-radius
		      :grasp-max-radius grasp-max-radius
		      :grasp-step grasp-step
		      :depth (+ depth 1)
		      :max-depth max-depth
		      )
      (cylinder-split cylinder
		      :top-pos c-pos
		      :bot-pos bot-pos
		      :radius radius
		      :grasp-min-radius grasp-min-radius
		      :grasp-max-radius grasp-max-radius
		      :grasp-step grasp-step
		      :depth (+ depth 1)
		      :max-depth max-depth
		      ))))))

(defun gen-contact-state-for-face-contact
  (pos
   n
   &key
   (name :rarm)
   (contact-coords (send *robot* name :end-coords))
   (contact-n (float-vector 0 -1 0))
   (contact-parameters
    (list :force0 (float-vector 0 0 0 0 0 0)
	  :ux 0.3 :uy 0.3 :uz 0.3 :lx 0.05 :ly 0.1))
   (rotate-cnt 8)
   (rotate-step (/ 360 rotate-cnt))
   (rotate-id-list
    (let* ((buf) (id -1))
      (dotimes (i rotate-cnt) (push (incf id) buf))
      buf))
   (buf
    (list
     n
     ;;(transform (send contact-coords :worldrot)
     contact-n))
   (target-coords
    (make-coords :pos (copy-seq pos)
		 :rot
		 (transpose
		  (matrix-exponent
		   (normalize-vector (apply #'v* buf))
		   (acos (apply #'v. (reverse buf)))))))
   ret
   )
  (dolist (i rotate-id-list)
    (push
     (instance* simple-contact-state :init :name name
		:contact-coords contact-coords
		:contact-n contact-n
		:target-coords
		(make-coords
		 :pos (copy-seq (send target-coords :worldpos))
		 :rot (m* (send target-coords :worldrot)
			  (matrix-exponent
			   (scale (deg2rad (* i rotate-step)) contact-n))))
		contact-parameters)
     ret)
    )
  ;; (dotimes (i rotate-cnt)
  ;;   (push
  ;;    (instance* simple-contact-state :init :name name
  ;; 		:contact-coords contact-coords
  ;; 		:contact-n contact-n
  ;; 		:target-coords target-coords
  ;; 		contact-parameters)
  ;;    ret)
  ;;   (setq target-coords
  ;; 	  (make-coords
  ;; 	   :pos (copy-seq (send target-coords :worldpos))
  ;; 	   :rot (m* (send target-coords :worldrot)
  ;; 		    (matrix-exponent
  ;; 		     (scale (deg2rad rotate-step) contact-n))))))
  ret)

(defun gen-contact-state-for-grasp-contact
  (pos dir
   &key
   (name :rarm)
   (contact-coords (send *robot* name :end-coords))
   (contact-n
    (if (eq name :rarm) (float-vector 0 -1 0) (float-vector 0 1 0)))
   (contact-z (normalize-vector (float-vector -0.2 0 0.8)))
   (contact-parameters
    (list :force0 (float-vector 0 0 0 0 0 0)
	  :ux 0.3 :uy 0.3 :uz 0.3 :lx 0.05 :ly 0.1))
   (n dir);; (let* ((v (normalize-vector (v* dir #F(1 0 0)))))
      ;; 	(if (< (norm v) 1e-3)
      ;; 	    (setq v (normalize-vector (v* dir #F(0 1 0)))))
      ;; 	v))
   (rotate-cnt 8)
   (rotate-step (/ 360 rotate-cnt))
   (rotate-id-list
    (let* ((buf) (id -1))
      (dotimes (i rotate-cnt) (push (incf id) buf))
      buf))
   (buf
    (list
     n
     ;; (transform (send contact-coords :worldrot)
     contact-z))
   (target-coords
    (make-coords :pos (copy-seq pos)
		 :rot
		 (transpose
		  (matrix-exponent
		   (normalize-vector (apply #'v* buf))
		   (acos (apply #'v. (reverse buf)))))))
   ret
   )
  (dolist (i rotate-id-list)
    (push
     (instance* simple-contact-state :init :name name
		:contact-coords contact-coords
		:contact-n contact-n
		:target-coords
		(make-coords
		 :pos (copy-seq (send target-coords :worldpos))
		 :rot (m* (send target-coords :worldrot)
			  (matrix-exponent
			   (scale (deg2rad (* i rotate-step)) contact-z))))
		contact-parameters)
     ret)
    )
  ;; (dotimes (i rotate-cnt)
  ;;   (push
  ;;    (instance* simple-contact-state :init :name name
  ;; 		:contact-coords contact-coords
  ;; 		:contact-n contact-n
  ;; 		:target-coords target-coords
  ;; 		contact-parameters)
  ;;    ret)
  ;;   (setq target-coords
  ;; 	  (make-coords
  ;; 	   :pos (copy-seq (send target-coords :worldpos))
  ;; 	   :rot (m* (send target-coords :worldrot)
  ;; 		    (matrix-exponent
  ;; 		     (scale (deg2rad rotate-step) contact-z))))))
  ret)

(defun gen-primitive-contact-states
  (obj
   &key
   (grasp-contact-name '(:rarm :larm))
   (grasp-cascoords
    (mapcar
     #'(lambda (k)
	 (if (find k '(:rarm :larm))
	     (send *robot* k :end-coords)
	   (cascoords-collection
	    :limb-key k
	    :coords (make-coords
		     :pos
		     (v+ #F(80 0 0)
			 (send *robot* k :end-coords :worldpos))
		     :rot
		     (copy-object (send *robot* k :end-coords :worldrot))))))
     grasp-contact-name))
   (place-contact-name '(:rleg :lleg))
   (place-cascoords
    (mapcar
     #'(lambda (k)
	 (if (find k '(:rarm :larm))
	     (send *robot* k :end-coords)
	   (cascoords-collection
	    :limb-key k
	    :coords (make-coords
		     :pos
		     (v+ #F(80 0 0)
			 (send *robot* k :end-coords :worldpos))
		     :rot
		     (copy-object (send *robot* k :end-coords :worldrot))))))
     ;;(send *robot* k :end-coords))
     place-contact-name))
   (grasp-n-vector
    (mapcar #'(lambda (n)
		(cond
		 ((eq n :rarm) (float-vector 0 -1 0))
		 ((eq n :larm) (float-vector 0 1 0))
		 (t (float-vector 0 0 1))))
	    grasp-contact-name))
   (grasp-z-vector
    (mapcar #'(lambda (a)
		(normalize-vector (float-vector -0.3 0 1)))
	    grasp-cascoords))
   (grasp-contact-parameters
    (mapcar
     #'(lambda (k)
	 (list :force0 #F(50 50 -10 5 5 5)
	       :ux 0.7 :uy 0.7 :uz 0.1 :lx 0.04 :ly 0.1))
     grasp-contact-name))
   (place-n-vector
    (mapcar #'(lambda (n)
		(cond
		 ((eq n :rarm) (float-vector 0 -1 0))
		 ((eq n :larm) (float-vector 0 1 0))
		 (t (float-vector 0 0 1))))
	    place-contact-name))
   (place-contact-parameters
    (mapcar
     #'(lambda (k)
	 (list :force0 (float-vector 0 0 0 0 0 0)
	       :ux 0.3 :uy 0.3 :uz 0.3 :lx 0.05 :ly 0.1))
     place-contact-name))
   ;;
   (grasp-extra-args (make-list (length grasp-contact-name)))
   (place-extra-args (make-list (length place-contact-name)))
   ;;
   (bodies
    (union
     (remove-if
      #'(lambda (a) (not (and (class a) (subclassp (class a) body))))
      (flatten (send obj :slots)))
     nil))
   (cylinders
    (remove-if
     #'(lambda (a)
	 (not (substringp "(:cylinder" (format nil "~A" a))))
     bodies))
   (prisms
    (remove-if
     #'(lambda (a)
	 (not (or (substringp "(:prism" (format nil "~A" a))
		  (substringp "(:cube" (format nil "~A" a)))))
     bodies))
   ;; (log1 (format t "log1: bodies(~A) >= prism(~A) + cylidner(~A)~%"
   ;; (length bodies) (length prisms) (length cylinders)))
   ;;
   (max-depth 5)
   (face-step 30)
   (face-contact-candidate-points
    (mapcar
     #'(lambda (obj)
	 (mapcar
	  #'(lambda (face)
	      (send face :put :contact-candidates
		    (face-circle-split face :face-step face-step :max-depth max-depth)))
	  (send obj :faces)))
     prisms))
   ;; (log2 (format t "log2: face split done~%"))
   ;;
   (grasp-min-radius 10)
   (grasp-max-radius 40)
   (grasp-step 100)
   (grasp-contact-candidate-points
    (mapcar
     #'(lambda (obj)
	 (send obj :put :grasp-contact-candidates
	       (cylinder-split
		obj :grasp-min-radius grasp-min-radius
		:grasp-max-radius grasp-max-radius
		:grasp-step grasp-step
		:max-depth max-depth)))
     cylinders))
   ;; (log3 (format t "log3: grasp split done~%"))
   ;;
   (draw? t)
   )
  (list
   (flatten
    (mapcar
     #'(lambda (obj)
	 (mapcar
	  #'(lambda (face)
	      ;; draw
	      (if draw?
		  (send-all (send face :get :contact-candidates) :draw-on :flush t :color #F(1 0 0) :width 1))
	      (mapcar
	       #'(lambda (p)
		   (if t ;;(not (check-point-collision p))
		       (mapcar #'(lambda (name cs n param ext)
				   (apply
				    #'gen-contact-state-for-face-contact
				    (append
				     (list p (send face :normal)
					   :name name :contact-coords cs
					   :contact-n n
					   :contact-parameters param)
				     ext)))
			       place-contact-name place-cascoords
			       place-n-vector place-contact-parameters
			       place-extra-args)))
	       (send face :get :contact-candidates)))
	  (send obj :faces)))
     prisms))
   ;; (format t "log4: face contact generation done~%")
   (flatten
    (mapcar
     #'(lambda (obj)
	 ;; draw
	 (cond
	  ((not (send obj :get :no-grasp))
	   (if draw?
	       (send-all (send obj :get :grasp-contact-candidates) :draw-on :flush t :color #F(0 1 0) :width 1))
	   (mapcar
	    #'(lambda (p)
		(mapcar #'(lambda (name cs n z param ext)
			    (apply
			     #'gen-contact-state-for-grasp-contact
			     (append
			      (list p
				    (normalize-vector
				     (v-
				      (send (send obj :find-named-face :top)
					    :worldpos)
				      (send (send obj :find-named-face :bottom)
				     :worldpos)))
				    :name name :contact-coords cs
				    :contact-n n
				    :contact-z z
				    :contact-parameters param)
			      ext)))
			grasp-contact-name grasp-cascoords
			grasp-n-vector grasp-z-vector
			grasp-contact-parameters grasp-extra-args))
	    (send obj :get :grasp-contact-candidates)))))
     cylinders)
    ;; (format t "log5: grasp contact generation done~%")
    )))


;(defvar *robot* (hrp2jsknts-simple-detail))
;(defvar *sample-contact-states* (now-contact-state))

#|

(defun pm
  (m)
  (format t "#2f~%")
  (dotimes (i (length (matrix-column m 0)))
    (format t "|")
    (dotimes (j (length (matrix-row m 0)))
      (format t "~A~0,2f " (if (minusp (aref m i j)) "-" "+") (abs (aref m i j))))
    (format t "|~%")))

(in-package "GEOMETRY")

