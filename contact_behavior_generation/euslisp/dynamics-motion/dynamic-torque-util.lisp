;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "../robot-state-data2.l")
(require "../contact-state.l")
(require "../math/matrix-util.l")

(defmethod bodyset-link
  (:force (&optional f) (if f (setq force f) force))
  (:moment (&optional m) (if m (setq moment m) moment))
  (:angular-velocity (&optional av)
		     (if av (setq angular-velocity av) angular-velocity))
  (:angular-acceleration (&optional aa)
			 (if aa (setq angular-acceleration aa) angular-acceleration))
  (:spacial-velocity (&optional sv)
		     (if sv (setq spacial-velocity sv) spacial-velocity))
  (:spacial-acceleration (&optional sa)
			 (if sa (setq spacial-acceleration sa) spacial-acceleration))
  )

(defmethod robot-model
  (:torque-vector2
   (&key (contact-states
	  (mapcar
	   #'(lambda (k)
	       (list
		(cons :link (send self k :end-coords :parent))
		(cons :worldcoords
		      (copy-object (send self k :end-coords :worldcoords)))
		(cons :ext-force (float-vector 0 0 0))
		(cons :ext-moment (float-vector 0 0 0))))
	   '(:rarm :larm :rleg :lleg)))
;	 (gravity; #F(0 0 0))
;	  (scale (* 1e-3 9.8 (send self :weight))
;		 #F(0 0 -1)))
	 (root-angular-velocity (float-vector 0 0 0))
	 (root-spacial-velocity (float-vector 0 0 0))
	 (root-angular-acceleration (float-vector 0 0 0))
	 (root-spacial-acceleration (float-vector 0 0 0))
	 (root-link (car (send self :links)))
         ret
	 (debug-view nil)
         (jvv (instantiate float-vector (calc-target-joint-dimension joint-list)))
         (jav (instantiate float-vector (calc-target-joint-dimension joint-list)))
         (calc-torque-buffer-args (send self :calc-torque-buffer-args))
	 (debug? nil))
;   (send (car (send self :links)) :force gravity)
   (mapcar #'(lambda (cs)
               (send (cdr (assoc :link cs)) :ext-force
		     (or (cdr (assoc :ext-force cs)) (float-vector 0 0 0)))
               (let* ((moment-offset
		       (v* (scale 1e-3
				  (send (cdr (assoc :worldcoords cs))
					:worldpos))
			   (or (cdr (assoc :ext-force cs)) (float-vector 0 0 0)))))
		 (send (cdr (assoc :link cs)) :ext-moment
		       (v+ moment-offset (or (cdr (assoc :ext-moment cs))
					     (float-vector 0 0 0))))))
	   contact-states)
   (setq
    ret
    (send self :calc-torque-from-vel-acc
	  :debug-view debug-view
	  :root-angular-acceleration root-angular-acceleration
	  :root-angular-velocity root-angular-velocity
	  :root-spacial-acceleration root-spacial-acceleration
	  :root-spacial-velocity root-spacial-velocity
	  :jvv jvv
	  :jav jav
	  :calc-torque-buffer-args calc-torque-buffer-args))
   (cond
    ((and debug?
	  (>
	   (norm
	    (concatenate
	     float-vector
	     (send root-link :force)
	     (send root-link :moment)))
	   1))
     (format t "[torque-vector2] root force detected ~A >1~%"
	     (concatenate
	      float-vector
	      (send root-link :force)
	      (send root-link :moment)))))
   ret
   )
  )

;; (eval
;;  (list
;;   'defmethod
;;   'hrp2
;;   (cadr (find-method (instance robot-model) :torque-vector2))))

(defun calc-jacobian-fill0
  (&key
   (robot *robot*)
   (all-links (cdr (send robot :links)))
   (move-target (send robot :rleg :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (transform-coords (make-coords))
   (J (send robot :calc-jacobian-from-link-list
	    link-list
	    :move-target move-target
	    :transform-coords transform-coords
	    :translation-axis '(t)
	    :rotation-axis '(t)))
   (zero-vector #F(0 0 0 0 0 0))
   (index 0)
   )
  (make-matrix
   (length all-links)
   (length zero-vector)
   (mapcar
    #'(lambda (l)
	(if (setq index (position l link-list))
	    (matrix-column J index)
	  zero-vector))
    all-links)))

(defun calc-dd-matrix ;; calc H of Hddq + b = t + Jf
  (&key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (root-pos (send (send root-link :worldcoords) :transform-vector
		   (send root-link :get-val 'acentroid)))
   (all-links (remove root-link (send robot :links)))
   (tau-target (instantiate float-vector (length all-links)))
   (dq (scale 0 (send robot :angle-vector)))
   (q (send robot :angle-vector))
   (contact-states
    (mapcar
     #'(lambda (k)
	 (list
	  (cons :f0 #F(0 0 0))
	  (cons :m0 #F(0 0 0))
	  (cons :link (send robot k :end-coords :parent))
	  (cons :worldcoords
		(copy-object
		 (send robot k :end-coords :worldcoords)))))
     '(:rleg :lleg)))
   (root-angular-velocity #F(0 0 0))
   (root-spacial-velocity #F(0 0 0))
   (base-torque? nil)
   ;;  buf
   (ddq (scale 0 (send robot :angle-vector)))
   (root-angular-acceleration #F(0 0 0))
   (root-spacial-acceleration #F(0 0 0))
   ddq-coeff dv-coeff b
   )
  (send robot :angle-vector q)
  (send robot
	:torque-vector2
	:contact-states contact-states
	:root-angular-velocity root-angular-velocity
	:root-spacial-velocity root-spacial-velocity
	:jvv dq)
  (setq b
	(append
	 (send-all (send-all all-links :joint) :joint-torque)
	 (if base-torque?
	     (concatenate
	      cons
	      (send root-link :force)
	      (v- (send root-link :moment)
		  (scale
		   1e-3
		   (v* root-pos
		       (send root-link :force))))))))
  (dotimes (i (length ddq))
    (if (>= (- i 1) 0) (setf (aref ddq (- i 1)) 0))
    (setf (aref ddq i) 1)
    (send robot
	  :torque-vector2
	  :contact-states contact-states
	  :root-angular-velocity root-angular-velocity
	  :root-spacial-velocity root-spacial-velocity
	  :jvv dq
	  :jav ddq)
    (push
     (mapcar
      #'-
      (append
       (send-all (send-all all-links :joint) :joint-torque)
       (if base-torque?
	   (concatenate
	    cons
	    (send root-link :force)
	    (v- (send root-link :moment)
		(scale
		 1e-3
		 (v* root-pos
		     (send root-link :force)))))))
      b)
     ddq-coeff)
    )
  (setf (aref ddq (- (length ddq) 1)) 0)
  (setq ddq-coeff
	(transpose (make-matrix (length ddq) (length (car ddq-coeff))
				(reverse ddq-coeff))))
  ;;
  (dotimes (i 3)
    (if (>= (- i 1) 0) (setf (aref root-spacial-acceleration (- i 1)) 0))
    (setf (aref root-spacial-acceleration i) 1)
    (send robot
	  :torque-vector2
	  :contact-states contact-states
	  :root-angular-velocity root-angular-velocity
	  :root-spacial-velocity root-spacial-velocity
	  :root-spacial-acceleration root-spacial-acceleration
	  :jvv dq)
    (push
     (mapcar
      #'-
      (append
       (send-all (send-all all-links :joint) :joint-torque)
       (if base-torque?
	   (concatenate
	    cons
	    (send root-link :force)
	    (v- (send root-link :moment)
		(scale
		 1e-3
		 (v* root-pos
		     (send root-link :force)))))))
      b)
     dv-coeff))
  (setf (aref root-spacial-acceleration 2) 0)
  (dotimes (i 3)
    (if (>= (- i 1) 0) (setf (aref root-angular-acceleration (- i 1)) 0))
    (setf (aref root-angular-acceleration i) 1)
    (send robot
	  :torque-vector2
	  :contact-states contact-states
	  :root-angular-velocity root-angular-velocity
	  :root-spacial-velocity root-spacial-velocity
	  :root-angular-acceleration root-angular-acceleration
	  :jvv dq)
    (push
     (mapcar
      #'-
      (append
       (send-all (send-all all-links :joint) :joint-torque)
       (if base-torque?
	   (concatenate
	    cons
	    (send root-link :force)
	    (v- (send root-link :moment)
		(scale
		 1e-3
		 (v* root-pos
		     (send root-link :force)))))))
      b)
     dv-coeff))
  (setf (aref root-angular-acceleration 2) 0)
  (setq dv-coeff (transpose (make-matrix 6 (length (car dv-coeff))
					 (reverse dv-coeff))))
  (list (cons :ddq-coeff ddq-coeff) (cons :dv-coeff dv-coeff))
  )

(defun calc-torque-eq
  ;;
  ;; [I,JT] [t;f;m] = ID(0,0,0)
  ;; if ! base-torque?; then
  ;;   [0,G] [t;f;m] = [0,0,0,0,0,0]
  ;;
  (&key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (root-pos (send (send root-link :worldcoords) :transform-vector
		   (send root-link :get-val 'acentroid)))
   (all-links (remove root-link (send robot :links))) ;; without root-link
   (tau-target (instantiate float-vector (length all-links)))
   (ddq (scale 0 (send robot :angle-vector)))
   (dq (scale 0 (send robot :angle-vector)))
   (q (send robot :angle-vector))
   (contact-states
    (mapcar
     #'(lambda (k)
	 (list
	  (cons :f0 #F(0 0 0))
	  (cons :m0 #F(0 0 0))
	  (cons :link (send robot k :end-coords :parent))
	  (cons :worldcoords
		(copy-object
		 (send robot k :end-coords :worldcoords)))))
     '(:rleg :lleg)))
   (root-angular-velocity #F(0 0 0))
   (root-spacial-velocity #F(0 0 0))
   (root-angular-acceleration #F(0 0 0))
   (root-spacial-acceleration #F(0 0 0))
   (base-torque? nil)
   tau0 fb0 mb0 IJ J G OG force-target move-target
   )
  (send robot :angle-vector q)
  (send robot
	:torque-vector2
	:contact-states contact-states
	:root-angular-velocity root-angular-velocity
	:root-spacial-velocity root-spacial-velocity
	:root-angular-acceleration root-angular-acceleration
	:root-spacial-acceleration root-spacial-acceleration
	:jvv dq
	:jav ddq)
  (setq tau0
	(coerce
	 (send-all (send-all all-links :joint) :joint-torque)
	 float-vector))
  (setq fb0
	(send root-link :force))
  (setq mb0
	(v- (send root-link :moment)
	    (scale 1e-3 (v* root-pos fb0))))
  (setq force-target
	(apply #'concatenate
	       (cons float-vector
		     (flatten
		      (mapcar
		       #'(lambda (cs)
			   (list (or (cdr (assoc :f0 cs)) #F(0 0 0))
				 (or (cdr (assoc :m0 cs)) #F(0 0 0))))
		       contact-states)))))
  ;;
  (setq move-target
	(mapcar
	 #'(lambda (l)
	     (make-cascoords
	      :init :link-list
	      :parent (cdr (assoc :link l))
	      :coords (cdr (assoc :worldcoords l))))
	 contact-states))
  (setq
   J
   (mapcar
    #'(lambda (l mt)
	(calc-jacobian-fill0
	 :robot robot
	 :all-links all-links
	 :move-target mt
	 :link-list (send robot :link-list (cdr (assoc :link l)))))
    contact-states move-target))
  (setq
   J
   (matrix-append
    (if base-torque?
	(mapcar
	 #'(lambda (J cs)
	     (matrix-append
	      (list J
		    (send robot :calc-grasp-matrix
			  (list
			   (v-
			    (send (cdr (assoc :worldcoords cs)) :worldpos)
			    root-pos))))
	      '(1 0)))
	 J contact-states)
      J)
    '(0 1)))
  (mapcar #'(lambda (mt) (send (send mt :parent) :dissoc mt)) move-target)
  (setq IJ
	(matrix-append
	 (list
	  (unit-matrix (+ (length all-links)
			  (if base-torque? 6 0)))
	  J)
	 '(0 1)))
  ;;
  (if (not base-torque?)
      (setq OG
	    (matrix-append
	     (list
	      (make-matrix 6 (length all-links))
	      (setq
	       G
	       (send robot :calc-grasp-matrix
		     (mapcar
		      #'(lambda (l)
			  (v- (send (cdr (assoc :worldcoords l))
				    :worldpos)
			      root-pos))
		      contact-states))))
	     '(0 1))))
  ;;
  (list
   (cons :tau0 tau0) (cons :fb0 fb0) (cons :mb0 mb0)
   (cons :tau-target tau-target) (cons :force-target force-target)
   (cons :IJ IJ) (cons :J J) (cons :OG OG) (cons :G G)))

(defun calc-torque-check
  (&key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (all-links (remove root-link (send robot :links)))
   (ddq (scale 0 (send robot :angle-vector)))
   (dq (scale 0 (send robot :angle-vector)))
   (q (send robot :angle-vector))
   (root-angular-velocity #F(0 0 0))
   (root-spacial-velocity #F(0 0 0))
   (root-angular-acceleration #F(0 0 0))
   (root-spacial-acceleration #F(0 0 0))
   (contact-states
    (mapcar
     #'(lambda (k)
	 (list
	  (cons :link (send robot k :end-coords :parent))
	  (cons :worldcoords
		(copy-object
		 (send robot k :end-coords :worldcoords)))))
     '(:rleg :lleg)))
   (weight-vector
    (apply #'concatenate
	   (cons float-vector
		 (append
		  (list (make-list (length all-links) :initial-element 1))
		  (mapcar
		   #'(lambda (cs)
		       (cond
			((find (cdr (assoc :link cs))
			       (append
				(send robot :rarm :links)
				(send robot :larm :links)))
			 #F(0.2 0.2 0.2 5 5 5))
			(t #F(1 1 1 10 10 10))))
		   contact-states)))))
   (weight-matrix (diag weight-vector))
   ;; (weight-matrix-1 (pseudo-inverse weight-matrix))
   (check? t)
   (index 0))
  (let* ((cteq (calc-torque-eq
		:robot robot
		:root-link root-link
		:root-angular-velocity root-angular-velocity
		:root-spacial-velocity root-spacial-velocity
		:root-angular-acceleration root-angular-acceleration
		:root-spacial-acceleration root-spacial-acceleration
		:contact-states contact-states
		:all-links all-links :ddq ddq :dq dq :q q))
	 (tfm0 (concatenate
		float-vector
		(cdr (assoc :tau0 cteq))
		(cdr (assoc :fb0 cteq))
		(cdr (assoc :mb0 cteq))))
	 (tfm-target
	  (concatenate
	   float-vector
	   (cdr (assoc :tau-target cteq))
	   (cdr (assoc :force-target cteq))))
	 (mat (matrix-append
	       (list (cdr (assoc :Ij cteq))
		     (cdr (assoc :og cteq)))
	       '(1 0)))
;	 (IIO (matrix-append
;	       (list (unit-matrix (+ 6 (length all-links)))
;		     (matrix-append
;		      (list (make-matrix 6 (length all-links))
;			    (unit-matrix 6))
;		      '(0 1)))
;	       '(1 0)))
	 (est
	  (solve-linear-equation
	   :W weight-matrix
	   :x0 tfm-target
	   :y tfm0
	   :J mat))
	  ;; (v+
	  ;;  tfm-target
	  ;;  (transform
	  ;;   (m* weight-matrix
	  ;; 	(pseudo-inverse-loop (m* mat weight-matrix)))
	  ;;   (v- tfm0
	  ;; 	(transform mat tfm-target)))))
	 ;;(est (transform (pseudo-inverse mat) left))
	 (tau-est (subseq est 0 (length all-links)))
	 (fm-list-est
	  (mapcar #'(lambda (k)
		      (subseq est (+ (* 6 index) (length all-links))
			      (+ (* 6 (incf index)) (length all-links))))
		  contact-states))
	 (tv
	  (if check?
	      (progn
		(send *robot*
		      :torque-vector2
		      :contact-states
		      (mapcar
		       #'(lambda (cs fm)
			   (append
			    cs
			    (list
			     (cons :ext-force (subseq fm 0 3))
			     (cons :ext-moment (subseq fm 3 6)))))
		       contact-states fm-list-est)
		      :root-angular-velocity root-angular-velocity
		      :root-spacial-velocity root-spacial-velocity
		      :root-angular-acceleration root-angular-acceleration
		      :root-spacial-acceleration root-spacial-acceleration
		      :jvv dq
		      :jav ddq
		      :debug? t
		      )
		(coerce
		 (send-all (send-all all-links :joint) :joint-torque)
		 float-vector))))
	 )
    (cond
     ((and check? (> (norm (v- tv tau-est)) 1))
      (print (norm (v- tv tau-est)))
      (format t " [check-torque-calculation] FIALURE!!!~%")
      (print fm-list-est)
      (print tv))
;     (check?
;      (print 'SUCCESS!!!))
     )
    (cons tau-est fm-list-est)))

(defun calc-torque-eq-with-ddq
  (&rest
   args
   &key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (all-links (remove root-link (send robot :links)))
   (dq (scale 0 (send robot :angle-vector)))
   (q (send robot :angle-vector))
   (root-angular-velocity #F(0 0 0))
   (root-spacial-velocity #F(0 0 0))
   (contact-states
    (mapcar
     #'(lambda (k)
	 (list
	  (cons :link (send robot k :end-coords :parent))
	  (cons :worldcoords
		(copy-object
		 (send robot k :end-coords :worldcoords)))))
     '(:rleg :lleg)))
   (offset? t)
   )
  (let* ((cteq (calc-torque-eq
		:robot robot
		:root-link root-link
		:root-angular-velocity root-angular-velocity
		:root-spacial-velocity root-spacial-velocity
		:contact-states contact-states
		:all-links all-links :dq dq :q q
		:base-torque? t
		))
	 (dd (calc-dd-matrix
	      :robot robot
	      :root-link root-link
	      :root-angular-velocity root-angular-velocity
	      :root-spacial-velocity root-spacial-velocity
	      :contact-states contact-states
	      :all-links all-links :dq dq :q q
	      :base-torque? t
	      ))
	 (tfm0 (concatenate
		float-vector
		(cdr (assoc :tau0 cteq))
		(cdr (assoc :fb0 cteq))
		(cdr (assoc :mb0 cteq))
		))
	 (HBIJ
	  (matrix-append
	   (list
	    (scale-matrix -1 (cdr (assoc :ddq-coeff dd)))
	    (scale-matrix -1 (cdr (assoc :dv-coeff dd)))
	    (cdr (assoc :IJ cteq)))
	   '(0 1)))
	 (mat (matrix-append
	       (list HBIJ
		     (matrix-set
		      (make-matrix 6 (send HBIJ :get-val 'dim1))
		      (unit-matrix 6)
		      (+ (length q) 6 (length all-links))
		      0))
	       '(1 0)))
	 (offset
	  (if offset?
	      (apply
	       #'calc-offset-base-torque-ddqtfm
	       (append
		(list
		 :check? t
		 :est (transform (pseudo-inverse mat)
				 (concatenate
				  float-vector
				  tfm0
				  #F(0 0 0 0 0 0))))
		args))
	    #F(0 0 0 0 0 0)))
	 )
    (dotimes (i 6)
      (setf (aref tfm0 (- (length tfm0) (+ i 1)))
	    (+
	     (aref tfm0 (- (length tfm0) (+ i 1)))
	     (aref offset (- 6 (+ i 1))))
	    ))
    (append
     (list
      (cons :tfm0 tfm0)
      (cons :HBIJ/00I0 mat))
     cteq)))

(defun calc-offset-base-torque-ddqtfm
  (&key
   (robot *robot*)
   (root-link (car (send *robot* :links)))
   (root-pos (send (send root-link :worldcoords) :transform-vector
		   (send root-link :get-val 'acentroid)))
   (all-links (remove root-link (send robot :links)))
   (dq (scale 0 (send robot :angle-vector)))
   (root-angular-velocity #F(0 0 0))
   (root-spacial-velocity #F(0 0 0))
   (contact-states
    (mapcar
     #'(lambda (k)
	 (list
	  (cons :link (send robot k :end-coords :parent))
	  (cons :worldcoords
		(copy-object
		 (send robot k :end-coords :worldcoords)))))
     '(:rleg :lleg)))
   (est
    (instantiate float-vector
		 (+ (length dq) 6 (length all-links) 6 (* 6 (length contact-states)))))
   (ddq (subseq est 0 (length dq)))
   (root-spacial-acceleration (subseq est
				      (length dq)
				      (+ 3 (length dq))))
   (root-angular-acceleration (subseq est
				      (+ 3 (length dq))
				      (+ 6 (length dq))))
   (tau-est
    (subseq est
	    (+ 6 (length dq))
	    (+ 6 (length dq) (length all-links))))
   (base-torque
    (subseq est
	    (+ 6 (length dq) (length all-links))
	    (+ 6 (length dq) (length all-links) 6)))
   (index 0)
   (fm-list-est
    (mapcar #'(lambda (k)
		(subseq est
			(+ (* 6 index)
			   (+ 6 (length dq) (length all-links) 6))
			(+ (* 6 (incf index))
			   (+ 6 (length dq) (length all-links) 6))))
	    contact-states))
   (tv
    (progn
      (send *robot*
	    :torque-vector2
	    :contact-states
	    (mapcar
	     #'(lambda (cs fm)
		 (append
		  cs
		  (list
		   (cons :ext-force (subseq fm 0 3))
		   (cons :ext-moment (subseq fm 3 6)))))
	     contact-states fm-list-est)
	    :root-angular-velocity root-angular-velocity
	    :root-spacial-velocity root-spacial-velocity
	    :root-angular-acceleration root-angular-acceleration
	    :root-spacial-acceleration root-spacial-acceleration
	    :jvv dq
	    :jav ddq
	    )
      (coerce
       (send-all (send-all all-links :joint) :joint-torque)
       float-vector)))
   (check? t)
   (real-base-torque
    (concatenate
     float-vector
     (send root-link :force)
     (v- (send root-link :moment)
	 (scale
	  1e-3
	  (v* root-pos (send root-link :force))))))
   offset
   &allow-other-keys
   )
  (if  (> (norm (v- tv tau-est)) 1)
      (progn
	(format t " [check-torque-calculation] FIALURE!!! (err=~A)~%"
		(norm (v- tv tau-est)))
	(print
	 (sort
	  (mapcar #'cons
		  (send-all (send-all all-links :joint) :name)
		  (map cons #'abs (v- tau-est tv)))
	  #'(lambda (a b) (< (cdr a) (cdr b)))))))
  (setq offset
	(v- real-base-torque
	    base-torque))
  (if (> (norm offset) 1)
      (format t "[calc-offset-base-torque-ddqtfm]~% offset detected ~A~%"
	      offset))
  (cond
   (check? offset)
   (t
    (flatten
     (list ddq root-spacial-acceleration root-angular-acceleration
	   tau-est real-base-torque fm-list-est))))
  )

(defun calc-torque-check-with-ddq
  (&rest
   args
   &key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (all-links (remove root-link (send robot :links)))
   (q-links  ;; for ddq
    (cond
     ((find-method robot :actuators)
      (send-all (send robot :actuators) :parent))
     (t (cdr (send robot :links)))))
   ;;(q-links (send-all (send robot :actuators) :parent)) ;; for ddq
   (dq (scale 0 (send robot :angle-vector)))
   (q (send robot :angle-vector))
   (root-angular-velocity #F(0 0 0))
   (root-spacial-velocity #F(0 0 0))
   (ddq-target (instantiate float-vector (length dq)))
   (dv-target (instantiate float-vector 6))
   (contact-states
    (mapcar
     #'(lambda (k)
	 (list
	  (cons :link (send robot k :end-coords :parent))
	  (cons :worldcoords
		(copy-object
		 (send robot k :end-coords :worldcoords)))))
     '(:rleg :lleg)))
   (weight-vector
    (apply #'concatenate
	   (cons float-vector
		 (append
		  (list (make-list (length dq) :initial-element 10000))
		  (list (make-list 6 :initial-element 10000))
		  (list (make-list (length all-links) :initial-element 1))
		  (list (make-list 6 :initial-element 1))
		  (mapcar
		   #'(lambda (cs)
		       (cond
			((find (cdr (assoc :link cs))
			       (append
				(send robot :rarm :links)
				(send robot :larm :links)))
			 #F(0.2 0.2 0.2 0.05 0.05 0.05))
			(t #F(1 1 1 0.1 0.1 0.1))))
		   contact-states)))))
   (weight-matrix (diag weight-vector))
   (check? nil);;t)
   (offset? t)
   (index 0))
  (let* ((cteq2
	  (calc-torque-eq-with-ddq
	   :robot robot
	   :root-link root-link
	   :root-angular-velocity root-angular-velocity
	   :root-spacial-velocity root-spacial-velocity
	   :contact-states contact-states
	   :all-links all-links :dq dq :q q
	   :offset? offset?))
	 (tfm0
	  (concatenate float-vector
		       (cdr (assoc :tfm0 cteq2))
		       #F(0 0 0 0 0 0)))
	 (tfm-target
	  (concatenate
	   float-vector
	   ddq-target
	   dv-target
	   (cdr (assoc :tau-target cteq2))
	   #F(0 0 0 0 0 0)
	   (cdr (assoc :force-target cteq2))))
	 ;;
	 (mat (cdr (assoc :HBIJ/00I0 cteq2)))
	 (est (solve-linear-equation
	       :J mat
	       :x0 tfm-target
	       :W weight-matrix
	       :y tfm0))
	 )
    (apply
     #'calc-offset-base-torque-ddqtfm
     (append
      (list
       :check? check?
       :est est)
      args))))

(defun calc-zmp-from-fm-list
  (fm-list
   pos-list
   &key
   (robot *robot*)
   (height
    (* 1e-3 (aref (send robot :rleg :end-coords :worldpos) 2)))
   (mg (scale (* 1e-3 (send robot :weight)) #F(0 0 -9.8)))
   (c (scale 1e-3 (send robot :centroid)))
   (Efi+mg
    (v+ mg
	(reduce #'v+
		(mapcar
		 #'(lambda (v) (subseq v 0 3))
		 fm-list))))
   (Emi+pfi
    (reduce #'v+
	    (mapcar #'(lambda (fm p)
			(v+ (subseq fm 3 6)
			    (v* (scale 1e-3 p) (subseq fm 0 3))))
		    fm-list pos-list)))
   (mat
    (matrix-append
     (list (outer-product-matrix (v+ Efi+mg mg))
	   (make-matrix 1 3 '((0 0 1))))
     '(1 0)))
   (vec
    (concatenate
     float-vector
     (v+ Emi+pfi (v* c mg))
     (float-vector (* +1 height)))))
  (transform
   (pseudo-inverse mat)
   (scale +1000 vec)))

(defun random-calc-torque-check
  (&rest args)
  (apply
   #'calc-torque-check
   (append
    (list
     :ddq (map float-vector #'(lambda (a) (- (random 10.0) 5.0))
	       (send *robot* :angle-vector))
     :dq (map float-vector #'(lambda (a) (- (random 10.0) 5.0))
	      (send *robot* :angle-vector))
     :root-spacial-acceleration (random-vector 10.0)
     :root-angular-acceleration (random-vector 10.0)
     :root-spacial-velocity (random-vector 10.0)
     :root-angular-velocity (random-vector 10.0))
    args)
   ))

(defun random-calc-torque-check2
  (&rest args)
  (send *robot* :angle-vector
	(map float-vector
	     #'(lambda (d) (* 180 (- (random 2.0) 1.0)))
	     (send *robot* :angle-vector)))
  (send *robot* :newcoords
	(make-coords
	 :pos (scale 1000 (random-vector 1.0))
	 :rpy (random-vector 3.14)))
  (apply
   #'calc-torque-check-with-ddq
   (append
    (list
     :dq (map float-vector
	      #'(lambda (a) (- (random 10.0) 5.0))
	      (send *robot* :angle-vector))
     :ddq-target
     (map float-vector
	  #'(lambda (a) (- (random 10.0) 5.0))
	  (send *robot* :angle-vector))
     :root-spacial-velocity (random-vector 10.0)
     :root-angular-velocity (random-vector 10.0))
    args)
   ))


#|

(progn (send *robot* :newcoords (make-coords :pos (random-vector 1000.0) :rpy (random-vector 3.0))) (send *robot* :angle-vector (map float-vector #'(lambda (a) (* 100 (- (random 2.0) 1.0))) (send *robot* :angle-vector))) (random-calc-torque-check2))