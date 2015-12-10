
;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *log-stream* t)

;; (defvar *robot-type* :staro)
(require "robot-param.lisp")
(require "my-util.lisp")
(require "util/min-power.lisp")
(require "util/spline.lisp")
(require "pose-generator-with-contact-state.lisp")
(require "slide-util.lisp")

(defun funcall-with-time
  (func
   &rest
   args
   &key
   (index 0)
   (time-buf #F(0))
   (call-cnt #F(0))
   &allow-other-keys)
  (let (mtime ret)
    (setq
     mtime
     (bench2
      (setq ret
	    (apply func (remove-args-values
			 args
			 (list :index :time-buf :call-cnt))))))
    (setf (aref time-buf index)
	  (+ (aref time-buf index) mtime))
    (setf (aref call-cnt index)
	  (+ (aref call-cnt index) 1))
    ret))

(setq *pg-use-toes* '(nil nil))
(setq *kca-cog-gain* 3)
(defun remove-motion-sequence
  (&rest
   args
   &key
   (robot *robot*)
   (remove-limb :rarm)
   (now-rsd (instance robot-state-data2 :init))
   (now-cs (send now-rsd :contact-states))
   ;;
   (rest-cs
    (flatten (list (find-if
		    #'(lambda (cs) (eq remove-limb (send cs :name)))
		    now-cs))))
   (target-cs (remove-if #'(lambda (cs) (eq remove-limb (send cs :name)))
			 now-cs))
   (non-stop t)
   (tmax-leg-rate 0.4)
   (tmax-hand-rate 0.4)
   (ik-debug-view nil)
   (buf nil)
   (mseq-mode :remove)
   (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.5 0.5))
   (pose-generate-stop 20)
   (centroid-thre-rate 0.95)
   optimize-brli-args
   &allow-other-keys)
  (format *log-stream* " ~A-mseq ~A~% target-cs=~A, rest-cs=~A~%"
	  mseq-mode remove-limb target-cs rest-cs)
  (let ((ret (pose-generate-with-contact-state-with-time
	      target-cs
	      :rest-contact-states rest-cs
	      :non-stop non-stop
	      :stop pose-generate-stop
	      :ik-debug-view ik-debug-view
	      :tau-gain 0.1
	      :centroid-thre-rate centroid-thre-rate
	      :tmax-leg-rate tmax-leg-rate
	      :tmax-hand-rate tmax-hand-rate
	      :root-link-virtual-joint-weight
	      root-link-virtual-joint-weight
	      :buf buf
	      :optimize-brli-args optimize-brli-args
	      )))
    (cond
     (ret (send ret :buf :mseq-mode mseq-mode)
	  (send ret :buf :remove-limb remove-limb)))
    ret))

(defun reach-motion-sequence
  (&rest
   args
   &key
   (robot *robot*)
   (remove-limb :rarm)
   (remove-cs
    (find-if #'(lambda (cs) (eq remove-limb (send cs :name)))
	     now-cs))
   (now-rsd (instance robot-state-data2 :init))
   remove-rsd
   (gbrli (send now-rsd :gbrli))
   (now-cs (send now-rsd :contact-states))
   (cs-candidates
    (remove-if #'default-contact-state-filter
	       (remove-if
		#'(lambda (cs) (not (eq remove-limb (send cs :name))))
		(progn
		  (if (not (and (boundp '*contact-states*)
				*contact-states*))
		      (demo-climb-setup))
		  *contact-states*))))
   (contact-trans-mode :walk)
   (first-contact-trans-mode contact-trans-mode)
   (slide-cs-candidates
    (if (eq contact-trans-mode :slide)
	(remove-if #'(lambda (cs)
		       (not (contact-plane-condition-check
			     cs remove-cs)))
		   cs-candidates)))
   sorted-cs-candidates
   ;;
   (tmax-leg-rate 0.6)
   (tmax-hand-rate 0.4)
   (ik-debug-view nil)
   (rms-loop-cnt 0)
   (rms-loop-max 5)
   (trajectory-check-func
    '(lambda (new-rsd now-rsd)
       (format *log-stream* " :reach-mseq traj-check-func always ok!!~%")
       t))
   (brlv-thre-for-slide 0.5)
   slide-buf slide-constraints-args
   ret
   buf tmp-error-flag
   &allow-other-keys)
  ;; (send-all (send-all cs-candidates :target-coords) :draw-on :flush t)
  ;; (unix:sleep 1)
  (format *log-stream* " :reach-mseq ~A~%" remove-limb)
  (labels
   ((cs-select
     (&optional
      (now-cs? remove-cs)
      ret)
     (setq ret
	   (mapcar
	    #'cdr
	    (sort
	     (mapcar
	      #'(lambda (cs)
		  (cons
		   (send cs :eval
			 now-cs?
			 now-rsd
			 :force-zero (send now-rsd :contact-zero))
		   cs))
	      (if (eq contact-trans-mode :walk) cs-candidates
		slide-cs-candidates))
	     #'(lambda (a b) (> (car a) (car b))))))
     (dotimes (i (length ret))
       (cond
	((and
	  now-cs?
	  (< (norm (send (send (car ret) :default-target-coords)
			 :difference-position (send now-cs? :default-target-coords)))
	     1))
	 (setq ret (append (cdr ret) (list (car ret)))))
	(t (return-from nil nil))))
     ;;     (if (find now-cs? ret)
     ;;	 (append (remove now-cs? ret) (list now-cs?))
     (if (eq contact-trans-mode :walk)
	 (setq cs-candidates ret)
       (setq slide-cs-candidates ret))
     ret)
    (reach-ik
     (move-cs
      &optional
      (now-cs  (union *pg-optional-ik-contact-states* now-cs)))
     (format *log-stream* "[reach-ik] swap ~A(~A) and ~A E ~A~%"
	     move-cs remove-limb remove-cs  now-cs)
     (ik-trial
      :trial 2
      :target
      (cons
       (list
	(cons :target (send move-cs :contact-coords))
	(cons :coords (send move-cs :target-coords))
	(cons :translation-axis (send move-cs :translation-axis))
	(cons :rotation-axis (send move-cs :rotation-axis))
	(cons :thre 20))
       (mapcar
	#'(lambda (cs)
	    (list
	     (cons :target (send cs :contact-coords))
	     (cons :coords (send cs :target-coords))
	     (cons :translation-axis (send cs :translation-axis))
	     (cons :rotation-axis (send cs :rotation-axis))))
	(remove-if #'(lambda (cs) (eq remove-limb (send cs :name))) now-cs)))
      ;; (mapcar
      ;; 	#'(lambda (k)
      ;; 	    (list (cons :target k)
      ;; 		  (cons :coords (send robot k :end-coords :copy-worldcoords))))
      ;; 	(remove remove-limb all-limbs)))
      :centroid-thre 1000
      :balance-leg nil
      :target-centroid-pos nil
      :collision-avoidance-link-pair nil
      :max #F(500 500 1000 300 300 300)
      :root-link-virtual-joint-weight root-link-virtual-joint-weight
      :stop 50
      :debug-view ik-debug-view
      :flush (not (null ik-debug-view))
      :warnp nil
      ;;      :revert-if-fail nil
      )))
   (format *log-stream* "[reach-motino-sequence] ~A/~A~%" rms-loop-cnt rms-loop-max)
   (cond
    ((and (setq tmp-error-flag
		(or
		 tmp-error-flag
		 (and (eq contact-trans-mode :walk)
		      (or  (null cs-candidates)
			   (not (and remove-rsd (send remove-rsd :full-constrainted)))))))
	  (not (eq first-contact-trans-mode contact-trans-mode)))
     ;; ! walkable?
     (format *log-stream* " :reach-mseq no-candidates ~A~%" remove-limb)
     (best-rsd (flatten buf) :buf (list :stop-reason :no-candidates)))
    ((and (setq tmp-error-flag
		(or
		 tmp-error-flag
		 (and (eq contact-trans-mode :slide) (null slide-cs-candidates))))
	  (not (eq first-contact-trans-mode contact-trans-mode)))
     ;; ! slidable?
     (format *log-stream* " :reach-mseq no-slide-candidates ~A~%" remove-limb)
     (format *log-stream* "             :walk transition mode~%")
     (best-rsd (flatten buf) :buf (list :stop-reason :no-slide-candidates)))
    ((and (setq tmp-error-flag (or tmp-error-flag (> rms-loop-cnt rms-loop-max)))
	  (not (eq first-contact-trans-mode contact-trans-mode)))
     (format *log-stream* " :reach-mseq too-deep ~A~%" remove-limb)
     (best-rsd (flatten buf) :buf (list :stop-reason :too-deep)))
    (tmp-error-flag
     ;; if ! walkable?; then slidable?; else walkable?; fi
     (apply #'reach-motion-sequence
	    (append (remove-args-values
		     args (list :contact-trans-mode
				:rms-loop-cnt))
		    (list :contact-trans-mode (if (eq contact-trans-mode :walk) :slide :walk)
			  :first-contact-trans-mode first-contact-trans-mode
			  :rms-loop-cnt 0
			  :cs-candidates cs-candidates
			  :slide-cs-candidates slide-cs-candidates))))
    ((and
      ;;(setq cs-candidates (cs-select))
      (setq sorted-cs-candidates (cs-select))
      (let ((buf (send (car sorted-cs-candidates) :contact-coords)))
	(format *log-stream* "  select ~A link toward(~A) ~A E ~A~%"
		(send (send buf :parent) :name) contact-trans-mode
		(car sorted-cs-candidates) sorted-cs-candidates)
	t)
      (or ;; (null (reach-ik (car cs-candidates)))
	  (and (eq contact-trans-mode :slide)
	       (not
		(and ;; slidable?
		 remove-cs
		 ;; (print 'slide-motion-search) (print slide-cs-candidates)
		 (setq slide-constraints-args
		       (slide-friction-condition-extra-args
			:contact-states now-cs
			:from remove-cs :to (car slide-cs-candidates)))
		 (setq
		  slide-buf
		  (apply #'remove-motion-sequence
			 (append (remove-args-values
				  args (list :buf :remove-limb :mseq-mode :optimize-brli-args))
				 (list :remove-limb nil
				       :mseq-mode :remove
				       :optimize-brli-args
				       slide-constraints-args))))
		 (send slide-buf :full-constrainted)
		 (let* ((danger1 (if (and remove-rsd (send remove-rsd :full-constrainted))
				     (calc-walk-danger-object-function remove-rsd)))
			(lmd (cadr (member :local-move-dir slide-constraints-args)))
			danger2)
		   (send slide-buf :buf :f/fmax
			 (mapcar
			  #'(lambda (f/fmax cs)
			      (if (eq (send cs :name) remove-limb)
				  (concatenate
				   float-vector
				   (v+
				    (scale brlv-thre-for-slide lmd)
				    (v- (subseq f/fmax 0 3)
					(scale (v. (subseq f/fmax 0 3) lmd) lmd)))
				   (subseq f/fmax 3))
				f/fmax))
			  (send slide-buf :f/fmax)
			  (send slide-buf :contact-states)))
		   (setq danger2 (calc-walk-danger-object-function slide-buf))
		   (format *log-stream* " brli compare wark vs slide = ~A vs ~A~%"
			   danger1 danger2)
		   (send slide-buf :buf :walk-brlv danger1)
		   (send slide-buf :buf :slide-brlv danger2)
		   (if remove-rsd (send remove-rsd :buf :walk-brlv danger1))
		   (if remove-rsd (send remove-rsd :buf :slide-brlv danger2))
		   (send slide-buf :buf :slide t)
		   (send slide-buf :buf :remove-limb remove-limb)
		   (if (and remove-rsd (send remove-rsd :full-constrainted))
		       (> danger1 danger2) t)
		   ))))
	  ;; (and (format *log-stream* " collision-resolve-move: ~A~%" (collision-resolve-move)) nil)
	  (null (reach-ik (car sorted-cs-candidates)))
	  )
      )
     (if (eq contact-trans-mode :slide)
	 (format *log-stream* " :reach-mseq slide impossible ~A~%" remove-limb)
       (format *log-stream* " :reach-mseq ik-solution-not-found ~A~%" remove-limb))
     (apply #'reach-motion-sequence
	    (append (remove-args-values args (list :cs-candidates
						   :slide-cs-candidates
						   :rms-loop-cnt))
		    (list :rms-loop-cnt (+ rms-loop-cnt 1)
			  :first-contact-trans-mode first-contact-trans-mode
			  :slide-cs-candidates
			  (if (eq contact-trans-mode :slide)
			      (cdr slide-cs-candidates)
			    slide-cs-candidates)
			  :cs-candidates
			  (if (eq contact-trans-mode :walk)
			      (cdr cs-candidates)
			    cs-candidates)))))
    ((progn
       ;; (send *viewer* :draw-objects) (read-line)
       (setq
	ret
	(apply #'remove-motion-sequence
	       (append (list :now-rsd
			     (if (eq contact-trans-mode :walk) now-rsd
			       (instance robot-state-data2 :init
					 :contact-states
                                         ;; (cons (car slide-cs-candidates)
                                         ;; (remove remove-cs now-cs))
                                         (mapcar
                                          #'(lambda (cs) (if (eq cs remove-cs)
                                                             (car slide-cs-candidates) cs))
                                          now-cs)
                                         ))
			     :rest-cs
			     (if (eq contact-trans-mode :walk) (list (car cs-candidates)))
			     :remove-limb
			     (if (eq contact-trans-mode :walk) remove-limb)
			     :mseq-mode :reach
			     :optimize-brli-args
			     (if (eq contact-trans-mode :walk) nil slide-constraints-args)
			     )
		       (remove-args-values
			args (list :buf :remove-limb :mseq-mode
				   :now-rsd :now-cs :optimize-brli-args))
		       )))
       (cond
	(ret (send ret :buf :slide slide-buf)
	     (send ret :buf :remove-limb remove-limb)))
       (or (null ret)
	   (not (send ret :full-constrainted))
	   (and (functionp trajectory-check-func)
		(not (funcall trajectory-check-func ret (or slide-buf now-rsd)))))
       )
     (format *log-stream* " :reach-mseq reach-impossible ~A~%" remove-limb)
     (send robot :angle-vector (copy-object (send now-rsd :angle-vector)))
     (send robot :newcoords (copy-object (send now-rsd :root-coords)))
     (if ret (send ret :set-val 'full-constrainted nil))
     (apply #'reach-motion-sequence
	    (append (remove-args-values
		     args
		     (list :cs-candidates :slide-cs-candidates
			   :now-rsd :gbrli :buf :rms-loop-cnt))
		    (list :rms-loop-cnt (+ rms-loop-cnt 1)
			  :first-contact-trans-mode first-contact-trans-mode
			  :now-rsd now-rsd
			  :gbrli (send (or ret now-rsd) :gbrli)
			  :slide-cs-candidates
			  (if (eq contact-trans-mode :slide)
			      (cdr slide-cs-candidates)
			    slide-cs-candidates)
			  :cs-candidates
			  (if (eq contact-trans-mode :walk)
			      (cdr cs-candidates)
			    cs-candidates)
			  :buf (cons ret buf))))
     )
    (t (send ret :buf :rest-candidates
	     (if (eq contact-trans-mode :walk)
		 (cdr cs-candidates)
	       cs-candidates))
       (send ret :buf :rest-slide-candidates
	     (if (eq contact-trans-mode :slide)
		 (cdr slide-cs-candidates)
	       slide-cs-candidates))
       (format *log-stream* " :reach-mseq success! ~A rest candidates=~A(mode=~A)~%"
	       remove-limb (length (cdr sorted-cs-candidates)) contact-trans-mode)
       (send ret :buf :mseq-mode :reach)
       ret))))

(defun suspend-motion-sequence
  (&rest args)
  (apply #'remove-motion-sequence
	 (append (list :remove-limb nil)
		 (list :mseq-mode :suspend)
		 args)))

(defvar *failures* nil)
(defun demo-motion-sequence
  (&rest
   args
   &key
   (robot *robot*)
   (remove-limb :rarm)
   (now-rsd
    (instance robot-state-data2 :init
	      :contact-states
	      (now-contact-state :limb-keys '(:rarm :larm :rleg :lleg))))
   (gbrli (send now-rsd :gbrli))
   (now-cs (send now-rsd :contact-states))
   (all-cs-candidates
    (progn
      (if (not (and (boundp '*contact-states*)
		    *contact-states*))
	  (demo-climb-setup :param-ladder))
      *contact-states*))
   (cs-filter 'default-contact-state-filter)
   (cs-candidates
    (remove-if
     #'(lambda (cs)
	 (or (not (eq remove-limb (send cs :name)))
	     (and (null (send cs :sequence-select))
		  (funcall cs-filter cs))))
     all-cs-candidates))
   (slide-cs-candidates)
   (select-limb-function)
   ;;
   (all-limbs '(:rarm :larm :rleg :lleg))
   (loop-cnt 1)
   (rec-cnt 1)
   (loop-max 4)
   (rms-loop-max 5)
   (back-track t)
   (non-stop t)
   (tmax-leg-rate 0.5)
   (tmax-hand-rate 0.5)
   (brlv-thre-for-slide 0.5)
   (centroid-thre-rate 0.95)
   (ik-debug-view :no-message)
   (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.5 0.5))
   ;;
   buf
   buf2
   ret
   remove
   slide optimize-brli-args contact-trans-mode
   reach
   suspend
   (failure-collect? nil)
   (first-flag t)
   ;;
   (time-buf
    (instantiate
     float-vector
     (length '(:remove :reach :suspend))))
   (time-bt-buf (copy-seq time-buf))
   (cnt-buf (copy-seq time-buf))
   (cnt-bt-buf (copy-seq time-buf))
   (prev-time-buf (copy-seq time-buf))
   (pose-generate-stop 20)
   trajectory-check-func
   &allow-other-keys)
  (if (and (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects))
  (format *log-stream* "[motion-seq] ~A(~A)/~A for ~A~%"
	  loop-cnt rec-cnt loop-max remove-limb)
  (cond
   ((if (functionp loop-max) (funcall loop-max loop-cnt)
      (> loop-cnt loop-max))
    (format *log-stream* "[motion-seq] too deep~%")
    ret)
   ((and
     (not first-flag)
     (setq contact-trans-mode :walk)
     (not ;; remove
      (and
       (or remove
	   (if (not (find :remove args))
	       (setq remove
		     (funcall-with-time
		      #'remove-motion-sequence
		      :time-buf time-buf
		      :call-cnt cnt-buf
		      :index 0
		      ;;
		      :robot robot
		      :remove-limb remove-limb
		      :now-rsd now-rsd
		      :non-stop non-stop
		      :tmax-hand-rate tmax-hand-rate
		      :tmax-leg-rate tmax-leg-rate
		      :ik-debug-view ik-debug-view
		      :centroid-thre-rate centroid-thre-rate
		      :root-link-virtual-joint-weight
		      root-link-virtual-joint-weight
		      :pose-generate-stop pose-generate-stop
		      ))))
       (send remove :full-constrainted)
       (< (calc-walk-danger-object-function remove)
	  brlv-thre-for-slide)
       ))
     (setq contact-trans-mode :slide)
     nil) ;; always false
    ;; this line will never be evaluated
    (format *log-stream*
	    "[motion-seq] ?? BUG :remove-mseq fail back-track ~A!!~%"
	    remove-limb)
    (if failure-collect? (push remove *failures*))
    (setf (aref time-bt-buf 0)
	  (+ (aref time-bt-buf 0)
	     (- (aref time-buf 0) (aref prev-time-buf 0))))
    (setf (aref cnt-bt-buf 0) (+ (aref cnt-bt-buf 0) 1))
    (or remove now-rsd))
   ((and
     (not first-flag)
     (not ;; reach
      (and
       (or
	reach
	(setq reach
	      (funcall-with-time
	       #'reach-motion-sequence
	       :time-buf time-buf
	       :call-cnt cnt-buf
	       :index 1
	       :brlv-thre-for-slide brlv-thre-for-slide
	       ;;
	       :contact-trans-mode contact-trans-mode
	       :remove-rsd remove
	       ;;
	       :rms-loop-max rms-loop-max
	       :robot robot
	       :remove-limb remove-limb
	       :now-rsd (or remove now-rsd)
	       :cs-candidates cs-candidates
	       :slide-cs-candidates
	       (setq slide-cs-candidates
		     (or slide-cs-candidates
			 (if (not (find :slide-cs-candidates args))
			     (remove-if #'(lambda (cs)
					    (not (contact-plane-condition-check
						  cs
						  (find-if
						   #'(lambda (cs) (eq (send cs :name) remove-limb))
						   now-cs))))
					cs-candidates))))
	       :tmax-leg-rate tmax-leg-rate
	       :tmax-hand-rate tmax-hand-rate
	       :ik-debug-view ik-debug-view
	       :centroid-thre-rate centroid-thre-rate
	       :root-link-virtual-joint-weight
	       root-link-virtual-joint-weight
	       :pose-generate-stop pose-generate-stop
	       :trajectory-check-func trajectory-check-func
	       )))
       (or (not back-track) (send reach :full-constrainted)))))
    (format *log-stream*
	    "[motion-seq] :reach-mseq fail back-track !! ~A~%"
	    remove-limb)
    (if failure-collect? (push reach *failures*))
    (setf (aref time-bt-buf 1)
	  (+ (aref time-bt-buf 1)
	     (- (aref time-buf 1) (aref prev-time-buf 1))))
    (setf (aref cnt-bt-buf 1) (+ (aref cnt-bt-buf 1) 1))
    (or reach remove now-rsd))
   ((and
     (not first-flag)
     (not ;; suspend
      (and
       (or suspend
	   (setq suspend
		 (funcall-with-time
		  #'suspend-motion-sequence
		  :time-buf time-buf
		  :call-cnt cnt-buf
		  :index 2
		  ;;
		  :robot robot
		  :remove-limb remove-limb
		  :now-rsd reach
		  :tmax-hand-rate tmax-hand-rate
		  :tmax-leg-rate tmax-leg-rate
		  :ik-debug-view ik-debug-view
		  :centroid-thre-rate centroid-thre-rate
		  :root-link-virtual-joint-weight
		  root-link-virtual-joint-weight
		  :buf (list (send reach :copy))
		  :pose-generate-stop pose-generate-stop
		  ))) ;; at least one solution
       (or (not back-track) (send suspend :full-constrainted)))))
    (format *log-stream*
	    "[motion-seq] ?? BUG :suspend-mseq fail back-track !! ~A~%"
	    remove-limb)
    (if failure-collect? (push suspend *failures*))
    (setf (aref time-bt-buf 2)
	  (+ (aref time-bt-buf 2)
	     (- (aref time-buf 2) (aref prev-time-buf 2))))
    (setf (aref cnt-bt-buf 2) (+ (aref cnt-bt-buf 2) 1))
    (or suspend reach remove now-rsd))
   ((and ;; next remove-limb
     (setq prev-time-buf (v- time-buf prev-time-buf))
     (setq
      buf
      (demo-motion-sequence
       :robot robot
       :remove-limb
       (cond
	(first-flag remove-limb)
	((functionp select-limb-function)
	 (funcall select-limb-function remove-limb all-limbs cs-candidates))
	(t (cadr (member remove-limb (append all-limbs all-limbs)))))
       :now-rsd (if first-flag now-rsd suspend)
       :all-cs-candidates all-cs-candidates
       :cs-filter cs-filter
       :select-limb-function select-limb-function
       :loop-cnt (if first-flag loop-cnt (+ loop-cnt 1))
       :rec-cnt (if first-flag rec-cnt (+ rec-cnt 1))
       :all-limbs ;; all-limbs
       (cond
	(first-flag all-limbs)
	((> (length all-limbs) 1)
	 (let* ((id (position remove-limb all-limbs)))
	   (append (subseq all-limbs (+ 1 id)) (subseq all-limbs 0 (+ 1 id)))))
	(t all-limbs))
       :loop-max loop-max
       :rms-loop-max rms-loop-max
       :non-stop non-stop
       :back-track back-track
       :tmax-leg-rate tmax-leg-rate
       :tmax-hand-rate tmax-hand-rate
       :brlv-thre-for-slide brlv-thre-for-slide
       :ik-debug-view ik-debug-view
       :centroid-thre-rate centroid-thre-rate
       :root-link-virtual-joint-weight
       root-link-virtual-joint-weight
       :time-buf time-buf
       :time-bt-buf time-bt-buf
       :cnt-buf cnt-buf
       :cnt-bt-buf cnt-bt-buf
       :failure-collect? failure-collect?
       :ret (append
	     (if (not first-flag)
		 (list suspend reach (or (send reach :buf :slide) remove)))
	     ret)
       :pose-generate-stop pose-generate-stop
       :trajectory-check-func trajectory-check-func
       :first-flag nil
       ))
     (listp buf))
    buf)
   ((or (and (eq :contact-trans-mode :walk) (null (send reach :buf :rest-candidates)))
	(and (eq :contact-trans-mode :slide) (null (send reach :buf :rest-slide-candidates))))
    (format *log-stream* "[motion-seq] ~A candidates~%"
	    (send reach :buf))
    (or buf suspend reach remove now-rsd))
   ((progn
      (format *log-stream* "[motion-seq] parallel track~%")
      ;; (send robot :angle-vector (copy-object (send remove :angle-vector)))
      ;; (send robot :newcoords (copy-object (send remove :root-coords)))
      (send (or (if (and remove (send remove :full-constrainted)) remove) now-rsd)
	    :draw :friction-cone? nil)
      (setf (aref cnt-bt-buf 1) (+ (aref cnt-bt-buf 1) 1))
      (setf (aref cnt-bt-buf 2) (+ (aref cnt-bt-buf 2) 1))
      (setf (aref time-bt-buf 1)
	    (+ (aref time-bt-buf 1) (aref prev-time-buf 1)))
      (setf (aref time-bt-buf 2)
	    (+ (aref time-bt-buf 2) (aref prev-time-buf 2)))
      (setq buf2
	    (demo-motion-sequence
	     :robot robot
	     :remove-limb remove-limb
	     :now-rsd (or (if (and remove (send remove :full-constrainted)) remove) now-rsd)
	     :gbrli (send (or buf remove now-rsd) :gbrli)
	     :all-cs-candidates all-cs-candidates
	     :cs-filter cs-filter
	     :cs-candidates (if reach (send reach :buf :rest-candidates))
	     :slide-cs-candidates (if reach (send reach :buf :rest-slide-candidates))
	     :select-limb-function select-limb-function
	     :all-limbs all-limbs
	     :loop-cnt loop-cnt
	     :rec-cnt (+ rec-cnt 1)
	     :loop-max loop-max
	     :rms-loop-max rms-loop-max
	     :non-stop non-stop
	     :back-track back-track
	     :tmax-leg-rate tmax-leg-rate
	     :tmax-hand-rate tmax-hand-rate
	     :brlv-thre-for-slide brlv-thre-for-slide
	     :ik-debug-view ik-debug-view
	     :centroid-thre-rate centroid-thre-rate
	     :root-link-virtual-joint-weight
	     root-link-virtual-joint-weight
	     :time-buf time-buf
	     :time-bt-buf time-bt-buf
	     :cnt-buf cnt-buf
	     :cnt-bt-buf cnt-bt-buf
	     :ret ret
	     :failure-collect? failure-collect?
	     :remove remove
	     :pose-generate-stop pose-generate-stop
	     :trajectory-check-func trajectory-check-func
	     :first-flag nil
	     ))
      (and buf2 (listp buf2)))
    buf2)
   (t (setf (aref cnt-bt-buf 0) (+ (aref cnt-bt-buf 0) 1))
      (setf (aref time-bt-buf 0)
	    (+ (aref time-bt-buf 0) (aref prev-time-buf 0)))
      (or buf2 buf suspend reach remove now-rsd))))

(defun demo-motion-sequence-with-timer
  (&rest args
	 &key
	 (log-file (format nil "log/~A.~A" "mseq" (log-surfix)))
	 (log-stream (open log-file :direction :output))
	 (log-buf *log-stream*)
	 &allow-other-keys)
  (setq *failures* nil)
  (let (mtime ret forward backtrack
	(time-buf (instantiate float-vector 3))
	(time-bt-buf (instantiate float-vector 3))
	(cnt-buf (instantiate float-vector 3))
	(cnt-bt-buf (instantiate float-vector 3)))
    (setq *log-stream* log-stream)
    (setq mtime
	  (bench2
	   (setq ret
		 (apply 'demo-motion-sequence
			(append args
				(list :time-buf time-buf
				      :time-bt-buf time-bt-buf
				      :cnt-buf cnt-buf
				      :cnt-bt-buf cnt-bt-buf))))))
    (format *log-stream*
	    "[demo-motion-sequence-with-timer] total ~A sec~%"
	    mtime)
    (format *log-stream*
	    " @ each seq total ~A sec = ~A + ~A + ~A (~A)%~%"
	    (setq forward (+ (aref time-buf 0)
			     (aref time-buf 1)
			     (aref time-buf 2)))
	    (aref time-buf 0) (aref time-buf 1) (aref time-buf 2)
	    (/ (* 100.0 forward) mtime))
    (mapcar
     #'(lambda (i name)
	 (format *log-stream*
		 "   @ ~A(x~A) ~A sec = ~A%~%"
		 name
		 (aref cnt-buf i)
		 (aref time-buf i)
		 (/ (* 100.0 (aref time-buf i))
		    forward)))
     '(0 1 2) (list "remove" "reach" "suspend"))
    (format *log-stream*
	    " @ backtrack ~A sec = ~A%~%"
	    (setq backtrack (+ (aref time-bt-buf 0)
			       (aref time-bt-buf 1)
			       (aref time-bt-buf 2)))
	    (/ (* 100.0 backtrack) mtime))
    (mapcar
     #'(lambda (i name)
	 (format *log-stream*
		 "   @ ~A(x~A) ~A sec = ~A%~%"
		 name
		 (aref cnt-bt-buf i)
		 (aref time-bt-buf i)
		 (/ (* 100.0 (aref time-bt-buf i))
		    backtrack)))
     '(0 1 2) (list "remove" "reach" "suspend"))
    (setq *log-stream* log-buf)
    (cond
     ((streamp log-stream)
      (format *log-stream* "close stream ~A~%" log-stream)
      (close log-stream)))
    (cond
     ((and ret (stringp log-file))
      (format *log-stream* "output rsd to ~A.rsd~%" log-file)
      (rsd-serialize :rsd-list ret :file (format nil "~A.rsd" log-file))
      ))
    ret
    ))

(defun demo-climb-setup
  (&optional
   (mode :param-ladder)
   (rotate? nil))
  (init-pose)
  (if rotate? (send *robot* :rotate (deg2rad 180) :z))
  ;;(send *robot* :translate #F(-100 0 0) :world)
  ;;
  ;; define :contact-coords for feet
  (dolist (limb '(:rleg :lleg))
    (send *robot* :put
          (read-from-string (format nil "~A-end-coords2" limb))
          (make-cascoords
           :coords (send (send (send *robot* limb :end-coords) :copy-worldcoords) :translate (float-vector 80 0 0))
           :name (read-from-string (format nil "~A-end-coords2" limb))))
    (send (send *robot* limb :end-coords :parent) :assoc
          (send *robot* :get (read-from-string (format nil "~A-end-coords2" limb)))))
  ;; torque limit
  (if (not (eq mode :kirin-ladder))
      (mapcar
       #'(lambda (k)
	   (if (and (find-method (send *robot* :rarm) k)
		    (send *robot* :rarm k))
	       (let ((buf (send *robot* :rarm k :max-joint-torque)))
		 (send *robot* :rarm k :max-joint-torque (* buf 2.))
		 (send *robot* :larm k :max-joint-torque (* buf 2.)))))
       '(:wrist-p :wrist-r :wrist-y)))
  ;; clear variables
  (setq *pg-float-cascoords* nil)
  (setq *kca-ik-param* nil)
  ;; gen climb object
  (cond
   ((eq mode :simple-floor)
    (require "package://contact_behavior_generation/euslisp/model/simple-floor.lisp")
    (setq *climb-obj* (instance simple-floor :init :z 0))
    (cond
     ((eq *robot-type* :staro)
      (setq *best-facefall*
	    (caar (rsd-deserialize
		   :file
		   (format nil "~A/euslisp/log/best-facefall.rsd"
			   (ros::rospack-find "contact_behavior_generation")))))
      (send *best-facefall* :draw :friction-cone? nil)
      (dolist (k '(:rarm :larm))
	(let* ((tag (read-from-string (format nil "~A-simple-floor-end-coords" k)))
	       (cc (make-cascoords
		    :coords (make-coords :pos (copy-seq (send *robot* k :end-coords :worldpos)))
		    :parent (send *robot* k :end-coords :parent)
		    :name tag)))
	  (send *robot* :put tag cc)))
      (setq *facefall-contact-state*
	    (let* ((name '(:rarm :larm :rleg :lleg))
		   (cc ;;(mapcar #'(lambda (k) (send *robot* k :end-coords)) name))
		    (list (send *robot* :get :rarm-simple-floor-end-coords)
			  (send *robot* :get :larm-simple-floor-end-coords)
			  (send *robot* :rleg :end-coords)
			  (send *robot* :lleg :end-coords)))
		   (tc (send-all cc :copy-worldcoords))
		   (cn (mapcar #'(lambda (a) (float-vector 0 0 1)) name))
		   (ux (mapcar #'(lambda (a) 0.5) name))
		   (uy (mapcar #'(lambda (a) 0.5) name))
		   (uz (mapcar #'(lambda (a) 0.5) name))
		   (lx (mapcar #'(lambda (a) 0.1) name))
		   (ly (mapcar #'(lambda (a) 0.1) name))
		   (f0 (mapcar #'(lambda (a) (float-vector 0 0 0 0 0 0)) name)))
	      (mapcar
	       #'(lambda (nm cc cn ux uy uz lx ly f0 tc)
		   (instance simple-contact-state
			     :init
			     :name nm :contact-coords cc :contact-n cn
			     :contact-plane-obj :floor
			     :ux ux :uy uy :uz uz :lx lx :ly ly
			     :force0 f0 :target-coords tc))
	       name cc cn ux uy uz lx ly f0 tc)))))
    )
   ((eq mode :four-leg-seat)
    (require "package://contact_behavior_generation/euslisp/model/four-leg-seat.lisp")
    (setq *climb-obj* (instance four-leg-seat :init))
    (send *climb-obj* :translate
	  (float-vector (- (/ (send *climb-obj* :get-val 'depth) -2.0) 100) 0 0)
	  :world)
    (setq *pg-float-cascoords*
          (append *pg-float-cascoords*
        	  (mapcar #'(lambda (k) (send *robot* k :end-coords)) '(:rarm :larm))))
    (setq *kca-ik-param*
	  (append *kca-ik-param*
		  (list :additional-weight-list
			(mapcar
			 #'(lambda (l) (list l 0.05))
			 (flatten (send *robot* :arms :links))))))
    (setup-hip-end-coords))
   ((eq mode :rock-wall)
    (require "package://contact_behavior_generation/euslisp/model/random-rock-wall.lisp")
    (setq *climb-obj* (instance random-rock-wall :init))
    (send *climb-obj* :translate #F(400 0 1500) :world)
;    (gl::transparent (*climb-obj* . wall-obj) 0.4)
    )
   ((eq mode :kirin-ladder)
    (require "package://contact_behavior_generation/euslisp/model/kirin-ladder.lisp")
    (setq *climb-obj* (instance kirin-ladder :init))
    (send *climb-obj* :translate #F(150 0 0) :world))
   ((eq mode :hasegawa-ladder)
    (require "package://contact_behavior_generation/euslisp/model/hasegawa-ladder.lisp")
    (setq *climb-obj* (instance hasegawa-ladder :init))
    (send *climb-obj* :translate #F(270 0 0) :world))
   ((eq mode :big-ladder)
    (require "package://contact_behavior_generation/euslisp/model/big-ladder.lisp")
    (setq *climb-obj* (instance big-ladder :init)))
   ((eq mode :garakuta-car)
    (require "package://contact_behavior_generation/euslisp/model/garakuta-car.lisp")
    (setq *kca-cog-gain* 7.0)
    ;; (setq *pg-optional-ik-contact-states*
    ;;       (append *pg-optional-ik-contact-states*
    ;;     	  (list
    ;;     	   (instance simple-contact-state :init
    ;;     		     :name :larm
    ;;     		     :translation-axis nil
    ;;     		     :rotation-axis nil))))
    (send *robot* :torso :waist-y :min-angle -15)
    (send *robot* :torso :waist-y :max-angle +15)
    (send *robot* :torso :waist-p :max-angle +40)
    (setq *kca-ik-param*
	  (append *kca-ik-param*
		  (list ;;
                        ;; :collision-avoidance-link-pair ;;nil
			;; (apply
			;;  #'append
			;;  (mapcar
			;;   #'(lambda (arm)
			;;       (mapcar
			;;        #'(lambda (leg) (list arm leg))
			;;        (append
			;; 	(subseq (send *robot* :rleg :links) 2 4)
			;; 	(subseq (send *robot* :lleg :links) 2 4)
                        ;;         (list (car (send *robot* :links)))
                        ;;         )))
			;;   (send *robot* :larm :links)))
			:avoid-collision-distance 40
			:avoid-collision-null-gain 0.8
			:avoid-collision-joint-gain 0.8
                        ;;:root-link-virtual-joint-weight
                        ;;(scale 0.005 #F(1 1 1 1 1 1))
			:additional-weight-list
                        (append
                         (mapcar
                          #'(lambda (l) (list l 1))
                          (send *robot* :larm :links))
                         (mapcar
                          #'(lambda (l) (list l 0.01))
                          ;;(send *robot* :torso :links)
                          (list (send *robot* :torso :waist-y))
                          ))
			)))
    (setq *climb-obj* (instance garakuta-car :init))
    (send *climb-obj* :rotate (deg2rad -90) :z)
    (send *climb-obj* :translate #F(-550 0 0) :world))
   (t ; :param-ladder
    (require "package://contact_behavior_generation/euslisp/model/param-ladder.lisp")
    (setq *climb-obj*
	  (instance param-ladder :init
		    :step-vector
		    (float-vector
		     (/ 200 (tan (deg2rad 90))) 0 200)
		    :step-count 20))
    (send *climb-obj* :translate #F(400 0 0) :world)
    (gl::transparent *climb-obj* 0.4)))
  (if (or (not (boundp '*viewer*))
	  (null *viewer*))
      ())
  (objects (list *robot* *climb-obj*))
  (cond
   ((eq mode :four-leg-seat)
    (send *irtviewer* :viewer :viewing :look
	  #f(500 3500 1000) (send *robot* :centroid)))
   ((eq mode :rock-wall)
    (send *irtviewer* :viewer :viewing :look
	  #f(-4500 3500 75) (send *robot* :centroid)))
   ((eq mode :simple-floor)
    (send *irtviewer* :viewer :viewing :look
	  #f(6000 3000 3000) (send *robot* :centroid)))
   )
  (send *viewer* :viewsurface :bg-color #F(1 1 1))
  (setq *contact-states*
	(or (apply #'append
		   (send *climb-obj* :gen-contact-states))
	    (flatten (gen-primitive-contact-states *climb-obj*))))
  (send *viewer* :draw-objects)
  ;; collision check defo
  ;; env
  (setq *env-collision-check-list*
	(if (find-method *climb-obj* :gen-collision-check-list)
	    (send *climb-obj* :gen-collision-check-list)
	  (list
	   (list (cons :name mode)
		 (cons :n (scale -1 (send *climb-obj* :vertical-vector)))
		 (cons :a0 (send *climb-obj* :worldpos)))
	   )))
  )

