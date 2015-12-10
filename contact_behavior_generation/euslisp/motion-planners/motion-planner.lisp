;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *log-stream* t)

(require "motion-sequencer.lisp")

;;
;; input: contact states
;; methods:
;;  contact-evaluation
;;  rsd-generate-with-contact-evaluation
;;  rsd error check
;;
(defclass abstract-motion-planner
  :super propertied-object
  :slots (robot
	  candidate-contact-states
	  sorted-contact-states
	  now-robot-state
	  ;; next-robot-state
	  rsd-list
	  error-value
	  error-thre
	  contact-state-filter-func
	  sorted-contact-state-filter-func
	  ;;
	  now-cs-key
	  cs-object-key
	  cs-eval-scale-key
	  candidates-sequencial-factor
	  sequencial-select-factor-map
	  cs-eval-gain
	  planner-sequencial-factor
	  planner-sequencial-factor-rate
	  ))
(defmethod abstract-motion-planner
  (:init
   (&rest
    args
    &key
    (name :nothing)
    ((:robot rb) *robot*)
    ((:candidate-contact-states cs) (now-contact-state))
    ((:now-robot-state rs)
     (instance robot-state-data2 :init :contact-states
	       (now-contact-state :limb-keys '(:rleg :lleg))))
    ((:error-thre et) 0.7)
    ((:contact-state-filter-func csff) 'default-contact-state-filter)
    ((:sorted-contact-state-filter-func scsff))
    ((:candidates-sequencial-factor csf) 0.8)
    ((:sequencial-select-factor-map ssfm) (make-hash-table))
    ((:planner-sequencial-factor psf) 1.0)
    ((:planner-sequencial-factor-rate psfr) 0.8)
    ((:cs-eval-gain cseg) 1.0)
    &allow-other-keys)
   (send-super :name name)
   (setq now-robot-state rs)
   (setq robot rb)
   (setq contact-state-filter-func csff)
   (setq sorted-contact-state-filter-func scsff)
   (setq candidates-sequencial-factor csf)
   (setq candidate-contact-states
	 (remove-if contact-state-filter-func (flatten cs)))
   ;;csf)
   (setq planner-sequencial-factor psf)
   (setq planner-sequencial-factor-rate psfr)
   (setq sequencial-select-factor-map ssfm)
   ;;
   (setq now-cs-key (read-from-string (format nil "~A-now-cs" name)))
   (setq cs-object-key (read-from-string (format nil "~A-objective" name)))
   (setq cs-eval-gain cseg)
   ;; (setq cs-eval-scale-key (read-from-string (format nil "~A-eval-scale" name)))
   ;; (send-all candidate-contact-states :buf cs-eval-scale-key 1)
   (setq error-thre et)
   )
  (:pop-contact-states
   (target-cs)
   (setq sorted-contact-states
	 (remove target-cs sorted-contact-states))
   (setq candidate-contact-states (remove target-cs candidate-contact-states)))
  ;; @abstract
  (:rsd-generate-with-contact-evaluation
   (&rest args &key (pop? t) target-cs black-list &allow-other-keys)
   (cond
    (pop? (send self :pop-contact-states target-cs)))
   )
  (:rsd-error-value
   (&rest args) ;; BRLV > thre?
   (setq error-value
	 (flatten
	  (mapcar
	   #'(lambda (rsd)
	       (if (and (class rsd)
			(subclassp (class rsd) robot-state-data2))
		   (mapcar
		    #'abs
		    (apply #'concatenate
			   (flatten (list cons
					  (send rsd :f/fmax)
					  (send rsd :t/tmax)))))
		 *inf*))
	   rsd-list)
	  ))
   (if (find-if #'numberp error-value) (apply #'max error-value))
   )
  (:rsd-error-check
   (&rest args) ;; BRLV > thre?
   (> (send self :rsd-error-value) error-thre))
  (:contact-evaluation
   (&key ret best now-cs buf)
   (cond
    (candidate-contact-states
     (setq ret nil)
     ;; (setq best (car candidate-contact-states))
     (dolist (cs candidate-contact-states)
       (setq now-cs
	     (find-if #'(lambda (cs2) (eq (send cs :name) (send cs2 :name)))
		      (send now-robot-state :contact-states)))
       (send cs :buf now-cs-key now-cs)
       (send cs :buf cs-object-key
	     (*
	      cs-eval-gain
	      (or (send sequencial-select-factor-map :get (send cs :name))
		  1.0)
	      (send cs :eval now-cs now-robot-state
		    :force-zero (send now-robot-state :contact-zero))))
       (cond
	((send self :sorted-contact-state-filter-func cs) 'nop)
	((or (not best)
	     (< (send best :buf cs-object-key) (send cs :buf cs-object-key)))
	 (if best (push best ret))
	 (setq best cs))
	(t (push cs ret))))
     (if best (setq ret (cons best ret)))))
   (let* ((best-cs (car ret))
	  (log))
     (dolist (cs ret) ;; logger
       ;;(setq ret (remove-if
       ;;#'(lambda (cs)
       ;;(let* ((ret (send self :sorted-contact-state-filter-func cs)))
       ;;(cond
       ;;((not ret)
       ;; (setq best-cs (or best-cs cs))
       (cond
	((not (vectorp (cdr (assoc (send cs :name) log)))) ;; initiallize
	 (setq log
	       (cons (cons (send cs :name)
			   (float-vector (or (send sequencial-select-factor-map :get (send cs :name)) 1.0)
					 1 1e+10 -1e+10))
		     log)))
	(t (setf (aref (cdr (assoc (send cs :name) log)) 1)
		 (+ (aref (cdr (assoc (send cs :name) log)) 1) 1))
	   (setf (aref (cdr (assoc (send cs :name) log)) 2)
		 (min (aref (cdr (assoc (send cs :name) log)) 2)
		      (send cs :buf cs-object-key)))
	   (setf (aref (cdr (assoc (send cs :name) log)) 3)
		 (max (aref (cdr (assoc (send cs :name) log)) 3)
		      (send cs :buf cs-object-key)))
	   )))
   ;; ret))
   ;; ret))
     (send sequencial-select-factor-map :enter
	   (send best-cs :name)
	   (* (or
	       (send sequencial-select-factor-map :get (send best-cs :name))
	       1.0) candidates-sequencial-factor))
     (format *log-stream* "[~A-eval-sequential-factor-log] ~A~%" (send self :name) log)
     )
   ;; (print (length ret))
   (setq sorted-contact-states ret)
   ;; (mapcar
   ;;  #'(lambda (cs)
   ;; 	(if (eq (send cs :name)
   ;; 		(send (car sorted-contact-states) :name))
   ;; 	    (send cs :buf cs-eval-scale-key
   ;; 		  (* (if (send cs :buf cs-eval-scale-key)
   ;; 			 (send cs :buf cs-eval-scale-key) 1.0)
   ;; 		     candidates-sequencial-factor))))
   ;;  sorted-contact-states)
   (if ret (send (car ret) :buf cs-object-key) nil))
  (:sorted-contact-state-filter-func
   (cs) (if (functionp sorted-contact-state-filter-func)
	    (funcall sorted-contact-state-filter-func cs)))
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )

;;
(defclass static-walk-motion-planner
  :super abstract-motion-planner
  :slots (remove reach suspend remove-map))
(defmethod static-walk-motion-planner
  ;; @overwrite
  (:init
   (&rest args)
   (send-super* :init
		(append args
			(list :name :static-walk))))
  ;; @overwrite
  (:rsd-generate-with-contact-evaluation
   (&rest
    args
    &key
    (robot robot)
    (time-buf (float-vector 0 0 0))
    (cnt-buf (copy-seq time-buf))
    ;;
    (update-contact-evaluation? t)
    (target-cs
     (progn
       (if update-contact-evaluation? (send self :contact-evaluation))
       (car sorted-contact-states)))
    (remove-limb (if target-cs (send target-cs :name)))
    (now-rsd now-robot-state)
    (non-stop t)
    (tmax-hand-rate 1.0)
    (tmax-leg-rate 1.0)
    (ik-debug-view nil)
    (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.5 0.5))
    (pose-generate-stop 20)
    (trajectory-check-func nil)
    (pop? t)
    args-buf
    &allow-other-keys
    )
   (send-super* :rsd-generate-with-contact-evaluation
		:pop? pop? :target-cs target-cs args)
   (if (not target-cs)
       (return-from :rsd-generate-with-contact-evaluation :no-candidate))
   (setq args-buf (list :robot robot :time-buf time-buf :call-cnt cnt-buf :update-contact-evaluation? update-contact-evaluation? :remove-limb remove-limb :now-rsd now-rsd :non-stop non-stop :tmax-hand-rate tmax-hand-rate :tmax-leg-rate tmax-leg-rate :ik-debug-view ik-debug-view :root-link-virtual-joint-weight root-link-virtual-joint-weight :pose-generate-stop pose-generate-stop))
   (setq remove
	 (or
	  ;; remove
	  (cdr (assoc remove-limb remove-map))
	  (apply #'funcall-with-time
		 (append (list #'remove-motion-sequence)
			 (list :index 0)
			 args-buf))))
   (if (not (assoc remove-limb remove-map))
       (push (cons remove-limb remove) remove-map))
   (cond
    ((or (not remove)
	 (not (send remove :full-constrainted)))
     (format *log-stream* "[:rsd-generate-with-contact-evaluation] unremovable link detected ~A for static walk planner~%"
	     remove-limb)
     (setq sorted-contact-states
	   (remove-if #'(lambda (cs) (eq (send cs :name) remove-limb)) sorted-contact-states))
     (setq candidate-contact-states sorted-contact-states)
     (return-from :rsd-generate-with-contact-evaluation :remove-impossible)))
   ;; (if (or (not remove) (not (send remove :full-constrainted)))
   ;; (return-from :rsd-generate-with-contact-evaluation :remove-fail))
   (setq reach
	 (apply #'funcall-with-time
		(append (list #'reach-motion-sequence)
			(list :index 1 :remove-rsd remove :contact-trans-mode :walk :now-rsd (or remove now-rsd)  :now-cs (send (or remove now-rsd) :contact-states) :cs-candidates (list target-cs) :slide-cs-candidates nil :trajectory-check-func trajectory-check-func)
			args-buf)))
   (if (or (not reach)
	   (not (send reach :full-constrainted)))
       (return-from :rsd-generate-with-contact-evaluation :unreachable))
   (setq suspend
	 (apply #'funcall-with-time
		(append (list #'suspend-motion-sequence)
			(list :index 2 :now-rsd reach :now-cs (send reach :contact-states) :buf (list (send reach :copy)))
			args-buf)))
   (setq rsd-list (list now-rsd remove reach suspend))
   )
  )

(defclass slide-motion-planner
  :super abstract-motion-planner
  :slots (remove reach suspend))
(defmethod slide-motion-planner
  ;; @overwrite
  (:init
   (&rest
    args
    &key
    ;; (now-cs-key :slide-now-cs)
    hoge
    &allow-other-keys)
   (send-super*
    :init
    (append
     args
     (list :name :slide
	   ;; :now-cs-key now-cs-key
	   ;; :sorted-contact-state-filter-func
	   ;; (list 'lambda '(cs)
	   ;; 	 (list 'or (list 'not (list 'send 'cs :buf now-cs-key))
	   ;; 	       (list 'not (list 'contact-plane-condition-check
	   ;; 				'cs (list 'send 'cs :buf now-cs-key)))))
	   ))))
  ;; @overwrite
  (:sorted-contact-state-filter-func
   (cs)
   ;; (format t "  slide-filter: ~A vs ~A" cs (send cs :buf now-cs-key))
   (or (not (send cs :buf now-cs-key))
       (not (contact-plane-condition-check cs (send cs :buf now-cs-key)))
       (not (eq (send cs :name) (send (send cs :buf now-cs-key) :name)))
       (eq cs (send cs :buf now-cs-key))))
  ;; @overwrite
  (:rsd-generate-with-contact-evaluation
   (&rest
    args
    &key
    (robot robot)
    (time-buf (float-vector 0 0 0))
    (cnt-buf (copy-seq time-buf))
    ;;
    (update-contact-evaluation? t)
    (target-cs
     (progn
       (if update-contact-evaluation? (send self :contact-evaluation))
       (car sorted-contact-states)))
    (remove-limb (if target-cs (send target-cs :name)))
    (now-rsd now-robot-state)
    (non-stop t)
    (tmax-hand-rate 1.0)
    (tmax-leg-rate 1.0)
    (ik-debug-view nil)
    (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.5 0.5))
    (pose-generate-stop 20)
    (trajectory-check-func nil)
    (pop? t)
    args-buf
    &allow-other-keys
    )
   (send-super* :rsd-generate-with-contact-evaluation
		:pop? pop? :target-cs target-cs args)
   (if (not target-cs)
       (return-from :rsd-generate-with-contact-evaluation :no-candidate))
   (setq args-buf (list :robot robot :time-buf time-buf :call-cnt cnt-buf :update-contact-evaluation? update-contact-evaluation? :remove-limb remove-limb :now-rsd now-rsd :non-stop non-stop :tmax-hand-rate tmax-hand-rate :tmax-leg-rate tmax-leg-rate :ik-debug-view ik-debug-view :root-link-virtual-joint-weight root-link-virtual-joint-weight :pose-generate-stop pose-generate-stop))
   (setq remove ;; not used?
	 (apply #'funcall-with-time
		(append (list #'remove-motion-sequence)
			(list :index 0)
			args-buf)))
   ;;(if (or (not remove) (not (send remove :full-constrainted)))
   ;; (return-from :rsd-generate-with-contact-evaluation :remove-fail))
   (setq reach
	 (apply #'funcall-with-time
		(append (list #'reach-motion-sequence)
			(list :index 1 :contact-trans-mode :slide :now-rsd (or remove now-rsd)  :now-cs (send (or remove now-rsd) :contact-states) :remove-cs (find-if #'(lambda (cs) (eq remove-limb (send cs :name))) (send (or remove now-rsd) :contact-states)) :cs-candidates nil :slide-cs-candidates (list target-cs) :trajectory-check-func trajectory-check-func)
			args-buf)))
   (if (or (not reach)
	   (not (send reach :full-constrainted)))
       (return-from :rsd-generate-with-contact-evaluation :unreachable))
   (setq remove (send reach :buf :slide))
   (if (or (not remove)
	   (not (send remove :full-constrainted)))
       (return-from :rsd-generate-with-contact-evaluation :unremovable))
   (setq suspend
	 (apply #'funcall-with-time
		(append (list #'suspend-motion-sequence)
			(list :index 2 :now-rsd reach :now-cs (send reach :contact-states) :buf (list (send reach :copy)))
			args-buf)))
   (if (or (not suspend)
	   (not (send suspend :full-constrainted)))
       (return-from :rsd-generate-with-contact-evaluation :unstable))
   (setq rsd-list (list now-rsd remove reach suspend))
   )
  )

(defclass remove-motion-planner
  :super abstract-motion-planner
  :slots (remove suspend remove-tag))
(defmethod remove-motion-planner
  ;; @overwrite
  (:init
   (&rest
    args
    &key
    ((:remove-tag rt) :torso)
    &allow-other-keys)
   (send-super*
    :init
    (append
     args
     (list :name (read-from-string (format nil "~A-remove" (setq remove-tag rt)))
	   :contact-state-filter-func '(lambda (&rest args) nil)))
    ))
  ;; @Overwrite
  (:contact-evaluation
   (&rest args)
   (setq sorted-contact-states
	 (flatten (list (car candidate-contact-states))))
   (setq candidate-contact-states sorted-contact-states)
   (cond
    ((null sorted-contact-states) nil)
    ((and
      (> (length (send now-robot-state :contact-states)) 3)
      (find remove-tag
	    (send-all (send now-robot-state :contact-states) :name)))
     (send (car sorted-contact-states) :buf cs-object-key 1e+100)
     (send (car sorted-contact-states) :buf cs-object-key))
    (t nil)))
  ;; @Overwrite
  ;; (:pop-contact-states (&rest args) nil)
  ;; @overwrite
  (:rsd-generate-with-contact-evaluation
   (&rest
    args
    &key
    (robot robot)
    (time-buf (float-vector 0 0 0))
    (cnt-buf (copy-seq time-buf))
    ;;
    (update-contact-evaluation? t)
    (target-cs
     (progn
       (if update-contact-evaluation? (send self :contact-evaluation))
       (car sorted-contact-states)))
    (remove-limb remove-tag)
    (now-rsd now-robot-state)
    (non-stop t)
    (tmax-hand-rate 1.0)
    (tmax-leg-rate 1.0)
    (ik-debug-view nil)
    (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.5 0.5))
    (pose-generate-stop 20)
    (trajectory-check-func nil)
    (pop? t)
    args-buf
    &allow-other-keys
    )
   (send-super* :rsd-generate-with-contact-evaluation
		:pop? pop? :target-cs target-cs args)
   (setq args-buf (list :robot robot :time-buf time-buf :call-cnt cnt-buf :remove-limb remove-limb :now-rsd now-rsd :non-stop non-stop :tmax-hand-rate tmax-hand-rate :tmax-leg-rate tmax-leg-rate :ik-debug-view ik-debug-view :root-link-virtual-joint-weight root-link-virtual-joint-weight :pose-generate-stop pose-generate-stop))
   (setq remove
	 (apply #'funcall-with-time
		(append (list #'remove-motion-sequence)
			(list :index 0)
			args-buf)))
   (if (or (not remove) (not (send remove :full-constrainted)))
       (return-from :rsd-generate-with-contact-evaluation :remove-fail))
   (setq suspend
	 (pose-generate-with-contact-state
	  (remove-if #'(lambda (cs) (eq (send cs :name) remove-limb))
		     (send remove :contact-states))))
   (setq suspend
	 (apply #'funcall-with-time
		(append (list #'suspend-motion-sequence)
			(list :index 2 :now-rsd suspend :now-cs (send suspend :contact-states))
			args-buf)))
   (setq rsd-list (list now-rsd remove suspend))
   )
  )


(defun gen-comparable-object-from-planner
  (pln target-cs)
  (list (send pln :name)
	target-cs
	(send (send pln :now-robot-state) :contact-states)))

(defun compare-planner-data
  (pln1 pln2)
  (and (eq (nth 0 pln1) (nth 0 pln2))
       (eq (nth 1 pln1) (nth 1 pln2))
       (eq (length (nth 2 pln1)) (length (nth 2 pln2)))
       (zerop (count nil
		     (mapcar
		      #'(lambda (cs) (find cs (nth 2 pln2)))
		      (nth 2 pln1))))))

(defclass stack-list
  :super cons
  :slots (lst max-stack))
(defmethod stack-list
  (:init (&rest args &key
		((:list l) nil)
		((:max-stack ms) 1000)
		&allow-other-keys)
	 (setq max-stack ms)
	 (setq lst l))
  (:list nil lst)
  (:car nil (car lst))
  (:cdr nil (cdr lst))
  (:pop nil (pop lst))
  (:push (a) (push a lst)
	 (if (> (length lst) max-stack)
	     (setq lst (subseq lst 0 max-stack)))
	 lst)
  )

(defun object-serial-id
  (obj
   &key
   (cls (if (class obj) (send (class obj) :name)))
   (id (if (class obj) (coerce (butlast (cdr (member #\# (cdr (coerce (format nil "~A" obj) cons))))) string)))
   (extra ""))
  (if (class obj)
      (read-from-string
       (format nil "~A~A~A" cls id extra))
    obj))

(defclass planner-graph-object
  :super propertied-object
  :slots (rsd from-node to-node
	      from-id to-id
	      from-name to-name
	      planner-obj planner-name))
(defmethod planner-graph-object
  (:init
   (from to
	 &key
	 ((:planner pln))
	 ((:rsd-list rsd-list))
	 ((:from-id fi) 0)
	 ((:to-id ti) 0)
	 ((:from-name fnm)
	  (object-serial-id from :extra fi))
	 ((:to-name tnm)
	  (object-serial-id to :extra ti))
	 ((:planner-name pname)
	  (if (and (class pln) (find-method pln :name))
	      (send pln :name) pln))
	 )
   (setq from-node from
	 to-node to
	 planner-obj pln
	 planner-name pname
	 rsd rsd-list
	 from-id fi
	 to-id ti
	 from-name fnm
	 to-name tnm))
  (:from-name nil from-name)
  (:to-name nil to-name)
  (:from nil from-node)
  (:to nil to-node)
  (:planner nil planner-obj)
  (:planner-name nil planner-name)
  (:rsd-list nil rsd-list))

(defun sequencial-select-filter
  (cs-list &optional ret all-cs-tag-name ref-order sequencial-select-factor-map)
  ;; seuqencial factor filter
  ;; (print 'sequencial-select-filter)
  ;; (print ret)
  (let* ((max 6)
	 (rret (reverse (flatten ret)))
	 (rret-tag (flatten (send-all rret :buf :remove-limb)))
	 (keys
	  (if ref-order
	      (let* ((p (if (car rret-tag)
			    (position (car rret-tag) ref-order))))
		(reverse
		 (if (and p (< (+ 1 p) (length ref-order)))
		     (append (subseq ref-order (+ 1 p))
			     (subseq ref-order 0 (+ 1 p)))
		   ref-order)))
	    (subseq
	     rret-tag 0 (* max (length all-cs-tag-name)))))
	 (len (max max (length keys)))
	 id ret
	 (sequence-scale
	  (mapcar
	   #'(lambda (k)
	       ;; (setq ret 1)
	       ;; (setq id 0)
	       ;; (dotimes (i max)
	       ;; 	 (setq id (position k keys :start id))
	       ;; 	 (if (not id) (return-from nil nil))
	       ;; 	 (setq ret (* ret (/ (+ id 1.0) len))))
	       (setq ret 1)
	       (setq id 0)
	       (dotimes (i max)
		 (setq id (position k keys :start id))
		 (if (not id) (return-from nil nil))
		 (setq ret (* ret (- len id))))
	       (setq ret (/ 1.0 (+ 1.0 ret)))
	       (send sequencial-select-factor-map :enter k ret)
	       (cons k ret))
	   all-cs-tag-name)))
    (format *log-stream* "[sequential-factor] ~A~%" sequence-scale)
    ;;(mapcar
    ;;#'(lambda (cs)
    ;;(send cs :set-val 'eval-scale (cdr (assoc (send cs :name) sequence-scale)))
    ;;cs)
    ;;cs-list)
    cs-list
    ))

(defvar *graph-data-buf*)
(defun demo-motion-sequence2
  (&rest
   args
   &key
   (robot *robot*)
   ;; (now-rsd
   ;;  (instance robot-state-data2 :init
   ;; 	      :contact-states
   ;; 	      (now-contact-state :limb-keys '(:rarm :larm :rleg :lleg))))
   (now-rsd
    (optimize-brli
     :robot robot
     :contact-states
     (now-contact-state :limb-keys '(:rleg :lleg))))
   (gbrli (send now-rsd :gbrli))
   (now-cs (send now-rsd :contact-states))
   (all-cs-candidates
    (progn
      (if (not (and (boundp '*contact-states*)
		    *contact-states*))
	  (demo-climb-setup :param-ladder))
      (flatten *contact-states*)))
   (all-cs-tag-name
    (union (send-all all-cs-candidates :name) nil))
   ;;
   ret
   ;;
   (loop-cnt 1)
   (rec-cnt 0)
   (loop-max 4)
   (rms-loop-max 4)
   (rec-max rms-loop-max)
   (rec-max-without-best-rsd rec-max)
   (back-track t)
   (non-stop t)
   (tmax-leg-rate 1.0)
   (tmax-hand-rate 1.0)
   (error-thre 0.7)
   (ik-debug-view :no-message)
   (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.5 0.5))
   (cs-filter-func 'sequencial-select-filter)
   (ref-order nil)
   (motion-planner-names
    (list static-walk-motion-planner slide-motion-planner))
   (sequencial-select-factor-map (make-hash-table))
   (cs-eval-gains ;; default priority mode
    (let* ((buf) (gain 1.0) (step 0.99))
      (dolist (l motion-planner-names)
	(push gain buf) (setq gain (* step gain)))
      (reverse buf)))
   (motion-planners
    (let ((cs-candidates (if (functionp cs-filter-func)
			     (funcall cs-filter-func
				      all-cs-candidates
				      ret
				      all-cs-tag-name
				      ref-order
				      sequencial-select-factor-map
				      )
			   all-cs-candidates)))
      (mapcar
       #'(lambda (name g)
	   (instance* name :init
		      :candidate-contact-states cs-candidates
		      :sequencial-select-factor-map
		      (copy-object sequencial-select-factor-map)
		      :now-robot-state now-rsd
		      :cs-eval-gain g
		      args))
       motion-planner-names cs-eval-gains)))
   ;;
   (graph-stack nil) ;;(instance stack-list :init))
   (time-buf (float-vector 0 0 0))
   (time-bt-buf (copy-seq time-buf))
   (cnt-buf (copy-seq time-buf))
   (cnt-bt-buf (copy-seq time-buf))
   (prev-time-buf (copy-seq time-buf))
   (pose-generate-stop 20)
   (mseq-log-stream *log-stream*)
   trajectory-check-func
   new-rsd-check-func
   ;;
   new-rsd best-rsd best-rsd-buf
   selected-planner
   cs-evaluated-value-list
   (black-list (instance stack-list :init))
   comp-plan;; deep-track-call?
   greedy-search-buf tmp
   &allow-other-keys)
  ;; remove black-listed args
  (setq args (remove-args-values args (list :remove-limb)))
  ;;
  (send now-rsd :draw :friction-cone? nil) ;; for back track
  (if (or (not now-rsd) (not (send now-rsd :full-constrainted)))
      (return-from demo-motion-sequence2 :agry-now-rsd))
  (if (and (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects))
  (format mseq-log-stream "[motion-seq] ~A(~A)/~A(~A|~A) for~%"
	  loop-cnt rec-cnt loop-max rec-max rec-max-without-best-rsd)
  (cond
   ;;
   ;; loop exceeded check
   ((if (functionp loop-max) (funcall loop-max loop-cnt)
      (> loop-cnt loop-max))
    (format mseq-log-stream "[motion-seq] too deep~%")
    (cons :loop-exeeded ret))
   ;;
   ;; candidates existance check
   ((null
     (flatten
      (setq cs-evaluated-value-list
	    (send-all motion-planners :contact-evaluation))))
    (format mseq-log-stream "[motion-seq] no candidate~%")
    (cons :no-candidates ret))
   ;;
   ;; leaf count check
   ((or
     (and best-rsd
	  (> rec-cnt rec-max))
     (> rec-cnt rec-max-without-best-rsd))
    (format mseq-log-stream
	    "[motion-seq] pararrel search loop exceed~%")
    (setq best-rsd
	  (sort best-rsd #'(lambda (a b) (< (car a) (car b)))))
    (format mseq-log-stream "[motion-seq] check bad rsd again ~A~%"
	    best-rsd)
    (setq greedy-search-buf nil)
    (dolist (rsd best-rsd)
      (format mseq-log-stream "[motion-seq] bad deep track ~A/~A~%"
	      loop-cnt rec-cnt)
      (if (and
	   (setq greedy-search-buf
		 (apply #'demo-motion-sequence2
			(append
			 (list :now-rsd (car (last rsd))
			       :ret (append ret (cdr rsd))
			       :loop-cnt (+ loop-cnt 1)
			       :rec-cnt 0
			       :black-list black-list
			       :best-rsd nil
			       :graph-stack graph-stack)
			 (remove-args-values
			  args
			  (list :motion-planners ;; regen planner
				:now-rsd :ret
				:loop-cnt :rec-cnt :black-list
				:best-rsd :graph-stack)))))
	   (find (car greedy-search-buf) (list :loop-exeeded)))
	  (return-from nil nil))
      (format mseq-log-stream "[motion-seq] bad deep back ~A/~A~%"
	      loop-cnt rec-cnt)
      (if graph-stack
	  (send graph-stack :push
		(instance planner-graph-object :init
			  (car (last rsd)) now-rsd
			  :from-id 0
			  :to-id (or (send (car (last rsd))
					   :buf :rec-cnt) 0)
			  :planner-name (or (car greedy-search-buf)
					    :no-solution))))
      )
    (or greedy-search-buf (cons :parrarel-loop-exceeded ret)))
   ;;
   ;; select and call motion-planner
   ((and
     (setq selected-planner
	   (cdar (sort
		  (apply
		   #'append
		   (mapcar
		    #'(lambda (pl val)
			(if val
			    (list
			     (cons (* (send pl :planner-sequencial-factor) val)
				   pl))))
		    motion-planners cs-evaluated-value-list))
		  #'(lambda (a b) (> (car a) (car b))))))
     (or (format mseq-log-stream "[motion-seq] planner=~A~%"
		 (send selected-planner :name))
	 (send selected-planner :set-val
	       'planner-sequencial-factor
	       (* (send selected-planner :planner-sequencial-factor-rate)
		  (send selected-planner :planner-sequencial-factor)))
	 t)
     (setq comp-plan
	   (gen-comparable-object-from-planner
	    selected-planner
	    (car (send selected-planner :sorted-contact-states))))
     (or (format mseq-log-stream "[motion-seq] selected cs=~A(~A)~%"
		 (car (send selected-planner :sorted-contact-states))
		 (if (car (send selected-planner :sorted-contact-states))
		     (send (car (send selected-planner :sorted-contact-states))
			   :name))
		 ) t)
     (let ((buf (find-if
		 #'(lambda (pl) (compare-planner-data comp-plan pl))
		 (send black-list :list))))
       (cond
	(buf
	 (format mseq-log-stream "[motion-seq] detect wanted~A~%" buf)
	 (send selected-planner :pop-contact-states
	       (car (send selected-planner :sorted-contact-states)))))
       (not buf))
     (send black-list :push comp-plan)
     (or (format mseq-log-stream "[motion-seq] update black-list=~A~%"
		 comp-plan) t)
     (setq new-rsd
	   (send*
	    selected-planner
	    :rsd-generate-with-contact-evaluation
	    :update-contact-evaluation? nil
	    args))
     (listp new-rsd)
     (not (find-if #'(lambda (rsd)
		       (not (and (class rsd)
				 (subclassp (class rsd)
					    robot-state-data2))))
		   new-rsd))
     (or (not (functionp new-rsd-check-func))
	 (funcall new-rsd-check-func new-rsd))
     (progn
       (if graph-stack
	   (send graph-stack :push
		 (instance planner-graph-object :init
			   now-rsd (car (last new-rsd))
			   :from-id rec-cnt :to-id 0
			   :planner selected-planner
			   :planner-name
			   (format nil "~A(BRLV=~A/~A)"
				   (send selected-planner :name)
				   (send selected-planner
					 :rsd-error-value)
				   error-thre
				   ))))
       t)
     (zerop (count nil (send-all new-rsd :full-constrainted)))
     (or (format mseq-log-stream "[motion-seq] succeed to satisfy all constraints=~A~%" (send selected-planner :name)) t)
     (send (car (last new-rsd)) :buf :rec-cnt rec-cnt)
     (send (car (last new-rsd)) :buf :selected-planner selected-planner)
     (setq best-rsd-buf
	   (cons (send selected-planner :rsd-error-value) new-rsd))
     (or (format mseq-log-stream "[motion-seq] BRLV(~A) > ~A?~%" (car best-rsd-buf) error-thre) t)
     (not (send selected-planner :rsd-error-check))
     (progn
       (format mseq-log-stream "[motion-seq] deep track ~A/~A~%"
	       loop-cnt rec-cnt)
       t)
     (setq greedy-search-buf
	   (apply #'demo-motion-sequence2
		  (append
		   (list :now-rsd (car (last new-rsd))
			 :ret (append ret new-rsd)
			 :loop-cnt (+ loop-cnt 1)
			 :rec-cnt 0
			 :best-rsd nil
			 :black-list black-list
			 :graph-stack graph-stack)
		   (remove-args-values
		    args
		    (list :motion-planners ;; regen planner
			  :now-rsd :ret :loop-cnt
			  :rec-cnt :best-rsd :black-list
			  :graph-stack)))))
     (progn
       (format mseq-log-stream "[motion-seq] deep back ~A/~A~%"
	       loop-cnt rec-cnt)
       (setq best-rsd-buf nil) ;;(cdr best-rsd-buf))
       (if graph-stack
	   (send graph-stack :push
		 (instance planner-graph-object :init
			   (car (last new-rsd)) now-rsd
			   :from-id 0 :to-id rec-cnt
			   :planner-name (or (car greedy-search-buf) :no-solution))))
       t)
     (find (car greedy-search-buf) (list :loop-exeeded)))
    ;; greedy deep search and success
    ;; (if graph-stack
    ;; (send (send graph-stack :car) :put :end t))
    greedy-search-buf
    )
   (t
    (format mseq-log-stream "[motion-seq] parrarel track ~A/~A~%"
	    loop-cnt rec-cnt)
    (if graph-stack
	(send graph-stack :push
	      (instance planner-graph-object :init
			now-rsd now-rsd
			:from-id rec-cnt :to-id (+ rec-cnt 1)
			;;:planner-name :parrarel-track
			:planner-name
			(if selected-planner
			    (format nil "~A(BRLV=~A/~A)"
				    (send selected-planner :name)
				    (send selected-planner
					  :rsd-error-value)
				    error-thre
				    )
			  :no-planner)
			)))
    (setq greedy-search-buf
	  (apply #'demo-motion-sequence2
		 (append
		  (list :ret ret
			:now-rsd now-rsd
			:best-rsd
			(if best-rsd-buf
			    (cons best-rsd-buf best-rsd) best-rsd)
			:rec-cnt (+ rec-cnt 1)
			:loop-cnt loop-cnt
			:black-list black-list
			:motion-planners motion-planners
			:graph-stack graph-stack)
		  (remove-args-values
		   args
		   (list :ret :now-rsd :best-rsd
			 :rec-cnt :loop-cnt :black-list
			 :motion-planners :graph-stack)))))
    (format mseq-log-stream "[motion-seq] parrarel back ~A/~A~%"
	    loop-cnt rec-cnt)
    (if graph-stack
	(send graph-stack :push
	      (instance planner-graph-object :init
			now-rsd  now-rsd
			:from-id (+ rec-cnt 1) :to-id rec-cnt
			:planner-name (or (car greedy-search-buf) :no-solution))))
    ;; (find (car greedy-search-buf) (list :loop-exeeded)))
    ;; greedy pararrel search and success
    greedy-search-buf
    )
   ))

(defun demo-motion-sequence2-with-timer
  (&rest
   args
   &key
   (tag "demo-motion-sequence2-with-timer")
   (log-root (format nil "log/~A_~A" tag (log-surfix)))
   (log-file (format nil "~A/tmp" log-root))
   (log-stream
    (progn
      (if (not (probe-file log-root))
	  (unix:system (format nil "mkdir -p ~A" log-root)))
      (open log-file :direction :output)))
   (dump-graphviz? nil)
   (graph-stack (setq *sl* (instance stack-list :init)))
   (log-stream-buf *log-stream*)
   &allow-other-keys)
  (let* (tm ret)
    (setq *log-stream* log-stream)
    (setq tm
	  (bench2
	   (setq
	    ret
	    (apply #'demo-motion-sequence2
		   (append
		    args
		    (list :graph-stack graph-stack))))))
    (format *log-stream* (format nil "time: ~A sec~%" tm))
    (close *log-stream*)
    (setq *log-stream* log-stream-buf)
    (if dump-graphviz?
	(dump-graphviz-file *sl* :output-dir log-root :red-rsd-list ret))
    (send-all (cdr ret) :buf :selected-planner nil)
    (rsd-serialize :rsd-list ret :file (format nil "~A.rsd" log-file))
    (unix:system
     (format nil
	     "rm -rf latest_~A; ln -s ~A latest_~A;"
	     tag log-root tag))
    ret
    ))

;; (progn (setq *ret* (demo-standup-motion-plan :graph-stack (setq *sl* (instance stack-list :init)))) (dump-graphviz-file *sl* :red-rsd-list *ret*))
;; (dump-graphviz-file *sl* :red-rsd-list *ret*)
(defun dump-graphviz-file
  (graph-stack
   &key
   (red-rsd-list nil)
   (output-dir (remove #\: (format nil "graphviz/~A" (log-surfix))))
   (output-path (format nil "~A/graphviz.dot" output-dir))
   (output
    (progn
      (if (not (probe-file output-dir))
	  (unix:system (format nil "mkdir -p ~A" output-dir)))
      (open output-path :direction :output)))
   (image-output? t)
   black-list (first-green? nil)
   )
  (format t "[dump-graphviz-file] write to ~A~%" output-path)
  (labels ((ng-string
	    (str
	     &key
	     (ng-data (list (cons #\- #\_)
			    (cons #\: nil)))
	     (str-list (coerce (format nil "~A" str) cons))
	     tmp
	     buf)
	    (dolist (d (reverse str-list))
	      (setq d (char-downcase d))
	      (push (if (assoc d ng-data)
			(cdr (assoc d ng-data))
		      d) buf))
	    (coerce (flatten buf) string)))
    (format output "digraph {~%")
    (format output "node [shape = circle];~%")
    (format output "graph [size=\"8.3,11.7\", ratio=\"1.4\"]~%")
    (format output "edge [fontsize=120, arrowsize = 10]~%")
    (if image-output?
	(dolist (graph (send graph-stack :list))
	  (cond
	   ((and (class (send graph :from))
		 (subclassp (class (send graph :from))
			    robot-state-data2)
		 ;;(not (find (ng-string (send graph :from-name))
		 ;;black-list :test #'string-equal))
		 )
	    (push (ng-string (send graph :from-name)) black-list)
	    (send (send graph :from) :draw :rest
		  (list *climb-obj*))
	    (send *viewer* :viewsurface :write-to-jpg-file
		  (format nil "~A/~A.jpg"
			  output-dir
			  (ng-string (send graph :from-name))))
	    (format output
		    "~A [image=\"~A\", color=\"~A\"];~%"
		    (ng-string (send graph :from-name))
		    (format nil "~A.jpg"
			    (ng-string (send graph :from-name)))
		    (cond
		     ((or
		       ;;(eq graph (car (send graph-stack :list)))
		       (eq graph
			   (car (last (send graph-stack :list))))
		       (send graph :get :end)
		       )
		      "#FF0000")
		     ((and
			 (find (send graph :to) red-rsd-list)
			 (find (send graph :from) red-rsd-list)
			 (class (send graph :planner))
			 (subclassp (class (send graph :planner))
				    abstract-motion-planner))
		      (cond
		       ((not first-green?)
			(setq first-green? t)
			(send graph :put :to-color "#FF0000")))
		      "#00FF00")
		     (t "#000000")))
	    ))
	  (cond
	   ((and (class (send graph :to))
		 (subclassp (class (send graph :to))
			    robot-state-data2)
		 (not (find (ng-string (send graph :to-name))
			    black-list :test #'string-equal)))
	    (push (ng-string (send graph :to-name)) black-list)
	    (send (send graph :to) :draw :rest
		  (list *climb-obj*))
	    (send *viewer* :viewsurface :write-to-jpg-file
		  (format nil "~A/~A.jpg"
			  output-dir
			  (ng-string (send graph :to-name))))
	    (format output
		    "~A [image=\"~A\", color=\"~A\"];~%"
		    (ng-string (send graph :to-name))
		    (format nil "~A.jpg"
			    (ng-string (send graph :to-name)))
		    (or (send graph :get :to-color)
			"#000000"))))
	  ))
    (dolist (graph (send graph-stack :list))
      (format output
	      "~A -> ~A [label=\"~A\", dir=~A];~%"
	      (ng-string (send graph :from-name))
	      (ng-string (send graph :to-name))
	      (cond
	       ((eq :loop-exeeded (send graph :planner-name)) "")
	       ((eq :parrarel-track (send graph :planner-name)) "")
	       (t (ng-string (send graph :planner-name))))
	      (cond
	       ((eq :parrarel-track (send graph :planner-name))
		"none")
	       (t "forward"))
	      ))
    (format output "}~%")
    (close output)
    output-dir))


#|

(defun demo-static-walk-motion-planner
  nil
  (cond
   ((not (and (boundp '*viewer*) *viewer*))
    (pickview :no-menu t)
    (objects (list *robot*))))
  (setq *swmp* (instance static-walk-motion-planner :init))
  (setq *walk-rsd* (send *swmp* :rsd-generate-with-contact-evaluation))
  (if (listp *walk-rsd*)
      (rsd-play :rsd-list (reverse *walk-rsd*) :graph nil :auto? t))
  *walk-rsd*)

(defun demo-slide-motion-planner
  nil
  (cond
   ((not (and (boundp '*viewer*) *viewer*))
    (pickview :no-menu t)
    (objects (list *robot*))))
  (setq *smp* (instance slide-motion-planner :init))
  (setq *slide-rsd* (send *smp* :rsd-generate-with-contact-evaluation))
  (if (listp *slide-rsd*)
      (rsd-play :rsd-list (reverse *slide-rsd*) :graph nil :auto? t))
  *slide-rsd*)

roseus
(defvar *robot-type* :staro)
(load "motion-planner.lisp")

(demo-climb-setup :simple-floor)
(send *best-facefall* :draw :friction-cone? nil)

(demo-motion-sequence2
 :now-rsd;; (pose-generate-with-contact-state (now-contact-state))
 (let* ((rsd (pose-generate-with-contact-state
	      (now-contact-state)))
	(cs (send rsd :contact-states)))
   (mapcar #'(lambda (cs) (send cs :set-val 'contact-plane-obj :floor)) cs)
   rsd)
 :rec-max 30
 ;; :cs-filter-func nil
 )


(setq *smp*
      (instance slide-motion-planner :init
		:now-robot-state
		(let* ((rsd (pose-generate-with-contact-state
			     (now-contact-state)))
		       (cs (send rsd :contact-states)))
		  (mapcar #'(lambda (cs) (send cs :set-val 'contact-plane-obj :floor)) cs)
		  rsd)
		:candidate-contact-states (flatten *contact-states*)))

(defun pop-test
  (&key (cnt 0)
	(stack nil))
  (push cnt stack)
  (cond
   ((eq cnt 10) nil)
   (t
    (pop-test :cnt (+ cnt 1)
	      :stack stack)
    (print stack))))


(let* ((id -1))
  (mapcar
   #'(lambda (rsd)
       (send rsd :draw :rest (list *climb-obj*))
       (send *viewer* :viewsurface :write-to-jpg-file
	     (format nil "staro_standup_log/standup_pose~A.jpg"
		     (incf id))))
   (reverse (cdr *ret*))))
(rsd-play :rsd-list (cdr *ret*) :auto? t)

(require "util/gp-util.lisp")
(mapcar
 #'(lambda (graph)
     (if t
	 (graph-panel2gp-graph graph :ylabel "BRLV ELEMENTS" :xlabel "ROBOT STATE IDs" :save? (format nil "staro_standup_log/~A_brlv.eps" (remove #\: (send graph :name))) :ratio 1.0)))
 *graph-sample*)
