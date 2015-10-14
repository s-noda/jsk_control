;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "my-util.lisp")
(require "contact-state.lisp")
(require "robot-param.lisp")
(require "ik-without-collision-org.lisp")

(defvar *log-stream* t)

(defclass robot-state-data2
  :super object
  :slots (contact-keys
	  contact-states
	  contact-zero
	  contact-forces
	  angle-vector
	  hand-angle-vector
	  root-coords
	  root-acc
	  ;;
	  torque-vector
	  contact-torque
	  contact-torque-max
	  friction-brli
	  torque-brli
	  brli-gain
	  brli
	  gbrli
	  gbrli-list
	  ;;
	  zmp
	  f/fmax
	  t/tmax
	  tf-error
	  full-constrainted
	  buf
	  ;;
	  robot2root-coords ;; transformation may be changed
	  ;;
	  ;;	  contact-links
	  ))

(defmethod robot-state-data2
  (:init
   (&key
    (robot *robot*)
    ((:contact-states cs) (now-contact-state))
    ((:contact-keys ck) (send-all cs :name))
    ((:contact-coords cc) (send-all cs :contact-coords))
    ((:contact-n cn) (send-all cs :contact-n))
    ((:contact-forces cf)
     (make-list (length ck) :initial-element #F(0 0 0 0 0 0)))
    ((:contact-zero cz)
     (scale (/ 1.0 (length cc))
	    (reduce #'v+ (append (list #F(0 0 0) #F(0 0 0))
				 (send-all
				  (send-all cs :target-coords)
				  :worldpos)))))
    ((:root-acc ra)
     (let ((mg (scale (* 1e-3 (send robot :weight)) (scale -1e-3 *g-vec*))))
       ;;#F(0 0 -9.8))))
       (v+ (transform
	    (send robot :calc-grasp-matrix
		  (mapcar #'(lambda (v) (v- v cz)) (send-all cc :worldpos)))
	    (apply #'concatenate float-vector cf))
	   (concatenate float-vector mg
			(scale 1e-3
			       (v* (v- (send robot :centroid) cz) mg))))))
    ((:angle-vector av) (copy-seq (send robot :angle-vector)))
    ((:hand-angle-vector hav)
     (if (find-method (send robot :rarm) :hand)
	 (copy-object (send robot :arms :hand :angle-vector))))
    ((:root-coords rc) (copy-object (send robot :worldcoords)))
    ((:contact-links cl)
     (mapcar
      #'(lambda (c l)
	  (send robot :link-list (send c :parent)
		(if (find-method robot l) (send robot l :root-link))))
      cc ck))
    (torque-check t)
    ((:torque-vector tv)
     (if torque-check
	 (copy-object
	  (send robot :torque-vector
		:target-coords cc
		:force-list (mapcar #'(lambda (f) (subseq f 0 3)) cf)
		:moment-list (mapcar #'(lambda (f) (subseq f 3 6)) cf)))))
    ((:contact-torque ct)
     (mapcar #'(lambda (ll)
		 (coerce (send-all (send-all ll :joint) :joint-torque)
			 float-vector)) cl))
    ((:contact-torque-max ctm)
     (mapcar #'(lambda (ll)
		 (coerce (send-all (send-all ll :joint) :max-joint-torque)
			 float-vector)) cl))
    ((:friction-brli fb)
     (mapcar #'(lambda (cs cf) (send cs :force-error-world cf)) cs cf))
    ((:f/fmax ff) (send-all cs :f/fmax))
    ((:t/tmax tt) (mapcar
		   #'(lambda (ct ctm) (map float-vector #'/ ct ctm))
		   ct ctm))
    ((:tf-error tf) (apply #'concatenate float-vector (append tt ff)))
    ((:torque-brli tb)
     (mapcar #'(lambda (tb j)
		 (transform (pseudo-inverse (transpose j)) tb))
	     tt
	     (mapcar
	      #'(lambda (l c)
		  (send robot :calc-jacobian-from-link-list
			l
			:move-target c
			:transform-coords (make-coords)
			:translation-axis '(t)
			:rotation-axis '(t)))
	      cl cc)))
    ((:brli-gain bg) 1)
    ((:brli br)
     (mapcar #'(lambda (fb tb) (v- fb (scale bg tb))) fb tb))
    ((:gbrli-list gbrl)
     (mapcar
      #'(lambda (grasp br) (transform grasp br))
      (mapcar
       #'(lambda (v) (send robot :calc-grasp-matrix (list (v- v cz))))
       (send-all cc :worldpos)) br))
    ((:gbrli gbr)
     (cond ((> (length gbrl) 1) (reduce #'v+ gbrl))
	   (t (car gbrl))))
    ((:full-constrainted fc)
     (and
      (< (norm ra) 1)
      (<
       (apply #'max
	      (mapcar #'abs
		      (flatten
		       (apply #'concatenate
			      (cons cons
				    (append
				     (mapcar #'(lambda (f) (setf (aref f 2) 0) f) ff)
				     tt))))))
       1.05)))
    ((:robot2root-coords r2c)
     (send (copy-object (send robot :worldcoords))
	   :transformation
	   (send (car (send robot :links)) :copy-worldcoords)))
    ((:buf b) nil)
    (log nil)
    &allow-other-keys
    )
   ;;
   (format *log-stream* "[rsd initialize]~%")
   (setq contact-keys ck)
   (setq contact-forces cf)
   (setq contact-states cs)
   (setq contact-zero cz)
   (setq angle-vector av)
   (setq torque-vector tv)
   (setq hand-angle-vector hav)
   (setq root-coords rc)
   (setq root-acc ra)
   ;;
   (setq contact-torque ct)
   (setq contact-torque-max ctm)
   (setq friction-brli fb)
   (setq torque-brli tb)
   (setq brli-gain bg)
   (setq brli br)
   (setq gbrli gbr)
   (setq gbrli-list gbrl)
   ;;
   (setq f/fmax ff)
   (setq t/tmax tt)
   (setq tf-error tf)
   (setq full-constrainted fc)
   (setq buf b)
   ;;
   (setq robot2root-coords r2c)
   ;;
   (if log (send self :log))
   self
   )
  (:fix-coords
   nil
   (send-all contact-states :fix-coords))
					;   (setq contact-states
					;	 (mapcar
					;	  #'(lambda (cs)
					;	      (send cs :copy
					;		    :target-coords
					;		    (copy-object
					;		     (send (send cs :contact-coords)
					;			   :worldcoords))))
					;	  contact-states)))
  (:copy
   (&rest args)
   (eval
    (append
     '(instance robot-state-data2 :init)
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
  (:get-link-list
   (&key (robot *robot*))
   (mapcar
    #'(lambda (c l)
	(send robot :link-list (send c :parent)
	      (if (find-method robot l) (send robot l :root-link))))
    (send-all contact-states :contact-coords) contact-keys))
  (:torque-vector
   (&rest args)
   (cond
    ((vectorp (car args)) (setq torque-vector (car args)))
    ((cadr (member :update? args))
     (send *robot* :torque-vector
	   :force-list (mapcar #'(lambda (v) (subseq v 0 3)) contact-forces)
	   :moment-list (mapcar #'(lambda (v) (subseq v 3 6)) contact-forces)
	   :target-coords (send-all (send self :contact-states) :contact-coords)))
    (t torque-vector)))
  (:log
   (&key
    (robot *robot*)
    (contact-links (send self :get-link-list :robot robot))
					;     (mapcar
					;      #'(lambda (c l)
					;	  (send robot :link-list (send c :parent) (send robot l :root-link)))
					;      (send-all contact-states :contact-coords) contact-keys))
    (torque-draw? nil))
   (format *log-stream* "[rsd log]~%")
   (if (collision-resolve-move) (format *log-stream* " collision detected~%"))
   ;;
   (format *log-stream* " --------------~%")
   (if (and torque-draw? (boundp '*viewer*) *viewer*)
       (send robot :draw-torque *viewer* :color #f(1 0 0)
	     :torque-vector torque-vector))
   (if (not (find-if #'identity
		     (flatten
		      (mapcar
		       #'(lambda (ll tv)
			   (map cons
				#'(lambda (j val)
				    (cond
				     ((> (abs val) 1.01)
				      (format *log-stream* " torque overthre ~A at ~A~%"
					      val (send j :name)))
				     ((> (abs val) 0.7)
				      (format *log-stream* " torque warn ~A > 0.7 at ~A~%"
					      val (send j :name)))
				     )
				    (> (abs val) 1.01))
				(send-all ll :joint) tv))
		       contact-links t/tmax))))
       (format *log-stream* " torque ok~%"))
   ;;
   (format *log-stream* " --------------~%")
   (mapcar #'(lambda (k f)
	       (format *log-stream* " | ~A f = ~A~%" k f))
	   contact-keys contact-forces)
   (if (not (find-if #'identity
		     (flatten
		      (mapcar
		       #'(lambda (ck fv)
			   (map cons
				#'(lambda (axis val)
				    (if (> (abs val) 1.01)
					(format *log-stream* " ~A axis overthre ~A at ~A~%"
						axis val ck))
				    (> (abs val) 1.01))
				(list "x" "y" "z" "roll" "pitch" "yaw") fv))
		       contact-keys f/fmax))))
       (format *log-stream* " force & moment ok~%"))
   (format *log-stream* " --------------~%")
   (if (> (norm root-acc) 1) (format *log-stream* " acc dynamic =~A~%" root-acc)
     (format *log-stream* " acceralation static~%"))
   (format *log-stream* " --------------~%")
   (format *log-stream*
	   "BRLV: norm=~A/max=~A~%"
	   (norm tf-error)
	   (apply #'max (map cons #'abs tf-error)))
   (if (not full-constrainted)
       (format *log-stream*
	       "!!! not full-constrainted rsd detected !!!~%"))
   (format *log-stream* "now-brli =~%")
   (if *log-stream* (pprint brli *log-stream*))
   (format *log-stream* "now-torque-brli =~%")
   (if *log-stream* (pprint torque-brli *log-stream*))
   (format *log-stream* "now-friction-brli =~%")
   (if *log-stream* (pprint friction-brli *log-stream*))
   (if (and torque-draw? (boundp '*viewer*) *viewer*)
       (send *viewer* :flush))
   )
  (:old-serialize
   nil
   (list (cons :angle-vector angle-vector)
	 (cons :hand-angle-vector hand-angle-vector)
	 (cons :pos (send root-coords :worldpos))
	 (cons :rot (send root-coords :worldrot))
	 (cons :grasp-hand
	       (cond
		((and (find :rarm contact-keys)
		      (find :larm contact-keys)) :both)
		((find :rarm contact-keys) :rarm)
		((find :larm contact-keys) :larm)
		(t nil)))
	 (cons :balance-leg
	       (cond
		((and (find :rleg contact-keys)
		      (find :lleg contact-keys)) :both)
		((find :rleg contact-keys) :rleg)
		((find :lleg contact-keys) :lleg)
		(t nil)))
	 (cons :toe-vector
	       (let ((k (or (find :rleg contact-keys)
			    (find :lleg contact-keys))))
		 (if k
		     (transform (transpose (send *robot* k :end-coords :worldrot))
				(v- (send (send self :contact-coords k) :worldpos)
				    (send *robot* k :end-coords :worldpos)))
		   #F(0 0 0))))
	 (cons :contact-keys contact-keys)
	 (cons :contact-forces contact-forces)
	 (cons :contact-pos (send-all (send self :contact-coords) :worldpos))
	 (cons :contact-rot (send-all (send self :contact-coords) :worldrot))
	 (cons :right-force
	       (subseq (or (send self :contact-forces :rarm)
			   #F(0 0 0 0 0 0)) 0 3))
	 (cons :left-force
	       (subseq (or (send self :contact-forces :larm)
			   #F(0 0 0 0 0 0)) 0 3))
	 (cons :right-moment
	       (subseq (or (send self :contact-forces :rarm)
			   #F(0 0 0 0 0 0)) 3 6))
	 (cons :left-moment
	       (subseq (or (send self :contact-forces :larm)
			   #F(0 0 0 0 0 0)) 3 6))))
  (:contact-coords
   (&optional limb)
   (if limb (send (send self :contact-states limb) :contact-coords)
     (send-all (send self :contact-states) :contact-coords)))
  (:contact-n
   (&optional limb)
   (if limb (send (send self :contact-states limb) :contact-n)
     (send-all (send self :contact-states) :contact-n)))
  (:gbrli-dec-move
   (&key
    (mg (* 1e-3 (send *robot* :weight) (* 1e-3 (aref *g-vec* 2)))) ;;9.8))
    (g-1 (make-matrix
	  3 6
	  (list (list 0 0 0 0 (/ 1 mg) 0)
		(list 0 0 0 (/ -1 mg) 0 0)
		(list 0 0 0 0 0 0))))
    (mv (mapcar #'(lambda (gbrl) (transform g-1 gbrl)) gbrli-list)))
   (format *log-stream* "[:gbrli-dec-move]~%")
   (mapcar
    #'(lambda (k mv) (format *log-stream* " | ~A = ~A~%" k mv))
    contact-keys mv)
   (cond
    ((> (length mv) 1) (reduce #'v+ mv))
    (t (car mv))))
  (:draw
   (&key
    (robot *robot*)
    (pv (if (and (boundp '*irtviewer*) *irtviewer*) *irtviewer*))
    (rest nil) (friction-cone? t) (torque-draw? t)
    (offset-pos (float-vector 0 0 0)))
   ;; check robot2root-coords transformation
   (let* ((r2c (send (copy-object (send robot :worldcoords))
		     :transformation
		     (send (car (send robot :links)) :copy-worldcoords)))
	  dif)
     (setq dif
	   (concatenate float-vector
			(send robot2root-coords :difference-position r2c)
			(send robot2root-coords :difference-rotation r2c)))
     (cond
      ((> (norm dif) 1)
       (format *log-stream*
	       "[rsd :draw] root coords transformation detected~% dif: ~A~%"
	       dif)
       (send (car (send robot :links))
	     :transform
	     (send
	      (send (car (send robot :links)) :copy-worldcoords)
	      :transformation
	      (send (copy-object (send robot :worldcoords))
		    :transform robot2root-coords :local)
	      :local))
       )))
   (send robot :newcoords (copy-object root-coords))
   (send robot :angle-vector angle-vector)
   ;;   (mapcar
   ;;    #'(lambda (k hav) (send robot k :hand-angle-vector hav))
   ;;    '(:rarm :larm) hand-angle-vector)
   (send-all contact-states :gen-friction-cone-object :offset-pos offset-pos)
   (mapcar #'(lambda (cs f)
	       (send cs :gen-force-vector-object (subseq f 0 3) :offset-pos offset-pos))
	   contact-states contact-forces)
   (if (and pv friction-cone?)
       (send pv :objects
	     (union
	      nil
	      (append
					;(send pv :objects)
	       rest
	       (list robot)
	       (send-all contact-states :friction-cone-object)
	       (send-all contact-states :force-vector-object)))))
   (if pv (send pv :draw-objects))
   (send self :log :torque-draw? torque-draw?))
  (:clear
   nil
   (send self :buf :slide (not (not (send self :buf :slide))))
   (send self :buf :selected-planner nil) ;; todo
   (send-all
    (remove-if #'(lambda (cs)
		   (not (and (class cs)
			     (subclassp (class cs) simple-contact-state))))
	       (flatten (send self :slots)))
    :clear))
  (:buf (&rest args)
	(cond
	 ((null args) buf)
	 ((= 1 (length args)) (cdr (assoc (car args) buf)))
	 (t (setq buf
		  (cons (cons (car args) (cadr args))
			(remove-if #'(lambda (kv) (eq (car kv) (car args))) buf))))))
  (:full-constrainted (&optional val)
		      (if val (setq full-constrainted val) full-constrainted))
  (:nomethod
   (&rest args)
   (let ((ret) buf)
     (setq ret
	   (cond
	    ((and (car args)
		  (find-method (car args) :pname)
		  (setq buf
			(find-if #'(lambda (a)
				     (and
				      (car a)
				      (find-method (car a) :pname)
				      (stringp (send (car a) :pname))
				      (string= (send (car a) :pname)
					       (send (car args) :pname))))
				 (send self :slots))))
	     (cdr buf))
	    (t nil)))
     (cond
      ((or (null ret)
	   (not (keywordp (cadr args)))) ret)
      ((and (listp ret)
	    (find (cadr args) contact-keys)
	    (eq (length contact-keys) (length ret)))
       (cdr (assoc (cadr args) (mapcar #'cons contact-keys ret))))
      ((find-method ret (cadr args)) (send ret (cadr args)))
      (t nil))))
  )


;;
(defvar *sample-rsd* (instance robot-state-data2 :init))

(require "util/list2file.lisp")
(require "util/graph-sample.lisp")

(defun rsd-serialize
  (&key rsd (rsd-list (list rsd))
	(file (format nil "log/~A.rsd" (log-surfix))))
  (let ((buf))
    (mapcar #'(lambda (v)
		(if (and (class v) (find-method v :clear))
		    (send v :clear))
		)
	    (flatten rsd-list))
    (setq buf (object-serializer rsd-list))
    (if (stringp file) (list2file buf file))
    buf))

(defun rsd-deserialize
  (&key (file "log/tmp.rsd") (rsd (file2list file)))
  (eval (object-deserializer rsd)))

(defun rsd-play
  (&key
   (file "log/tmp.rsd")
   (rsd-list (flatten (rsd-deserialize :file file)))
   (rsd *sample-rsd*);(find-if #'(lambda (rsd) (eq (class rsd) robot-state-data2)) rsd-list))
   (other-obj (if (boundp '*climb-obj*) (list *climb-obj*)))
   (torque-max 1.0)
   (offset-pos #F(100 0 0))
   (contact-keys (send rsd :contact-keys))
   (joint-list
    (mapcar #'(lambda (cs k)
		(send-all
		 (send *robot* :link-list
		       (if (find-method *robot* k)
			   (send *robot* k :end-coords :parent)
			 (send (send cs :contact-coords) :parent))
		       (if (find-method *robot* k) (send *robot* k :root-link)
			 (car (send *robot* :links))))
		 :joint))
	    (send rsd :contact-states) contact-keys))
   ;;    (mapcar #'(lambda (ll) (send-all ll :joint))
   ;;	    (send rsd :get-link-list)))
   (graph-size '(300 300))
   (graph (mapcar
	   #'(lambda (k l)
	       (create-graph (format nil "~A" k)
			     :size graph-size
			     :name-list
			     (append
			      (list "MAX")
			      (list "MIN")
			      (list "--- Force ---")
			      (list "Fx" "Fy" "Fz" "Mx" "My" "Mz")
			      (list "--- Torque ---")
			      (mapcar
			       #'(lambda (l)
				   (cond
				    ((stringp (send l :name))
				     (send l :name))
				    ((keywordp (send l :name))
				     (send (send l :name) :pname))
				    (t (format nil "~A" (send l :name)))))
			       l)
			      )
			     :range (list #F(0 -1) #F(90 1))))
	   contact-keys joint-list))
   ret
   (index -1)
   (auto? nil)
   break?)
  (place-graph-in-order :graph graph :size graph-size)
  (setq
   ret
   (mapcar
    #'(lambda (rsd)
	(cond
	 (break? 'break)
	 ((eq (class rsd) robot-state-data2)
	  (send rsd :draw
		:rest other-obj
		:offset-pos offset-pos
		:friction-cone? t :torque-draw? nil)
					;(send rsd :log)
	  (cond
	   (graph
	    (mapcar
	     #'(lambda (k g jlist)
		 (add-data-to-graph
		  (append
		   (list torque-max)
		   (list (* -1 torque-max))
		   (list 0)
		   (coerce (or (send rsd :f/fmax k)
			       (instantiate float-vector 6))
			   cons)
		   (list 0)
		   (coerce (or (send rsd :t/tmax k)
			       (instantiate float-vector (length jlist)))
			   cons))
		  :pre-pos (float-vector index 0)
		  :title-change? nil
		  :draw? nil
		  :graph g))
	     contact-keys graph joint-list)
	    (send-all graph :simple-draw-with-line)))
	  (incf index)
	  (let* ((buf (if auto? (progn (unix:usleep (round (* 1e+3 100))) "") (read-line)))
		 (tar (if (plusp (length buf)) (aref buf 0) nil)))
	    (case tar (#\q (setq break? t))))
	  (send rsd :full-constrainted))
	 (t (format *log-stream* "non rsd data for rsd-play~%"))))
    (reverse rsd-list)))
  (if graph (send-all graph :fit-draw))
  ret)


#|

(rsd-play :file "log/Tue_Sep_10_22:40:24_2013/kirin-ladder.rsd")

(setq a (car *graph-sample*))
(mapcar
 #'(lambda (a)
     (send a :set-range #F(0 -1) #F(100 1))
     (send a :set-data
	   (let ((buf (send a :data)) tmp1 tmp2)
	     (append
	      (progn
		(setq tmp2 (subseq buf 10 (length buf)))
		(send-all tmp2 :name "")
		tmp2)
	      (progn
		(setq tmp1 (subseq buf 3 9))
		(send-all tmp1 :name "")
		tmp1)
	      (subseq buf 0 2))))
     (send a :simple-draw-with-line))
 *graph-sample*)

(load "../../../util/gp-util.lisp")

(mapcar
 #'(lambda (a)
     (graph-panel2gp-graph
      a :ylabel "BRLV ELEMENTS" :xlabel "LOOP COUNT" :save? t))
 *graph-sample*)


roscd euslib/demo ;
svn up s-noda ;
roscd climb/work && roseus "(progn (defvar *robot-type* :staro) (load \"motion-sequencer.l\") (demo-climb-setup :garakuta-car) (progn (init-pose) (setq *ret* (demo-motion-sequence :all-limbs '(:rarm :rleg :lleg) :now-rsd (instance robot-state-data2 :init :contact-states (now-contact-state :limb-keys '(:rarm :rleg :lleg))) :tmax-leg-rate 1.0 :loop-max 3)) (rsd-play :rsd-list *Ret* :auto? t) (require \"dynamic-connector.l\") (connect-rsd :rsd-list *ret*) (quit-graph))"
