
;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsk-gazebo)

(require "robot-param.l")
(require "motion-sequencer.l")
(require "model/simple-floor.l")
(require "ik-without-collision.l")

(defun simple-contact-state-transition
  (cs-start cs-end
	    &key
	    (force nil)
	    (remove? t)
	    (reach? t))
  (let* ((cmm (remove-if #'(lambda (st) (not (find st cs-end))) cs-start))
	 (sto (set-difference cs-start cmm))
	 (eno (set-difference cs-end  cmm))
	 ret)
    (cond
     ((and
       remove?
       (progn
	 (format *log-stream*
		 "[simple-contact-state-transition] :remove~%")
	 (push
	  (pose-generate-with-contact-state-with-time
	   cmm
	   :rest-contact-states sto
	   :tau-gain 0.1
	   :centroid-thre-rate 0.9
	   :non-stop t :best-rsd? t) ret)
	 (not (or force (and (car ret) (send (car ret) :full-constrainted))))))
      nil)
     ((and
       reach?
       (progn
	 (format *log-stream*
		 "[simple-contact-state-transition] :reach~%")
	 (push (pose-generate-with-contact-state-with-time
		cmm
		:rest-contact-states eno
		:tau-gain 0.1
		:centroid-thre-rate 0.9
		:non-stop t :best-rsd? t) ret)
	 (not (or force (and (car ret) (send (car ret) :full-constrainted))))))
      nil)
     ((progn
	(format *log-stream*
		"[simple-contact-state-transition] :suspend~%")
	(push (pose-generate-with-contact-state-with-time
	       (append cmm eno)
	       :rest-contact-states nil
	       :tau-gain 0.1
	       :centroid-thre-rate 0.9
	       :non-stop t :best-rsd? t) ret)
	(not (or force (and (car ret) (send (car ret) :full-constrainted)))))
      nil)
     (t (reverse ret)))))

(defun simple-contact-states-transition
  (cs-list
   &key
   (force nil)
   ret)
  (while (and cs-list (cdr cs-list) (or (null ret) (car ret)))
    (push
     (simple-contact-state-transition
      (car cs-list) (cadr cs-list))
     ret)
    (setq cs-list (cdr cs-list)))
  ret)

(defun gen-straight-contact-states
  (&key
   (seed
    (mapcar
     #'(lambda (cs)
	 (send cs :copy
	       :target-coords
	       (copy-object (send (send cs :contact-coords) :worldcoords))))
     *hihi-contact-state*))
   (dir #F(100 0 0))
   (name-list (send-all seed :name))
   (depth 1)
   (ret (list seed)))
  (while (>= (decf depth) 0)
    (dolist (name name-list)
      (let* ((keep (remove-if #'(lambda (cs) (eq name (send cs :name))) (car ret)))
	     (move (find-if #'(lambda (cs) (eq name (send cs :name))) (car ret)))
	     (coords (send move :target-coords)))
	(push
	 (cons
	  (send move :copy
		:target-coords
		(make-coords :pos (v+ dir (send coords :worldpos))
			     :rot (copy-object (send coords :worldrot))))
	  keep)
	 ret))))
  (reverse ret))


;; samples

(defun demo-contact-sequence
  nil
  ;; gen env
  (demo-climb-setup :simple-floor)
  (setq *env-collision-check-link-list*
	(list
	 (list
	  (cons :link *climb-obj*)
	  (cons :centroid-dot-normal
		(mapcar
		 #'(lambda (f)
		     (let ((buf
			    (union
			     (send f :vertices)
			     nil)))
		       (list
			(scale (/ 1.0 (length buf))
			       (reduce #'v+ buf))
			(normalize-vector
			 (v*
			  (v- (nth 1 buf) (nth 0 buf))
			  (v- (nth 2 buf) (nth 0 buf)))))))
		 (send *climb-obj* :faces)))
	  (cons :dist-func
		#'(lambda (rb env)
		    (let* ((rbl (cdr (assoc :link rb)))
			   (evl (cdr (assoc :link env)))
			   (dst (pqp-collision-distance rbl evl))
			   (evp (nth 2 dst))
			   ret)
		      (cond
		       ((< (car dst) 1)
			(format *log-stream* "  collision: ~A vs ~A~%"
				(send (cdr (assoc :link env)) :name)
				(send (cdr (assoc :link rb)) :name))
			(setq ret
			      (append
			       (list nil #F(0 0 1))
			       ;; (car
			       ;;  (sort
			       ;;   (cdr (assoc :centroid-dot-normal env))
			       ;;   #'(lambda (a b)
			       ;; 	   (< (norm2 (v- (car a) evp))
			       ;; 	      (norm2 (v- (car b) evp))))))
			       dst))))
		      ret))))))
  ;;
  (setq *robot-collision-check-link-list*
	(append
	 (mapcar
	  #'(lambda (l)
	      (list (cons :link l)))
	  (send *robot* :links))
	 (if (find-method (send *robot* :rarm) :hand)
	     (apply
	      #'append
	      (mapcar
	       #'(lambda (k)
		   (mapcar
		    #'(lambda (l)
			(list (cons :link l)
			      (cons :parent (send *robot* k :end-coords :parent))))
		    (send *robot* k :hand :links)))
	       '(:rarm :larm))))))
  ;; robot contact states
  (send *robot* :angle-vector (scale 0 (send *robot* :angle-vector)))
  (send *robot* :arms :shoulder-p :joint-angle -180)
  (send *robot* :newcoords
	(make-coords
	 :pos (float-vector 0 0 300)
	 :rot #2f((0 0 1) (0 1 0) (-1 0 0))))
  (setq
   *hihi-contact-state*
   (append
    (mapcar
     #'(lambda (k)
	 (let* ((el (send (send *robot* k :elbow-p) :worldpos))
		(wl (send *robot* k :wrist-r))
		(c (make-cascoords :init :link-list
				   :parent
				   (cond
				    ((find-method wl :child-link)
				     (send wl :child-link))
				    ((find-method wl :parent)
				     (send (send wl :parent) :parent)))
				   :name (read-from-string (format nil "~A-wrist-contact" k))
				   :coords (make-coords
					    :pos (copy-object (send wl :worldpos))))))
	   (instance simple-contact-state
		     :init
		     :name k
		     :contact-n #F(0 0 1)
		     :force0 #F(0 0 0 0 0 0)
		     :contact-coords c
		     :rotation-axis :y
		     :target-coords
		     (make-coords :pos (float-vector (- (aref el 0) 50)
						     (aref el 1)
						     80)))))
     '(:rarm :larm))
    (mapcar
     #'(lambda (k)
	 (let* ((kn (send *robot* k :knee-p))
		(cr (send (send *robot* k :crotch-p) :worldpos))
		(c (make-cascoords :init :link-list
				   :parent
				   (cond
				    ((find-method kn :child-link)
				     (send kn :child-link))
				    ((find-method kn :parent)
				     (send (send kn :parent) :parent)))
				   :name (read-from-string (format nil "~A-knee-contact" k))
				   :coords (make-coords
					    :pos (send kn :worldpos)))))
	   (instance simple-contact-state
		     :init
		     :name k
		     :contact-n #F(0 0 1)
		     :force0 #F(0 0 0 0 0 0)
		     :contact-coords c
		     :rotation-axis :y
		     :target-coords
		     (make-coords :pos (float-vector (aref cr 0)
						     (aref (send kn :worldpos) 1)
						     120)))))
     '(:rleg :lleg))
    ))
  (send *robot* :reset-manip-pose)
  (simple-fullbody
   :target
   (mapcar
    #'(lambda (cs ta)
	(list (cons :target (send cs :contact-coords))
	      (cons :coords (send cs :target-coords))
	      (cons :translation-axis ta)))
    *hihi-contact-state*
    (list t t t t))
   :stop 100
   :balance-leg nil
   :target-centroid-pos nil
   :collision-avoidance-link-pair nil
   :debug-view :no-message)
  (send-all *hihi-contact-state* :fix-coords)
  ;; pose generation
  (pose-generate-with-contact-state *hihi-contact-state* :non-stop t)
  )

;; get (list (list rarm-contact-state larm-contact-state rleg-contact-state lleg-contact-state) ...) from gait mode
(defun get-contact-state-list-from-gait-mode
  (contact-states ;; candidates of contact-states
   gait-mode
   &key (hand-foot-offset-ladder-count 4) ;; offset ladder step count between hand and foot, 0 is same step
        (hand-coordinates-index 0) (foot-coordinates-index 2) (candidate-num 3) ;; currently param-ladder only
        (start-index 0) ;; index of start ladder
        (switch-step-num 2)) ;; steps to climb up or climb down
  (labels
      ((extract-limb-contact-states
        (limb)
        (remove-if-not #'(lambda (x) (eq limb (send x :name))) contact-states))
       (calc-gait-contact-states
        (rarm-idx-offset larm-idx-offset rleg-idx-offset lleg-idx-offset)
        (list (elt (extract-limb-contact-states :rarm)
                   (+ (* candidate-num (+ hand-foot-offset-ladder-count start-index rarm-idx-offset)) hand-coordinates-index))
              (elt (extract-limb-contact-states :larm)
                   (+ (* candidate-num (+ hand-foot-offset-ladder-count start-index larm-idx-offset)) hand-coordinates-index))
              (elt (extract-limb-contact-states :rleg)
                   (+ (* candidate-num (+ start-index rleg-idx-offset)) foot-coordinates-index))
              (elt (extract-limb-contact-states :lleg)
                   (+ (* candidate-num (+ start-index lleg-idx-offset)) foot-coordinates-index)))))
    (case gait-mode
      (:gallop
       (labels
           ((calc-gallop-contact-states
             (arm-idx-offset leg-idx-offset)
             (calc-gait-contact-states
              arm-idx-offset arm-idx-offset leg-idx-offset leg-idx-offset)))
         (append
          (list (calc-gallop-contact-states 0 0)) ;; initial
          (mapcan
           #'append
           (mapcar
            #'(lambda (i)
                (list
                 (calc-gallop-contact-states i (+ 1 i))
                 (calc-gallop-contact-states (+ 1 i) (+ 1 i))))
            (range switch-step-num)
            )))))
      (:pace
       (labels
           ((calc-pace-contact-states
             (right-idx-offset left-idx-offset)
             (calc-gait-contact-states
              right-idx-offset left-idx-offset right-idx-offset left-idx-offset)))
         (append
          (list (calc-pace-contact-states 0 0)) ;; initial
          (let (ret)
            (dotimes (i switch-step-num)
               (push
                (if (evenp i)
                    (calc-pace-contact-states i (1+ i))
                  (calc-pace-contact-states (1+ i) i))
                ret))
             (reverse ret))
          (list (calc-pace-contact-states switch-step-num switch-step-num)) ;; finalize
          )))
      (:trot
       (labels
           ((calc-trot-contact-states
             (idx-offset0 idx-offset1)
             (calc-gait-contact-states
              idx-offset0 idx-offset1 idx-offset1 idx-offset0)))
         (append
          (list (calc-trot-contact-states 0 0)) ;; initial
          (let (ret)
            (dotimes (i switch-step-num)
               (push
                (if (evenp i)
                    (calc-trot-contact-states i (1+ i))
                  (calc-trot-contact-states (1+ i) i))
                ret))
             (reverse ret))
          (list (calc-trot-contact-states switch-step-num switch-step-num)) ;; finalize
          )))
      (:crawl
       (append
        (list (calc-gait-contact-states 0 0 0 0)) ;; initial
        (mapcan
         #'append
         (mapcar
          #'(lambda (i)
              (list
               ;; (calc-gait-contact-states i i i (1+ i))
               ;; (calc-gait-contact-states i i (1+ i) (1+ i))
               ;; (calc-gait-contact-states i (1+ i) (1+ i) (1+ i))
               ;; (calc-gait-contact-states (1+ i) (1+ i) (1+ i) (1+ i))
               (calc-gait-contact-states (1+ i) i i i)
               (calc-gait-contact-states (1+ i) (1+ i) i i)
               (calc-gait-contact-states (1+ i) (1+ i) (1+ i) i)
               (calc-gait-contact-states (1+ i) (1+ i) (1+ i) (1+ i))
               ))
          (range switch-step-num)))))
      )
    ))

(defun test-get-contact-state-list-from-gait-mode
  ()
  (labels
      ((view-contact-states
        (cs-list)
        (dolist (cs cs-list)
          (send *irtviewer* :draw-objects :flush nil)
          (send-all (send-all cs :target-coords) :draw-on :flush t :color #f(1 0 0) :size 100)
          (print ";; press Enter")
          (read-line))))
  (print ";; gallop test")
  (view-contact-states
   (get-contact-state-list-from-gait-mode *contact-states* :gallop :switch-step-num 2))
  ;;
  (print ";; pace test")
  (view-contact-states
   (get-contact-state-list-from-gait-mode *contact-states* :pace :switch-step-num 2))
  ;;
  (print ";; trot test")
  (view-contact-states
   (get-contact-state-list-from-gait-mode *contact-states* :trot :switch-step-num 2))
  ;;
  (print ";; crawl test")
  (view-contact-states
   (get-contact-state-list-from-gait-mode *contact-states* :crawl :switch-step-num 2))
  ))

#|
(setq a (gen-straight-contact-states :dir #F(100 0 0) :depth 10))
(mapcar #'(lambda (a) (send *irtviewer* :draw-objects) (send-all (send-all a :target-coords) :draw-on :flush t :color #F(1 0 0)) (read-line)) (gen-straight-contact-states :depth 10))

(let ((buf (gen-straight-contact-states :dir #F(100 0 0) :depth 1))
      cmm)
  (mapcar
   #'(lambda (end start)
       (setq cmm
	     (remove-if #'(lambda (a) (not (find a end))) start))
       (list
	(pose-generate-with-contact-state
	 cmm
	 :rest-contact-states (set-difference end cmm)
	 :non-stop t)
;	(pose-generate-with-contact-state
;	 end
;	 :rest-contact-states nil
;	 :non-stop t)
	)
       )
   (cdr buf) buf))

|#