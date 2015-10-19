(defvar *robot-type* :hrp2jsknt-collada)

(require "ik-with-torque.lisp")
(require "package://eus_gui/graph-sample.lisp")

(defvar *ball-radius* 900)
(defvar *human-ball* (make-sphere (* 1 *ball-radius*)))
(send *human-ball* :set-color #f(0 1 0))
(gl::transparent *human-ball* 0.3)
;; (send *irtviewer* :change-background (float-vector 1 1 1))
;; (send *pickview* :viewer :viewsurface :bg-color #F(1 1 1))

(mapcar
 #'(lambda (k)
     (if (and (find-method (send *robot* :rarm) k)
	      (send *robot* :rarm k))
	 (let ((buf (send *robot* :rarm k :max-joint-torque)))
	   (send *robot* :rarm k :max-joint-torque (* buf 3.))
	   (send *robot* :larm k :max-joint-torque (* buf 3.)))))
 '(:wrist-p :wrist-r :wrist-y))
;; (send-all (send *robot* :joint-list) :max-joint-torque 1000)

;; (mapcar
;;  #'(lambda (k)
;;      (send (send (send *robot* k :hand) :parent)
;; 	   :dissoc
;; 	   (send *robot* k :hand))
;;      (send (send *robot* k :hand) :locate #F(0 0 0) :world)
;;      ;;(gl::transparent (send *robot* k :hand) 0.0)
;;      )
;;  '(:rarm :larm))
(defvar *robot-hand* (send *robot* :links))

(defun vector-matrix
  (ref &optional
       (axis (float-vector 0 0 1))
       (rot-axis (v* ref axis))
       (rot-dot (v. (normalize-vector ref)
		    (normalize-vector axis)))
       (rot-angle
	(let ((av) buf)
	  (cond
	   ((eps= (norm rot-axis) 0.0 0.01)
	    (while (zerop (norm rot-axis))
	      (setq buf (random-vector 1.0))
	      (setq rot-axis (v* ref buf)))
	    (if (eps= rot-dot 0.0 0.01)
		0 (deg2rad 180)))
	   (t (acos rot-dot))))))
  (matrix-exponent
   (scale (* -1 rot-angle) (normalize-vector rot-axis)))
  )

(defun random-sphere-coords
  (&key
   (key :rleg)
   (axis
    (case key
	  (:rleg (float-vector 0 0 -1))
	  (:lleg (float-vector 0 0 -1))
	  (:rarm (float-vector +1 0 0))
	  (:larm (float-vector +1 0 0))))
   (seed
    (case key
      (:rleg
       (normalize-vector
	(float-vector (* +0.25 (- (random 2.0) .0))
		      (* -0.5 (random 1.0))
		      (+ -0.5 (* -0.5 (random 1.0))))))
      (:lleg
       (normalize-vector
	(float-vector (* -0.25 (- (random 2.0) .0))
		      (* +0.5 (random 1.0))
		      (+ -0.5 (* -0.5 (random 1.0))))))
      (:rarm
       (normalize-vector
	(float-vector (- (random 2.0) 1.0)
		      (* -1 (random 1.0))
		      (* +0.5 (- (random 2.0) 0.0)))))
      (:larm
       (normalize-vector
	(float-vector (- (random 2.0) 1.0)
		      (* +1 (random 1.0))
		      (* +0.5 (- (random 2.0) 0.0)))))
      ))
   (pos (scale (* 0.9 *ball-radius*) seed))
   (rot
    ;;(m*
    ;;(vector-matrix axis #F(0 0 1))
    (vector-matrix seed axis)
    )
   )
  (make-coords :pos pos :rot rot))

(defun human-ball-pose
  (&key
   (key-list '(:rleg :lleg :rarm :larm)))
  (send *robot* :reset-manip-pose)
  (send *robot* :newcoords (make-coords :pos #F(0 0 -100)))
  (send (car (send *robot* :links))
	:newcoords (make-coords :pos #F(0 0 -100)))
  (send-all (send *robot* :links) :worldcoords)
  (objects (flatten (list *robot-hand* *human-ball*)))
  (send *irtviewer* :change-background (float-vector 1 1 1))
  (let* ((move-target
	  (mapcar
	   #'(lambda (k) (send *robot* k :end-coords))
	   key-list))
	 (link-list
	  (mapcar
	   #'(lambda (mt)
	       (send *robot* :link-list (send mt :parent)))
	   move-target))
	 (target-coords
	  (send-all move-target :copy-worldcoords))
	 (rotation-axis (list :z :z :x :x))
	 (translation-axis (list t t t t))
	 )
    (setq *ik-convergence-user-check* 0.0)
    (send *robot* :fullbody-inverse-kinematics
	  (mapcar
	   #'(lambda (k)
	       (random-sphere-coords :key k))
	   key-list)
	  :move-target move-target
	  :link-list link-list
	  :target-centroid-pos nil
	  :debug-view :no-message
	  :min #F(-1000 -1000 -1000 -400 -400 -400)
	  :max #F(+1000 +1000 +1000 +400 +400 +400)
	  :collision-avoidance-link-pair nil
	  ;;:root-link-virtual-joint-weight
	  ;;(float-vector 0.1 0.1 0.1 0.1 0.1 0.1)
	  :thre
	  (subseq (list 100 100 100 100)
		  0 (length key-list))
	  :rthre
	  (subseq (list 0.1 0.1 0.1 0.1)
		  0 (length key-list))
    	  :rotation-axis
    	  (subseq rotation-axis 0 (length key-list))
	  :warnp nil
	  ;;:revert-if-fail nil
	  )
    ;; (send *robot* :fullbody-inverse-kinematics
    ;; 	  (mapcar
    ;; 	   #'(lambda (k)
    ;; 	       (random-sphere-coords :key k))
    ;; 	   key-list)
    ;; 	  :move-target move-target
    ;; 	  :link-list link-list
    ;; 	  :target-centroid-pos nil
    ;; 	  :debug-view :no-message
    ;; 	  :rotation-axis
    ;; 	  (subseq rotation-axis 0 (length key-list))
    ;; 	  :translation-axis
    ;; 	  (subseq translation-axis 0 (length key-list))
    ;; 	  :min #F(-1000 -1000 -1000 -400 -400 -400)
    ;; 	  :max #F(+1000 +1000 +1000 +400 +400 +400)
    ;; 	  :collision-avoidance-link-pair nil
    ;; 	  :root-link-virtual-joint-weight
    ;; 	  (float-vector 0.1 0.1 0.1 0.01 0.01 0.01)
    ;; 	  ;;:stop 1
    ;; 	  ;;:revert-if-fail nil
    ;; 	  )
    )
  )

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
		  (copy-object (send c :worldcoords))
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

(defun test-sphere-human-ball
  (&key
   (key-list '(:rleg :lleg :rarm :larm))
   (rotation-axis
    (subseq
     (list :z :z :x :x)
     ;;(list t t t t)
     0 (length key-list)))
   (debug-view :no-message)
   (callback nil)
   (gain1) (gain2)
   (rest-torque-ik-args)
   torque-ik-args
   )
  (let* ((inital-av) init-rsd torque brlv)
    (setq *ik-convergence-user-check* 0.0)
    (setq torque-ik-args
	  (list :key-list key-list
		:wrench-key-list key-list
		:init nil
		:null-max 0.3
		:gtol 0.001
		:stop 100
		:linear-brli nil
		:debug-view debug-view
		;;:root-link-virtual-joint-weight
		;;(float-vector 0.1 0.1 0.1 0.01 0.01 0.01)
		:rotation-axis rotation-axis))
    (while (or (null
		(setq inital-av
		      (print (human-ball-pose :key-list key-list))))
	       (progn
		 (apply 'simple-calc-torque-gradient
			(append rest-torque-ik-args torque-ik-args))
		 (not
		  (or (send *now-rsd* :full-constrainted)
		      (send *now-rsd* :buf :contact-wrench-optimize-skip))))))
    (setq inital-av (copy-object inital-av))
    (setq init-rsd *now-rsd*)
    (setq torque-ik-args
	  (append torque-ik-args
		  (list :now-rsd init-rsd
			:best-rsd init-rsd)))
    ;;(send init :draw :rest (list *human-ball*))
    (setq *rsd-queue* nil)
    (setq
     torque
     (apply
      'test-torque-ik
      (append
       rest-torque-ik-args
       torque-ik-args
       (list :mode :normal :gain gain1))))
    (if (not torque)
	(return-from test-sphere-human-ball nil))
    (send *viewer* :draw-objects)
    (send torque :buf :gain *simple-calc-torque-gradient-gain*)
    ;; (setq init (car (last *rsd-queue*)))
    ;; (send init :draw :rest (list *human-ball*))
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
    (if (not brlv)
	(return-from test-sphere-human-ball nil))
    (send *viewer* :draw-objects)
    (send brlv :buf :gain *simple-calc-torque-gradient-gain*)
    (format t
	    (concatenate
	     string
	     " [sphere-human-ball-test] time ~A/~A = ~A~%"
	     "                          eval ~A/~A = ~A~%")
	     (send brlv :buf :time)
	     (send torque :buf :time)
	     (/ (send brlv :buf :time)
		(send torque :buf :time))
	     ;; (/ (norm (send brlv :torque-vector))
	     ;; 	(norm (send init :torque-vector)))
	     ;; (/ (norm (send torque :torque-vector))
	     ;; 	(norm (send init :torque-vector)))
	     ;; (/ (norm (send brlv :torque-vector))
	     ;; 	(norm (send torque :torque-vector)))
	     (/ (norm (send brlv :buf :tau))
		(norm (send init-rsd :buf :tau)))
	     (/ (norm (send torque :buf :tau))
		(norm (send init-rsd :buf :tau)))
	     (/ (norm (send brlv :buf :tau))
		(norm (send torque :buf :tau)))
	     )
    (if (functionp callback) (funcall callback torque brlv))
    (send-all (list init-rsd torque brlv) :clear)
    (list init-rsd torque brlv)))

;; 0.030954 ;; 0.013033
;; 1.814612e+05 ;; 19500.8
(defun test-sphere-human-ball-loop
  (&rest
   args
   &key
   (key-list '(:rleg :lleg :rarm :larm))
   (loop-max 300)
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
   (if (not (plusp (decf cnt))) (return-from nil nil))
   (sys::gc)
   (setq
    buf
    (apply
     'test-sphere-human-ball
     :key-list key-list :debug-view debug-view :gain1 gain1 :gain2 gain2
     args))
   (cond
    ((or (not buf)
	 ;;(<
	 ;;(length (flatten (send-all (cdr buf) :full-constrainted)))
	 ;;2)
	 )
     (incf fail-cnt))
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

#|

(test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm :larm) :debug-view nil :gain1 0.530954 :gain2 1.814612e+05 :rest-torque-ik-args (list :contact-wrench-optimize? nil))
(test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm :larm) :debug-view nil :gain1 3.530954 :gain2 1.814612e+05 :rest-torque-ik-args (list :contact-wrench-optimize? t))

(dotimes (i 1000) (random 1.0))
(setq
 *test*
 (let ((surfix "best-rsd "))
 (reverse
  (list
   (test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm :larm) :debug-view nil :surfix surfix)
   (test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm) :debug-view nil :surfix surfix)
   (test-sphere-human-ball-loop :key-list '(:rleg :lleg) :debug-view nil :surfix surfix)
   ))))

(mapcar
 #'(lambda (a)
     (if (and (class a) (find-method a :clear))
	 (send a :clear)))
 (flatten *test*))
(dump-loadable-structure "log/20140525-human-ball-test.log" *test*)

(progn (rsd-serialize :rsd-list *test*) nil)
(setq *test* (car (rsd-deserialize :file "log/...rsd")))

(mapcar #'(lambda (a) (send (nth 2 a) :draw :rest (list *human-ball*)) (send *viewer* :draw-objects) (read-line)) (cdaddr *test*))

(setq *test* (car (rsd-deserialize :file "log/20140526-2-rt(:z :z :x :x)-optimize/conv.rsd")))
(let ((id -1))
  (dolist (d (cdaddr *test*))
    (incf id)
    (if (>= id 8) (return-from nil nil))
    (send (nth 0 d) :draw :rest (list *human-ball*))
    (send *viewer* :draw-objects)
    (send *viewer* :viewsurface :write-to-jpg-file (format nil "init~A.jpg" id))
    (send (nth 1 d) :draw :rest (list *human-ball*))
    (send *viewer* :draw-objects)
    (send *viewer* :viewsurface :write-to-jpg-file (format nil "torque~A.jpg" id))
    (send (nth 2 d) :draw :rest (list *human-ball*))
    (send *viewer* :draw-objects)
    (send *viewer* :viewsurface :write-to-jpg-file (format nil "brlv~A.jpg" id))
    ))

(require "package://eus_gui/gp-util.lisp")
(let ((graph (nth 2 *graph-sample*)))
  (mapcar
   #'(lambda (d name) (send d :name name))
   (send graph :data)
   (list "Torque gradient" "Torque-Force gradient"))
  (graph-panel2gp-graph graph
                        :ylabel "Time [sec]"
                        :xlabel "|Joint Torque| [Nm]"
                        :font "Times-Roman,36"
                        :width 1
                        :save? "rlegllegrarmlarm.eps"
                        :ratio 1.0)
  )

(defmethod graph-panel
  (:simple-draw
   nil
   (send self :clear)
   (send self :color #xFFFFFF)
   ;; (send self :fill-rectangle
   ;; 	 0 0 (send self :width) (send self :height))
   ;; (send self :color #x000000)
   (send self :draw-axis)
   (send self :plot-data)
   (send self :draw-label)
   (send self :message-string "plot")
   (send self :repaint)
   self))


(let ((graph (nth 2 *graph-sample*))
      (av (mapcar
	   #'(lambda (d)
	       (scale (/ 1.0 (length (send d :data)))
		      (reduce #'v+ (send d :data))))
	   (send graph :data))))
  (mapcar
   #'(lambda (d name col)
       (send d :name name)
       (send d :color col))
   (send graph :data)
   (list "Torque gradient" "Torque-Force gradient")
   (list #xFF8888 #x8888FF))
  (send graph :set-range #F(0 0.7) #F(35 1.0))
  (send graph :resize 400 400)
  (send graph :simple-draw)
  (send graph :color #xFFFFFF)
  (send graph :plot-screen (car av) 14)
  (send graph :color #xFF3333)
  (send graph :plot-screen (car av) 8)
  (send graph :color #xFFFFFF)
  (send graph :plot-screen (cadr av) 14)
  (send graph :color #x3333FF)
  (send graph :plot-screen (cadr av) 8)
  (send graph :repaint)
  (print av)
  )
