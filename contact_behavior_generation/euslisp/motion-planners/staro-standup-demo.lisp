;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :staro)
(require "motion-planners/motion-planner.l")

(demo-climb-setup :simple-floor)

(defun setup-body-end-coords
  nil
  (let* ((plink (car (send *robot* :links)))
	 (hip (instance bodyset-link :init
			(make-cascoords
			 :coords
			 ;; (send plink :copy-worldcoords)
			 (make-coords
			  :pos (copy-seq (send plink :worldpos))
			  :rot (transpose (send plink :worldrot))
			  ;; zero coords for now body link coords
			  )
			 )
			:name "body_contact_link"
			:bodies (list (make-cube 10 10 10))))
	 (joint (instance rotational-joint :init :min 0.0 :max 0.0
			  :name "body_contact_fixed_joint"
			  :child-link hip
			  :parent-link plink))
	 (hip-end-coords
	  (make-cascoords :coords (send hip :copy-worldcoords)
			  :parent hip
			  :name :body-end-coords))
	 )
    (cond
     ((send *robot* :get :body-end-coords)
      (send *robot* :set-val 'links
	    (remove
	     (send (send *robot* :get :body-end-coords) :parent)
	     (append (send *robot* :get-val 'links) (list hip))))
      (send (send (send *robot* :get :body-end-coords) :parent)
	    :dissoc
	    (send *robot* :get :body-end-coords))
      )
     (t
      (send *robot* :set-val 'links
	    (append (send *robot* :get-val 'links) (list hip)))))
    (send *robot* :put :body-end-coords hip-end-coords)
    (send hip :add-joint joint)
    (send hip :add-parent-link plink)
    (send plink :assoc hip)
    ;; (send joint :set-val 'default-coords
    ;; (make-coords :pos (float-vector -50 0 -40)))
    (send joint :joint-angle 0)
    ;; (send *robot* :set-val 'joint-list
    ;; (append (send *robot* :get-val 'joint-list) (list joint)))
    ))

(defun gen-body-contact-state
  nil
  (setup-body-end-coords)
  (instance
   simple-contact-state
   :init
   :name :torso
   :contact-coords (send *robot* :get :body-end-coords)
   :contact-n #F(0 0 1)
   :force0 #F(0 0 0 0 0 0)
   :ux 1.0 :uy 1.0 :uz 1.0 :lx 0.3 :ly 0.3
   :target-coords
   (send (send *robot* :get :body-end-coords) :copy-worldcoords)))

(defun test-slide-vs-walk-standup
  nil
  (let* ((buf)
	 (root-path (format nil "log/slide_vs_walk_~A" (log-surfix)))
	 (slide-path (format nil "~A/slide" root-path))
	 (walk-path  (format nil "~A/walk" root-path)))
    (push (demo-standup-motion-plan :log-root slide-path) buf)
    (dump-graphviz-file *sl* :output-dir slide-path :red-rsd-list (car buf))
    (push (demo-standup-motion-plan :log-root walk-path
				    :motion-planner-names
				    (list static-walk-motion-planner
					  remove-motion-planner))
	  buf)
    (dump-graphviz-file *sl* :output-dir walk-path :red-rsd-list (car buf))
    (unix:system
     (format nil
	     "rm -rf latest; ln -s ~A latest;"
	     root-path))
    (reverse buf)
    ))

;; (demo-standup-motion-plan :graph-stack (setq sl (instance stack-list :init)))
(defun demo-standup-motion-plan
  (&key
   (graph-stack (setq *sl* (instance stack-list :init)))
   (log-root "log")
   (log-file (format nil "~A/tmp" log-root))
   (log-stream
    (progn
      (if (not (probe-file log-root))
	  (unix:system (format nil "mkdir -p ~A" log-root)))
      (open log-file :direction :output)))
   (log-stream-buf *log-stream*)
   (motion-planner-names
    (list static-walk-motion-planner
	  slide-motion-planner remove-motion-planner))
   ret
   (timer (instance mtimer :init))
   )
  (setq *log-stream* log-stream)
  ;; roseus
  ;; (defvar *robot-type* :staro) ;; staro expected
  ;; (load "motion-planner.l")
  (send *best-facefall* :draw :friction-cone? nil)
  (send-all *contact-states* :set-val 'gain '(1.0 1.0 100))
  ;;
  (setq
   ret
   (demo-motion-sequence2
    :now-rsd;; (pose-generate-with-contact-state (now-contact-state))
    (let* ((rsd (pose-generate-with-contact-state
		 *facefall-contact-state*))
	   (cs (send rsd :contact-states)))
      (mapcar #'(lambda (cs) (send cs :set-val 'contact-plane-obj :floor)) cs)
      rsd)
    :loop-max;; 10
    '(lambda (cnt &rest args)
       (let* ((ret (simple-fullbody :target (list (list (cons :target :rarm)) (list (cons :target :larm)) (list (cons :target :rleg)) (list (cons :target :lleg))) :debug-view nil :stop 30 :centroid-thre 100 :warnp nil)))
	 (if ret (send *viewer* :draw-objects))
	 ret))
    :motion-planner-names motion-planner-names
    ;; :cs-filter-func
    ;; #'(lambda (cs-list &rest args) cs-list)
    :rec-max-without-best-rsd 20
    :non-stop t
    :error-thre 0.5
    :graph-stack graph-stack
    ;; :cs-filter-func nil
    ))
  (format *log-stream* (format nil "time: ~A sec~%" (send timer :stop)))
  (setq *log-stream* log-stream-buf)
  (send-all (cdr ret) :buf :selected-planner nil)
  (rsd-serialize :rsd-list ret :file (format nil "~A.rsd" log-file))
  ret)

(defun demo-standup-motion-plan-torso-contact
  (&key (graph-stack (setq *sl* (instance stack-list :init))))
  ;; roseus
  ;; (defvar *robot-type* :staro) ;; staro expected
  ;; (load "motion-planner.l")
  ;; (demo-climb-setup :simple-floor)
  (send *best-facefall* :draw :friction-cone? nil)
  (send-all *contact-states* :set-val 'gain '(0.7 1.0 10))
  ;;
  (demo-motion-sequence2
   :now-rsd;; (pose-generate-with-contact-state (now-contact-state))
   (let* ((rsd (pose-generate-with-contact-state
		(list (gen-body-contact-state))))
	  (cs (send rsd :contact-states)))
     (mapcar #'(lambda (cs) (send cs :set-val 'contact-plane-obj :floor)) cs)
     rsd)
   :loop-max;; 10
   '(lambda (cnt &rest args)
      (let* ((ret (simple-fullbody :target (list (list (cons :target :rarm)) (list (cons :target :larm)) (list (cons :target :rleg)) (list (cons :target :lleg))) :debug-view nil :stop 30 :centroid-thre 100 :warnp nil)))
	(if ret (send *viewer* :draw-objects))
	;; (print ret)
	;; (read-line)
	ret))
   :motion-planner-names
   (list static-walk-motion-planner slide-motion-planner remove-motion-planner)
   :cs-filter-func
   #'(lambda (cs-list) cs-list)
   :rec-max-without-best-rsd 20
   :non-stop nil
   :error-thre 0.7
   :graph-stack graph-stack
   ;; :cs-filter-func nil
   ))

(defun generate-random-pose
  nil
  (mapcar
   #'(lambda (j)
       (send j :joint-angle
	     (+ (random (- (send j :max-angle)
			   (send j :min-angle)))
		(send j :min-angle))))
   (send *robot* :joint-list))
  (send *robot* :newcoords
	(make-coords
	 :pos (float-vector 0 0 1000)
	 :rpy (random-vector 3.14)))
  (send *viewer* :draw-objects)
  )

#|

(flatten
 (mapcar
  #'(lambda (d)
      (if (send d :from)
	  (format t " ~A ~A~%"
		  (send-all (send (send d :to) :contact-states)
			    :name)
		  (send d :planner-name))
	))
  (send *sl* :list)))


(mapcar
 #'(lambda (l)
     (pqp-collision-distance l *climb-obj*))
 (send *robot* :links))
