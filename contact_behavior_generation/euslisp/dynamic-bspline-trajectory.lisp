
;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsknts-collada)

(require "motion-sequencer.l")
(if (not (functionp 'float-limb-trajectory))
    (require "dynamic-connector.l"))
(require "dynamics-motion/dynamic-torque-util.l")
(require "dynamics-motion/dynamic-trajectory.l")
(require "util/partition-spline/partition-spline.l")
(require "euslib/irteus_proposals/motion-lib-proposal.l")
;; (require "motion-lib-proposal.l")
;; (require "math/matrix-util.l")

;;
;; rsd-list
;;  -> (av-c-list, (move-target, move-coords, fix-target, fix-coords, cog-traj)-list)
;;             ↓↑
;;  -> (cs-fixed-av-c-list, cs-list)
;;

(defvar *order-factor* 7)

;; (progn (init-kirin-ladder-demo) (dynamic-contact-transition-demo :cog-gain (list -1e-0 -1e-0 -1e-6) :loop-max 10))
(defun init-kirin-ladder-demo
  nil
  (if (not (and (boundp '*climb-obj*)
		*climb-obj*
		(eq (send *climb-obj* :name) :kirin-ladder)))
      (demo-climb-setup :kirin-ladder))
  (setq *rsd* (flatten (rsd-deserialize :file "log/kirin-climb-seq.rsd")))
  (setq *rsd-splitted* (suspend-split (reverse *rsd*)))
  (setq *climbup-rsd* (reverse (nth 3 *rsd-splitted*)))
  ;; (setq *climbup-rsd* (reverse (nth 5 *rsd-splitted*)))
  ;;
  (let* ((id 0) (step (/ 4.0 (- (length *climbup-rsd*) 1))))
    (mapcar
     #'(lambda (p) (send p :buf :time id) (setq id (+ id step)))
     (reverse *climbup-rsd*))))

;; (bench (progn (init-walk-demo) (dynamic-contact-transition-demo :cog-gain (list 1e+3 1e+3 1e-6) :acc-cog-gain (list 1e+1 1e+1 1e-6) :loop-max 100 :graph? nil)))
;; (bench (progn (init-walk-demo) (dynamic-contact-transition-demo :cog-gain (list 1e+3 1e+3 1e-6) :acc-cog-gain (list 1e+1 1e+1 1e-6) :loop-max 1 :graph? nil) (dynamic-contact-transition-demo :cog-gain (list 1e+3 1e+3 1e-6) :acc-cog-gain (list 1e+1 1e+1 1e-6) :loop-max 100 :graph? nil :convergence-thre 0.8 :return-best? nil)))
(defun init-walk-demo
  (&key stand-rsd remove-rsd
	reach-rsd suspend-rsd
	(time-span 2.15)
	(lx 0.1)
	(ly 0.1))
  (demo-climb-setup :simple-floor)
  (init-pose)
  (setq stand-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:rleg :lleg)
			    :lx (list lx lx) :ly (list ly ly))))
  (send *robot* :move-centroid-on-foot :lleg '(:rleg :lleg))
  (setq remove-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:lleg)
			    :lx (list lx) :ly (list ly))
	 :rest-contact-states
	 (now-contact-state :limb-keys '(:rleg)
			    :lx (list lx) :ly (list ly))))
  (send remove-rsd :buf :remove-limb :rleg)
  (send *robot* :inverse-kinematics
	(send (send *robot* :rleg :end-coords :copy-worldcoords)
	      :translate #F(150 0 0) :world)
	:move-target (send *robot* :rleg :end-coords)
	:link-list (send *robot* :link-list (send *robot* :rleg :end-coords :parent)))
  (send *robot* :move-centroid-on-foot :lleg '(:rleg :lleg))
  (setq reach-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:lleg)
			    :lx (list lx) :ly (list ly))
	 :rest-contact-states
	 (now-contact-state :limb-keys '(:rleg)
			    :lx (list lx) :ly (list ly))))
  (send reach-rsd :buf :remove-limb :rleg)
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq suspend-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:rleg :lleg)
			    :lx (list lx lx) :ly (list ly ly))))
  (setq *rsd* (reverse (list stand-rsd remove-rsd reach-rsd suspend-rsd)))
  (setq *rsd-splitted* (list *rsd*))
  (setq *climbup-rsd* *rsd*)
  ;;
  (let* ((id 0) (step (/ time-span (- (length *climbup-rsd*) 1))))
    (mapcar
     #'(lambda (p) (send p :buf :time id) (setq id (+ id step)))
     (reverse *climbup-rsd*))))

;; (setq a (progn (init-2step-walk-demo) (setq a (dynamic-contact-transition-demo :cog-gain (list -1e-0 -1e-0 -1e-10) :loop-max 1 :bspline nil :graph? t))))
(defun init-2step-walk-demo
  (&key stand-rsd remove-rsd reach-rsd suspend-rsd remove-rsd2 reach-rsd2 suspend-rsd2)
  (demo-climb-setup :simple-floor)
  (init-pose)
  (setq stand-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:rleg :lleg))))
  (send *robot* :move-centroid-on-foot :lleg '(:rleg :lleg))
  (setq remove-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:lleg))
	 :rest-contact-states
	 (now-contact-state :limb-keys '(:rleg))))
  (send remove-rsd :buf :remove-limb :rleg)
  (send *robot* :inverse-kinematics
	(send (send *robot* :rleg :end-coords :copy-worldcoords)
	      :translate #F(150 0 0) :world)
	:move-target (send *robot* :rleg :end-coords)
	:link-list (send *robot* :link-list (send *robot* :rleg :end-coords :parent)))
  (send *robot* :move-centroid-on-foot :lleg '(:rleg :lleg))
  (setq reach-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:lleg))
	 :rest-contact-states
	 (now-contact-state :limb-keys '(:rleg))))
  (send reach-rsd :buf :remove-limb :rleg)
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq suspend-rsd
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:rleg :lleg))))
  ;;
  (send *robot* :move-centroid-on-foot :rleg '(:rleg :lleg))
  (setq remove-rsd2
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:rleg))
	 :rest-contact-states
	 (now-contact-state :limb-keys '(:lleg))))
  (send remove-rsd2 :buf :remove-limb :lleg)
  (send *robot* :inverse-kinematics
	(send (send *robot* :lleg :end-coords :copy-worldcoords)
	      :translate #F(150 0 0) :world)
	:move-target (send *robot* :lleg :end-coords)
	:link-list (send *robot* :link-list (send *robot* :lleg :end-coords :parent)))
  (send *robot* :move-centroid-on-foot :rleg '(:rleg :lleg))
  (setq reach-rsd2
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:rleg))
	 :rest-contact-states
	 (now-contact-state :limb-keys '(:lleg))))
  (send reach-rsd2 :buf :remove-limb :lleg)
  (send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq suspend-rsd2
	(pose-generate-with-contact-state
	 (now-contact-state :limb-keys '(:rleg :lleg))))
  (setq *rsd*
	(reverse (list stand-rsd remove-rsd reach-rsd suspend-rsd
		       remove-rsd2 reach-rsd2 suspend-rsd2)))
  (setq *rsd-splitted* (list *rsd*))
  (setq *climbup-rsd* *rsd*)
  ;;
  (let* ((id 0) (step (/ (* 6.0 (/ (- (length *climbup-rsd*) 1) 3))
			 (- (length *climbup-rsd*) 1))))
    (mapcar
     #'(lambda (p) (send p :buf :time id) (setq id (+ id step)))
     (reverse *climbup-rsd*))))

;; dynamic-contact-transition-demo :loop-max 1 :graph? nil :bspline nil
;; (dynamic-contact-transition-demo
;;  :loop-max 1 :graph? nil :bspline nil
;;  :trajectory-extra-args (list :rsd-list (reverse *climbup-rsd*))
;;  :init (setq *cog-buf* nil *avc-buf* nil))

(defun dynamic-contact-transition-demo
  (&key
   (loop-max 10)
   (cog-gain (list -1e+1 -1e+1 -1e-6))
   (acc-cog-gain (list 0 0 0))
   ;; (init (instance partition-spline-contact-wrench-trajectory :init))
   (cog-traj)
   (graph? t)
   (optimize-extra-args)
   (trajectory-extra-args)
   ;; buf
   (bspline) ;; (instance partition-spline-contact-wrench-trajectory :init))
   ret
   init
   (convergence-thre 1.0)
   (return-best? t)
   )
  ;; (init-kirin-ladder-demo)
  (if (and graph? (boundp '*graph-sample*) *graph-sample*) (quit-graph))
  (dotimes (i loop-max)
    (setq bspline
	  (instance* partition-spline-contact-wrench-trajectory :init
		     (append
		      trajectory-extra-args
		      (if (or cog-traj *cog-buf*)
			  (list :cog-trajectory (or cog-traj *cog-buf*))))))
    (cond
     ((null (send bspline :trajectory-elem-list))
      (format *log-stream* "[dynamic-trans] :ik-no-solution~%")
      (return-from nil :ik-no-solution)))
    (cond
     ((send* bspline :optimize optimize-extra-args)
      (if graph?
	  (send bspline :gen-graph :name "force-graph"
		:dim-list (send bspline :force-range)))
      )
     (t
      (format *log-stream* "[dynamic-trans] :qp-no-solution~%")
      (return-from nil :qp-no-solution)))
    (cond
     ((and (not (send bspline :get :optimize-success))
	   (find-if #'(lambda (b) (send b :get :optimize-success)) ret))
      (format *log-stream* "[dynamic-trans] :qp-awkward~%")
      (return-from nil :qp-awkward)))
    ;;
    (push bspline ret)
    (setq cog-traj (send bspline :regenerate-cog-trajectory
			 :cog-gain cog-gain
			 :acc-cog-gain acc-cog-gain))
    (let* ((cnt 0)
	   (max -1) buf
	   (sum (apply
		 #'+
		 (mapcar #'(lambda (d)
			     (setq cnt (+ cnt (+ (length (cdr (assoc :f/fmax d)))
						 (length (cdr (assoc :t/tmax d))))))
			     (setq max
				   (max
				    max
				    (apply #'max (map cons #'abs (cdr (assoc :f/fmax d))))
				    (apply #'max (map cons #'abs (cdr (assoc :t/tmax d))))))
			     (+ (norm2 (cdr (assoc :f/fmax d)))
				(norm2 (cdr (assoc :t/tmax d)))))
			 (send bspline :descrete-dynamics-value)))))
      (send bspline :put :evaluated-value (/ sum (max 1.0 (* 1.0 cnt))))
      (send bspline :put :max-brlv max)
      )
    ;;
    (format *log-stream* "[dynamic-trans] :log~% :objective ~A~% :state ~A~%"
	    (send-all ret :get :evaluated-value) ;;(send (car ret) :get :all-evaluated-value)
	    (mapcar #'cons (send-all ret :get :optimize-success)
		    (send-all ret :get :optimize-trial-cnt))
	    )
    ;;
    (cond
     ((< (send cog-traj :get :diff-integral) convergence-thre)
      (format *log-stream* "[dynamic-trans] :convergence~%")
      (return-from nil :convergence)))
    )
  (cond
   (ret
    (mapcar
     #'(lambda (bs)
	 (send bs :put :all-evaluated-value
	       (send-all ret :get :evaluated-value)))
     ret)
    (format *log-stream* "[dynamic-trans] :done~% :objective ~A~% :state ~A~%"
	    (send-all ret :get :evaluated-value) ;;(send (car ret) :get :all-evaluated-value)
	    (mapcar #'cons (send-all ret :get :optimize-success)
		    (send-all ret :get :optimize-trial-cnt))
	    )
    (setq ret
	  (sort ret #'(lambda (a b) (< (send a :get :evaluated-value)
				       (send b :get :evaluated-value)))))))
  (if return-best?
      (or (find-if #'(lambda (b) (send b :get :optimize-success)) ret)
	  (car ret)) ret)
  )

(defun suspend-split
  (rsd &optional (p))
  (cond
   ((null (cdr rsd)) nil)
   ((or
     (null
      (setq
       p
       (position-if
	#'(lambda (rsd) (eq (send rsd :buf :mseq-mode) :suspend))
	(cdr rsd))))
     (>= (+ 1 (setq p (+ p 1))) (length rsd))) (list rsd))
   (t
    ;; (print (send-all (subseq rsd 0 (+ 1 p)) :buf :mseq-mode))
    (cons (subseq rsd 0 (+ 1 p)) (suspend-split (subseq rsd p))))
   ))

(defun angle-vector-root-coords-vector
  nil
  (concatenate
   float-vector
   (send *robot* :angle-vector)
   (send (send (car (send *robot* :links)) :worldcoords) :worldpos)
   (matrix-log (send (car (send *robot* :links)) :worldrot))))

(defun show-angle-vector-root-coords-vector
  (v &key
     (av-end (length (send *robot* :angle-vector)))
     (pos-end (+ av-end 3))
     (rot-end (+ pos-end 3)))
  (send *robot* :angle-vector (subseq v 0 av-end))
  (dotimes (i 3)
    (send *robot*
	  :transform
	  (send
	   (send (car (send *robot* :links)) :copy-worldcoords)
	   :transformation
	   (make-coords
	    :pos (subseq v av-end pos-end)
	    :rot (matrix-exponent (subseq v pos-end rot-end))))
	  :local)))

(defun animate-bspline-data
  (bspline)
  (mapcar
   #'(lambda (data)
       (show-angle-vector-root-coords-vector (cadr data))
       (send *viewer* :draw-objects)
       (unix:usleep (round (* 0.1 1000 1000))))
   (reverse (send bspline :get :data))))

;; rsd-list
;;  -> (av-c-list, (move-target, move-coords, fix-target, fix-coords, cog-traj)-list)
(defun calc-angle-vector-root-coords-vector-trajectory
  (&rest
   args
   &key
   (rsd-list (reverse *climbup-rsd*))
   &allow-other-keys)
  (apply
   #'minjerk-interpole-partition-spline-vector
   (append
    args
    (list
     :dimension (+ 6 (length (send *robot* :angle-vector)))
     :id-max (* *order-factor* (length rsd-list))
     :start-pos
     (progn
       (send (car rsd-list) :draw :friction-cone? nil)
       (angle-vector-root-coords-vector))
     :end-pos
     (progn
       (send (car (last rsd-list)) :draw :friction-cone? nil)
       (angle-vector-root-coords-vector))
     :mid-pos
     (mapcar #'(lambda (rsd)
		 (send rsd :draw :friction-cone? nil)
		 (angle-vector-root-coords-vector))
	     (cdr (butlast rsd-list)))
     :mid-pos-x
     (let* ((id 0) (step (/ 1.0 (length rsd-list))))
       (mapcar
	#'(lambda (p) (send p :buf :time))
	(cdr (butlast rsd-list))))
     :x-min 0.0
     :x-max 1.0
     :print-x-step 0.033))))

(defun calc-cog-trajectory
  (&rest
   args
   &key
   (rsd-list (reverse *climbup-rsd*))
   &allow-other-keys)
  (apply
   #'minjerk-interpole-partition-spline-vector
   (append
    args
    (list
     :dimension 3
     :id-max (* *order-factor* (length rsd-list))
     :start-pos
     (progn
       (send (car rsd-list) :draw :friction-cone? nil)
       (copy-seq (send *robot* :centroid)))
     :end-pos
     (progn
       (send (car (last rsd-list)) :draw :friction-cone? nil)
       (copy-seq (send *robot* :centroid)))
     :mid-pos
     (mapcar #'(lambda (rsd)
		 (send rsd :draw :friction-cone? nil)
		 (copy-seq (send *robot* :centroid)))
	     (cdr (butlast rsd-list)))
     :mid-pos-x
     (let* ((id 0) (step (/ 1.0 (length rsd-list))))
       (mapcar
	#'(lambda (p) (send p :buf :time))
	(cdr (butlast rsd-list))))
	;;(reverse (subseq rsd-list 1 (- (length rsd-list) 1)))))
     :x-min 0.0
     :x-max 1.0
     :print-x-step 0.033))))

(defun linear-float-limb-trajectory
  (&key
   (now-contact)
   (goal-contact)
   (split-cnt 30))
  (reverse
   (mapcar
    #'(lambda (d)
	(let* ((id 0) (v (cadr d)))
	  (mapcar
	   #'(lambda (cs)
	       (make-coords
		:pos (subseq v id (setq id (+ 3 id)))
		:rot (matrix-exponent
		      (subseq v id (setq id (+ 3 id))))))
	   now-contact)))
    (send
     (minjerk-interpole-partition-spline-vector
      :dimension (* 6 (length now-contact))
      :start-pos
      (apply #'concatenate
	     (cons float-vector
		   (flatten
		    (mapcar
		     #'(lambda (mc)
			 (list
			  (send (send mc :target-coords) :worldpos)
			  (matrix-log (send (send mc :target-coords) :worldrot))))
		     now-contact))))
      :end-pos
      (apply #'concatenate
	     (cons float-vector
		   (flatten
		    (mapcar
		     #'(lambda (mc)
			 (list
			  (send (send mc :target-coords) :worldpos)
			  (matrix-log (send (send mc :target-coords) :worldrot))))
		     goal-contact))))
      :x-min 0.0
      :x-max 1.0
      :print-x-step (/ 1.0 split-cnt))
     :get :data))))

;;  -> (cs-fixed-av-c-list, cs-list)
(defvar *avc-buf*)
(defvar *cog-buf*)
(defvar *dbt-land-height* 10)
(defvar *dbt-away-dist* 30)
(defvar *dbt-col-dist* 0.8)
(defun fix-angle-vector-root-coords-vector
  (&rest
   args
   &key
   (rsd-list (reverse *climbup-rsd*))
   (angle-vector-root-coords-vector-trajectory
    (or
     *avc-buf*
     (setq
      *avc-buf*
      (apply #'calc-angle-vector-root-coords-vector-trajectory
	     (append (list :output-stream nil) args)))
     ))
   (cog-trajectory
    (or *cog-buf*
	(setq *cog-buf*
	      (apply #'calc-cog-trajectory args))
	))
   data-list
   (move-coords-interpole-type :float)
   (animate-time 10.0)
   ;; root-vel-acc limits
   (root-velocity-limit
    (scale 1e+3 (float-vector 100 100 100 1 1 1)))
   (root-acceleration-limit
    (scale 100 (float-vector 500 500 500 5 5 5)))
   root-limit-min-buf
   root-limit-max-buf
   (root-pos-0) (root-pos-1) (root-pos-2)
   ;; return
   coords-list
   position-list
   (freq ;;0
    (/ (- (length (send cog-trajectory :get :data)) 1.0)
       (- (car (car (send cog-trajectory :get :data)))
	  (caar (last (send cog-trajectory :get :data)))))
    )
   time-list
   ret
   &allow-other-keys)
  (catch :ik-no-solution
    ;; (format t " arg: ~A~%" args)
    (setq data-list
	  (reverse
	   (mapcar
	    #'(lambda (avc cog)
		(list (cons :time (car avc))
		      (cons :avc (cadr avc))
		      (cons :cog (cadr cog))))
	    (send angle-vector-root-coords-vector-trajectory
		  :get :data)
	    (send cog-trajectory :get :data))))
    (scale (/ 1.0 freq) root-velocity-limit root-velocity-limit)
    (scale (/ 1.0 (* freq freq))
	   root-acceleration-limit root-acceleration-limit)
    (show-angle-vector-root-coords-vector
     (cdr (assoc :avc (car data-list))))
    (setq root-pos-0
	  (subseq (cdr (assoc :avc (car data-list)))
		  (length (send *robot* :angle-vector))))
    (setq root-pos-1
	  (subseq (cdr (assoc :avc (car data-list)))
		  (length (send *robot* :angle-vector))))
    (setq root-pos-2
	  (subseq (cdr (assoc :avc (car data-list)))
		  (length (send *robot* :angle-vector))))
    (mapcar
     #'(lambda (goal-rsd now-rsd)
	 ;; (send now-rsd :draw :rest (list *climb-obj*)) ;; :friction-cone? nil)
	 (let* ((d-list
		 (remove-if
		  #'(lambda (d)
		      (not
		       (and (<= (cdr (assoc :time d))
				(if (eq goal-rsd (car (last rsd-list)))
				    1e+100
				  (send goal-rsd :buf :time)))
			    (> (cdr (assoc :time d))
			       (if (eq now-rsd (car rsd-list))
				   -1e+100
				 (send now-rsd :buf :time))))))
		  data-list))
		(move-contact
		 (if (and (send now-rsd :buf :remove-limb)
			  (eq (send now-rsd :buf :remove-limb)
			      (send goal-rsd :buf :remove-limb)))
		     (flatten
		      (list (find-if
			     #'(lambda (cs)
				 (eq (send now-rsd :buf :remove-limb)
				     (send cs :name)))
			     (send now-rsd :contact-states))))))
		(goal-contact
		 (if (and (send now-rsd :buf :remove-limb)
			  (eq (send now-rsd :buf :remove-limb)
			      (send goal-rsd :buf :remove-limb)))
		     (flatten
		      (list (find-if
			     #'(lambda (cs)
				 (eq (send goal-rsd :buf :remove-limb)
				     (send cs :name)))
			     (send goal-rsd :contact-states))))))
		(fix-contact
		 (remove-if
		  #'(lambda (cs) (find cs move-contact))
		  (send now-rsd :contact-states)))
		(fix-target (send-all fix-contact :contact-coords))
		(fix-coords (send-all fix-target :copy-worldcoords))
		;; (move-target (send-all move-contact :contact-coords))
		;; (hoge (format t " move-contact=~A~% d-list=~A~%" move-contact d-list))
		(move-coords-list
		 (progn
		   ;; (require "dynamic-connector.l")
		   (or (send now-rsd :buf :move-coords-list)
		       (if (null move-contact)
			   (if (null d-list) (print 'nil-data-list)
			     (make-list (length d-list))))
		       (cond
			((eq move-coords-interpole-type :linear)
			 (linear-float-limb-trajectory
			  :now-contact move-contact
			  :goal-contact goal-contact
			  :split-cnt (length d-list)))
			(t
			 (mapcar
			  #'list
			  (float-limb-trajectory
			   now-rsd goal-rsd
			   :split-cnt (- (length d-list) 1)
			   :away-dist *dbt-away-dist*
			   :land-height *dbt-land-height*
			   :col-dist *dbt-col-dist*
			   :time-list '(0 0.5 0.8 1.0)
			   :debug-view :no-message)
			  ))))))
		(prev-avc)
		(id -1) (blend-rate) (id^ (- (length d-list) 1.0))
		)
	   ;; (format t " ~A -> ~A~%" now-rsd goal-rsd)
	   ;; (read-line)
	   (mapcar
	    #'(lambda (d mc)
		(incf id)
		(setq prev-avc (angle-vector-root-coords-vector))
		;; triangle function
		(cond
		 ((> id (* id^ 0.66))
		  (setq blend-rate (/ (* 3.0 (- id^ id)) id^)))
		 ((< id (* id^ 0.33))
		  (setq blend-rate (/ (* 3.0 id) id^)))
		 (t (setq blend-rate 1)))
		(show-angle-vector-root-coords-vector
		 (v+ (scale blend-rate prev-avc)
		     (scale (- 1 blend-rate) (cdr (assoc :avc d)))))
		;; acc-limit
		(setq root-pos-2 root-pos-1)
		(setq root-pos-1 root-pos-0)
		(setq root-pos-0
		      (subseq (angle-vector-root-coords-vector)
			      (length (send *robot* :angle-vector))))
		(setq root-limit-min-buf
		      (vmax
		       (v+ (scale -1 root-acceleration-limit)
			   (v+ (v+ (scale -1 root-pos-0)
				   (scale 2 root-pos-1))
			       (scale -1 root-pos-2)))
		       (v+ (scale -1 root-velocity-limit)
			   (v+ (scale -1 root-pos-0) root-pos-1))))
		(setq root-limit-max-buf
		      (vmin
		       (v+ (scale +1 root-acceleration-limit)
			   (v+ (v+ (scale -1 root-pos-0)
				   (scale 2 root-pos-1))
			       (scale -1 root-pos-2)))
		       (v+ (scale +1 root-velocity-limit)
			   (v+ (scale -1 root-pos-0) root-pos-1))))
		(format *log-stream* "~A < v < ~A~%" root-limit-min-buf root-limit-max-buf)
		;;
		(cond
		 ((apply
		   #'ik-wrapper
		   (append
		    args
		    (list
		     :target
		     (append
		      (mapcar
		       #'(lambda (cs tc cc)
			   (list (cons :target tc)
				 (cons :coords cc)
				 (cons :translation-axis (send cs :translation-axis))
				 (cons :rotation-axis (send cs :rotation-axis))))
		       fix-contact fix-target fix-coords)
		      (mapcar
		       #'(lambda (cs c)
			   (list (cons :target (send cs :contact-coords))
				 (cons :coords c)
				 (cons :translation-axis (send cs :translation-axis))
				 (cons :rotation-axis (send cs :rotation-axis))))
		       goal-contact ;;move-contact
		       mc))
		     :debug-view nil
		     :target-centroid-pos (cdr (assoc :cog d))
		     :cog-gain 0.5
		     :centroid-thre 50
		     :balance-leg nil
		     :collision-avoidance-link-pair nil
		     :max
		     (map float-vector #'*
			  (list 1 1 1
				(rad2deg 1) (rad2deg 1) (rad2deg 1))
			  root-limit-max-buf)
		     :min
		     (map float-vector #'*
			  (list 1 1 1
				(rad2deg 1) (rad2deg 1) (rad2deg 1))
			  root-limit-min-buf)
		     ;; :max #F(500 500 1000 300 300 300)
		     ;; :min #F(-500 -500 -1000 -300 -300 -300)
		     :root-link-virtual-joint-weight
		     (float-vector 0.1 0.1 0.1 0.01 0.01 0.01)
		     :stop 30
		     :revert-if-fail t)
		    ))
		  (send *viewer* :draw-objects)
		  (push (send (car (send *robot* :links))
			      :copy-worldcoords) coords-list)
		  (push (copy-seq (send *robot* :angle-vector))
			position-list)
		  (format *log-stream* "- fix-dif: ~A~%" (mapcar #'(lambda (cs) (norm (concatenate float-vector (scale 1e-3 (send (send cs :contact-coords) :difference-position (send cs :target-coords))) (send (send cs :contact-coords) :difference-rotation (send cs :target-coords))))) fix-contact))
		  (push (cdr (assoc :time d)) time-list)
		  (if animate-time
		      (unix:usleep
		       (round
			(* (/ animate-time (length (send angle-vector-root-coords-vector-trajectory :get :data)))
			   1000 1000)))))
		 (t (throw :ik-no-solution nil)))
		)
	    d-list move-coords-list)
	   ))
     (cdr rsd-list) rsd-list)
    (setq ret
	  (gen-dynamic-trajectory
	   :freq freq :position-list position-list
	   :coords-list coords-list :fix-length t))
    (mapcar #'(lambda (traj tm) (send traj :put :time tm))
	    ret (reverse time-list))
    ret))

(defun test-regenerate-cog-trajectory
  (bspline
   &key
   (x-min (send bspline :x-min))
   (x-max (send bspline :x-max))
   (x-step (/ (- x-max x-min) 3.0))
   (x-cnt (- x-min x-step))
   (x-list
    (let* ((buf nil))
      (while (< (setq x-cnt (+ x-cnt x-step))
		(+ x-max (/ x-step 2.0)))
	(push x-cnt buf))
      (reverse buf)))
   (mid-x-list (cdr (butlast x-list)))
   (force-gradient nil)
   (mid-dc-list
    (mapcar
     #'(lambda (x)
	 (let* (min max)
	   (format t "[force-delta] ~A E " x)
	   (mapcar
	    #'(lambda (gmin gmax)
		(cond
		 ((and (<= x (cdr (assoc :time gmax)))
		       (>= x (cdr (assoc :time gmin))))
		  (setq min gmin) (setq max gmax))
		 ((and (>= x (cdr (assoc :time gmax)))
		       (<= x (cdr (assoc :time gmin))))
		  (setq min gmax) (setq max gmin))))
	    (cdr force-gradient) force-gradient)
	   (format t "[~A ~A]~%"
		   (cdr (assoc :time min))
		   (cdr (assoc :time max)))
	   (cond
	    ((and min max)
	     (list (cons :near-min-grad min)
		   (cons :near-max-grad max)
		   (cons :dc-average
			 (scale
			  (/ 1.0
			     (abs (- (cdr (assoc :time max))
				     (cdr (assoc :time min)))))
			  (v+
			   (scale
			    (abs (- x (cdr (assoc :time max))))
			    (cdr (assoc :dc min)))
			   (scale
			    (abs (- x (cdr (assoc :time min))))
			    (cdr (assoc :dc max)))))))))
	   ;;(v+ (cdr (assoc :dc min))
	   ;;(cdr (assoc :dc max))))
	   ))
     mid-x-list))
   ;;
   (cog-gain '(5e-1 5e-1 1e-6))
   (acc-cog-gain cog-gain)
   ;;
   (regen-bspline
    (minjerk-interpole-partition-spline-vector
     :dimension (length (send bspline :calc 0))
     :id-max (send bspline :id-max)
     :recursive-order (send bspline :recursive-order)
     :x-min x-min :x-max x-max
     :start-pos (send bspline :calc x-min)
     :end-pos (send bspline :calc x-max)
     :start-acc (send bspline :calc-delta x-min :n 2 :discrete? nil)
     :end-acc (send bspline :calc-delta x-max :n 2 :discrete? nil)
     ;;
     :mid-pos-x mid-x-list
     :mid-acc-x mid-x-list
     :mid-pos
     (mapcar #'(lambda (x grad)
		 (let* ((ret (send bspline :calc x))
			(dc
			 (if grad
			     (map float-vector
				  #'*
				  cog-gain
				  (subseq (cdr (assoc :dc-average grad)) 0 3)))))
		   (format t "  mid-pos(~A) ~A -+ ~A~%" x ret dc)
		   (if grad (setq ret (v+ ret dc)))
		   ret))
	     mid-x-list mid-dc-list)
     :mid-acc
     (mapcar #'(lambda (x grad)
		 (let* ((ret (send bspline :calc-delta x :n 2 :discrete? nil))
			(ddc
			 (if grad
			     (map float-vector
				  #'*
				  acc-cog-gain
				  (subseq (cdr (assoc :dc-average grad)) 3 6)))))
		   (format t "  mid-pos(~A) ~A -+ ~A~%" x ret ddc)
		   (if grad (setq ret (v+ ret ddc)))
		   ret))
	     mid-x-list mid-dc-list)
     :debug? nil
     ))
   ;;
   (debug? t)
   (print-x-step 0.033)
   (print-tm (- x-min print-x-step))
   (dif-intergral 0)
   &allow-other-keys
   )
  (format *log-stream* "[test-regenerate-cog-trajectory]~%")
  (send bspline :get-descrete-points
	:print-x-step print-x-step
	:print-tm print-tm
	:output-stream nil)
  (send regen-bspline :get-descrete-points
	:print-x-step print-x-step
	:print-tm print-tm
	:output-stream nil)
  (mapcar
   #'(lambda (d1 d2)
       (setq dif-intergral
	     (+ dif-intergral
		(* print-x-step (norm (v- (cadr d1) (cadr d2))))))
       (format t " ~A: org=~A dif=~A~%"
	       (car d1) (cadr d1) (v- (cadr d1) (cadr d2))))
   (send bspline :get :data)
   (send regen-bspline :get :data))
  (format *log-stream* "  diff-integral=~A~%" dif-intergral)
  ;; (format *log-stream* "[test-regenerate-cog-trajectory]~%")
  (send regen-bspline :put :diff-integral dif-intergral)
  regen-bspline)

;; if no solution, check if sep is enough large
(defclass partition-spline-contact-wrench-trajectory
  :super partition-spline-vector
  :slots (robot
	  rsd-list
	  contact-name-list
	  contact-constraints-list
	  contact-switch-time-list
	  contact-time-list
	  ;;
	  trajectory-elem-list
	  force-range
	  force-range-list
	  tm-list
	  ;;
	  objective-vector-buf
	  objective-matrix
	  ;;
	  descrete-dynamics-param
	  descrete-dynamics-value
	  cog-trajectory
	  angle-vector-root-coords-vector-trajectory
	  ;;
	  force-gradient-param
	  ))
(defmethod partition-spline-contact-wrench-trajectory
  (:init
   (&rest
    args
    &key
    (robot *robot*)
    (rsd-list (reverse *climbup-rsd*))
    (contact-name-list
     (union
      (send-all
       (flatten (send-all rsd-list :contact-states)) :name) nil))
    (contact-constraints-list
     (mapcar
      #'(lambda (goal-rsd now-rsd)
	  (mapcar
	   #'(lambda (name)
	       (cond
		((or (null (send now-rsd :contact-states name))
		     (and (eq name (send now-rsd :buf :remove-limb))
			  (eq name (send goal-rsd :buf :remove-limb))))
		 (list (unit-matrix 6) (instantiate float-vector 6) nil))
		(t
		 (let* ((cs (send now-rsd :contact-states name))
			(r (matrix-append
			    (list (transpose (send cs :z-rot))
				  (transpose (send cs :z-rot)))
			    '(1 1))))
		   (list
		    (m*
                     (let ((tmp-contact-constraint
                            (instance default-contact-constraint :init
                                      :mu-trans (aref (send cs :slip-matrix) 0 2)
                                      :mu-rot (aref (send cs :slip-matrix) 5 2)
                                      :l-max-x
                                      (* +1e+3 (aref (send cs :slip-matrix) 4 2))
                                      :l-min-x
                                      (* -1e+3 (aref (send cs :slip-matrix) 4 2))
                                      :l-max-y
                                      (* +1e+3 (aref (send cs :slip-matrix) 3 2))
                                      :l-min-y
                                      (* -1e+3 (aref (send cs :slip-matrix) 3 2)))))
                       (send tmp-contact-constraint :calc-constraint-matrix (make-coords)))
		     r)
		    (float-vector (* -1 (aref (send cs :force0) 0))
				  (* -1 (aref (send cs :force0) 0))
				  (* -1 (aref (send cs :force0) 1))
				  (* -1 (aref (send cs :force0) 1))
				  (* -1 (aref (send cs :force0) 5))
				  (* -1 (aref (send cs :force0) 5))
				  (* -1 (aref (send cs :force0) 4))
				  (* -1 (aref (send cs :force0) 4))
				  (* -1 (aref (send cs :force0) 3))
				  (* -1 (aref (send cs :force0) 3))
				  (* -1 (aref (send cs :force0) 2))
				  ;; 0
				  )
		    cs
		    )
		   ))))
	   contact-name-list))
      (cdr rsd-list) rsd-list))
    ;;
    (force-range-list
     (let ((id -1))
       (mapcar
	#'(lambda (a) (mapcar #'(lambda (b) (incf id)) (make-list 6)))
	(make-list (length contact-name-list)))))
    (force-range (apply #'append force-range-list))
    (id-max (* 1 *order-factor* (length rsd-list)))
    (_recursive-order 3)
    (recursive-order
     (make-list (* 6 (length contact-name-list))
		:initial-element _recursive-order))
    (tm-list (let ((tmp -1))
	       (mapcar #'(lambda (hoge) (incf tmp))
		       (make-list id-max))))
    (x-min (apply #'min (or (remove-if-not #'numberp (send-all rsd-list :buf :time)) (list 0))))
    (x-max (apply #'max (or (remove-if-not #'numberp (send-all rsd-list :buf :time)) (list 2.0))))
    (contact-switch-time-list
     (let* ((id 0) (step (/ (- x-max x-min) (- (length rsd-list) 1))))
       (mapcar
	#'(lambda (p)
	    (send p :buf :time
		  (or (send p :buf :time) id))
	    (setq id (+ step (send p :buf :time)))
	    (send p :buf :time))
	rsd-list)))
    (contact-time-list
     (mapcar
      #'(lambda (next-sw now-sw)
	  (let* ((bef
		  (round (/ (* 1.0 (- id-max _recursive-order)
			       (- now-sw x-min)) (- x-max x-min))))
		 (aft
		  (round (+ _recursive-order
			    (/ (* 1.0 (- id-max _recursive-order)
				  (- next-sw x-min)) (- x-max x-min)))))
		 (N (- aft bef)))
	    (decf bef)
	    (mapcar #'(lambda (v) (incf bef)) (make-list N))))
      (cdr contact-switch-time-list) contact-switch-time-list))
    angle-vector-root-coords-vector-trajectory
    cog-trajectory
    (freq 30.0)
    (static? nil)
    (trajectory-elem-list
     (apply
      #'fix-angle-vector-root-coords-vector
      (append
       args
       (list
	:angle-vector-root-coords-vector-trajectory
	(or
	 angle-vector-root-coords-vector-trajectory
	 ;; *avc-buf*
	 (setq
	  angle-vector-root-coords-vector-trajectory
	  (setq
	   *avc-buf*
	   (apply #'calc-angle-vector-root-coords-vector-trajectory
		  (append (list :output-stream nil
				:x-min x-min :x-max x-max
				:print-x-step (/ 1.0 freq)
				:rsd-list rsd-list)
			  args)))))
	:cog-trajectory
	(or cog-trajectory  ;; *cog-buf*
	    (setq
	     cog-trajectory
	     (setq *cog-buf*
		   (apply #'calc-cog-trajectory
			  (append
			   (list :x-min x-min :x-max x-max
				 :print-x-step (/ 1.0 freq)
				 :rsd-list rsd-list)
			   args)))))
	:x-min x-min :x-max x-max :rsd-list rsd-list
	:freq (if static? 0 freq) ;;freq
	:animate-time 1.0))))
    &allow-other-keys
    )
   ;; (format t " trajectory-elem-list = ~A~%" trajectory-elem-list)
   (send-super* :init
		:dimension (+ 1 (car (last force-range)))
		:id-max id-max
		:recursive-order recursive-order
		:x-min x-min
		:x-max x-max
		args
		)
   ;;
   ;; filter
   (if (not (listp trajectory-elem-list))
       (setq trajectory-elem-list nil))
   (dotimes (i 1);; 3)
   	     ;; (max
   	     ;;  1
   	     ;;  (round (/ (* (/ (length trajectory-elem-list) 60.0)
   	     ;; 		   6.0
   	     ;; 		   (- x-max x-min))
   	     ;; 		(length rsd-list)))))
     (mapcar
      #'(lambda (tj1 tj2 tj3)
   	  (mapcar
   	   #'(lambda (k)
   	       (send tj2 k
   		     (v+ (scale (/ 1.0 3) (send tj1 k))
   			 (scale (/ 1.0 3) (v+ (send tj2 k)
					      (send tj3 k))))))
   	   '(:velocity :acceleration :dx :ddx :w :dw))
   	  )
      ;; (butlast trajectory-elem-list)
      ;; trajectory-elem-list
      ;; (cdr trajectory-elem-list))
      (append (flatten (list (car trajectory-elem-list)))
	      (butlast trajectory-elem-list))
      trajectory-elem-list
      (append (cdr trajectory-elem-list)
	      (last trajectory-elem-list)))
     )
   ;;
   (setq objective-vector-buf
	 (mapcar
	  #'(lambda (bs)
	      (instantiate float-vector
			   (+ (send bs :recursive-order) 1)))
	  (send self :partition-spline-list)))
   (setq objective-matrix
	 (convert-vertical-coeff-matrix-for-gain-vector
	  (convert-horizontal-coeff-matrix-for-gain-vector
	   (matrix-append
	    (mapcar
	     #'(lambda (bs buf)
		 (send bs :calc-integral-objective-coeff-matrix
		       :n 2 :retv buf))
	     (send self :partition-spline-list)
	     objective-vector-buf)
	    '(1 1))
	   :dimension (+ 1 (car (last force-range))))
	  :dimension (+ 1 (car (last force-range)))))
   ;;
   (mapcar
    #'(lambda (k-d-v)
	(send self
	      (read-from-string (format nil ":~A" (car k-d-v)))
	      (eval (car k-d-v))))
    (send self :slots))
   )
  (:calc-dynamics
   (traj tm &key
	 (n 0)
	 (contact-states
	  (mapcar #'caddr (car contact-constraints-list)))
	 (dim-list
	  (apply #'append
		 (mapcar #'(lambda (cs range) (if cs range))
			 contact-states force-range-list)))
	 mat)
   ;; (print contact-states)
   (send robot :angle-vector
	 (copy-seq (send traj :position)))
   (send robot :transform
	 (send (send (car (send robot :links)) :copy-worldcoords)
	       :transformation
	       (send traj :coords))
	 :local)
   ;; (format t  "~A ~A~%" contact-states traj)
   (setq mat
	 (calc-torque-eq
	  :robot robot
	  :root-link (car (send robot :links))
	  ;; :root-pos
	  ;; (let* ((vbuf (float-vector 0 0 0)) (cnt 0))
	  ;;   (dolist (cs (flatten contact-states))
	  ;;     (v+ vbuf (send (send cs :target-coords) :worldpos) vbuf)
	  ;;     (incf cnt))
	  ;;   (scale (/ 1.0 cnt) vbuf vbuf)
	  ;;   vbuf)
	  :contact-states
	  (mapcar #'(lambda (cs)
		      (list
		       (cons :link
			     (send (send cs :contact-coords) :parent))
		       (cons :worldcoords
			     (copy-object (send cs :target-coords)))))
		  (flatten contact-states))
	  :all-links (cdr (send robot :links))
	  :dq (map float-vector #'deg2rad (send traj :velocity))
	  :ddq (map float-vector #'deg2rad (send traj :acceleration))
	  :root-angular-velocity (send traj :w)
	  :root-angular-acceleration (send traj :dw)
	  :root-spacial-velocity (scale 1e-3 (send traj :dx))
	  :root-spacial-acceleration (scale 1e-3 (send traj :ddx))
	  ))
   ;;
   (list
    (cons :contact-states contact-states)
    (cons :mat mat)
    (cons :time tm)
    (cons :force-range dim-list)
    (cons :Gf
	  (m*
	   (cdr (assoc :G mat))
	   (send self :calc-pos-constraints-coeff-matrix-for-gain-vector
		 :tm tm
		 :delta n
		 :dim-list dim-list)))
    (cons :JTf
	  (m*
	   (cdr (assoc :J mat))
	   (send self :calc-pos-constraints-coeff-matrix-for-gain-vector
		 :tm tm
		 :delta n
		 :dim-list dim-list)))
    (cons :Tmin (v- (cdr (assoc :tau0 mat))
		    (map float-vector
			 #'(lambda (l) (send (send l :joint) :max-joint-torque))
			 (cdr (send robot :links)))))
    (cons :Tmax (v+ (cdr (assoc :tau0 mat))
		    (map float-vector
			 #'(lambda (l) (send (send l :joint) :max-joint-torque))
			 (cdr (send robot :links)))))
    (cons :mg (concatenate float-vector (cdr (assoc :fb0 mat))
			   (cdr (assoc :mb0 mat))))))
  (:friction
   (&key (tm-list (car contact-time-list))
	 (contact-constraints (car contact-constraints-list))
	 (dim-list force-range))
   (send self :calc-coeff-matrix-for-gain-vector
	 (matrix-append
	  (mapcar #'car contact-constraints) '(1 1))
	 :dim-list dim-list
	 :tm-list tm-list))
  (:calc-descrete-dynamics-param
   (&key (id -1)
	 (sep
	  (max
	   1
	   (round (/ (length trajectory-elem-list)
		     (* (/ *order-factor* 2.0) (length rsd-list))))
	   ;; (round (/ (* (/ (length trajectory-elem-list) 60.0)
	   ;; 6.0
	   ;; (- x-max x-min))
	   ;; (length rsd-list)))
	   )))
   (setq descrete-dynamics-param
	 (apply #'append
		(mapcar
		 #'(lambda (traj)
		     (if (eq 0 (mod (incf id) sep))
			 (list
			  (send self :calc-dynamics
				traj (send traj :get :time)
				:contact-states
				(or
				 (mapcar
				  #'caddr
				  (caddr
				   (find-if
				    #'(lambda (grsd-nrsd-cst)
					(and
					 (<= (send traj :get :time)
					     (send (car grsd-nrsd-cst)
						   :buf :time))
					 (>= (send traj :get :time)
					     (send (cadr grsd-nrsd-cst)
						   :buf :time))))
				    (mapcar #'list (cdr rsd-list) rsd-list
					    contact-constraints-list))))
				 (mapcar #'caddr
					 (car (last contact-constraints-list))))
				))))
		 trajectory-elem-list))))
  (:calc-descrete-dynamics-value
   (&key
    (sep
     (max
      1
      (round (/ (length trajectory-elem-list)
		(* (/ *order-factor* 2.0) (length rsd-list))))
      ))
    (update? t))
   (if update? (send self :calc-descrete-dynamics-param :sep sep))
   (setq descrete-dynamics-value
	 (mapcar
	  #'(lambda (d)
	      (let* (;;(range (cdr (assoc :force-range d)))
		     ;;(force-buf (instantiate float-vector (length range)))
		     (force (send self :calc (cdr (assoc :time d))))
		     (cs (cdr (assoc :contact-states d)))
		     (flist
		      (let* ((id -1) buf)
			(dolist (c cs)
			  (push (subseq force (* 6 (incf id)) (* 6 (+ id 1))) buf))
			(reverse buf)))
		     (force-buf
		      (apply
		       #'concatenate
		       (cons float-vector
			     (flatten (mapcar
				       #'(lambda (cs f)
					   (cond (cs f) (t nil)))
				       cs flist)))))
		     (f/fmax
		      (mapcar
		       #'(lambda (cs f)
			   (cond
			    (cs
			     (list (send cs :force-error-world f)
				   (copy-seq (send cs :f/fmax))))
			    (t nil)))
		       cs flist))
		     (i -1) tau)
		;; (dolist (r range)
		;; (setf (aref force-buf (incf i)) (aref force r)))
		(list (cons :time (cdr (assoc :time d)))
		      (cons :force force-buf)
		      (cons :flist flist)
		      (cons :f/fmax
			    (apply
			     #'concatenate
			     (cons float-vector (flatten (mapcar #'cadr f/fmax)))))
		      (cons :f/fmax-world
			    (apply
			     #'concatenate
			     (cons float-vector (flatten (mapcar #'car f/fmax)))))
		      (cons :f/fmax-list
			    (mapcar #'(lambda (f) (or (cadr f) (instantiate float-vector 6)))
				    f/fmax))
		      (cons :jacobian (cdr (assoc :J (cdr (assoc :mat d)))))
		      (cons :grasp-matrix (cdr (assoc :G (cdr (assoc :mat d)))))
		      (cons :torque
			    (setq
			     tau
			     (v-
			      (transform (cdr (assoc :J (cdr (assoc :mat d)))) force-buf)
			      (cdr (assoc :tau0 (cdr (assoc :mat d)))))))
		      (cons :t/tmax
			    (map float-vector
				 #'/ tau
				 (send-all
				  (send-all (cdr (send robot :links)) :joint)
				  :max-joint-torque)))
		      )))
	  descrete-dynamics-param))
   )
  (:calc-force-delta
    (&key
     (update-dynamic? t)
     (dynamic-value
      (if update-dynamic? (send self :calc-descrete-dynamics-value)
	(send self :descrete-dynamics-value)))
     (force-filter
      #'(lambda (d)
	  (v+ (scale 1 (cdr (assoc :f/fmax-world d)))
	      (scale
	       -1e-1 ;;-1e-3
	       (transform (pseudo-inverse (cdr (assoc :jacobian d)))
			  (cdr (assoc :t/tmax d)))))))
     (filtered-force (mapcar force-filter dynamic-value))
     (m (* 1e-3 (send robot :weight)))
     (g (scale -1e-3 *g-vec*)) ;;(float-vector 0 0 -9.8))
     (gen-grad-matrix
      (mapcar
       #'(lambda (dv df)
	   (let* ((c (send cog-trajectory :calc (cdr (assoc :time dv))))
		  (ddc (send cog-trajectory :calc-delta (cdr (assoc :time dv))
			     :n 2 :discrete? nil))
		  (gm (cdr (assoc :grasp-matrix dv)))
		  I-mI/mddcx-mgx-mcx buf)
	     ;; mI        -mI  dc
	     ;; mddcx-mgx -mcx ddc
	     ;; (scale 0 ddc ddc)
	     (scale 0 c c) ;; cog coordinates
	     (setq
	      I-mI/mddcx-mgx-mcx
	      (matrix-append
	       (list
		(matrix-append
		 (list (scale-matrix m (unit-matrix 3))
		       (scale-matrix (* -1 m) (unit-matrix 3)))
		 '(0 1))
		(matrix-append
		 (list (outer-product-matrix (scale m (v- ddc g)))
		       (outer-product-matrix (scale (* -1 m) c)))
		 '(0 1)))
	       '(1 0)))
	     (setq buf (m* (pseudo-inverse I-mI/mddcx-mgx-mcx) GM))
	     (list (cons :time (cdr (assoc :time dv)))
		   (cons :G gm)
		   (cons :dc-mat I-mI/mddcx-mgx-mcx)
		   (cons :dc^-1G buf)
		   (cons :df df)
		   (cons :ddc-now ddc)
		   (cons :dc (transform buf (scale -1 df))))
	     ))
       dynamic-value filtered-force))
     )
    (setq force-gradient-param gen-grad-matrix)
    )
  ;;
  ;;  (:
  (:regenerate-cog-trajectory
   (&rest args) (apply #'test-regenerate-cog-trajectory
		       (append
			(list cog-trajectory
			      :force-gradient
			      (send self :calc-force-delta))
			args)))
  (:optimize
   (&key (id -1)
	 (trial-cnt 10)
	 (ineq-offset 20)
	 (ineq-scale 1.5)
	 (trial-cnt-feedback #i(0))
	 ret)
   (send-all (send *robot* :joint-list) :joint-torque 0)
   (send self :calc-descrete-dynamics-param)
   ;;
   (setq
    ret
    (solve-qp
     :debug? t
     :trial-cnt trial-cnt
     :trial-cnt-feedback trial-cnt-feedback
     :ineq-offset ineq-offset
     :ineq-scale ineq-scale
     :initial-state (setq gain-vector (instantiate float-vector (* id-max dimension)))
     :eval-weight-matrix ;; (unit-matrix (* id-max dimension)) ;;objective-matrix
     (m+ (scale-matrix (* 1e-1 (expt 10 (/ (length rsd-list) 4.0)))
		       objective-matrix)
	 (unit-matrix (* id-max dimension)))
     :equality-matrix
     (matrix-append
      (flatten
       (list
	;; Gf
	(mapcar #'(lambda (d) (cdr (assoc :Gf d))) descrete-dynamics-param)
	;; zero force
	(mapcar
	 #'(lambda (tm-list constraints)
	     (if (find-if #'(lambda (c) (null (caddr c))) constraints)
		 (send self :friction :tm-list tm-list
		       :dim-list
		       (apply #'append
			      (mapcar #'(lambda (cs range) (if (not cs) range))
				      (mapcar #'caddr constraints)
				      force-range-list))
		       :contact-constraints
		       (remove-if #'caddr constraints))))
	 contact-time-list contact-constraints-list)
	))
      '(1 0))
     :equality-vector
     (apply
      #'concatenate
      (cons
       float-vector
       (flatten
	(list
	 ;; base torque
	 ;; equation of motion
	 (mapcar #'(lambda (d) (cdr (assoc :mg d))) descrete-dynamics-param)
	 ;; zero force
	 (mapcar
	  #'(lambda (tm-list constraints)
	      (if (find-if #'(lambda (c) (null (caddr c))) constraints)
		  (send self
			:constant-vector
			(apply
			 #'concatenate
			 (cons
			  float-vector
			  (mapcar
			   #'cadr
			   (remove-if #'(lambda (c) (caddr c)) constraints))))
			;; :dim-list
			;; (apply #'append
			;;        (mapcar #'(lambda (cs range) (if (not cs) range))
			;; 	       (mapcar #'caddr constraints)
			;; 	       force-range-list))
			;;(apply #'concatenate
			;;(cons float-vector
			;;(mapcar #'cadr constraints)))
			:tm-list tm-list)))
	  contact-time-list contact-constraints-list)))))
     :inequality-matrix
     (matrix-append
      (flatten
       (list
	;; friction
	(mapcar
	 #'(lambda (tm-list constraints)
	     (send self :friction :tm-list tm-list
		   :dim-list
		   (apply #'append
			  (mapcar #'(lambda (cs range) (if cs range))
				  (mapcar #'caddr constraints)
				  force-range-list))
		   :contact-constraints
		   (remove-if #'(lambda (c) (null (caddr c))) constraints)
		   ))
	 contact-time-list contact-constraints-list)
	(mapcar #'(lambda (d) (cdr (assoc :JTf d))) descrete-dynamics-param)
	(mapcar #'(lambda (d) (scale-matrix -1 (cdr (assoc :JTf d))))
		descrete-dynamics-param)
	))
      '(1 0))
     :inequality-min-vector
     (apply
      #'concatenate
      (cons float-vector
	    (flatten
	     (list
	      (mapcar
	       #'(lambda (tm-list constraints)
		   (send self
			 :constant-vector
			 (apply #'concatenate
				(cons float-vector
				      (mapcar
				       #'cadr
				       (remove-if #'(lambda (c) (null (caddr c))) constraints)
				       )))
			 ;; :dim-list
			 ;; (apply #'append
			 ;; 	(mapcar #'(lambda (cs range) (if cs range))
			 ;; 		(mapcar #'caddr constraints)
			 ;; 		force-range-list))
			 :tm-list tm-list))
	       contact-time-list contact-constraints-list)
	      (mapcar #'(lambda (d) (cdr (assoc :Tmin d))) descrete-dynamics-param)
	      (mapcar #'(lambda (d) (scale -1 (cdr (assoc :Tmax d))))
		      descrete-dynamics-param)
	      ))))))
   (cond
    ((vectorp ret)
     (send self :convert-gain-vector-to-gain-matrix gain-vector)
     ;; (send self :put :descrete-force
     ;; 	   (mapcar
     ;; 	    #'(lambda (dv)
     ;; 		(send self :calc (cdr (assoc :time dv))))
     ;; 	    descrete-dynamics-param))
     ))
   (send self :put :optimize-result ret)
   (send self :put :optimize-trial-cnt (aref trial-cnt-feedback 0))
   (send self :put :optimize-success (eq (aref trial-cnt-feedback 0) trial-cnt))
   ret
   )
  (:gen-brlv-graph
   (&key (sep 1) (torque? t))
   (let* ((val (send self :calc-descrete-dynamics-value :sep sep))
	  graph
	  (tml (mapcar #'(lambda (a) (cdr (assoc :time a))) val)))
     (setq
      graph
      (mapcar
       #'(lambda (k)
	   (create-graph
	    (format nil "~A" k)
	    :size '(400 400)
	    :name-list
	    (append
	     (list "MAX")
	     (list "MIN")
	     (list "--- Force ---")
	     (list "Fx" "Fy" "Fz" "Mx" "My" "Mz")
	     (if torque? (list "--- Torque ---"))
	     (if torque?
		 (let* ((id -1))
		   (mapcar
		    #'(lambda (j) (format nil "Tau~A" (incf id)))
		    (send robot k :joint-list))))
	     )
	    :range (list (float-vector (apply #'min tml) -1.1)
			 (float-vector (apply #'max tml) 1.1))
	    :data-list
	    (append
	     (list
	      (list
	       (float-vector (cdr (assoc :time (car val))) 1.0)
	       (float-vector (cdr (assoc :time (car (last val)))) 1.0))
	      (list
	       (float-vector (cdr (assoc :time (car val))) -1.0)
	       (float-vector (cdr (assoc :time (car (last val)))) -1.0))
	      nil)
	     (let* ((pos (position k contact-name-list)) buf)
	       (dotimes (i 6)
		 (push (mapcar
			#'(lambda (v)
			    (float-vector
			     (cdr (assoc :time v))
			     (aref (nth pos (cdr (assoc :f/fmax-list v))) i)))
			val) buf))
	       (reverse buf))
	     (if torque? (list nil))
	     (if torque?
		 (let* (buf (jlist (send robot k :joint-list)))
		   (dolist (j jlist)
		     (push (mapcar
			    #'(lambda (v)
				(map cons
				     #'(lambda (l t/tmax)
					 (send l :put :t/tmax t/tmax))
				     (send-all (cdr (send robot :links)) :joint)
				     (cdr (assoc :t/tmax v)))
				(float-vector (cdr (assoc :time v)) (send j :get :t/tmax)))
			    val) buf))
		   (reverse buf)))
	     )))
       contact-name-list))
     (send-all graph :simple-draw-with-line)
     graph))
  (:gen-acc-graph
   nil
   (let* ((g
	   (create-graph
	    "Base Link Acceralation"
	    :name-list (list "x" "y" "z")
	    :data-list
	    (let* (x y z)
	      (mapcar
	       #'(lambda (trj)
		   (push (float-vector (send trj :get :time)
				       (* 1e-3 (aref (send trj :ddx) 0)))
			 x)
		   (push (float-vector (send trj :get :time)
				       (* 1e-3 (aref (send trj :ddx) 1)))
			 y)
		   (push (float-vector (send trj :get :time)
				       (* 1e-3 (aref (send trj :ddx) 2)))
			 z))
	       (send self :trajectory-elem-list))
	      (list x y z))
	    )))
     (send g :fit-draw)
     g))
  )



