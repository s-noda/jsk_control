;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(require "util/graph-sample.l")

(unless (boundp '*real-flag*)
  (defvar *real-flag* t))
(unless (boundp '*sim-flag*)
  (defvar *sim-flag* nil))
(unless (boundp '*node-name*)
  (defvar *node-name* "noda"))

(cond
 (*real-flag*
  (unix:system "source ~/.bashrc")
  (unix:system "rossethrp2017")
  (unix:system "~/prog/hrp2/scripts/Auditor/add-hrp-network.sh")
  (setup-real-for-hrp2)
;  (send *ci* :set-interpolation-method :hoffarbib "wor")
  )
 (*sim-flag*
  (require "package://rats/app/test-rats-plugins-and-server.l")
  (setup-connection))
 (t
  (setup-model-for-hrp2 :no-view t)
;  (defvar *ri* nil)
  (defvar *ci* nil))
 )
(defvar *real-robot*)
(defvar *robot* *hrp2*)

;()
;(setq *robot* (hrp2-simple "HRP2JSKNTS"))
;(send *irtviewer* :objects (list *robot*))
;(send *irtviewer* :look-all

(defun model2real
  (&key (robot *robot*) (sleep-time 5000))
  (send *ci* :angle-vector
	(send robot :angle-vector) sleep-time "wor")
  (if (find-method *ci* :hand-angle-vector)
      (send *ci* :hand-angle-vector
	    (apply #'concatenate
		   float-vector (send robot :arms :hand :angle-vector))
	    sleep-time))
  )

(defun real2model
  (&key (robot *robot*))
  (send robot :angle-vector (send *ci* :state :potentio-vector)))

(defun reset-hand
  (&key (robot *robot*))
  (send *ci* :set-ref-force #f(0 0 0) 3000)
  (send *ci* :set-ref-moment #f(0 0 0) 3000)
  (send *ci* :set-impedance-param :axis t)
  (send robot :angle-vector (send *ci* :state :potentio-vector))
  (send robot :arms :hand :standard-pose)
  (model2real))

(defun zmp-draw
  nil
  (setq *drawing* t)
  (sys:thread
   #'(lambda nil
       (while *drawing*
	 (send *robot* :angle-vector (send *ci* :state :potentio-vector))
	 (send *irtviewer* :draw-objects)
	 (send (send *ci* :state :zmp-vector)
	       :draw-on :width 3 :flush t :color #f(1 0 0))
	 (unix:sleep 2)))))
(defun draw-stop nil (setq *drawing* nil))


(defvar *graph-draw-data-func*
  #'(lambda nil
      (cons 0.7
	    (map cons #'(lambda (a b) (/ (abs a) b))
		 (send *ci* :state :torque-vector)
		 (send *robot* :max-torque-vector)))))
(defun graph-draw
  (&key
   (robot *robot*)
   (name-list (cons :error (send-all (send robot :joint-list) :name)))
   (range (list #f(0 -0.1) #f(500 1.1)))
   (get-data-func *graph-draw-data-func*))
  (let* ((name-list (mapcar #'(lambda (name) (format nil "~A" name)) name-list))
	 (graph (create-graph "graph-draw" :name-list name-list :range range)))
    (setq *graph-draw-data-func* get-data-func)
    (setq *drawing* t)
    (sys:thread
     #'(lambda nil
	 (while *drawing*
	   (add-data-to-graph (funcall *graph-draw-data-func*))
	   (unix:sleep 1))))))