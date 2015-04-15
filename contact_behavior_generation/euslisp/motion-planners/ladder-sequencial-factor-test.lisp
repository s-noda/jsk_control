;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsknts-collada)
(require "motion-planners/motion-planner.l")

(demo-climb-setup :kirin-ladder)

(setq *contact-states*
      (flatten
       (gen-primitive-contact-states
	*climb-obj* :draw? nil
	:grasp-contact-name '(:rarm :larm)
	:grasp-z-vector (list (float-vector 0 0 1) (float-vector 0 0 1))
	:grasp-extra-args
	(list
	 (list
	  :rotate-step 45
	  :rotate-id-list '(-2 -1 0 1 2))
	 (list
	  :rotate-step 45
	  :rotate-id-list '(2 1 0 -1 -2)))
	:place-extra-args
	(list
	 (list
	  :rotate-step 30
	  :rotate-id-list '(-1 0 1))
	 (list
	  :rotate-step 30
	  :rotate-id-list '(-1 0 1)))
	)))

(send-all *contact-states* :set-val 'gain '(1.1 1.0 10 10))
;; (setq *dcsf-min-dist-scale* 0.05)
(setq *dcsf-max-dist-scale* 1.3)
;; (setq *dcsf-use-heuristic* nil)

(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords) :both)
(let* ((rsd1
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg))))
       (rsd2
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg)))))
  ;; (send rsd1 :buf :remove-limb :rleg)
  ;; (send rsd2 :buf :remove-limb :lleg)
  (setq
   ret1
   (demo-motion-sequence2-with-timer
    :now-rsd rsd1
    ;; :ret (list rsd1 rsd2)
    :rms-loop-max 5 :ik-debug-view nil
    :loop-max
    '(lambda (&rest args) (> (aref (send *robot* :centroid) 2) 1300))
    :dump-graphviz? t
    :tag "sequencial_ladder"
    ))
  )

(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords) :both)
(let* ((rsd1
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg))))
       (rsd2
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg)))))
  (send rsd1 :buf :remove-limb :rleg)
  (send rsd2 :buf :remove-limb :lleg)
  (setq
   ret2
   (demo-motion-sequence2-with-timer
    :now-rsd rsd1
    :ret (list rsd1 rsd2)
    :rms-loop-max 5 :ik-debug-view nil
    :loop-max
    '(lambda (&rest args) (> (aref (send *robot* :centroid) 2) 1300))
    :cs-filter-func '(lambda (&rest args) (car args))
    :dump-graphviz? t
    :tag "non_sequencial_ladder_rms5"
    ))
  )

(mapcar
 #'(lambda (tag ret)
     (let* ((max 10))
       (dotimes (i (+ max 1))
	 (send (nth (round (* (/ i (+ max 1.0))
			      (min 50 (length (cdr ret))))) (cdr ret))
	       :draw :rest (list *climb-obj*))
	 (send *viewer* :viewsurface :write-to-jpg-file (format nil "img/~A~A.jpg" tag i)))))
 (list "seq" "nseq") (list ret1 ret2))


#|

(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords) :both)
(let* ((rsd1
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg))))
       (rsd2
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg)))))
  (send rsd1 :buf :remove-limb :rleg)
  (send rsd2 :buf :remove-limb :lleg)
  (setq
   ret
   (demo-motion-sequence2-with-timer
    :now-rsd rsd1
    :ret (list rsd1 rsd2)
    :rms-loop-max 30 :ik-debug-view nil
    :loop-max
    '(lambda (&rest args) (> (aref (send *robot* :centroid) 2) 1300))
    :cs-filter-func '(lambda (&rest args) (car args))
    :dump-graphviz? t
    :tag "non_sequencial_ladder_rms30"
    ))
  )



;; (rsd-play :rsd-list (reverse ret) :graph nil)
#|

(send *robot* :reset-pose)
(send *robot* :fix-leg-to-coords (make-coords) :both)
(let* ((rsd1
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg))))
       (rsd2
	(optimize-brli
	 :contact-states
	 (now-contact-state :limb-keys '(:rleg :lleg)))))
  (send rsd1 :buf :remove-limb :rleg)
  (send rsd2 :buf :remove-limb :lleg)
  (setq
   ret
   (demo-motion-sequence2-with-timer
    :now-rsd rsd1
    :ret (list rsd1 rsd2)
    :rms-loop-max 5 :ik-debug-view nil
    :loop-max
    '(lambda (&rest args) (> (aref (send *robot* :centroid) 2) 1300))
    :dump-graphviz? t
    :tag "non_sequencial_ladder"
    ))
  )
