#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://eus_caffe/euslisp/eus-log.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require "models/arrow-object.l")
(require "package://eus_caffe/sample/scalable_test/gen-solver.l")

(defclass vector-coords
  :super propertied-object
  :slots (coords mode v-buf c-buf))
(defmethod vector-coords
  (:init (c
          &key ((:mode m) :coords))
         (setq coords c) (setq mode m)
         (setq c-buf (make-coords))
         (setq v-buf (instantiate float-vector (send self :length))))
  (:worldcoords
   nil
   (let* ((c (cond ((functionp coords) (funcall coords))
                   (t coords))))
     (cond
      ((not (find-method c :worldcoords))
       (warning-message 1 "[coords2vector] invalid coords format ~A~%" coords)
       (throw :vector-coords-exception t)))
     (send c :worldcoords)))
  ;;
  (:vector
   (&key
    (org-coords (make-coords))
    (offset-coords)
    (ret v-buf) (i 0))
   (let* ((_c (send self :worldcoords))
	  (c (if offset-coords (send (send _c :copy-worldcoords)
				     :transform offset-coords :world) _c)))
     (cond
      ((eq mode :coords)
       (let* (pos rot)
         (setq pos (send (send org-coords :worldcoords)
                         :difference-position c))
         (setq rot (send (send org-coords :worldcoords)
                         :difference-rotation c))
         (dotimes (j 3) (setf (aref ret (+ i j)) (* 1e-3 (aref pos j))))
         (dotimes (j 3) (setf (aref ret (+ 3 i j)) (aref rot j)))))
      ((eq mode :pos)
       (let* (pos)
         (setq pos (send (send org-coords :worldcoords)
                         :difference-position
                         c))
         (dotimes (j 3) (setf (aref ret (+ i j)) (* 1e-3 (aref pos j))))
         ;; (setq ret (scale 1e-3 pos))
         ))
      (t
       (warning-message 1 "[coords2vector] invalid mode ~A~%" mode)
       (throw :vector-coords-exception t)))
     ret))
  (:length
   (&optional vec ret)
   (setq ret
         (cond ((eq mode :coords) 6) ((eq mode :pos) 3)
               (t (warning-message 1 "[coords-vector] mode invalid~A~%" mode)
                  (throw :vector-coords-exception t))))
   (cond
    ((and vec (not (eq ret (length vec))))
     (warning-message 1 "[coords-vector] invalid length~A~%" vec)
     (throw :vector-coords-exception t)))
   ret)
  (:coords-from-vector
   (vec &key (org-coords (make-coords)) c)
   ;; check length
   (send self :length vec)
   ;;
   (cond
    ((eq mode :coords)
     (setq c (make-coords
              :pos (scale 1e+3 (subseq vec 0 3))
              :rot (matrix-exponent (subseq vec 3 6))))
     (setq c (send (send org-coords :copy-worldcoords)
                   :transform c :local))
     c)
    ((eq mode :pos)
     (setq c (make-coords :pos (scale 1e+3 vec)))
     (setq c (send (send org-coords :copy-worldcoords)
                   :transform c :local))
     (setq c (make-coords :pos (send c :worldpos)))
     c))
   c)
  (:nomethod
   (&rest args)
   (send* (send self :worldcoords) (car args) (cdr args)))
  (:transform-copy
   (c) (instance vector-coords :init (send (send self :copy-worldcoords) :transform c :world)))
  )

(defvar *robot* (hrp2jsknt))
(defvar *joint-list1* (append (butlast (send *robot* :rarm :joint-list))
                              (send *robot* :torso :joint-list)))
;; (setq *joint-list1* (list (send *robot* :rarm :elbow-p) (send *robot* :rarm :shoulder-p)))
(defvar *joint-list2* (append (butlast (send *robot* :rarm :joint-list))
                              (butlast (send *robot* :larm :joint-list))
                              (send *robot* :torso :joint-list)))
(defvar *joint-list3* (append (butlast (send *robot* :rarm :joint-list))
                              (butlast (send *robot* :larm :joint-list))
                              (butlast (send *robot* :rleg :joint-list))
                              (butlast (send *robot* :lleg :joint-list))
                              (send *robot* :torso :joint-list)))

(defvar *ef-coords* (list (instance vector-coords :init (send *robot* :rarm :end-coords))))
;; (defvar *ef-coords-length* (list 6))
(defvar *arrow* (mapcar '(lambda (ec) (arrow)) *ef-coords*))
(defvar *root-link* (car (send *robot* :links)))
(defvar *root-link-fix-coords-func* nil)

;;(defvar *learn-mode* :rarm-larm-rleg-lleg)
(defvar *learn-mode*
  (cond
   ((read-line (piped-fork "head predict_ik_net.prototxt -n 1 | grep 9s") nil) :rarm)
   ((read-line (piped-fork "head predict_ik_net.prototxt -n 1 | grep 16s") nil) :rarm-larm)
   ((read-line (piped-fork "head predict_ik_net.prototxt -n 1 | grep 28s") nil) :rarm-larm-rleg-lleg)
   (t :rarm-larm-rleg-lleg)))
(cond
 ((eq *learn-mode* :rarm-larm)
  (setq *joint-list1* *joint-list2*)
  (setq *ef-coords*
        (mapcar
         '(lambda (c mode) (instance vector-coords :init c :mode mode))
         (list (send *robot* :larm :end-coords)
               (send *robot* :rarm :end-coords))
         (list :coords :coords :coords :pos))))
 ((eq *learn-mode* :rarm-larm-rleg-lleg)
  (setq *joint-list1* *joint-list3*)
  (setq *ef-coords*
        (mapcar
         '(lambda (c mode) (instance vector-coords :init c :mode mode))
         (list (send *robot* :larm :end-coords)
               (send *robot* :rarm :end-coords)
               (send *robot* :lleg :end-coords)
               ;; '(lambda (&rest args)
               ;;    (make-coords :pos (send *robot* :centroid)))
               )
         (list :coords :coords :coords :pos)))
  (setq *root-link* (car (last (butlast (send *robot* :rleg :links)))))
  (setq *root-link-fix-coords-func*
        '(lambda nil (send *root-link* :worldcoords)
           (send *robot* :fix-leg-to-coords (make-coords) :rleg)))
  ))

(setq *arrow* (mapcar '(lambda (ec) (arrow)) *ef-coords*))
(send-all (send *robot* :links) :set-val 'analysis-level :coords)
(objects (flatten (list (car (send *robot* :links))
                        (reverse (send-all *joint-list3* :child-link))
                        *arrow*)))

(defvar *sample-points* ;;96)
  (let* ((org
          (min
           (round 1e+6)
           (expt (max
                  (min (round (expt 1e+6 (/ 1.0 (length *joint-list1*))))
                       4) 2) (length *joint-list1*))))
         (sep 32))
    (if (not (zerop (mod org sep)))
        (max 1 (round (* (ceiling (/ (* 1.0 org) sep)) sep)))
      org)))

(defvar *angle-vector-map*)
(defvar *ef-coords-map*)

(defun get-vector-from-ef-coords
  (&key
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (offset-coords (make-list (length ef-coords)))
   (org-coords (send root-link :worldcoords))
   )
  (apply 'concatenate
         (cons float-vector
               (mapcar #'(lambda (efc off)
			   (send efc :vector :org-coords org-coords :offset-coords off))
		       ef-coords offset-coords))))

(defun get-ef-coords-from-vector
  (&key
   (ef-coords *ef-coords*)
   (vector (get-vector-from-ef-coords))
   (root-link *root-link*)
   (org-coords (send root-link :worldcoords))
   (i 0) len
   ret)
  (dolist (efc ef-coords)
    (setq len (send efc :length))
    (push (send efc :coords-from-vector (subseq vector i (+ i len))
                :org-coords org-coords) ret)
    (setq i (+ i len)))
  (reverse ret))

(defun shuffle-random-vector-map
  (&key
   (sample-points *sample-points*)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   ;; (ef-coords-length *ef-coords-length*)
   (av-map *angle-vector-map*)
   (cc-map *ef-coords-map*)
   ;;
   (len (length (get-vector-from-ef-coords))) ;;(apply '+ ef-coords-length))
   )
  (labels
      ((swap (a i j)
             (let* ((c (aref a i)))
               (setf (aref a i) (aref a j))
               (setf (aref a j) c))))
    (dotimes (i sample-points)
      (let* ((r (random sample-points)))
        (dotimes (j (length joint-list))
          (swap av-map
                (+ j (* (length joint-list) i))
                (+ j (* (length joint-list) r))))
        (dotimes (j len)
          (swap cc-map (+ j (* i len)) (+ j (* r len))))))))

(defun gen-ik-learning-data
  (&key (sample-points *sample-points*)
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
        ;; (ef-coords-length *ef-coords-length*)
        (sep (floor (expt sample-points (/ 1.0 (length joint-list)))))
        (root-link *root-link*) ;;(car (send *robot* :links)))
        (i 0)
        (draw? nil)
        (surfix "")
        (len (length (get-vector-from-ef-coords)))
        )
  ;;
  (cond
   ((< sep 2)
    (warning-message 3 "large search space ~A^~A~% -> random DB~%"
                     (length joint-list) sep)
    (gen-random-ik-learning-data
     :sample-points sample-points
     :joint-list joint-list
     :ef-coords ef-coords
     :sep sep
     :root-link root-link
     :i i
     :draw? draw?
     :surfix surfix))
   (t
    ;;
    (setq *sample-points* sample-points)
    (cond
     ((not (eq (length *angle-vector-map*)
               (* (length joint-list) sample-points)))
      (warning-message 1 "angle-vector-map alloc~%")
      (setq *angle-vector-map*
            (instantiate float-vector
                         (* (length joint-list) sample-points)))))
    (cond
     ((not (eq (length *ef-coords-map*)
               (* len sample-points)))
      (warning-message 1 "ef-coords-map alloc~%")
      (setq *ef-coords-map*
            (instantiate float-vector (* len sample-points)))))
    (labels ((itter
              (jlist)
              (cond
               ((>= i sample-points)
                'nop)
               ((null jlist)
                (let* (val)
                  (send-all (send-all joint-list :child-link) :worldcoords)
                  (send-all ef-coords :worldcoords)
                  (cond
                   (draw?
                    (send *viewer* :draw-objects :flush nil)
                    (send-all ef-coords :draw-on :flush nil
                              :color (float-vector 1 0 0) :size 100)
                    (send *viewer* :viewsurface :flush)))
                  (dotimes (j (length joint-list))
                    (setf (aref *angle-vector-map* (+ j (* i (length joint-list))))
                          (deg2rad (send (nth j joint-list) :joint-angle))))
                  ;;
                  (setq val (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
                  (dotimes (j len)
                    (setf (aref *ef-coords-map* (+ j (* i len))) (aref val j)))
                  (setq i (+ i 1))
                  (if (and (> sample-points 100)
                           (zerop (mod i (round (/ sample-points 100)))))
                      (warning-message 2 "~A%~%" (/ i (round (/ sample-points 100)))))
                  ))
               (t
                (dotimes (j sep)
                  (send (car jlist) :joint-angle
                        (-
                         (+ (send (car jlist) :min-angle)
                            (* (/ j (- sep 1.0))
                               (- (send (car jlist) :max-angle)
                                  (send (car jlist) :min-angle))))
                         (send (car jlist) :joint-angle))
                        :relative t)
                  (itter (cdr jlist))))
               )))
      (warning-message 1 "generate sep DB ~A^~A~%" (length joint-list) sep)
      (itter joint-list)
      ;; fill random
      (warning-message 1 "generate random DB ~A~%" (- sample-points i))
      (gen-random-ik-learning-data
       :sample-points sample-points
       :joint-list joint-list
       :ef-coords ef-coords
       :sep sep
       :root-link root-link
       :i i
       :draw? draw?
       :dump? nil)
      ;;
      (warning-message 1 "shuffling DB~%")
      (shuffle-random-vector-map)
      (warning-message 1 "write to file .~A~%" surfix)
      (dump-ik-learning-data :surfix surfix)
      ))))

(defun gen-random-angle-vector
  (&optional (joint-list *joint-list1*))
  (dolist (j joint-list)
    (send j :joint-angle
          (-
           (+ (send j :min-angle)
              (* (random 1.0)
                 (- (send j :max-angle) (send j :min-angle))))
           (send j :joint-angle))
          :relative t)))

(defun gen-random-ik-learning-data
  (&key (sample-points *sample-points*)
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
        (sep (round (expt sample-points (/ 1.0 (length joint-list)))))
        (root-link *root-link*)
        ;; (root-link (car (send *robot* :links)))
        (i 0)
        (draw? nil)
        (dump? t)
        (surfix (if dump?
                    (format nil ".~A" (unix-command-line "basename `pwd`"))))
        (gen-random-angle-vector-func 'gen-random-angle-vector)
        ;;
        (len (length (get-vector-from-ef-coords)))
        &allow-other-keys
        )
  (setq *sample-points* sample-points)
  (cond
   ((not (eq (length *angle-vector-map*)
             (* (length joint-list) sample-points)))
    (warning-message 1 "angle-vector-map alloc~%")
    (setq *angle-vector-map*
          (instantiate float-vector
                       (* (length joint-list) sample-points)))))
  (cond
   ((not (eq (length *ef-coords-map*)
             (* len sample-points)))
    (warning-message 1 "ef-coords-map alloc~%")
    (setq *ef-coords-map*
          (instantiate float-vector (* len sample-points)))))
  ;;
  (let* (val)
    (dotimes (_i sample-points)
      (if (>= i sample-points) (return-from nil nil))
      (if (and (> sample-points 100)
               (zerop (mod _i (round (/ sample-points 100)))))
          (warning-message 3 "~A%~%" (/ _i (round (/ sample-points 100)))))
      ;; (dolist (j joint-list)
      ;;   (send j :joint-angle
      ;;         (-
      ;;          (+ (send j :min-angle)
      ;;     (* (random 1.0)
      ;;        (- (send j :max-angle) (send j :min-angle))))
      ;;          (send j :joint-angle))
      ;;         :relative t)
      ;;   )
      (if (functionp gen-random-angle-vector-func)
          (funcall gen-random-angle-vector-func joint-list)
        (warning-message 1 "invalid gen-random-angle-vector-func~%"))
      (send-all (send-all joint-list :child-link) :worldcoords)
      (send-all ef-coords :worldcoords)
      (dotimes (j (length joint-list))
        (setf (aref *angle-vector-map* (+ j (* i (length joint-list))))
              (deg2rad (send (nth j joint-list) :joint-angle))))
      ;;
      (setq val (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
      (dotimes (j len)
        (setf (aref *ef-coords-map* (+ j (* i len))) (aref val j)))
      ;;
      (incf i)
      )
    (if dump? (dump-ik-learning-data :surfix surfix))
    ))

;; gen-random-ik-learning-data-with-thre :sample-points 3 :thre 0.3 :max-time 5
;; check-test-data 0
;; ik-random-test-min :revert? t :random-angle (send-all *joint-list1* :joint-angle)
(defun gen-random-ik-learning-data-with-thre
  (&rest args
         &key
         (draw? t)
         (thre 0.3)
         (start-time (car (unix::gettimeofday)))
         (max-time (* 15 60))
         (_max-time (+ start-time max-time))
         (time-over-flag nil)
         (overthre-cnt 0)
         (nop-cnt 1)
         ret
         &allow-other-keys)
  (smart-initialize-net)
  (cpplog::change-output-stream "/dev/null")
  (setq
   ret
   (apply 'gen-random-ik-learning-data
          :gen-random-angle-vector-func
          #'(lambda (joint-list)
              (gen-random-angle-vector joint-list)
              (cond
               (time-over-flag
                (incf nop-cnt);; (print 'nop)
                )
               ((< _max-time (car (unix::gettimeofday)))
                (setq time-over-flag t)
                (warning-message 1 "time exceeded ~A~%" max-time))
               (t
                (let* ((dif))
                  (setq dif (apply 'ik-random-test-min
                                   ;; :redundancy-step 0.1
                                   :step-max 10
                                   :revert? t :initialize? nil
                                   :draw? draw? args))
                  (while (and
                          (not time-over-flag)
                          (not (setq time-over-flag (< _max-time (car (unix::gettimeofday)))))
                          (< (cdr (assoc :evaluation dif)) thre))
                    (if draw? (warning-message 6 "random av rejected: ~A < ~A~%" dif thre))
                    ;; (gen-random-angle-vector joint-list)
                    (setq dif (apply 'ik-random-test-min
                                     ;; :redundancy-step 0.1
                                     :step-max 10
                                     :revert? t :initialize? nil
                                     :draw? draw? args))
                    )
                  (cond
                   ((not time-over-flag) ;; (>= (cdr (assoc :state-diff-norm dif)) thre)
                    (incf overthre-cnt)
                    (if draw?
                        (warning-message 5 "random av accepted~A: ~A < ~A~%" overthre-cnt dif thre)))
                   ;; (t (incf nop-cnt))
                   )
                  ))))
          args))
  (warning-message 1 "[gen-random-ik-learning-data-with-thre] thre: ~A, over: ~A, nop: ~A~%"
                   thre overthre-cnt nop-cnt)
  (cpplog::change-output-stream "")
  ret
  )

(defun load-ik-learning-data
  (&key (root "teacher") (surfix ""))
  (setq *angle-vector-map* (cpplog::read-vector (format nil "~A/angle-vector-map.dat~A" root surfix)))
  (setq *ef-coords-map* (cpplog::read-vector (format nil "~A/ef-coords-map.dat~A" root surfix)))
  (length *angle-vector-map*))

(defun dump-ik-learning-data
  (&key (root "teacher") (surfix ""))
  (unix::system (format nil "mkdir -p ~A" root))
  (cpplog::write-vector (format nil "~A/angle-vector-map.dat~A" root surfix)
                        *angle-vector-map*)
  (cpplog::write-vector (format nil "~A/ef-coords-map.dat~A" root surfix)
                        *ef-coords-map*))

(defun ik-learn
  (&key (solver "ik_solver.prototxt")
        (solverstate "")
        &allow-other-keys)
  (cond
   ((or (zerop (length *ef-coords-map*))
        (zerop (length *angle-vector-map*)))
    (load-ik-learning-data)
    (setq *sample-points* ;;96)
          (/ (length *angle-vector-map*)
             (length *joint-list1*)))
    ))
  ;; (caffe::create-solver :solver solver :solverstate "")
  ;; (caffe::reset-memory-layer :name "input"
  ;;      :size (length *angle-vector-map*)
  ;;      :data *angle-vector-map*)
  ;; (caffe::reset-memory-layer :name "target"
  ;;      :size (length *ef-coords-map*)
  ;;      :data *ef-coords-map*)
  (cond
   ((or (zerop (length *ef-coords-map*))
        (zerop (length *angle-vector-map*)))
    (warning-message 1 "zero training DB~%"))
   (t
    (caffe::learn
     :solver solver
     :solverstate solverstate
     :size *sample-points*
     :ddata *ef-coords-map*
     :idata *angle-vector-map*
     ))))

(defun unix-command-line
  (cmd)
  (let* ((p (piped-fork cmd))
         (ret (read-line p nil)))
    (close p)
    ret))

(defun gen-snapshot-prefix
  (solver)
  (unix-command-line
   (concatenate string
                (format nil "cat ~A" solver)
                " | "
                "grep \"snapshot_prefix\""
                " | "
                "sed -e \"s/^snapshot_prefix: \\\"\\\(.\\\+\\\)\\\"$/\\\1/g\""
                )))

(defun link-best-settings
  (solver
   &key
   (best-path solver)
   (best-snap (gen-snapshot-prefix best-path))
   (caffe-snap
    (unix-command-line
     (format nil "ls ~A*caffemodel -t | head -n 1" best-snap)))
   (solver-snap
    (unix-command-line
     (format nil "ls ~A*solverstate -t | head -n 1" best-snap)))
   (solver-lnk
    (format nil "ln -snf ~A ik_solver.best.prototxt" best-path))
   (caffe-lnk
    (format nil "ln -snf ~A ik.best.caffemodel" caffe-snap))
   (solverstate-lnk
    (format nil "ln -snf ~A ik.best.solverstate" solver-snap)))
  ;;
  (warning-message 3 (format nil "~A~%" solver-lnk))
  (if (string-equal solver "ik_solver.best.prototxt")
      (print 'skip)
    (unix::system solver-lnk))
  (warning-message 3 (format nil "~A~%" caffe-lnk))
  (unix::system caffe-lnk)
  (warning-message 3 (format nil "~A~%" solverstate-lnk))
  (unix::system solverstate-lnk)
  )

(defun ik-learn-with-all-solver
  (&rest args)
  (let* ((ret
          (sort
           (mapcar
            #'(lambda (solver)
                (cons
                 solver
                 (apply 'ik-learn :solver solver args)))
            (remove-if-not
             #'(lambda (str) (and (substringp "ik_solver" str)
                                  (substringp "prototxt" str)))
             (directory)))
           #'(lambda (a b) (< (cdr a) (cdr b)))))
         )
    (link-best-settings (car (car ret)))
    ret))

(defun ik-learn-best
  (&rest args &key (thre) &allow-other-keys)
  (let* (config state)
    ;; detect solver config
    (setq config
          (or (unix-command-line "ls ik_solver.best.prototxt")
              (unix-command-line "ls -t ik_solver*prototxt")))
    ;;
    ;; detect solver state
    (setq state
          (or (unix-command-line "ls ik.best.solverstate")
              (unix-command-line "ls -t ik*solverstate")
              ""))
    ;;
    (warning-message 3 "config: ~A, state: ~A~%" config state)
    ;;
    (cond
     ((not (and config state))
      (warning-message 1 "empty setting files~%")
      (throw :settings-not-found nil)))
    (warning-message 3 "regenrate random traning DB~%")
    (if (numberp thre)
        (gen-random-ik-learning-data-with-thre :draw? nil :thre thre)
      (gen-random-ik-learning-data))
    (warning-message 3 "update solver~%")
    (let* ((p (apply 'update-solver :input "ik_solver.best.prototxt" args)))
      (unix::system (format nil "ln -snf ~A ik_solver.best.prototxt" p)))
    (warning-message 3 "start learning ... ~%")
    (apply 'ik-learn :solver config :solverstate state args)
    ;; (ik-learn :solver solver :solverstate state)
    (warning-message 3 "done~%")
    (link-best-settings "ik_solver.best.prototxt")
    (warning-message 3 "evaluate network~%")
    (cpplog::change-output-stream "/dev/null")
    (let* ((ret (ik-random-test-min-average :draw? nil)))
      (cpplog::change-output-stream "")
      ret)
    ))

(defun ik-learn-best-loop
  (&rest args
         &key
         (lp 10)
         ;;
         (base_lr-gain 1.0)
         (eq-scale-thre 0.01)
         (base_lr-gain-update-scale 1.1)
         (vain-cnt 0)
         (vain-cnt-max 5) prev now
         (thre
          (progn
            (cpplog::change-output-stream "/dev/null")
            (let* ((ret (ik-random-test-min-average :draw? nil)))
              (cpplog::change-output-stream "")
              ret)))
         ;;
         &allow-other-keys)
  (let* ((ret))
    (push thre ret)
    (dotimes (i lp)
      (unix::gc)
      (push (apply 'ik-learn-best :base_lr-gain base_lr-gain :thre (car ret) args) ret)
      (setq prev now)
      (setq now (car ret))
      ;; update base_lr-gain
      (cond
       ((or (not (numberp now)) (not (numberp prev)))
        'skip)
       ((eps= now prev (* now eq-scale-thre))
        ;;(setq base_lr-gain (* base_lr-gain base_lr-gain-update-scale))
        (setq base_lr-gain base_lr-gain-update-scale)
        (incf vain-cnt)
        )
       ((eps< now prev (* now eq-scale-thre))
        (setq base_lr-gain 1.0)
        (decf vain-cnt)
        'nop
        )
       ((eps> now prev (* now eq-scale-thre))
        ;; (setq base_lr-gain (/ base_lr-gain base_lr-gain-update-scale))
        (setq base_lr-gain (/ 1.0 base_lr-gain-update-scale))
        (incf vain-cnt)
        ))
      (warning-message
       3 (format nil "~A~%" (unix-command-line "ls -l ik.best.caffemodel | sed -e \"s/ /\\\n/g\" | tail -1")))
      (warning-message
       3 (format nil "~A~%" (unix-command-line "ls -l ik.best.solverstate | sed -e \"s/ /\\\n/g\" | tail -1")))
      (warning-message 3 "update base_lr-gain: ~A, vain: ~A, now: ~A~%"
                       base_lr-gain vain-cnt now)
      ;; convergence
      (cond
       ((>= vain-cnt vain-cnt-max)
        (warning-message 3 "loop exceeded, ~A > ~A~%"
                         vain-cnt vain-cnt-max)
        (return-from nil nil)))
      )
    ;; (setq *ret* ret)
    ret))

(defun check-test-data
  (&optional
   (i (random *sample-points*)) ;;0)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (len (length (get-vector-from-ef-coords)))
   (input (instantiate float-vector len))
   (output (instantiate float-vector (length joint-list)))
   (root-link-fix-coords-func *root-link-fix-coords-func*)
   c c-buf
   )
  (dotimes (j (length joint-list))
    (setf (aref output j)
          (aref *angle-vector-map* (+ (* i (length joint-list)) j))))
  (map cons #'(lambda (j a) (send j :joint-angle
                                  (- (rad2deg a) (send j :joint-angle))
                                  :relative t))
       joint-list output)
  (send-all (send-all joint-list :child-link) :worldcoords)
  (send-all ef-coords :worldcoords)
  ;;
  (if (functionp root-link-fix-coords-func)
      (funcall root-link-fix-coords-func))
  ;;
  (setq c-buf
        (get-ef-coords-from-vector
         :ef-coords ef-coords
         :vector (subseq *ef-coords-map* (* i len) (* (+ i 1) len))
         :root-link root-link))
  (mapcar
   '(lambda (ar c)
      (send ar :newcoords (send c :copy-worldcoords)))
   *arrow* c-buf)
  (send *viewer* :draw-objects :flush nil)
  (send *viewer* :viewsurface :flush)
  (print (apply 'concatenate (cons float-vector (flatten (mapcar '(lambda (ec c) (list (send c :difference-position ec) (send c :difference-rotation ec))) (send-all ef-coords :worldcoords) c-buf)))))
  )

(defun smart-initialize-net
  nil
  (let* ((caffemodel (unix-command-line "ls -t | grep -e \"caffemodel$\" | head -n 1")))
    (warning-message 6 "load caffe model ~A~%" caffemodel)
    (caffe::gen-test-net :netproto "predict_ik_net.prototxt"
                         :caffemodel caffemodel))
  )

(defun ik-random-test
  (&key (calc-forward 'caffe::calc-forward-double) ;;'caffe::memory-calc-forward)
        (draw? t) (revert? nil) (update-arrow-coords? t)
        tm
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
        (root-link *root-link*)
        (root-link-fix-coords-func *root-link-fix-coords-func*)
        (random-angle
         (mapcar '(lambda (j) (send j :joint-angle
                                    (-
                                     (+ (send j :min-angle)
                                        (* (random 1.0)
                                           (- (send j :max-angle)
                                              (send j :min-angle))))
                                     (send j :joint-angle))
                                    :relative t))
                 joint-list))
        (update-coords
         (progn (if (functionp root-link-fix-coords-func)
                    (funcall root-link-fix-coords-func))
                (send-all (send-all joint-list :child-link) :worldcoords)
                (send-all ef-coords :worldcoords)))
        (c (send-all (send-all ef-coords :worldcoords) :copy-worldcoords))
        (b (send root-link :copy-worldcoords))
        (av (instantiate float-vector (length joint-list)))
        (input (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
        (initialize? (smart-initialize-net))
        (redundancy;; (float-vector 0.5))
         ;;(fill
         (instantiate float-vector (- (caffe::get-blob-count "input") (length input)))
         ;;0.5)
         )
	;;
	(after-callback-func)
	(evaluation-callback-func
	 '(lambda (target-state target-task state task)
	    (sqrt
	     (/ (norm2 (map float-vector '(lambda (a b) (deg2rad (- a b)))
			    target-state (send-all state :joint-angle)))
		(length target-state)))))
        &allow-other-keys
        )
  (setq input (concatenate float-vector input redundancy))
  (setq
   tm
   (bench2
    (apply calc-forward
           (list :isize (length input) :input input
                 :osize (length av) :output av))
    ))
  (map cons #'(lambda (j a) (send j :joint-angle
                                  (- (rad2deg a) (send j :joint-angle))
                                  :relative t))
       joint-list av)
  (if (functionp root-link-fix-coords-func)
      (funcall root-link-fix-coords-func))
  (cond
   (update-arrow-coords?
    (mapcar
     '(lambda (ar c)
	(send ar :newcoords (send c :copy-worldcoords)))
     *arrow* c)))
  (cond
   ((or (not (functionp after-callback-func))
	(funcall after-callback-func av c ef-coords))
    (cond
     (draw?
      (send *viewer* :draw-objects :flush nil)
      (send *viewer* :viewsurface :flush)))
    (list
     (cons :time tm)
     (cons :state (map float-vector 'deg2rad (send-all joint-list :joint-angle)))
     (cons :redundancy (copy-seq redundancy))
     (cons :task-diff
	   (apply 'concatenate (cons float-vector (flatten (mapcar '(lambda (ec c) (list (scale 1e-3 (send c :difference-position ec)) (send c :difference-rotation ec))) (send-all ef-coords :worldcoords) c))))
	   )
     (cons :state-diff
	   (map float-vector '(lambda (a b) (deg2rad (- a b)))
		random-angle (send-all joint-list :joint-angle)))
     (cons :evaluation
	   (if (functionp evaluation-callback-func)
	       (funcall evaluation-callback-func random-angle c joint-list ef-coords)))
     (cons :revert?
	   (cond
	    (revert?
	     (mapcar '(lambda (j av) (send j :joint-angle (- av (send j :joint-angle))
					   :relative t))
		     joint-list random-angle)
	     (if (functionp root-link-fix-coords-func)
		 (funcall root-link-fix-coords-func)))))
     ))))

(defun ik-random-test-min
  (&rest
   args
   &key
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (root-link-fix-coords-func *root-link-fix-coords-func*)
   (random-angle
    (mapcar '(lambda (j) (send j :joint-angle
                               (-
                                (+ (send j :min-angle)
                                   (* (random 1.0)
                                      (- (send j :max-angle)
                                         (send j :min-angle))))
                                (send j :joint-angle))
                               :relative t))
            joint-list))
   (update-coords
    (progn (if (functionp root-link-fix-coords-func)
                    (funcall root-link-fix-coords-func))
           (send-all (send-all joint-list :child-link) :worldcoords)
           (send-all ef-coords :worldcoords)))
   (c (send-all (send-all ef-coords :worldcoords) :copy-worldcoords))
   (b (send root-link :copy-worldcoords))
   (av (instantiate float-vector (length joint-list)))
   (input (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
   (initialize? (smart-initialize-net))
   (redundancy
    ;;(fill
    (instantiate float-vector (- (caffe::get-blob-count "input") (length input)))
    ;;0.0)
    )
   (step-max 300)
   (redundancy-step
    (/ 1.0 (expt step-max (/ 1.0 (length redundancy)))))
   (redundancy-step-cnt
    (max 1 (floor (/ 1.0 redundancy-step))))
   (revert? nil) (draw? t)
   tmp buf ret (cnt 0) (update-arrow-coords? t)
   ;;
   (stop-func)
   (evaluation-func
    '(lambda (val soa)
       (< (cdr (assoc :evaluation val))
	  (cdr (assoc :evaluation soa)))))
   &allow-other-keys
   )
  (labels ((itter
            (depth &optional (depth0 depth) (prev 0))
            (cond
             ((and (zerop depth) (eq prev 0))
              (let* ((jmax (max 0 (- step-max (expt redundancy-step-cnt depth0)))))
                ;; (warning-message 1 "~A~%" jmax)
                (dotimes (j jmax)
                  (setf (aref redundancy (random depth0)) (random 1.0))
                  (itter depth depth0 -1))))
             ((zerop depth)
              ;; (print redundancy)
              ;; (print (incf cnt))
              (setq tmp (apply 'ik-random-test
                               :random-angle random-angle :update-coords update-coords
                               :c c :b b :av av :input input :redundancy redundancy
			       :update-arrow-coords? update-arrow-coords?
                               :initialize? nil :revert? revert? args))
	      (if update-arrow-coords? (setq update-arrow-coords? nil))
              ;;(push tmp buf)
              (cond
	       ((not tmp) 'nop)
               ((not ret) (setq ret tmp))
               ((and (functionp evaluation-func)
		     (funcall evaluation-func tmp ret))
		(setq ret tmp)))
	      ;;
	      (if (and (functionp stop-func)
		       (funcall stop-func ret)) (throw :stop-func-end nil))
	      )
             (t
              (dotimes (j redundancy-step-cnt)
                (setf (aref redundancy (- depth 1)) (* j redundancy-step))
                (itter (- depth 1) depth0 (+ prev (- redundancy-step-cnt j 1))))))))
    (catch :stop-func-end (itter (length redundancy))))
  ;; (dotimes (i (length redundancy))
  ;;   (dotimes (j (round (/ 1.0 redundancy-step)))
  ;;     (setf (aref redundancy i) (* j redundancy-step))
  ;;     (push (apply 'ik-random-test :random-angle random-angle :update-coords update-coords :c c :b b :av av :input input :redundancy redundancy :initialize? nil args) buf)))
  ;; (setq *ret* buf)
  ;; (setq ret (car (sort buf #'(lambda (a b) (< (cdr (assoc :state-diff-norm a))
  ;; (cdr (assoc :state-diff-norm b)))))))
  (cond
   (revert?
    (mapcar '(lambda (j av) (send j :joint-angle (- av (send j :joint-angle)) :relative t))
	    joint-list random-angle)
    (if (functionp root-link-fix-coords-func)
	(funcall root-link-fix-coords-func)))
   (ret
    (map cons '(lambda (j av) (send j :joint-angle (- (rad2deg av) (send j :joint-angle))
				    :relative t))
	 joint-list (cdr (assoc :state ret)))
    (if (functionp root-link-fix-coords-func)
	(funcall root-link-fix-coords-func))))
  (if (and draw? (or revert? ret)) (send *viewer* :draw-objects))
  ret
  )

(defun ik-random-test-min-average
  (&rest args &key (lp 100) (step-max 300) (variance (float-vector 0)) &allow-other-keys)
  (let* ((ret) avr var)
    (smart-initialize-net)
    (dotimes (i lp)
      (push (apply 'ik-random-test-min :initialize? nil :step-max step-max args) ret))
    (setq ret (remove nil ret))
    (if (not (eq lp (length ret)))
	(warning-message 1 "[ik-random-test-min-average] warn: ik-random-test-min sometimes redused? ~A/~A~%" (length ret) lp))
    ;; (setq *ret* ret)
    (setq avr (/ (apply '+ (mapcar '(lambda (d) (cdr (assoc :evaluation d))) ret))
		(length ret)))
    (setq var (mapcar #'(lambda (d) (- avr (cdr (assoc :evaluation d)))) ret))
    (setq var (apply '+ (mapcar '(lambda (d) (* d d)) var)))
    (setq var (sqrt (/ var (length ret))))
    (setf (aref variance 0) var)
    avr))

#|

(smart-initialize-net)
(cpplog::change-output-stream "/dev/null")

(defvar *collision-check-pair*
  (let* ((init (progn (send *robot* :init-pose)
		      (send-all (send *robot* :links) :worldcoords)))
	 (pair (send *robot* :collision-check-pairs))
	 bl1 bl2)
    (setq bl1 (send *robot* :self-collision-check))
    ;;
    (send *robot* :reset-manip-pose)
    (send-all (send *robot* :links) :worldcoords)
    (setq bl2 (send *robot* :self-collision-check))
    (setq pair
	  (remove-if
	   #'(lambda (ll)
	       (find-if #'(lambda (bl)
			    (or (and (eq (car ll) (car bl))
				     (eq (cdr ll) (cdr bl)))
				(and (eq (car ll) (cdr bl))
				     (eq (cdr ll) (car bl)))))
			(append bl1 bl2)))
	   pair))
    pair))


(let* ((ik-fail-cnt 0)
       (cl-fail-cnt 0)
       (joint-list *joint-list1*)
       (random-angle
	(progn
	  (while (progn
		   (mapcar '(lambda (j) (send j :joint-angle
					      (-
					       (+ (send j :min-angle)
						  (* (random 1.0)
						     (- (send j :max-angle)
							(send j :min-angle))))
					       (send j :joint-angle))
					      :relative t))
			   joint-list)
		   (send *robot* :self-collision-check :pairs *collision-check-pair*)))
	  (send-all joint-list :joint-angle)))
       (ret
	(bench
	 (ik-random-test-min
	  :initialize? nil
	  :random-angle random-angle
	  :step-max 30
	  :after-callback-func
	  #'(lambda (av c ef-coords)
	      (let* ((ret
		      (send *robot* :fullbody-inverse-kinematics
			    (cons (make-coords) (send-all c :copy-worldcoords))
			    :move-target
			    (cons (send *robot* :rleg :end-coords)
				  (send-all *ef-coords* :get-val 'coords))
			    :link-list
			    (mapcar
			     '(lambda (mt) (send *robot* :link-list (send mt :parent)))
			     (cons (send *robot* :rleg :end-coords)
				   (send-all *ef-coords* :get-val 'coords)))
			    :debug-view nil
			    :target-centroid-pos nil
			    :stop 8 :min-loop 0
			    :min (fill (instantiate float-vector 6) -1000)
			    :max (fill (instantiate float-vector 6) +1000)
			    :root-link-virtual-joint-weight
			    (fill (instantiate float-vector 6) 1)
			    :warnp nil
			    :revert-if-fail t))
		     (col))
		(cond
		 ((not ret) (incf ik-fail-cnt) nil)
		 ((setq col (send *robot* :self-collision-check
				  :pairs *collision-check-pair*))
		  (incf cl-fail-cnt) nil)
		 (t (and ret (not col))))))
	  :evaluation-callback-func
	  '(lambda (target-state target-task state task)
	     (norm (send *robot* :calc-torque)))
	  ;; :stop-func '(lambda (ret) ret)
	  :draw? t
	  )
	 )))
  (send *viewer* :draw-objects)
  (format t "ik: ~A, col:~A~%" ik-fail-cnt cl-fail-cnt)
  ret
  )

;; (progn (smart-initialize-net) (cpplog::change-output-stream "/dev/null") (let* ((var (float-vector 0)) avr (log (open "ik_random_test_min.log" :direction :output))) (setq avr (ik-random-test-min-average :draw? nil :variance var)) (format log "~A ~A~%" avr (aref var 0)) (close log) (unix::system "cat ik_random_test_min.log")))