#!/usr/bin/env roseus

(ros::roseus "hrp2007_seating")

(defvar *robot-type* :hrp2jsk-collada)
(require "package://contact_behavior_generation/euslisp/motion-sequencer.lisp")
(require "package://contact_behavior_generation/euslisp/dynamic-connector.lisp")
(require "package://contact_behavior_generation/euslisp/motion-planners/motion-planner.lisp")

(setq *force-observer-limb* :lleg) (require :lleg-force-observer "constraint-force-observer.l")
(setq *force-observer-limb* :rleg) (require :rleg-force-observer "constraint-force-observer.l")
(setq *force-observer-limb* :hip) (require :hip-force-observer "constraint-force-observer.l")
(require "constraint-torque-observer.l")

;; @Override
(defun demo-motion-sequence (&rest args) (let* ((ret (apply 'demo-motion-sequence2 (append args (list :error-thre 1.1))))) (cons (car ret) (reverse (cdr ret)))))

;; @Override for debug
(defvar *rsd-play* (function rsd-play))
(defun rsd-play (&rest args) (apply *rsd-play* :rsd-list *rsd* :graph nil args))

;; test-proc
;; test-proc :u-rate 2.0 :init (nth 5 (reverse *rsd*))
(setq  *kca-cog-gain* 1)
(defun test-proc
  (&rest args &key (anime? nil) (u-rate 1.0) (u (* u-rate 0.2))
         (init
          (progn (send *robot* :reset-manip-pose)
                 (send *robot* :reset-pose)
                 (send *robot* :arms :elbow-p :joint-angle -90)
                 (send *robot* :arms :shoulder-p :joint-angle -30)
                 (send *robot* :fix-leg-to-coords (make-coords)) ;; :pos (float-vector -10 0 0)))
                 (send *viewer* :draw-objects)
                 (optimize-brli :contact-states
                                (now-contact-state :limb-keys '(:rleg :lleg))
                                :rest-contact-states
                                (list (now-hip-contact-state)))
                 ))
         &allow-other-keys)
  (send init :draw :friction-cone? nil :torque-draw? nil)
  (demo-climb-setup :four-leg-seat)
  (send init :draw :friction-cone? nil :torque-draw? nil)
  ;;
  (setq *goal-contact-state* (nth 87 *contact-states*))
  (setf (aref (send (send *goal-contact-state* :target-coords) :worldpos) 0)
        (+ (aref (send (send *goal-contact-state* :target-coords) :worldpos) 0) 60))
  ;;
  (setq *contact-states*
        (remove-if '(lambda (d) (< (norm (send (send d :target-coords) :difference-position (send (send d :contact-coords) :worldcoords))) 20)) *contact-states*))
  (dolist (v (send-all (send-all *contact-states* :target-coords) :worldpos))
    (setf (aref v 2) (- (aref v 2) 10)))
  (dolist (c (remove-if '(lambda (cs) (not (eq (send cs :name) :hip)))
                        (append (send init :contact-states) *contact-states*)))
    (setf (aref (send c :slip-matrix) 0 2) u)
    (setf (aref (send c :slip-matrix) 1 2) u)
    (setf (aref (send c :slip-matrix) 5 2) u))
  ;;
  ;; (dolist (c (remove-if '(lambda (cs) (eq (send cs :name) :hip)) (send init :contact-states)))
  ;;   (setf (aref (send c :slip-matrix) 0 2) 0.4)
  ;;   (setf (aref (send c :slip-matrix) 1 2) 0.4)
  ;;   (setf (aref (send c :slip-matrix) 5 2) 0.4)
  ;;   (setf (aref (send c :slip-matrix) 3 2) 0.03)
  ;;   (setf (aref (send c :slip-matrix) 4 2) 0.03))
  ;; (dolist (c *contact-states*)
  ;; (send c :set-val 'gain '(1.0 1.0 1.0)))
  ;;
  (unix:system (format nil "mkdir -p ~A" *log-root*))
  (cond
   ((let* ((rsd (demo-motion-sequence-with-timer
                 :ik-debug-view *ik-debug-view*
                 :loop-max
                 '(lambda (id)
                    (< (norm (send (send *goal-contact-state* :target-coords)
                                   :difference-position
                                   (send (send *goal-contact-state* :contact-coords) :worldcoords)))
                       10))
                 :remove-limb :hip
                 :all-limbs '(:hip)
                 :now-rsd
                 ;;(instance robot-state-data2 :init :contact-states (cons (now-hip-contact-state) (now-contact-state :limb-keys '(:rleg :lleg))))
                 init
                 :tmax-hand-rate 1.0
                 :tmax-leg-rate 1.0
                 :log-file (format nil "~A/four-leg-seat-u~A" *log-root* u)
                 )))
      (setq *rsd* rsd)
      (cond
       ((and anime? (listp rsd))
        (rsd-play :file (format nil "~A/four-leg-seat-u~A.rsd" *log-root* u) :auto? t)
        (quit-graph)))
      (atom rsd))
    (print 'four-leg-seat-error)
    t)))

(defvar *external-force* (float-vector 0 0 (* 1e-3 9.8 (send *robot* :weight)) 0 0 0))
(defun slide-a-little
  (av1 av2
       &key
       (move-deg 10)
       (sleep-time 2500)
       (av3 (v+ (scale move-deg (normalize-vector (v- av2 av1)))
                av1))
       (rate 30)
       (min-cnt 10)
       (error-flag nil)
       (never-stop? nil)
       (off-error-thre 1.0)
       (hip-error-thre 1.0)
       (mu-buf (float-vector 0))
       rfs-org lfs-org
       )
  (ros::rate rate)
  (send *robot* :angle-vector av3)
  (send *viewer* :draw-objects)
  (model2real :sleep-time sleep-time :wait? nil)
  ;;
  (warning-message 6 "start sliding: off-thre:~A, hip-thre:~A~%" off-error-thre hip-error-thre)
  ;;
  (dotimes (i (round (* rate (/ sleep-time 1000.0))))
    (ros::sleep)
    (ros::spin-once)
    (send *ri* :state)
    (setq rfs-org (cdr (assoc :off-rfsensor (send *ri* :get-val 'robot-state))))
    (setq lfs-org (cdr (assoc :off-lfsensor (send *ri* :get-val 'robot-state))))
    (if (and rfs-org lfs-org) (return-from nil nil)))
  ;;
  (dotimes (i (round (* rate (/ sleep-time 1000.0))))
    (ros::sleep)
    (ros::spin-once)
    (send *ri* :state)
    (let* ((rfs (cdr (assoc :off-rfsensor (send *ri* :get-val 'robot-state))))
           (lfs (cdr (assoc :off-lfsensor (send *ri* :get-val 'robot-state))))
           (cog (if (and rfs lfs) (v- (v- *external-force* rfs) lfs)))
           )
      (if rfs (rleg_sensor_observer::update-error-vector nil :f rfs :off-f rfs-org))
      (if lfs (lleg_sensor_observer::update-error-vector nil :f lfs :off-f lfs-org))
      (if cog (hip_sensor_observer::update-error-vector nil :f cog)))
    ;;
    (lleg_sensor_observer::publish-error-vector)
    (rleg_sensor_observer::publish-error-vector)
    (hip_sensor_observer::publish-error-vector)
    ;;
    (ros::publish "/constraint_torque_observer/error/vector"
                  (instance std_msgs::float32multiarray :init
                            :data torque_sensor_observer::*error-vector*))
    ;;
    (ros::publish "/hrp2007_seating/slide/thre/p" (instance std_msgs::float32 :init :data 1))
    (ros::publish "/hrp2007_seating/slide/prediction/p" (instance std_msgs::float32 :init :data hip-error-thre))
    (ros::publish "/hrp2007_seating/slide/thre/n" (instance std_msgs::float32 :init :data -1))
    (ros::publish "/hrp2007_seating/slide/prediction/n" (instance std_msgs::float32 :init :data (* -1 hip-error-thre)))
    ;;
    (let* ((rleg_fmin
            (apply 'max (map cons 'abs (or rleg_sensor_observer::*error-vector* '(0)))))
           (lleg_fmin
            (apply 'max (map cons 'abs (or lleg_sensor_observer::*error-vector* '(0)))))
           (off_rleg_fmin
            (apply 'max (map cons 'abs (or rleg_sensor_observer::*off-error-vector* '(0)))))
           (off_lleg_fmin
            (apply 'max (map cons 'abs (or lleg_sensor_observer::*off-error-vector* '(0)))))
           (taumin
            (apply 'max (map cons 'abs (or torque_sensor_observer::*error-vector* '(0)))))
           (hip_fmin
            ;; (apply 'max (map cons 'abs (or hip_sensor_observer::*error-vector* '(0))))
            (if hip_sensor_observer::*error-vector*
                (abs (aref hip_sensor_observer::*error-vector* 0)) 0))
           )
      (cond
       ((> hip_fmin hip-error-thre)
        (setq error-flag t)
        (setf (aref mu-buf 0) hip_fmin)
        (warning-message 1 "hip force limitation ~A~%" hip_sensor_observer::*error-vector*))
       ((> rleg_fmin 1.0)
        (setq error-flag t)
        (warning-message 1 "right leg force limitation ~A~%" rleg_sensor_observer::*error-vector*))
       ((> lleg_fmin 1.0)
        (setq error-flag t)
        (warning-message 1 "left leg force limitation ~A~%" lleg_sensor_observer::*error-vector*))
       ((> off_rleg_fmin off-error-thre)
        (setq error-flag t)
        (warning-message 1 "right leg off-force limitation ~A~%" rleg_sensor_observer::*off-error-vector*))
       ((> off_lleg_fmin off-error-thre)
        (setq error-flag t)
        (warning-message 1 "left leg off-force limitation ~A~%" lleg_sensor_observer::*off-error-vector*))
       ((> taumin 1.0)
        (setq error-flag t)
        (warning-message 1 "torque limitation ~A~%" torque_sensor_observer::*error-vector*)))
      (cond
       ((and (minusp (decf min-cnt)) error-flag (not never-stop?))
        (warning-message 1 "revert in ~A msec~%" sleep-time)
        (send *robot* :angle-vector av1)
        (send *viewer* :draw-objects)
        (model2real :sleep-time sleep-time)
        (return-from nil nil)
        )
       (t
        (warning-message 6 "ok ~A/~A~%" i (round (* rate (/ sleep-time 1000.0))))
        ))))
  error-flag)

(defun demo-seating-proc
  (&optional (mu 0.2) (mu-buf (float-vector 0)))
  (cond
   ((not (and (boundp '*ri*) *ri*))
    (warning-message 1 "initalize robot-interface~%")
    (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
    (hrp2jsk-init)))
  (let* ((slide? nil))
    (dolist (rsd (reverse (cdr *rsd*)))
      ;; slide check
      (cond
       (slide?
        (warning-message 1 "slide a little~%")
        (if (y-or-n-p)
            (if (slide-a-little (copy-seq (send slide? :angle-vector))
                                (copy-seq (send rsd :angle-vector))
                                :sleep-time 5000
                                ;; :off-error-thre
                                ;; (max 0.1
                                ;;      (* 0.5 1.5
                                ;;         (+ (/ (abs (aref (send slide? :contact-forces :rleg) 0))
                                ;;               (max 1e-3 (abs (aref (send slide? :contact-forces :rleg) 2))))
                                ;;            (/ (abs (aref (send slide? :contact-forces :lleg) 0))
                                ;;               (max 1e-3 (abs (aref (send slide? :contact-forces :lleg) 2)))))))
                                :hip-error-thre mu
                                :mu-buf mu-buf
                                )
                (return-from demo-seating-proc slide?)))
        ;;
        (warning-message 1 "slide~%")
        (if (y-or-n-p)
            (slide-a-little (copy-seq (send slide? :angle-vector))
                            (copy-seq (send rsd :angle-vector))
                            :av3 (copy-seq (send rsd :angle-vector))
                            :sleep-time 5000
                            :never-stop? t
                            :hip-error-thre mu
                            :mu-buf mu-buf
                            ;; :off-error-thre
                            ;; (max 0.1
                            ;;      (* 0.5 1.5
                            ;;         (+ (/ (abs (aref (send rsd :contact-forces :rleg) 0))
                            ;;               (max 1e-3 (abs (aref (send rsd :contact-forces :rleg) 2))))
                            ;;            (/ (abs (aref (send rsd :contact-forces :lleg) 0))
                            ;;               (max 1e-3 (abs (aref (send rsd :contact-forces :lleg) 2)))))))
                            ))
        (setq slide? nil))
       (t
        (if (send rsd :buf :slide) (setq slide? rsd))
        ;; move
        (send rsd :draw :friction-cone? nil :torque-draw? nil)
        (warning-message 1 "ok?~%")
        (if (y-or-n-p)
            (model2real :sleep-time 5000))))
      ;;
      ))
  ;;
  (send (cadr *rsd*) :draw :friction-cone? nil :torque-draw? nil)
  (let* ((rleg (copy-seq (send *robot* :rleg :angle-vector)))
         (lleg (copy-seq (send *robot* :lleg :angle-vector))))
    (send *robot* :reset-pose)
    (send *robot* :arms :elbow-p :joint-angle -90)
    (send *robot* :rleg :angle-vector rleg)
    (send *robot* :lleg :angle-vector lleg)
    (send *viewer* :draw-objects))
  (warning-message 1 "last?~%")
  (if (y-or-n-p)
      (model2real :sleep-time 5000))
  t
  )

(defun rsd-union
  (rsd-list)
  (append
   (append
    (list (car rsd-list) (cadr rsd-list))
    (flatten
     (mapcar
      '(lambda (rsd1 rsd2)
         (if (< (print (norm (v- (send rsd1 :angle-vector) (send rsd2 :angle-vector)))) 10)
             nil rsd1))
      (cddr rsd-list) (cdr rsd-list))))))

;; (setq torque_sensor_observer::*torque-constraint-rate* -0.7)
(defvar *rsd-list*)
(defun demo-seating
  (&key (rsd nil) (rsd-list *rsd*) (mu-buf (float-vector 0)))
  (setq *rsd-list* nil)
  (dolist (u-rate '(1 2 4 8 16 32 62)) ;; 10 25))
    (warning-message 1 " -- gen rsd, U-RATE: ~A~%" u-rate)
    (cond
     ((> (aref mu-buf 0) (* u-rate 0.2))
      (warning-message 1 " -- skip mu=~A < ~A~%" (* u-rate 0.2) (aref mu-buf 0)))
     (t
      (setq rsd-list (or rsd-list
                         (progn
                           (apply 'test-proc :u-rate u-rate (if rsd (list :init rsd) nil))
                           *rsd*)))
      (setq rsd-list (rsd-union rsd-list))
      (push rsd-list *rsd-list*)
      (cond
       ((not (find :loop-exeeded rsd-list))
        (warning-message 1 " -- abort: no anster~%")
        (return-from demo-seating nil)))
      ;;
      (setq rsd (demo-seating-proc (* u-rate 0.2) mu-buf))
      (cond
       ((eq rsd t)
        (warning-message 1 " -- success!~%")
        (return-from demo-seating t))
       ((and (class rsd) (subclassp (class rsd) robot-state-data2))
        (send rsd :buf :slide nil)
        'next)
       (t (warning-message 1 " -- invalid return statement ~A~%" rsd)
          (return-from demo-seating rsd)
          )
       )
      (setq rsd-list nil)
      ))))

;;init

(ros::advertise "/hrp2007_seating/slide/thre/p" std_msgs::float32)
(ros::advertise "/hrp2007_seating/slide/prediction/p" std_msgs::float32)
(ros::advertise "/hrp2007_seating/slide/thre/n" std_msgs::float32)
(ros::advertise "/hrp2007_seating/slide/prediction/n" std_msgs::float32)

(progn (demo-climb-setup :four-leg-seat)
       ;;
       (dolist (v (send-all (send-all *contact-states* :target-coords) :worldpos))
         (setf (aref v 2) (- (aref v 2) 10)))
       ;; (setq *goal-contact-state* (nth 86 *contact-states*))
       (setq *goal-contact-state* (nth 87 *contact-states*))
       (send *robot* :reset-manip-pose)
       (send *robot* :reset-pose)
       (send *robot* :arms :elbow-p :joint-angle -90)
       (send *robot* :fix-leg-to-coords (make-coords)))

(defvar *rsd*)
(defvar *log-root* (format nil "log/~A" (log-surfix)))
(defvar *ik-debug-view* nil) ;;:no-message)
(unix:system (format nil "mkdir -p ~A" *log-root*))
(cond
 ((let* ((log (read-line (piped-fork "ls -t log/*/four-leg-seat.rsd | head -n 1") nil)))
    (and
     (and log (probe-file (format nil "~A" log)))
     (progn
       (unix::system (format nil "cp ~A ~A" log *log-root*))
       (setq *rsd* (car (rsd-deserialize :file (format nil "~A" log)))))))
  'nop)
 (t
  (if (test-proc)
      (warning-message 1 "not solutions found~%"))))
(setq *rsd-org* *rsd*)

#|

(defun demo-seating-old
  nil
  (cond
   ((not (and (boundp '*ri*) *ri*))
    (warning-message 1 "initalize robot-interface~%")
    (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
    (hrp2jsk-init)))
  ;; init
  (send (nth 1 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
  (warning-message 1 "intial pose?~%") (read-line)
  (model2real)
  ;; seating
  (send (nth 2 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
  (warning-message 1 "seating?~%") (read-line)
  (model2real :sleep-time 5000)
  ;; suspend
  (send (nth 4 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
  (warning-message 1 "suspend?~%") (read-line)
  (model2real :sleep-time 5000)
  ;; slide before
  (send (nth 5 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
  (warning-message 1 "slide before?~%") (read-line)
  (model2real :sleep-time 5000)
  ;; slide a little
  (warning-message 1 "slide a little?~%") (read-line)
  (if (slide-a-little (copy-seq (send (nth 5 (reverse *rsd*)) :angle-vector))
                      (copy-seq (send (nth 6 (reverse *rsd*)) :angle-vector)))
      (return-from demo-seating nil))
  ;; slide
  (send (nth 6 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
  (warning-message 1 "slide?~%") (read-line)
  (model2real :sleep-time 10000)
  ;; last
  (send (nth 7 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
  (let* ((rleg (copy-seq (send *robot* :rleg :angle-vector)))
         (lleg (copy-seq (send *robot* :lleg :angle-vector))))
    (send *robot* :reset-pose)
    (send *robot* :arms :elbow-p :joint-angle -90)
    (send *robot* :rleg :angle-vector rleg)
    (send *robot* :lleg :angle-vector lleg)
    (send *viewer* :draw-objects))
  (warning-message 1 "last?~%") (read-line)
  (model2real :sleep-time 5000)
  )
