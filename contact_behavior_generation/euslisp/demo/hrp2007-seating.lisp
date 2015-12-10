#!/usr/bin/env roseus

(defvar *robot-type* :hrp2jsk-collada)
(require "package://contact_behavior_generation/euslisp/motion-sequencer.lisp")
(require "package://contact_behavior_generation/euslisp/dynamic-connector.lisp")
(require "package://contact_behavior_generation/euslisp/motion-planners/motion-planner.lisp")

(setq *force-observer-limb* :lleg) (require :lleg-force-observer "constraint-force-observer.l")
(setq *force-observer-limb* :rleg) (require :rleg-force-observer "constraint-force-observer.l")
(require "constraint-torque-observer.l")

;; @Override
(defun demo-motion-sequence (&rest args) (let* ((ret (apply 'demo-motion-sequence2 (append args (list :error-thre 1.1))))) (cons (car ret) (reverse (cdr ret)))))


;; test-proc
;; test-proc :u-rate 2.0 :init (nth 5 (reverse *rsd*))
(defun test-proc
  (&rest args &key (anime? nil) (u-rate 1.0) (u (* u-rate 0.2))
         (init
          (progn (send *robot* :reset-manip-pose)
                 (send *robot* :reset-pose)
                 (send *robot* :arms :elbow-p :joint-angle -90)
                 (send *robot* :fix-leg-to-coords (make-coords))
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
        (+ (aref (send (send *goal-contact-state* :target-coords) :worldpos) 0) 50))
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

(defun slide-a-little
  (av1 av2
       &key
       (move-deg 10)
       (sleep-time 5000)
       (av3 (v+ (scale move-deg (normalize-vector (v- av2 av1)))
                av1))
       (rate 30)
       (error-flag nil)
       )
  (ros::rate rate)
  (send *robot* :angle-vector av3)
  (send *viewer* :draw-objects)
  (model2real :sleep-time sleep-time :wait? nil)
  (dotimes (i (round (* rate (/ sleep-time 1000.0))))
    (ros::sleep)
    (ros::spin-once)
    (let* ((rleg_fmin
            (apply 'min (coerce (or rleg_sensor_observer::*error-vector* '(0)) cons)))
           (lleg_fmin
            (apply 'min (coerce (or lleg_sensor_observer::*error-vector* '(0)) cons)))
           (taumin
            (apply 'min (coerce (or torque_sensor_observer::*error-vector* '(0)) cons))))
      (cond
       ((minusp rleg_fmin)
        (setq error-flag t)
        (warning-message 1 "right leg force limitation ~A~%" rleg_sensor_observer::*error-vector*))
       ((minusp lleg_fmin)
        (setq error-flag t)
        (warning-message 1 "left leg force limitation ~A~%" lleg_sensor_observer::*error-vector*))
       ((minusp taumin)
        (setq error-flag t)
        (warning-message 1 "torque limitation ~A~%" torque_sensor_observer::*error-vector*)))
      (cond
       (error-flag
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

(defun demo-seating-proc
  nil
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
        (warning-message 1 "slide~%")
        (if (or (not (y-or-n-p))
                (slide-a-little (copy-seq (send slide? :angle-vector))
                                (copy-seq (send rsd :angle-vector))))
            (return-from demo-seating-proc slide?))
        (setq slide? nil))
       ((send rsd :buf :slide)
        (setq slide? rsd)))
      ;; move
      (send rsd :draw :friction-cone? nil :torque-draw? nil)
      (warning-message 1 "ok?~%")
      (if (y-or-n-p)
          (model2real :sleep-time 5000))
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
  (&key (rsd nil) (rsd-list *rsd*))
  (setq *rsd-list* nil)
  (dotimes (u-rate 4)
    (warning-message 1 " -- gen rsd, U-RATE: ~A~%" (+ 1 u-rate))
    (setq rsd-list (or rsd-list
                       (progn
                         (apply 'test-proc :u-rate (+ 1 u-rate) (if rsd (list :init rsd) nil))
                         *rsd*)))
    (setq rsd-list (rsd-union rsd-list))
    (push rsd-list *rsd-list*)
    (cond
     ((not (find :loop-exeeded rsd-list))
      (warning-message 1 " -- abort: no anster~%")
      (return-from demo-seating nil)))
    ;;
    (setq rsd (demo-seating-proc))
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
    ))

;;(init
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
(cond
 ((let* ((log (read-line (piped-fork "ls -t log/*/four-leg-seat.rsd | head -n 1") nil)))
    (and
     (and log (probe-file (format nil "~A" log)))
     (setq *rsd* (car (rsd-deserialize :file (format nil "~A" log))))))
  'nop)
 (t
  (unix:system (format nil "mkdir -p ~A" *log-root*))
  (if (test-proc)
      (warning-message 1 "not solutions found~%"))))
(setq *rsd-org* *rsd*)

#|

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
(hrp2jsk-init)

(send (nth 1 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
(model2real)
;; init
(send (nth 2 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
(model2real :sleep-time 10000)
;; squat
(send (nth 4 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
(model2real :sleep-time 10000)
;; straight
(send (nth 5 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
(model2real :sleep-time 10000)
;; slide
(send (nth 6 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
(model2real :sleep-time 20000)
;; last
(send (nth 7 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)
(let* ((rleg (copy-seq (send *robot* :rleg :angle-vector)))
       (lleg (copy-seq (send *robot* :lleg :angle-vector))))
  (send *robot* :reset-pose)
  (send *robot* :arms :elbow-p :joint-angle -90)
  (send *robot* :rleg :angle-vector rleg)
  (send *robot* :lleg :angle-vector lleg)
  (send *viewer* :draw-objects))
(model2real :sleep-time 10000)
