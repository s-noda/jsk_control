#!/usr/bin/env roseus

(defvar *robot-type* :hrp2jsk-collada)
(require "package://contact_behavior_generation/euslisp/motion-sequencer.lisp")
(require "package://contact_behavior_generation/euslisp/dynamic-connector.lisp")
(require "package://contact_behavior_generation/euslisp/motion-planners/motion-planner.lisp")

;; @Override
(defun demo-motion-sequence (&rest args) (let* ((ret (apply 'demo-motion-sequence2 (append args (list :error-thre 1.1))))) (cons (car ret) (reverse (cdr ret)))))

(defun test-proc
  (key)
  (cond
   ((let* ((init
            (progn (demo-climb-setup :four-leg-seat)
                   ;;
                   (dolist (v (send-all (send-all *contact-states* :target-coords) :worldpos))
                     (setf (aref v 2) (- (aref v 2) 10)))
                   ;;
                   ;; (setq *goal-contact-state* (nth 86 *contact-states*))
                   (setq *goal-contact-state* (nth 87 *contact-states*))
                   (send *robot* :reset-manip-pose)
                   (send *robot* :reset-pose)
                   (send *robot* :arms :elbow-p :joint-angle -90)
                   (send *robot* :fix-leg-to-coords (make-coords))
                   (send *viewer* :draw-objects)
                   ))
           (rsd (demo-motion-sequence-with-timer
                 :ik-debug-view *ik-debug-view*
                 :loop-max 2
                 :remove-limb :hip
                 :all-limbs '(:hip)
                 :now-rsd
                 ;;(instance robot-state-data2 :init :contact-states (cons (now-hip-contact-state) (now-contact-state :limb-keys '(:rleg :lleg))))
                 (optimize-brli :contact-states
                                (now-contact-state :limb-keys '(:rleg :lleg))
                                :rest-contact-states
                                (list (now-hip-contact-state)))
                 :tmax-hand-rate 1.0
                 :tmax-leg-rate 1.0
                 :log-file (format nil "~A/four-leg-seat" *log-root*)
                 )))
      (setq *rsd* rsd)
      (cond
       ((listp rsd)
        (rsd-play :file (format nil "~A/four-leg-seat.rsd" *log-root*) :auto? t)
        (quit-graph)))
      (atom rsd))
    (print 'four-leg-seat-error)
    t)))

(defun demo-seating
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

;;(init
(progn (demo-climb-setup :four-leg-seat)
       ;;
       (dolist (v (send-all (send-all *contact-states* :target-coords) :worldpos))
         (setf (aref v 2) (- (aref v 2) 10)))
       ;;
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
 ((let* ((log (read-line (piped-fork "ls -t log | head -n 1") nil)))
    (and log (probe-file (format nil "log/~A/four-leg-seat.rsd" log)))
    (setq *rsd* (car (rsd-deserialize :file (format nil "log/~A/four-leg-seat.rsd" log)))))
  'nop)
 (t
  (unix:system (format nil "mkdir -p ~A" *log-root*))
  (if (test-proc :four-leg-seat)
      (warning-message 1 "not solutions found~%"))))

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
