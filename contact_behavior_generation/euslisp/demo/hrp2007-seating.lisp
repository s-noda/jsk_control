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
                   ;; (send *robot* :reset-pose)
                   ;; (send *robot* :arms :elbow-p :joint-angle -90)
                   (send *robot* :fix-leg-to-coords (make-coords))))
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

;; start test

(defvar *log-root* (format nil "log/~A" (log-surfix)))
(defvar *ik-debug-view* nil) ;;:no-message)
(unix:system (format nil "mkdir -p ~A" *log-root*))

(if (test-proc :four-leg-seat)
    (warning-message 1 "not solutions found~%"))

#|

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
(hrp2jsk-init)

(send (nth 0 (reverse *rsd*)) :draw :friction-cone? nil :torque-draw? nil)

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
(send-all (send *robot* :torso :joint-list) :joint-angle 0)
(send-all (send *robot* :head :joint-list) :joint-angle 0)
(send *viewer* :draw-objects)
(model2real :sleep-time 10000)
