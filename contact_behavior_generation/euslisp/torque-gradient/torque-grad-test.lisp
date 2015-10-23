#!/usr/bin/env roseus

(require "human-ball-test.lisp")
(require "rotational-6dof.lisp")

;; (test-sphere-human-ball :key-list '(:rleg :lleg :rarm :larm) :stop 50 :debug-view :no-message :null-max 0.3 :gain1 0.01 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send *robot* :newcoords (make-coords :pos (float-vector 0 0 -650))))))

;; (test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm :larm) :stop 50 :debug-view nil :gain1 0.530954 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send (car (send *robot* :links)) :newcoords (make-coords :pos (float-vector 0 0 -650))))))

(setq
 *test-human-ball-rsd*
 (test-sphere-human-ball-loop :key-list '(:rleg :lleg :rarm :larm) :stop 50 :null-max 0.3 :debug-view nil :gain1 0.01 :gain2 0.01 :rest-torque-ik-args (list :contact-wrench-optimize? t) :human-ball-pose-args (list :human-ball-init-pose '(progn (reset-pose) (send *robot* :newcoords (make-coords :pos (float-vector 0 0 -650)))))))

;; (dump-loadable-structure "log.test-human-ball.rsd" *test-human-ball-rsd*)
