;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-overwrite* t)
(defvar *robot-type* :hrp2jsknts-collada)
(defvar *robot*)

(if (or *robot-overwrite*
        (not (and (boundp '*robot*) *robot*)))
    (case *robot-type*
      ;; (:hrp2jsk-gazebo
      ;;  (load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.lisp")
      ;;  (require :hrp2jsk "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.lisp")
      ;;  (require :hrp2jsk-utils "package://jsk_hrpsys_ros_bridge/euslisp/hrp2jsk-utils.lisp")
      ;;  (setq *robot* (instance hrp2jsk-robot :init)))
      (:hrp2jsk-collada
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
       (setq *robot* (instance hrp2jsk-robot :init)))
      (:hrp2jsknt-collada
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.li")
       (setq *robot* (instance hrp2jsknt-robot :init))
       )
      (:hrp2jsknts-collada
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
       (setq *robot* (instance hrp2jsknts-robot :init))
       )
      (:staro
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/staro-interface.l")
       (setq *robot* (staro))
       )
      ;; (:hrp2jsk
      ;;  (setq *robot* (hrp2jsk-simple-detail)))
      ;; (:hrp2jsknts
      ;;  (setq *robot* (hrp2jsknts-simple-detail)))
      ;; (:hrp2jsknt
      ;;  (setq *robot* (hrp2jsknt-simple-detail)))
      ;; (:atlas
      ;;  (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.lisp")
      ;;  (require "my-util.lisp")
      ;;  (setq *robot* (atlas-with-hand))
      ;;  ;; (send-all (send *robot* :joint-list) :max-joint-torque 700)
      ;;  (defun init-pose
      ;; 	 (&rest args)
      ;; 	 (send *robot* :reset-manip-pose)
      ;; 	 (send *sandia_hand_left* :grasp-pose)
      ;; 	 (send *sandia_hand_right* :grasp-pose)
      ;; 	 (send (car (send *robot* :links)) :newcoords
      ;; 	       (make-coords
      ;; 		:pos
      ;; 		(float-vector
      ;; 		 -200 0
      ;; 		 (aref (v- (send *robot* :worldpos)
      ;; 			   (send *robot* :rleg :end-coords :worldpos)) 2)))))
      ;;  )
      (:jaxon
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
       (setq *robot* (jaxon))
       )
      (:jaxon_red
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
       (setq *robot* (jaxon_red))
       )
      (t
       (require "package://euslisp/jskeus/irteus/demo/sample-robot-model.lisp")
       (setq *robot* (instance sample-robot :init))
       ;; (send-all (send *robot* :joint-list) :max-joint-torque 100)
       )
      ))

(let* ((val 300))
  (dolist (j (send *robot* :joint-list))
    (cond
     ((< (abs (send j :max-joint-torque))
	 (if (eq *robot-type* :staro) 100 1e-3))
      (format t "[robot-param] ~A joint has 0 torque limit~%"
	      (send j :name))
      (format t "              set ~A Nm  as torque limit~%"
	      val (send j :name))
      (send j :max-joint-torque val)))))
