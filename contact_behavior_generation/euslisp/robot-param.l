;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(defvar *robot-type* :hrp2jsknts-collada)
(defvar *robot*)

(case *robot-type*
      (:hrp2jsk-gazebo
       (load "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
       (require :hrp2jsk "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
       (require :hrp2jsk-utils "package://jsk_hrpsys_ros_bridge/euslisp/hrp2jsk-utils.l")
       (setq *robot* (instance hrp2jsk-robot :init)))
      (:hrp2jsk-collada
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
       ;;(require :hrp2jsk "package://hrpsys_ros_bridge_tutorials/models/hrp2jsk.l")
       (setq *robot* (instance hrp2jsk-robot :init)))
      (:hrp2jsknt-collada
       ;;(require :hrp2jsknt "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknt.l")
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
       (setq *robot* (instance hrp2jsknt-robot :init))
       )
      (:hrp2jsknts-collada
       (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
       ;;(require :hrp2jsknts "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
       (setq *robot* (instance hrp2jsknts-robot :init))
       )
      (:staro
       ;;(require "package://hrpsys_ros_bridge_tutorials/models/staro.l")
       (require "package://jsk_hrpsys_ros_bridge/euslisp/staro-interface.l")
       (setq *robot* (staro))
       ;; (send-all (send *robot* :joint-list) :max-joint-torque 300)
       )
      (:hrp2jsk
       (setq *robot* (hrp2jsk-simple-detail)))
      (:hrp2jsknts
       (setq *robot* (hrp2jsknts-simple-detail)))
      (:hrp2jsknt
       (setq *robot* (hrp2jsknt-simple-detail)))
      (:atlas
       (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
       (require "my-util.l")
       (setq *robot* (atlas-with-hand))
       ;; (send-all (send *robot* :joint-list) :max-joint-torque 700)
       (defun init-pose
	 (&rest args)
	 (send *robot* :reset-manip-pose)
	 (send *sandia_hand_left* :grasp-pose)
	 (send *sandia_hand_right* :grasp-pose)
	 (send *robot* :newcoords
	       (make-coords
		:pos
		(float-vector
		 -200 0
		 (aref (v- (send *robot* :worldpos)
			   (send *robot* :rleg :end-coords :worldpos)) 2)))))
       )
      (t
       (require "package://euslisp/jskeus/irteus/demo/sample-robot-model.l")
       (setq *robot* (instance sample-robot :init))
       ;; (send-all (send *robot* :joint-list) :max-joint-torque 100)
       )
      )

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

#|

roseus
(defvar *robot-type* :staro)
(load "motion-sequencer.l")
(objects *robot*)
(send-all (send *robot* :joint-list) :max-joint-torque 300)

(defvar *psuedo-inverse* #'pseudo-inverse)
(defun pseudo-inverse
  (m &optional ret mat)
  (setq ret (funcall *psuedo-inverse* m))
  (while (null ret)
    (if (null mat) (setq mat (copy-object m)))
    (dotimes (i (send mat :get-val 'dim0))
      (dotimes (j (send mat :get-val 'dim1))
	(setf (aref mat i j)
	      (+ (aref mat i j) (- (random 0.01) 0.005)))))
    (setq ret (funcall *psuedo-inverse* mat)))
  ret)

(setq buf nil)
(setq *kca-cog-gain* 3)
(setq *pg-move-gain* (/ 10000 2))
(dotimes (i 100)
  (init-pose)
  (send *robot* :reset-manip-pose)
  (send *robot* :arms
   	:inverse-kinematics
   	(make-coords :pos #F(10000 0 0))
   	:revert-if-fail nil)
  (setq hoge i)
  (if (not
       (simple-fullbody
	:target
	(list
	 (list (cons :target :lleg)
	       (cons :move (float-vector 300 0 (* (+ 0 i) 10))))
	 (list (cons :target :rleg)))
	:root-link-virtual-joint-weight
	#F(1 1 1 1 1 1)
	:target-centroid-pos nil))
      (return-from nil i))
  (if (not
       (simple-fullbody
	:target
	(list
	 (list (cons :target :lleg))
	 (list (cons :target :rleg)))
	:root-link-virtual-joint-weight
	#F(1 1 1 1 1 1)
	:centroid-thre 10
	:target-centroid-pos
	(send *robot* :lleg :end-coords :worldpos)))
      (print 'hoge))
  (push
   (pose-generate-with-contact-state
    (now-contact-state :limb-keys '(:lleg))
    :rest-contact-states
    (now-contact-state :limb-keys '(:rleg))
    :ik-debug-view nil;; :no-message
    :root-link-virtual-joint-weight
    #F(1 1 1 1 1 1)
    :non-stop t
    :centroid-thre-rate 0.9
    :convergence-thre 0.1
    :tmax-leg-rate 0.7)
   buf)
  (send *viewer* :draw-objects)
  (if (not (and (car buf) (send (car buf) :full-constrainted)))
      (return-from nil i))
  (format t "height: ~A~%" (* i 10)))
(print hoge)

(mapcar
 #'(lambda (d)
     (send *robot* :angle-vector
	   (send d :angle-vector))
     (send *robot* :newcoords
	   (send d :root-coords))
     (draw-line-man :flush nil))
 (cdr (flatten buf)))
(send *viewer* :flush)

(rsd-play :rsd-list (cdr (flatten buf)) :graph-size '(640 640))
