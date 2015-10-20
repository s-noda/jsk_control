#!/usr/bin/env roseus

(defun add-6dof-rotational-joints
  (&key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (ref-joint (send robot :lleg :joint-list))
   ;;
   (link-list (list nil))
   (rlink root-link)
   rlinks
   )
  (dotimes (i 6)
    (let* ((j0 (nth i ref-joint))
	   (vlink
	    (instance
	     bodyset-link
	     :init (make-cascoords
		    :coords
		    (send (send j0 :child-link)
			  :copy-worldcoords))
	     :bodies (list (make-cube 10 10 10))
	     :name (format nil "~A~A" 'virtual-link_ i)
	     :weight 0 :centroid (float-vector 0 0 0)
	     :inertia-tensor (make-matrix 3 3)))
	   (j (instance rotational-joint :init
			:name (format nil "~A_virtual"
				      (send j0 :name))
			:axis (scale 1 (send j0 :get-val 'axis))
			:min (send j0 :min-angle)
			:max (send j0 :max-angle)
			:child-link rlink
			;;(if (eq rlink root-link) robot rlink)
			:parent-link vlink))
	   )
      (send j :joint-angle (send j0 :joint-angle))
      ;; (if (not (eq rlink root-link))
      (send vlink :assoc rlink)
      (send j :set-val 'default-coords
	    (send (send j0 :get-val 'default-coords)
		  :transformation (make-coords)))
      (send rlink :add-joint j)
      (send rlink :add-parent-link vlink)
      (send vlink :add-child-links rlink)
      (push rlink rlinks)
      (setq rlink vlink)))
  (send robot :put :root-link-org root-link)
  (send robot :put :links-org (send robot :links))
  (setq root-link rlink)
  (send robot :set-val 'links
	(append (cons root-link rlinks)
		(remove root-link (send robot :links))))
  ;; (send robot :set-val 'joint-list (flatten (send-all (send robot :links) :joint)))
  (list
   (cons :6dof-links
         (mapcar #'(lambda (l) (append rlinks l)) link-list))
   (cons :del-6dof-links
         (eval
          (list 'function
                (list 'lambda nil
		      (list 'send (send root-link :parent-link) :del-child-link root-link)
                      (list 'send root-link :del-joint)
                      (list 'send root-link :del-parent-link))))))
  )

(defun rotational-6dof-demo
  nil
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (defvar *robot* (instance hrp2jsknt-robot :init))
  (defvar *robot-org* *robot*)
  (setq *robot* (copy-object *robot-org*))
  (objects (list *robot*))
  (setq *v6dof* (add-6dof-rotational-joints))
  (send-all (send-all (cadr (assoc :6dof-links *v6dof*)) :joint) :joint-angle 0 :relative t)
  (send-all (send *robot* :links) :worldcoords)
  ;;
  (send *robot* :reset-pose)
  (send *robot* :inverse-kinematics
	(list
	 (send
	  (send *robot* :rarm :end-coords :copy-worldcoords)
	  :translate (float-vector 0 -100 0) :world)
	 (send *robot* :rleg :end-coords :copy-worldcoords)
	 (send *robot* :lleg :end-coords :copy-worldcoords))
	:move-target
	(list (send *robot* :rarm :end-coords)
	      (send *robot* :rleg :end-coords)
	      (send *robot* :lleg :end-coords))
	:link-list
	(list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
	      (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
	      (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
	      )
	;;
	:target-centroid-pos
	(scale 0.5 (v+ (send *robot* :rleg :end-coords :worldpos)  (send *robot* :lleg :end-coords :worldpos)))
	:centroid-thre 5
	:cog-null-space t
	:cog-gain 1.0
	:cog-translation-axis :z
	:centroid-offset-func
	#'(lambda (&rest args)
	    (send *robot* :centroid))
	;; :additional-weight-list
	;; (mapcar '(lambda (l) (list l 0.1))
	;; 	(cadr (assoc :6dof-links *v6dof*)))
	:debug-view :no-message))

(cond
 ((and (boundp '*robot*) *robot*)
  ;; (defvar *robot-org* *robot*)
  ;; (setq *robot* (copy-object *robot-org*))
  (setq *rotational-6dof-fix-leg* :new)
  (objects (list *robot*))
  (setq *v6dof* (add-6dof-rotational-joints))
  (send-all (send-all (cadr (assoc :6dof-links *v6dof*)) :joint) :joint-angle 0 :relative t)
  (send-all (send *robot* :links) :worldcoords)
  ;;
  (eval
   (list 'defmethod (send (class *robot*) :name)
	 '(:fullbody-inverse-kinematics
	   (target-coords
	    &rest args
	    &key
	    (move-target) (link-list)
	    (target-centroid-pos (apply #'midpoint 0.5 (send self :legs :end-coords :worldpos)))
	    (cog-gain 1.0)
	    (centroid-thre 5.0)
	    additional-weight-list
	    (root-link-virtual-joint-weight (float-vector 0.1 0.1 0.1 0.1 0.1 0.1))
	    &allow-other-keys)
	   (send* self :inverse-kinematics target-coords
		  :move-target move-target
		  :link-list link-list
		  :cog-gain cog-gain
		  :centroid-thre centroid-thre
		  :target-centroid-pos target-centroid-pos
		  :additional-weight-list
		  (append
		   additional-weight-list
		   (map cons
			'(lambda (l d) (list l d))
			(send *robot* :link-list (send *robot* :get :root-link-org)) root-link-virtual-joint-weight)
		   (remove nil
			   (list (if (send self :rleg :toe-p)
				     (list (send self :rleg :toe-p :child-link) 0))
				 (if (send self :lleg :toe-p)
				     (list (send self :lleg :toe-p :child-link) 0)))))
		  args)
	   )))
  ))

#|

roseus human-ball-test.lisp
(test-torque-ik :stop 20)
(load "rotational-6dof.lisp")
(test-torque-ik :gain 2e-4 :key-list '(:rleg :lleg) :init nil :root-link-virtual-joint-weight (scale 1e+3 (float-vector 1 1 1 1 1 1)))

(require "package://euslisp/jskeus/irteus/irtmodel.l")
(require "package://euslisp/jskeus/irteus/irtrobot.l")
(require "package://euslisp/jskeus/irteus/irtdyna.l")

