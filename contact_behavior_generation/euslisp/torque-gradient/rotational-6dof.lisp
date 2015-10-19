#!/usr/bin/env roseus

(defmethod rotational-joint
  (:joint-angle
   (&optional v &key relative &allow-other-keys)
   "Return joint-angle if v is not set, if v is given, set joint angle. v is ro\
tational value in degree."
   (let ()
     (when v
       (when (and joint-min-max-table joint-min-max-target)
         (setq min-angle (send self :joint-min-max-table-min-angle)
               max-angle (send self :joint-min-max-table-max-angle)))
       (if relative (setq v (+ v joint-angle)))
       (cond ((> v max-angle)
              (unless relative (warning-message 3 ";; ~A :joint-angle(~A) viola\
te max-angle(~A)~%" self v max-angle))
              (setq v max-angle)))
       (cond ((< v min-angle)
              (unless relative (warning-message 3 ";; ~A :joint-angle(~A) viola\
te min-angle(~A)~%" self v min-angle))
              (setq v min-angle)))
       (setq joint-angle v)
       (send child-link :replace-coords default-coords)
       (send child-link :rotate (deg2rad joint-angle) axis)
       (if (send self :get :joint-angle-transform-coords)
	   (send child-link :transform (send self :get :joint-angle-transform-coords)))
       )
     joint-angle))
  )

(defun reverse-assoc
  (&rest
   args
   &key
   (robot *robot*)
   (limb :lleg)
   (link (send robot limb :end-coords :parent))
   (root-link
    (or (send robot :get :root-link)
	(progn
	  (send robot :put :root-link link)
	  link)))
   ;; (root-link-org
   ;; (send robot :put :root-link-org (car (send robot :links))))
   (parent (send link :parent))
   (j (send link :joint))
   (jname (send-all (send robot :joint-list) :name))
   (av-org (copy-seq (send robot :angle-vector)))
   (init (progn (send-all (send robot :joint-list) :joint-angle 0)
		(send-all (send robot :links) :worldcoords)))
   &allow-other-keys
   )
  (print (send link :name))
  (cond
   ((or (null parent)
	(not (subclassp (class parent) bodyset-link))
	)
    (send root-link :del-parent-link)
    (send root-link :del-joint)
    (send robot :dissoc link)
    (send robot :assoc root-link)
    ;; (send robot :set-val 'joint-list
    ;; 	  (mapcar
    ;; 	   #'(lambda (name)
    ;; 	       (find-if #'(lambda (j) (and j (string-equal name (send j :name))))
    ;; 			(send-all (send robot :links) :joint)))
    ;; 	   jname))
    (send robot :put :root-link-org (car (send robot :links)))
    (send robot :set-val 'links
	  (cons root-link (remove root-link (send robot :links))))
    ;;
    (send robot :angle-vector av-org)
    (send-all (send robot :links) :worldcoords)
    root-link)
   (t
    (let* ((parent-parent (send parent :parent))
	   (pj (send parent :joint)) pc dc
	   )
      (send parent :dissoc link)
      (send parent :del-child-link link)
      (send j :set-val 'parent-link link)
      (send j :set-val 'child-link parent)
      (send j :set-val 'axis
	    (cond
	     ((vectorp (send j :get-val 'axis)) (scale -1 (send j :get-val 'axis)))
	     (t (send j :get-val 'axis))))
      (let* ((lorg (send (send link :worldcoords) :copy-worldcoords))
	     (porg (send (send parent :worldcoords) :copy-worldcoords))
      	     (jc (send (send (send parent :worldcoords) :copy-worldcoords)
		       :transform (send j :get-val 'default-coords)))
      	     (dc (send (send lorg :transformation (make-coords))
      	     	       :transform
      	     	       jc))
	     )
	(send j :set-val 'default-coords dc)
	(send j :put :joint-angle-transform-coords
	      (send (send lorg :transform dc) :transformation porg))
      	)
      (send parent :add-joint j)
      ;;
      (send link :assoc parent)
      (send link :add-child-links parent)
      (send parent :add-parent-link link)
      ;;
      (apply 'reverse-assoc
	     (append
	      (list :robot robot
		    :root-link root-link
		    :link parent
		    :parent parent-parent
		    :j pj :jname jname
		    :init nil :av-org av-org)
	      args)))
    )))

(defun rotational-6dof-demo
  nil
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (defvar *robot* (instance hrp2jsknt-robot :init))
  (defvar *robot-org* *robot*)
  (setq *robot* (copy-object *robot-org*))
  (objects (list *robot*))
  (reverse-assoc :robot *robot* :limb :lleg)
  ;;
  (send *robot* :reset-pose)
  (send *robot* :inverse-kinematics
	(list
	 (send
	  (send *robot* :rarm :end-coords :copy-worldcoords)
	  :translate (float-vector 0 -100 0) :world)
	 (send *robot* :rleg :end-coords :copy-worldcoords))
	:move-target
	(list (send *robot* :rarm :end-coords)
	      (send *robot* :rleg :end-coords))
	:link-list
	(list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
	      (send *robot* :link-list (send *robot* :rleg :end-coords :parent)))
	;;
	:target-centroid-pos
	(scale 0.5 (v+ (send *robot* :rleg :end-coords :worldpos)
		       (send *robot* :lleg :end-coords :worldpos)))
	:centroid-thre 5
	:cog-null-space t
	:cog-gain 2.0
	:cog-translation-axis :z
	:centroid-offset-func
	#'(lambda (&rest args)
	    (send *robot* :centroid))
	:debug-view :no-message))

(cond
 ((and (boundp '*robot*) *robot*)
  (defvar *rotational-6dof-fix-leg* :lleg)
  (reverse-assoc :robot *robot* :limb *rotational-6dof-fix-leg*)
  ;;
  (eval
   (list 'defmethod (send (class *robot*) :name)
	 '(:link-list
	   (to &optional from)
	   (cond
	    ((find to (send self *rotational-6dof-fix-leg* :links))
	     ;; (format t "~A detected in ~A~%" (send to :name) *rotational-6dof-fix-leg*)
	     (let* ((ret) (l to)
		    (root-links
		     (list from (send self :get :root-link-org))))
	       (while (and (not (find l root-links)) (send l :child-links))
		 (push l ret)
		 (setq l (car (flatten (send l :child-links))))
		 )
	       ;; (setq ret (reverse ret))
	       (if from (push from ret))
	       ret))
	    (t (send-super :link-list to from))))
	 '(:fullbody-inverse-kinematics
	   (target-coords
	    &rest args
	    &key
	    (move-target) (link-list)
	    (target-centroid-pos (apply #'midpoint 0.5 (send self :legs :end-coords :worldpos)))
	    (cog-gain 2.0)
	    (centroid-thre 5.0)
	    &allow-other-keys)
	   (let* ((p (position-if
		      #'(lambda (mt)
			  (eq (send mt :parent) (send *robot* *rotational-6dof-fix-leg* :end-coords :parent)))
		      move-target)))
	     (cond
	      (p
	       (setq target-coords (append (subseq target-coords 0 p)
					   (if (< (+ 1 p) (length target-coords))
					       (subseq target-coords (+ 1 p)))))
	       (setq move-target (append (subseq move-target 0 p)
					 (if (< (+ 1 p) (length move-target))
					     (subseq move-target (+ 1 p)))))
	       (setq link-list (append (subseq link-list 0 p)
				       (if (< (+ 1 p) (length link-list))
					   (subseq link-list (+ 1 p))))))))
	   (send* self :inverse-kinematics target-coords
		  :move-target move-target
		  :link-list link-list
		  :cog-gain cog-gain :centroid-thre centroid-thre
		  :target-centroid-pos target-centroid-pos
		  args)
	   ))
   )
  ;;
  ))


#|

;; (defvar *robot-type* :hrp2jsknt-collada)
;; (require "gradient-util.lisp")
;; (require "../optimize-brli.lisp")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(defvar *robot* (instance hrp2jsknt-robot :init))
(defvar *robot-org* *robot*)
(setq *robot* (copy-object *robot-org*))
(objects (list *robot*))
(reverse-assoc :robot *robot* :limb :lleg)

#|

(reverse-assoc :robot *robot* :limb :lleg)

(send *robot* :reset-pose)

(send *robot* :inverse-kinematics
      (list
       (send
	(send *robot* :rarm :end-coords :copy-worldcoords)
	:translate (float-vector 0 -100 0) :world)
       (send *robot* :rleg :end-coords :copy-worldcoords))
      :move-target
      (list (send *robot* :rarm :end-coords)
	    (send *robot* :rleg :end-coords))
      :link-list
      (list (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
	    (send *robot* :link-list (send *robot* :rleg :end-coords :parent)))
      ;;
      :target-centroid-pos
      (scale 0.5 (v+ (send *robot* :rleg :end-coords :worldpos)
		     (send *robot* :lleg :end-coords :worldpos)))
      :centroid-thre 5
      :cog-null-space t
      :cog-gain 2.0
      :cog-translation-axis :z
      :centroid-offset-func
      #'(lambda (&rest args)
	  (send *robot* :centroid))
      :debug-view :no-message)

(send *robot* :fullbody-inverse-kinematics
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
	    (send *robot* :link-list (send *robot* :lleg :end-coords :parent)))
      ;;
      :target-centroid-pos
      (scale 0.5 (v+ (send *robot* :rleg :end-coords :worldpos)
		     (send *robot* :lleg :end-coords :worldpos)))
      :centroid-thre 5
      :cog-null-space t
      :cog-gain 1.0
      :cog-translation-axis :z
      :centroid-offset-func
      #'(lambda (&rest args)
	  (send *robot* :centroid))
      :debug-view :no-message)

(defun pendulum
  (&optional (k :knee-p))
  (do-until-key
   (dotimes (i 10)
     (send *robot* :lleg k :joint-angle 3 :relative t)
     (send *viewer* :draw-objects))
   (dotimes (i 10)
     (send *robot* :lleg k :joint-angle -3 :relative t)
     (send *viewer* :draw-objects))))

(send-all (send *robot* :joint-list) :joint-angle 0)
(pendulum :crotch-p)
