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

(defun calc-jacobian-rotational
  (fik row column joint paxis child-link world-default-coords
       child-reverse
       move-target transform-coords rotation-axis translation-axis
       tmp-v0 tmp-v1 tmp-v2 tmp-v3 tmp-v3a tmp-v3b tmp-m33)
  (let* ((j-rot (calc-jacobian-default-rotate-vector
                 paxis world-default-coords child-reverse
		 transform-coords tmp-v3 tmp-m33))
         (p-diff (scale 1e-3
                        (transform (transpose (send transform-coords :worldrot) tmp-m33)
                                   (v- (send move-target :worldpos)
				       (send world-default-coords :worldpos)
				       tmp-v3a)
                                   tmp-v3a) tmp-v3a))
         (j-trs (v* j-rot p-diff tmp-v3b)))
    (setq j-trs (calc-dif-with-axis j-trs translation-axis tmp-v0 tmp-v1 tmp-v2))
    (dotimes (j (length j-trs)) (setf (aref fik (+ j row) column) (elt j-trs j)))
    (setq j-rot (calc-dif-with-axis j-rot rotation-axis tmp-v0 tmp-v1 tmp-v2))
    (dotimes (j (length j-rot)) (setf (aref fik (+ j row (length j-trs)) column) (elt j-rot j)))
    ))

(defun calc-inertia-matrix-rotational
  (mat row column
       paxis m-til c-til I-til axis-for-angular child-link world-default-coords
       translation-axis rotation-axis
       tmp-v0 tmp-v1 tmp-v2 tmp-va tmp-vb tmp-vc tmp-vd tmp-m)
  (let* ((ax (normalize-vector (send world-default-coords :rotate-vector paxis tmp-va) tmp-va))
         (mt (* 1e-3 m-til))
         (ct (scale 1e-3 c-til tmp-vb))
         (It (scale-matrix 1e-9 I-til tmp-m))
         (mj (scale mt
                    (v* ax (v- ct (scale 1e-3 (send world-default-coords :worldpos) tmp-vc) tmp-vc)
                        tmp-vd) tmp-vc))
         (hj (v- (v+ (v* ct mj tmp-vd) (transform It ax tmp-vb) tmp-vb)
                 (v* (scale 1e-3 axis-for-angular tmp-va) mj tmp-vd) tmp-vb)))
    (let ((mv (calc-dif-with-axis mj translation-axis tmp-v0 tmp-v1 tmp-v2)))
      (dotimes (i (length mv)) (setf (aref mat (+ row i) column) (elt mv i)))
      (let ((hv (calc-dif-with-axis hj rotation-axis tmp-v0 tmp-v1 tmp-v2)))
        (dotimes (i (length hv)) (setf (aref mat (+ row i (length mv)) column) (elt hv i)))
        ))))

(defmethod rotational-joint
  (:calc-spacial-velocity-jacobian
   (ax tmp-va tmp-vb)
   (v* (scale 0.001
	      (send (send self :joint-worldcoords) :worldpos)
	      tmp-va) ax tmp-vb))
  )


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
	    ;; ((find to (send self *rotational-6dof-fix-leg* :links))
	    ;;  ;; (format t "~A detected in ~A~%" (send to :name) *rotational-6dof-fix-leg*)
	    ;;  (let* ((ret) (l (car (flatten (send to :child-links))))
	    ;; 	    (root-links
	    ;; 	     (list from (send self *rotational-6dof-fix-leg* :end-coords :parent) (send self :get :root-link-org))))
	    ;;    (while (and (not (find l root-links)) (send l :child-links))
	    ;; 	 (push l ret)
	    ;; 	 (setq l (car (flatten (send l :child-links))))
	    ;; 	 )
	    ;;    ;; (setq ret (reverse ret))
	    ;;    (if (and l (send l :joint)) (push l ret))
	    ;;    ret))
	    (t (send-super :link-list to from))))
	 '(:fullbody-inverse-kinematics
	   (target-coords
	    &rest args
	    &key
	    (move-target) (link-list)
	    (target-centroid-pos (apply #'midpoint 0.5 (send self :legs :end-coords :worldpos)))
	    (cog-gain 2.0)
	    (centroid-thre 5.0)
	    additional-weight-list
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
		  :additional-weight-list
		  (append
		   additional-weight-list
		   (remove nil
			   (list (if (send self :rleg :toe-p)
				     (list (send self :rleg :toe-p :child-link) 0))
				 (if (send self :lleg :toe-p)
				     (list (send self :lleg :toe-p :child-link) 0)))))
		  args)
	   ))
   )
  ;;
  ))




#|

(send *robot* :legs :toe-p :min-angle 0)
(send *robot* :legs :toe-p :max-angle 0)

roseus human-ball-test.lisp
(test-torque-ik)
(load "rotational-6dof.lisp")
(send-all (send *robot* :joint-list) :put :local-axis-vector nil)
(send-all (send *robot* :joint-list) :local-axis-vector)
(test-torque-ik :gain 1e-4 :key-list '(:rleg))
