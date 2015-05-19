#!/usr/bin/env roseus

(defvar *elbow-end-coords*
  (mapcar
   #'(lambda (k name)
       (send
        *robot* :put name
        (make-cascoords
         :name name
         :parent (send *robot* k :elbow-p :child-link)
         :coords
         (send (send (send *robot* k :elbow-p :child-link) :copy-worldcoords)
               :translate (float-vector 0 0 -100) :local))))
   '(:rarm :larm) '(:rarm-elbow-cascoords :larm-elbow-cascoords)))
(defvar *ik-convergence-user-check* t)

(defmethod cascaded-link
  (:ik-convergence-check
   (success dif-pos dif-rot
    rotation-axis translation-axis thre rthre
    centroid-thre target-centroid-pos
    centroid-offset-func cog-translation-axis
    &key (update-mass-properties t) (success-buf t))
   (dotimes (i (length dif-pos))
     (setq success-buf
	   (and success-buf
		(if (elt translation-axis i)
		    (< (norm (elt dif-pos i)) (elt thre i)) t)
		(if (elt rotation-axis i)
		    (< (norm (elt dif-rot i)) (elt rthre i)) t))))
   (if target-centroid-pos
       (setq success-buf
	     (and
	      success-buf
	      (send self :cog-convergence-check
		    centroid-thre target-centroid-pos
		    :centroid-offset-func centroid-offset-func
		    :translation-axis cog-translation-axis
		    :update-mass-properties update-mass-properties))))
   (and *ik-convergence-user-check* success success-buf)))

(defun test-himo-ik
  (&rest
   args
   &key
   (tm (instance mtimer :init))
   (init
    (progn
      (cond
       ((not (and (boundp '*robot*) *robot*))
        (require "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
        (setq *robot* (instance hrp2jsknts-robot :init))))
      (cond
       ((not (and (boundp '*viewer*) *viewer*))
        (objects (list *robot*))))
      (send *robot* :reset-pose)
      (send *robot* :newcoords (make-coords))
      (send-all (send *robot* :links) :worldcoords)
      (send *viewer* :draw-objects)))
   (stop 50)
   (key-list '(:rarm :larm))
   (move-target
    (mapcar #'(lambda (k)
                (cond
                 ((eq k :rarm) (send *robot* :get :rarm-elbow-cascoords))
                 ((eq k :larm) (send *robot* :get :larm-elbow-cascoords))
                 (t (send *robot* k :end-coords))))
            key-list))
   (link-list
    (mapcar #'(lambda (mt) (send *robot* :link-list (send mt :parent)))
	    move-target))
   (target-coords (send-all move-target :copy-worldcoords))
   (translation-axis (make-list (length move-target) :initial-element t))
   (rotation-axis
    (mapcar #'(lambda (k)
                (cond
                 ((eq k :rarm) nil)
                 ((eq k :larm) nil)
                 (t t)))
            key-list))
   (thre (mapcar #'(lambda (k) 50) key-list))
   (rthre (mapcar #'(lambda (k) (deg2rad 12)) key-list))
   (root-link-virtual-joint-weight #F(0.1 0.1 0.1 0.1 0.1 0.1))
   (avoid-weight-gain 0.01)
   (cnt -1)
   (debug-view :no-message)
   ;;
   (best-rsd nil)
   (now-rsd nil)
   collision-avoidance-link-pair
   (min #F(-1000 -1000 -1000 -500 -500 -500))
   (max #F(1000 1000 1000 500 500 500))
   ret
   &allow-other-keys)
  (setq
   ret
   (catch :ik-stop
     (send* *robot* :fullbody-inverse-kinematics
	    target-coords
	    :move-target move-target
	    :link-list link-list
	    :collision-avoidance-link-pair
	    collision-avoidance-link-pair
	    :null-space
	    #'(lambda nil
		(x::window-main-one)
		(incf cnt)
                )
	    :target-centroid-pos target-centroid-pos
	    :min min :max max
	    :avoid-weight-gain avoid-weight-gain
	    :translation-axis translation-axis
	    :rotation-axis rotation-axis
	    :debug-view debug-view
	    :stop stop
	    :thre thre :rthre rthre
	    :root-link-virtual-joint-weight
	    root-link-virtual-joint-weight
	    args
	    )))
  (format t "    time: ~A~%" (send tm :stop))
  ret
  )


