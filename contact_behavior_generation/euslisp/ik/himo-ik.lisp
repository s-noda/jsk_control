#!/usr/bin/env roseus

(cond
 ((not (and (boundp '*robot*) *robot*))
  (require "package://hrpsys_ros_bridge_tutorials/models/hrp2jsknts.l")
  (setq *robot* (instance hrp2jsknts-robot :init))))

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
      (send *robot* :reset-manip-pose)
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
   (union-link-list (union (flatten link-list) nil))
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
   collision-avoidance-link-pair
   (min #F(-1000 -1000 -1000 -500 -500 -500))
   (max #F(1000 1000 1000 500 500 500))
   ret
   ;;
   (rope-initial-points
    (list (v+ (float-vector 0 -500 500)  (send (send *robot* :get :rarm-elbow-cascoords) :worldpos))
          (v+ (float-vector 300 0 0)  (send (send *robot* :get :rarm-elbow-cascoords) :worldpos))
          (v+ (float-vector -300 0 0)  (send (send *robot* :get :larm-elbow-cascoords) :worldpos))
          (v+ (float-vector 0 500 500)  (send (send *robot* :get :larm-elbow-cascoords) :worldpos))
          ))
   ;;
   (mt2rope-scale 1e-3)
   (rope2mt-scale 1e-2)
   (rope2length-scale 1e-3)
   &allow-other-keys)
  (setq *ik-convergence-user-check* nil)
  (setq
   ret
   (catch :ik-stop
     (send* *robot* :inverse-kinematics
	    target-coords
	    :move-target move-target
	    :link-list link-list
            :union-link-list union-link-list
	    :collision-avoidance-link-pair
	    collision-avoidance-link-pair
	    :null-space
	    #'(lambda nil
                (send-all union-link-list :put :move-angle 0)
		(x::window-main-one)
		(incf cnt)
                (mapcar
                 #'(lambda (pos1 pos2)
                     (send *viewer* :viewsurface :3d-line pos1 pos2))
                 (cdr rope-initial-points) rope-initial-points)
                ;;
                (let* ((rope-points (copy-seq rope-initial-points))
                       buf)
                  (mapcar
                   #'(lambda (mt ll pos pos-buf)
                       (map cons
                            #'(lambda (l dif) (send l :put :move-angle
                                                    (+ (send l :get :move-angle) dif)))
                            ll
                            (scale mt2rope-scale
                                   (transform
                                    (transpose
                                     (send *robot* :calc-jacobian-from-link-list ll
                                           :translation-axis '(t) :rotation-axis '(t) :move-target mt))
                                    (concatenate float-vector
                                                 (send (make-coords :pos pos) :difference-position mt)
                                                 (float-vector 0 0 0)))))
                       ;;
                       (setq buf (send (make-coords :pos pos) :difference-position mt))
                       ;; (send mt :difference-position (make-coords :pos pos)))
                       (dotimes (i 3)
                         (setf (aref pos-buf i)
                               (+ (aref pos-buf i) (* rope2mt-scale (aref buf i)))))
                       )
                   move-target link-list
                   (subseq rope-initial-points 1 (- (length rope-initial-points) 1))
                   (subseq rope-points 1 (- (length rope-points) 1)))
                  ;;
                  (mapcar
                   #'(lambda (from to) (dotimes (i 3) (setf (aref to i) (aref from i))))
                   rope-points rope-initial-points)
                  )
                ;;
                (print (coerce (send-all union-link-list :get :move-angle) float-vector))
                )
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
  (setq *ik-convergence-user-check* t)
  ret
  )


