#!/usr/bin/env roseus

(defun copy-virtual-joint-angle
  nil
  (dolist (j (virtual-joint-list))
    (send j :joint-angle
	  (send (send j :get :ref-joint) :joint-angle)))
  )

;; (defun add-6dof-rotational-joints
;;   (&key
;;    (robot *robot*)
;;    (root-link (car (send robot :links)))
;;    (ref-joint (subseq (send robot :lleg :joint-list) 0 6))
;;    ;;
;;    (link-list (list nil))
;;    (rlink root-link)
;;    rlinks
;;    ;;
;;    (org-coords
;;     (send (send (car (last ref-joint)) :child-link)
;; 	  :copy-worldcoords))
;;    )
;;   (dotimes (i (length ref-joint))
;;     (let* ((j0 (nth i ref-joint))
;; 	   (vlink
;; 	    (instance
;; 	     bodyset-link
;; 	     :init (make-cascoords)
;; 	     :bodies (list (make-cube 50 50 50))
;; 	     :name (format nil "~A~A" 'virtual-link_ i)
;; 	     :weight 0 :centroid (float-vector 0 0 0)
;; 	     :inertia-tensor (make-matrix 3 3)))
;; 	   (j (instance rotational-joint :init
;; 			:name (format nil "~A_virtual"
;; 				      (send j0 :name))
;; 			:axis (scale 1 (send j0 :get-val 'axis))
;; 			:min (send j0 :min-angle)
;; 			:max (send j0 :max-angle)
;; 			:child-link rlink
;; 			;;(if (eq rlink root-link) robot rlink)
;; 			:parent-link vlink))
;; 	   )
;;       ;; (send vlink :transform (send (send j0 :child-link) :copy-worldcoords))
;;       ;; (if (not (eq rlink root-link))
;;       (send vlink :assoc rlink)
;;       (send j :put :ref-joint j0)
;;       ;; (send vlink :transform (send j :get-val 'default-coords)) = (send (send rlink :transform (send j0 :get-val 'default-coords)) :transformation (make-coords))
;;       (send j :set-val 'default-coords
;; 	    (send (send j0 :get-val 'default-coords)
;; 		  :transformation (make-coords)))
;;       (send j :joint-angle (send j0 :joint-angle))
;;       (send rlink :add-joint j)
;;       (send rlink :add-parent-link vlink)
;;       (send vlink :add-child-links rlink)
;;       (push rlink rlinks)
;;       (setq rlink vlink)))
;;   (send robot :put :root-link-org root-link)
;;   (send robot :put :links-org (send robot :links))
;;   (send rlink :newcoords org-coords)
;;   (send robot :set-val 'links
;; 	(append (cons rlink rlinks)
;; 		(remove root-link (send robot :links))))
;;   (send-all (send robot :joint-list) :joint-angle 0 :relative t)
;;   (send-all (send robot :links) :worldcoords)
;;   (setq root-link (send root-link :parent-link))
;;   ;; (send robot :set-val 'joint-list (flatten (send-all (send robot :links) :joint)))
;;   (list
;;    (cons :6dof-links
;;          (mapcar #'(lambda (l) (append rlinks l)) link-list))
;;    (cons :del-6dof-links
;;          (eval
;;           (list 'function
;;                 (list 'lambda nil
;; 		      (list 'send (send root-link :parent-link) :del-child-link root-link)
;;                       (list 'send root-link :del-joint)
;;                       (list 'send root-link :del-parent-link))))))
;;   )

(defun add-6dof-rotational-joints
  (&key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (ref-joint (subseq (send robot :lleg :joint-list) 0 6))
   ;;
   (link-list (list nil))
   (rlink
    (instance
     bodyset-link
     :init (make-cascoords)
     :bodies (list (make-cube 50 50 50))
     :name (format nil "~A" 'virtual-root-link)
     :weight 0 :centroid (float-vector 0 0 0)
     :inertia-tensor (make-matrix 3 3)))
   rlinks
   ;;
   (org-coords
    (send (send (car (last ref-joint)) :child-link)
	  :copy-worldcoords))
   (org-av (copy-seq (send robot :angle-vector)))
   (jc0)
   )
  (send rlink :newcoords
	(send org-coords :copy-worldcoords))
  (send robot :init-pose)
  (send-all (send robot :links) :worldcoords)
  (dolist (j0 (reverse ref-joint))
    (if (not jc0) (setq jc0 (send (send (send j0 :parent-link) :copy-worldcoords) :transform (send j0 :get-val 'default-coords))))
    (let* ((jc
	    (send (send (send j0 :parent-link)
			:copy-worldcoords)
		  :transform
		  (send j0 :get-val 'default-coords)))
	   (vlink
	    (if (eq j0 (car ref-joint))
		root-link
	      (instance
	       bodyset-link
	       :init (make-cascoords)
	       :bodies (list (make-cube 50 50 50))
	       :name (format nil "~A_virtual_link"
			     (send j0 :name))
	       :weight 0 :centroid (float-vector 0 0 0)
	       :inertia-tensor (make-matrix 3 3))))
	   (j (instance rotational-joint :init
			:name
			(format nil "~A_virtual_joint"
				(send j0 :name))
			:axis (scale -1 (send j0 :get-val 'axis))
			:min (send j0 :min-angle)
			:max (send j0 :max-angle)
			:child-link vlink
			:parent-link rlink))
	   )
      (send rlink :assoc vlink)
      (send j :put :ref-joint j0)
      (send j :set-val 'default-coords
	    (send jc0 :transformation jc))
      (if (eq (car ref-joint) j0)
	  (send (send j :get-val 'default-coords)
		:transform
		(send
		 (send j0 :get-val 'default-coords)
		 :transformation (make-coords))))
      ;; (send (send j0 :get-val 'default-coords)
      ;; :transformation (make-coords)))
      (send j :joint-angle (send j0 :joint-angle))
      (send vlink :add-joint j)
      (send vlink :add-parent-link rlink)
      (send rlink :add-child-links vlink)
      (push rlink rlinks)
      (setq rlink vlink) (setq jc0 jc)))
  (push rlink rlinks)
  (setq rlinks (reverse rlinks))
  (send robot :put :root-link-org root-link)
  (send robot :put :links-org (send robot :links))
  (send robot :set-val 'links
	(append rlinks (send robot :links)))
  (send robot :angle-vector org-av)
  (copy-virtual-joint-angle)
  (send-all (send robot :links) :worldcoords)
  (setq root-link (send root-link :parent-link))
  ;; (send robot :set-val 'joint-list (flatten (send-all (send robot :links) :joint)))
  (list
   (cons :6dof-links
         (mapcar #'(lambda (l) (append (cdr rlinks) l)) link-list))
   (cons :del-6dof-links
         (eval
          (list 'function
                (list 'lambda nil
		      (list 'send (send root-link :parent-link) :del-child-link root-link)
                      (list 'send root-link :del-joint)
                      (list 'send root-link :del-parent-link))))))
  )

(defun demo-rotational-6dof
  nil
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (defvar *robot* (instance hrp2jsknt-robot :init))
  (defvar *robot-org* *robot*)
  (setq *robot* (copy-object *robot-org*))
  (objects (list *robot*))
  (setq *v6dof* (add-6dof-rotational-joints))
  ;;
  ;; (send *robot* :reset-pose)
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

(defun test-random-lleg-angle-vector
  nil
  (mapcar
   '(lambda (j) (send j :joint-angle (+ (* (random 1.0) (- (send j :max-angle) (send j :min-angle))) (send j :min-angle))))
   (send *robot* :lleg :joint-list))
  (send-all (send *robot* :links) :worldcoords)
  (copy-virtual-joint-angle)
  (send *viewer* :draw-objects))

(defun virtual-joint-list
  nil (send-all (send *robot* :link-list (send *robot* :get :root-link-org)) :joint))

(defun draw-virtual-links
  nil (send-all (send-all (virtual-joint-list) :parent-link) :draw-on :flush t :color (float-vector 1 0 0)))

(defun draw-lleg-links
  nil (send-all (send-all (send *robot* :lleg :links) :worldcoords) :draw-on :flush t :color (float-vector 0 1 0)))

(defun reset-pose
  nil
  (send *robot* :reset-pose)
  (copy-virtual-joint-angle)
  )

(defun squwat-pose
  nil
  (send *robot* :legs :move-end-pos (float-vector 0 0 50))
  (copy-virtual-joint-angle))

(defun squwat-pose2
  nil
  (reset-pose)
  (squwat-pose)
  (dotimes (i 10)
    (send *robot* :inverse-kinematics
	  (send (send (send *robot* :link "BODY") :copy-worldcoords) :translate (float-vector 0 0 -10))
	  :move-target
	  (or
	   (send *robot* :get :body-end-coords)
	   (send *robot* :put :body-end-coords
		 (make-cascoords
		  :name "body_link"
		  :parent (send *robot* :link "BODY")
		  :coords (send (send *robot* :link "BODY") :copy-worldcoords))))
	  :link-list
	  (send *robot* :link-list (send (send *robot* :get :body-end-coords) :parent))
	  :debug-view :no-message
	  :stop 10
	  )))


(cond
 ((and (boundp '*robot*) *robot*)
  ;; (defvar *robot-org* *robot*)
  ;; (setq *robot* (copy-object *robot-org*))
  (setq *rotational-6dof-fix-leg* :new)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :both)
  (objects (list *robot*))
  (setq *v6dof* (add-6dof-rotational-joints))
  ;; (send-all (send-all (cadr (assoc :6dof-links *v6dof*)) :joint) :joint-angle 0 :relative t)
  ;; (send-all (send *robot* :links) :worldcoords)
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
	    ;;
	    min max
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
			(send self :link-list (send self :get :root-link-org)) root-link-virtual-joint-weight)
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
(test-torque-ik :stop 50)
(load "rotational-6dof.lisp")
(send-all (send *robot* :joint-list) :put :local-axis-vector nil)
(send-all (send *robot* :joint-list) :local-axis-vector)
(reset-pose)
(test-torque-ik :init nil :gain nil :null-max 3)

(test-torque-ik :init nil)
(test-torque-ik :stop 50 :gain 100 :init nil)

(test-torque-ik :key-list '(:rleg :lleg) :init nil :null-max (* (rad2deg 1) 0.1) :gain (* (rad2deg 1) 1e-4) :null-space nil)


(require "package://euslisp/jskeus/irteus/irtmodel.l")
(require "package://euslisp/jskeus/irteus/irtrobot.l")
(require "package://euslisp/jskeus/irteus/irtdyna.l")

