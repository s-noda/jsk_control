#!/usr/bin/env roseus

(require "package://contact_behavior_generation/euslisp/euslib/irteus_proposals/motion-lib-proposal.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
;; (require "euslib/jsk/jsk.l")
;; (require "robot-state-subscriber.l")

(ros::roseus "constraint_force_observer")
(ros::load-ros-manifest "geometry_msgs")

(defvar *force-observer-limb* :rleg)
(defvar *force-observer-package-name*
  (format nil "~A_SENSOR_OBSERVER" (send *force-observer-limb* :pname)))
(if (not (find-package *force-observer-package-name*))
    (make-package *force-observer-package-name*))
(In-package *force-observer-package-name*)

(defvar *viewer?* nil)

(if (and (boundp 'user::*robot*) user::*robot*)
    (setq *robot* user::*robot*))
(cond
 ((not (and (boundp '*robot*) *robot*))
  (setq *robot* (user::hrp2jsk))))

(defvar *sensor* (instance geometry_msgs::WrenchStamped :init))
(defvar *contact-local-vertical-vector* #F(0 0 1))
(defvar *link-name*
  (case user::*force-observer-limb*
        (:lleg "lfsensor")
        (:rleg "rfsensor")
        (:larm "lhsensor")
        (:rarm "rhsensor")))
(defvar *link-cascoords*
  (find-if #'(lambda (l) (string-equal *link-name* (send l :name)))
           (append (send *robot* :links)
                   (send *robot* :force-sensors))))

(defvar *min-force* #F(0 0 0 0 0 0))
(defvar *force0*
  (float-vector (aref *min-force* 0) (aref *min-force* 0)
                (aref *min-force* 1) (aref *min-force* 1)
                (aref *min-force* 5) (aref *min-force* 5)
                (aref *min-force* 4) (aref *min-force* 4)
                (aref *min-force* 3) (aref *min-force* 3)
                (aref *min-force* 2)))
(defvar *lx* 50)
(defvar *ly* 50)
(defvar *-lx* -50)
(defvar *-ly* -50)
(defvar *mx* 0.9)
(defvar *my* 0.9)
(defvar *mz* 0.9)
(defvar *constraint-matrix*
  (let ((tmp-contact-constraint
         (instance user::default-contact-constraint :init
                   :mu-trans *mx*
                   :mu-rot *mz*
                   :l-max-x *lx*
                   :l-min-x *-lx*
                   :l-max-y *ly*
                   :l-min-y *-ly*)))
    (send tmp-contact-constraint :calc-constraint-matrix (user::make-coords))))
;; (calc-constraint-matrices
;;  (make-coords)
;;  :mu-trans (/ (+ *mx* *my*) 2.0)
;;  :mu-rot *mz*
;;  :l-min-x *-lx* :l-min-y *-ly*
;;  :l-max-x *lx* :l-max-y *ly*))

(defvar *skip-force* 50)
(defvar *last-echo-time* (car (unix::gettimeofday)))
(defvar *error-vector*)

(defun update-friction
  (mx my mz)
  (setq *lx* 50)
  (setq *ly* 50)
  (setq *-lx* -50)
  (setq *-ly* -50)
  (setq *mx* 0.9)
  (setq *my* 0.9)
  (setq *mz* 0.9)
  (setq *constraint-matrix*
        (let ((tmp-contact-constraint
               (instance default-contact-constraint :init
                         :mu-trans *mx*
                         :mu-rot *mz*
                         :l-max-x *lx*
                         :l-min-x *-lx*
                         :l-max-y *ly*
                         :l-min-y *-ly*)))
          (send tmp-contact-constraint :calc-constraint-matrix (user::make-coords)))))

(defun worldforce-transform
  (f)
  (let* ((buf
          (list
           (transform (send (send *link-cascoords* :worldcoords) :worldrot)
                      *contact-local-vertical-vector*)
           #F(0 0 1)))
         (m (user::matrix-exponent
             (user::normalize-vector (apply #'v* (reverse buf)))
             (acos (apply #'v. buf))))
         (ret
          (concatenate float-vector
                       (transform m (subseq f 0 3))
                       (transform m (subseq f 3 6)))))
    ;; (format t "--- ~A --> ~A~%" f ret)
    ret))

(defun copy-sensor-values
  (buf msg)
  (dolist (fm '(:force :torque))
    (dolist (xyz '(:x :y :z))
      (send buf :wrench fm xyz
            (send msg :wrench fm xyz))))
  ;;
  (let* ((f (float-vector (send msg :wrench :force :x)
                          (send msg :wrench :force :y)
                          (send msg :wrench :force :z)
                          (send msg :wrench :torque :x)
                          (send msg :wrench :torque :y)
                          (send msg :wrench :torque :z))))
    (if (< (norm f) *skip-force*)
        (setq *error-vector* nil)
      (setq *error-vector*
            (map float-vector
                 #'(lambda (val min name)
                     (cond
                      ((< val min)
                       (cond
                        ((> (- (car (unix::gettimeofday)) *last-echo-time*)
                            1)
                         (setq *last-echo-time* (car (unix::gettimeofday)))
                         (warning-message 1 "[~A] ~A = ~A > ~A ? ~A~%"
                                          (ros::get-name) name
                                          val min (> val min))))
                       (ros::publish (format nil "/constraint_force_observer/~A/error/string"
                                             (remove #\: (format nil "~A" user::*force-observer-limb*)))
                                     (instance std_msgs::string :init
                                               :data (format nil "~A(~A) error=~A < 0" *link-name* name (- val min))))
                       ))
                     (- val min))
                 (transform *constraint-matrix* (worldforce-transform f))
                 *force0*
                 (list "-fx" "+fx" "-fy" "+fy" "fz" "-nx" "+nx" "-ny" "+ny" "-nz" "+nz"))))))

(ros::subscribe
 (case user::*force-observer-limb*
       (:lleg "/off_lfsensor")
       (:rleg "/off_rfsensor")
       (:larm "/off_lhsensor")
       (:rarm "/off_rhsensor"))
 geometry_msgs::WrenchStamped
 #'copy-sensor-values *sensor*)
;; (ros::advertise "/constraint_force_observer/status/string"
;; std_msgs::string)
(ros::advertise (format nil "/constraint_force_observer/~A/error/string"
                        (remove #\: (format nil "~A" user::*force-observer-limb*)))
                std_msgs::string)
(ros::advertise (format nil "/constraint_force_observer/~A/error_vector"
                        (remove #\: (format nil "~A" user::*force-observer-limb*)))
                std_msgs::float32multiarray)

(defun main
  nil
  (ros::rate 10)
  (user::do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (ros::spin-once)
   (ros::sleep)
   (let* ((f (float-vector (send *sensor* :wrench :force :x)
                           (send *sensor* :wrench :force :y)
                           (send *sensor* :wrench :force :z)
                           (send *sensor* :wrench :torque :x)
                           (send *sensor* :wrench :torque :y)
                           (send *sensor* :wrench :torque :z))))
     (if (> (norm f) *skip-force*)
         (ros::publish
          (format nil "/constraint_force_observer/~A/error_vector"
                  (remove #\: (format nil "~A" user::*force-observer-limb*)))
          (instance std_msgs::float32multiarray :init
                    :data *error-vector*))))
   ))
