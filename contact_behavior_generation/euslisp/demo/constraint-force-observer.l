#!/usr/bin/env roseus

(require "package://contact_behavior_generation/euslisp/euslib/irteus_proposals/motion-lib-proposal.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
;; (require "euslib/jsk/jsk.l")
;; (require "robot-state-subscriber.l")

(defvar *force-observer-limb* :rleg)
(setq *force-observer-package-name*
      (format nil "~A_SENSOR_OBSERVER" (send *force-observer-limb* :pname)))

(ros::roseus "constraint_force_observer")
(ros::load-ros-manifest "geometry_msgs")

(if (not (find-package *force-observer-package-name*))
    (make-package *force-observer-package-name*))
(In-package *force-observer-package-name*)

(defvar *viewer?* nil)
(defvar *force-observer-limb* user::*force-observer-limb*)

(if (and (boundp 'user::*robot*) user::*robot*)
    (setq *robot* user::*robot*))
(cond
 ((not (and (boundp '*robot*) *robot*))
  (setq *robot* (user::hrp2jsk))))

(defvar *sensor* (instance geometry_msgs::WrenchStamped :init))
(defvar *contact-local-vertical-vector* #F(0 0 1))
(defvar *link-name*
  (case *force-observer-limb*
        (:lleg "lfsensor")
        (:rleg "rfsensor")
        (:larm "lhsensor")
        (:rarm "rhsensor")
        (t "sensor")))
(defvar *link-cascoords*
  (or
   (find-if #'(lambda (l) (string-equal *link-name* (send l :name)))
            (append (send *robot* :links)
                    (send *robot* :force-sensors)))
   (make-cascoords)))

(defvar *min-force* #F(0 0 0 0 0 0))
(defvar *force0*
  (float-vector (aref *min-force* 0) (aref *min-force* 0)
                (aref *min-force* 1) (aref *min-force* 1)
                (aref *min-force* 5) (aref *min-force* 5)
                (aref *min-force* 4) (aref *min-force* 4)
                (aref *min-force* 3) (aref *min-force* 3)
                (aref *min-force* 2)))
(defvar *lx* 100)
(defvar *ly* 100)
(defvar *-lx* -100)
(defvar *-ly* -100)
(defvar *mx* 0.4)
(defvar *my* 0.4)
(defvar *mz* 0.4)
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

(defvar *skip-force* 30)
(defvar *last-echo-time* (car (unix::gettimeofday)))
(defvar *error-vector*)
(defvar *off-error-vector*)
(defvar *error-vector-old*)

(defun update-friction
  (mx my mz)
  (setq *lx* 100)
  (setq *ly* 100)
  (setq *-lx* -100)
  (setq *-ly* -100)
  (setq *mx* mx)
  (setq *my* my)
  (setq *mz* mz)
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

(defun update-error-vector
  (msg
   &key
   (f (float-vector (send msg :wrench :force :x)
                    (send msg :wrench :force :y)
                    (send msg :wrench :force :z)
                    (send msg :wrench :torque :x)
                    (send msg :wrench :torque :y)
                    (send msg :wrench :torque :z)))
   (off-f (scale 0 f))
   )
  (if (< (norm f) *skip-force*)
      (setq *error-vector* nil)
    (progn
      (setq *error-vector*
            (let* ((_f (worldforce-transform f))
                   (fz (max (aref _f 2) 1e-3)))
              (float-vector (/ (aref _f 0) (* *mx* fz))
                            (/ (aref _f 1) (* *my* fz))
                            0
                            (/ (aref _f 3) (* *ly* fz))
                            (/ (aref _f 4) (* *lx* fz))
                            (/ (aref _f 5) (* *mz* fz)))))
      ;;
      (if off-f
          (setq *off-error-vector*
                (let* ((_f (worldforce-transform f))
                       (_off-f (worldforce-transform off-f))
                       (fz (max (aref _f 2) 1e-3)))
                  (float-vector (/ (- (aref _f 0) (aref _off-f 0)) (* *mx* fz))
                                (/ (- (aref _f 1) (aref _off-f 1)) (* *my* fz))
                                0
                                (/ (- (aref _f 3) (aref _off-f 3)) (* *ly* fz))
                                (/ (- (aref _f 4) (aref _off-f 4)) (* *lx* fz))
                                (/ (- (aref _f 5) (aref _off-f 5))  (* *mz* fz))))))
      ;;
      (setq *error-vector-old*
            (map float-vector
                 #'(lambda (val min name)
                     (cond
                      ((< val min)
                       (cond
                        ((> (- (car (unix::gettimeofday)) *last-echo-time*)
                            1)
                         (setq *last-echo-time* (car (unix::gettimeofday)))
                         (warning-message 1 "[~A] ~A = ~A > ~A ? ~A~%"
                                          ;;(ros::get-name)
                                          *force-observer-limb*
                                          name
                                          val min (> val min))))
                       (ros::publish (format nil "/constraint_force_observer/~A/error/string"
                                             (remove #\: (format nil "~A" *force-observer-limb*)))
                                     (instance std_msgs::string :init
                                               :data (format nil "~A(~A) error=~A < 0" *link-name* name (- val min))))
                       ))
                     (- val min)
                     )
                 (transform *constraint-matrix* (worldforce-transform f))
                 *force0*
                 (list "-fx" "+fx" "-fy" "+fy" "fz" "-nx" "+nx" "-ny" "+ny" "-nz" "+nz")))
      )))

(defun copy-sensor-values
  (buf msg)
  (dolist (fm '(:force :torque))
    (dolist (xyz '(:x :y :z))
      (send buf :wrench fm xyz
            (send msg :wrench fm xyz))))
  ;;
  (update-error-vector msg))

(ros::subscribe
 (case *force-observer-limb*
       (:lleg "/off_lfsensor")
       (:rleg "/off_rfsensor")
       (:larm "/off_lhsensor")
       (:rarm "/off_rhsensor")
       (t "/hoge"))
 geometry_msgs::WrenchStamped
 #'copy-sensor-values *sensor*)
;; (ros::advertise "/constraint_force_observer/status/string"
;; std_msgs::string)
(ros::advertise (format nil "/constraint_force_observer/~A/error/string"
                        (remove #\: (format nil "~A" *force-observer-limb*)))
                std_msgs::string)
(ros::advertise (format nil "/constraint_force_observer/~A/error/vector"
                        (remove #\: (format nil "~A" *force-observer-limb*)))
                std_msgs::float32multiarray)
(ros::advertise (format nil "/constraint_force_observer/~A/off_error/vector"
                        (remove #\: (format nil "~A" *force-observer-limb*)))
                std_msgs::float32multiarray)

(defun publish-error-vector
  nil
  (if *error-vector*
      (ros::publish
       (format nil "/constraint_force_observer/~A/error/vector"
               (remove #\: (format nil "~A" *force-observer-limb*)))
       (instance std_msgs::float32multiarray :init
                 :data *error-vector*)))
  (if *off-error-vector*
      (ros::publish
       (format nil "/constraint_force_observer/~A/off_error/vector"
               (remove #\: (format nil "~A" *force-observer-limb*)))
       (instance std_msgs::float32multiarray :init
                 :data *off-error-vector*))))

(defun main
  nil
  (ros::rate 10)
  (user::do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (ros::spin-once)
   (ros::sleep)
)
  )
