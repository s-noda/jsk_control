#!/usr/bin/env roseus

(defvar *viewer?* nil)

(require "package://contact_behavior_generation/euslisp/euslib/irteus_proposals/motion-lib-proposal.l")
;; (require "euslib/jsk/jsk.l")
;; (require "robot-state-subscriber.l")

(ros::roseus "constraint_force_observer")
(ros::load-ros-manifest "geometry_msgs")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
(setq *robot* (hrp2jsk))

(defvar *sensor* (instance geometry_msgs::WrenchStamped :init))
(defvar *contact-local-vertical-vector* #F(0 0 1))
(defvar *link-name* "lfsensor")
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
         (instance default-contact-constraint :init
                   :mu-trans *mx*
                   :mu-rot *mz*
                   :l-max-x *lx*
                   :l-min-x *-lx*
                   :l-max-y *ly*
                   :l-min-y *-ly*)))
    (send tmp-contact-constraint :calc-constraint-matrix (make-coords))))
;; (calc-constraint-matrices
;;  (make-coords)
;;  :mu-trans (/ (+ *mx* *my*) 2.0)
;;  :mu-rot *mz*
;;  :l-min-x *-lx* :l-min-y *-ly*
;;  :l-max-x *lx* :l-max-y *ly*))

(defvar *skip-force* -50)
(defvar *last-echo-time* (car (unix::gettimeofday)))

(defun worldforce-transform
  (f)
  (let* ((buf
          (list
           (transform (send (send *link-cascoords* :worldcoords) :worldrot)
                      *contact-local-vertical-vector*)
           #F(0 0 1)))
         (m (matrix-exponent
             (normalize-vector (apply #'v* (reverse buf)))
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
            (send msg :wrench fm xyz)))))

(ros::subscribe "/off_lfsensor"
                geometry_msgs::WrenchStamped
                #'copy-sensor-values *sensor*)
;; (ros::advertise "/constraint_force_observer/status/string"
;; std_msgs::string)
(ros::advertise "/constraint_force_observer/error/string"
                std_msgs::string)
(ros::advertise "/constraint_force_observer/error_vector"
                std_msgs::float32multiarray)

(ros::rate 10)
(do-until-key
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
        "/constraint_force_observer/error_vector"
        (instance std_msgs::float32multiarray :init
                  :data
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
                             (ros::publish "/constraint_force_observer/error/string"
                                           (instance std_msgs::string :init
                                                     :data (format nil "~A(~A) error=~A < 0" *link-name* name (- val min))))
                             ))
                           (- val min))
                       (transform *constraint-matrix* (worldforce-transform f))
                       *force0*
                       (list "-fx" "+fx" "-fy" "+fy" "fz" "-nx" "+nx" "-ny" "+ny" "-nz" "+nz")))))
   ))

