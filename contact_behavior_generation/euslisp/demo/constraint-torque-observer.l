#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "constraint_torque_observer")

(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
(setq *robot* (hrp2jsk))

(defvar *torque-constraint-rate* 0.7)
(defvar *last-echo-time* (car (unix::gettimeofday)))

(defun joint-state-callback
  (msg)
  (map cons
       #'(lambda (name tau)
	   (let* ((j (find-if #'(lambda (j) (string-equal (send j :name) name))
			      (send *robot* :joint-list))))
	     (cond
	      (j (send j :put :joint-torque tau)
		 (cond
                  ((> (abs tau) (* *torque-constraint-rate*
                                   (send j :max-joint-torque)))
                   (cond
                    ((> (- (car (unix::gettimeofday)) *last-echo-time*)
                        1)
                     (setq *last-echo-time* (car (unix::gettimeofday)))
                     (warning-message 1 "~A tau overthre ~A(~A)~%"
                                      (send j :name) (abs tau) *torque-constraint-rate*)))
                   (ros::publish "/constraint_torque_observer/error/string"
                                 (instance std_msgs::string :init
                                           :data (format nil "~A" name))))))
	      ((not (find-if #'(lambda (nm) (string-equal nm name))
			     *black-list-joint-name*))
	       (push name *black-list-joint-name*)
	       (format t "[~A] unknow joint name=~A~%"
		       (ros::get-name) name))
	      (t 'nop))))
       (send msg :name)
       (send msg :effort))
  )

(ros::subscribe "/joint_states" sensor_msgs::jointstate
		#'joint-state-callback 1)
(ros::advertise "/constraint_torque_observer/error/string"
		std_msgs::string)

(ros::rate 100)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep)
 )

