#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(require "package://hrpsys_gazebo_tutorials/euslisp/hrpsys-gazebo-utils.l")

(require "four-leg-seat.l")

(let* ((model (instance four-leg-seat :init)))
  (eus2urdf-for-gazebo model)
  ;; (send model :name
  ;; 	(remove #\: (format nil "~A" (send model :name))))
  ;; (collada::eus2collada model ".") ;;(format nil "~A.dae" (send model :name)))
  )

(exit 0)

