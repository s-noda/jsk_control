
(require "package://gazebo_drive_simulator/euslisp/misumi-ranger-xp900-chair.lisp")

(defclass car-chair
  :super misumi-ranger-xp900-chair ;;cascaded-link
  :slots (name
	  seat-plane back-plane
	  seat-height seat-width
	  ))
(defmethod car-chair
  (:init
   (&rest
    args
    &key
    ((:name nm) :car-chair)
    ((:seat-heigth sh) (float-vector 0 0 520))
    ((:seat-width sw) 460)
    &allow-other-keys)
   (send-super* :init :mode :double args)
   (setq seat-plane (nth 0 (send self :bodies)))
   (setq back-plane (nth 9 (send self :bodies)))
   (setq name nm)
   (setq seat-height sh)
   (setq seat-width sw)
   (send self :translate seat-height :world)
   self
   )
  (:back-plane-vertical-vector
   nil
   (transform (send back-plane :worldrot) #F(1 0 0)))
  (:seat-plane-vertical-vector
   nil
   (transform (send seat-plane :worldrot) #F(0 0 1)))
  (:calc-dist-from-seat-plane
   (pos &key
	(min-ccl 0)
	(vv (send self :seat-plane-vertical-vector))
	(a (send seat-plane :worldpos))
	(xv (transform (send seat-plane :worldrot) #F(1 0 0)))
	(yv (transform (send seat-plane :worldrot) #F(0 1 0)))
	(x (abs (v. xv (v- pos a))))
	(y (abs (v. yv (v- pos a))))
	(w 230) (h 500))
   (if (and (< x w) (< y h))
       (- (* -1 (v. vv (v- pos a))) min-ccl)
     1e+6)
   )
  (:calc-dist-from-back-plane
   (pos &key
	(min-ccl 0)
	(vv (send self :seat-back-vertical-vector))
	(a (send back-plane :worldpos))
	(xv (transform (send back-plane :worldrot) #F(0 0 1)))
	(yv (transform (send back-plane :worldrot) #F(0 1 0))))
   (send self :calc-dist-from-seat-plane pos
	 :min-ccl min-ccl :vv vv :a a :xv xv :yv yv))
  (:vertical-vector nil (send self :seat-plane-vertical-vector))
  (:gen-collision-check-list
   nil
   (list
    (list (cons :name name)
	  (cons :n (scale -1 (send self :vertical-vector)))
	  (cons :a0 (send seat-plane :worldpos))
	  (cons :check-func
		(list 'lambda '(env ccl)
		      (list 'let
			    '((x (funcall (cdr (assoc :dist ccl)))) (slf self))
			    '(min (send slf :calc-dist-from-seat-plane x)
				  (send slf :calc-dist-from-back-plane x))))))))
  (:gen-contact-states nil nil)
  )

;; (setq a (instance car-chair :init))
;; (objects (list a))

;; (mapcar #'(lambda (l) (send *viewer* :draw-objects) (send l :draw-on :flush t) (read-line)) (send a :bodies))
