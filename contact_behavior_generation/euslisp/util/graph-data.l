
(defun in-range?
  (min max data)
  (if (and
       (v> (v- data min) (scale 0 data))
       (v> (v- max data) (scale 0 data)))
      t
    nil))

(defun map-filter
  (list func)
  (if (null list)
      nil
    (if (funcall func (car list))
	(cons (car list) (map-filter (cdr list) func))
      (map-filter (cdr list) func))))

(defclass graph-data
  :super object
  :slots (col dat max-len nam))

(defmethod graph-data
  (:init
   (&key (color #x0) (data nil) (name "nothing") (max-length 100))
   (progn
     (setq col color)
     (setq nam name)
     (setq dat data)
     (setq max-len max-length)))
  (:name
   (&optional name)
   (if name (setq nam name) nam))
  (:color
   (&optional color)
   (if color (setq col color) col))
  (:data
   (&optional data)
   (if data (setq dat data) dat))
  (:data-delete
   nil
   (setq dat nil))
  (:max-length
   (&optional max)
   (if max (setq max-len max) max-len))
  (:add
   (d)
   (progn
     (push d dat)
     (if (> (length dat) max-len)
	 (setq dat
	       (remove-if #'(lambda (vec) t) dat :start (- (length dat) 1))))))
  (:range-filter
   (min max)
   (map-filter
    dat
    #'(lambda (data)
	(let ((ret (in-range? min max data)))
;	  (unless ret (format t "~A is out of range~%" (send data :name)))
	  ret))))
  )


#|
(setq a (instance graphData :init))
(send a :add #f(-1 -1))
(send a :add #f(0 0))
(send a :add #f(1 1))
(send a :range-filter #f(-0.5 -0.5) #f(0.5 0.5))

